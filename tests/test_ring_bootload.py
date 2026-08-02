import struct
import sys
import unittest
import zlib
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from firmware_image import (  # noqa: E402
    APP_LIMIT,
    MANIFEST_MAGIC,
    MANIFEST_PAGE_SIZE,
    build_manifest,
    image_metadata,
    parse_manifest,
    prepare_image,
)
from ring_bootload import (  # noqa: E402
    BOOT_FLAG_COMMITTED_VALID,
    BOOT_SUB_COMMIT_IMAGE,
    BOOT_SUB_GET_INFO,
    BOOT_SUB_WRITE_CHUNK,
    CMD_BOOT_BASE,
    CMD_BOOT_REPLY_BASE,
    BootloaderClient,
)
from ring_bus import RingTimeout  # noqa: E402


class FakeRing:
    def __init__(self, replies):
        self.replies = list(replies)
        self.sent = []

    def _send_frame(self, payload):
        self.sent.append(payload)

    def _recv_frame(self, timeout_ms=None):
        if not self.replies:
            raise RingTimeout("empty fake reply queue")
        reply = self.replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return reply


def info_reply(
    address,
    *,
    flags,
    size,
    crc32,
    version,
    protocol_version=1,
    diagnostics=None,
):
    detail = bytearray()
    detail.extend(struct.pack(">BHB B H H I I I", protocol_version, protocol_version, flags, address, 0x200, APP_LIMIT, size, crc32, version))
    detail.extend(struct.pack(">III", 0x11111111, 0x22222222, 0x33333333))
    if diagnostics is not None:
        detail.extend(struct.pack(">III", *diagnostics))
    return bytes([CMD_BOOT_REPLY_BASE + address, BOOT_SUB_GET_INFO, 0]) + bytes(detail)


class FirmwareImageTests(unittest.TestCase):
    def test_padding_crc_and_manifest_round_trip(self):
        raw = struct.pack("<II", 0x20000FF0, 0x00000009) + b"\x04"
        image = prepare_image(raw)
        self.assertEqual(image, raw + b"\xFF\xFF\xFF")
        meta = image_metadata(image, image_version=0x10203)
        self.assertEqual(meta.image_crc32, zlib.crc32(image) & 0xFFFFFFFF)

        page = build_manifest(image, image_version=0x10203)
        self.assertEqual(len(page), MANIFEST_PAGE_SIZE)
        self.assertEqual(struct.unpack_from("<I", page)[0], MANIFEST_MAGIC)
        self.assertEqual(parse_manifest(page), meta)
        self.assertEqual(page[32:], b"\xFF" * (MANIFEST_PAGE_SIZE - 32))

    def test_image_limit_is_enforced(self):
        with self.assertRaises(ValueError):
            prepare_image(b"x" * (APP_LIMIT + 1))

    def test_boot_vectors_are_validated_before_packaging(self):
        with self.assertRaisesRegex(ValueError, "initial stack"):
            prepare_image(struct.pack("<II", 0x1000, 0x00000009))
        with self.assertRaisesRegex(ValueError, "reset vector"):
            prepare_image(struct.pack("<II", 0x20000FF0, 0x00000008))

    def test_manifest_is_invalid_until_magic_is_written_last(self):
        image = prepare_image(
            struct.pack("<II", 0x20000FF0, 0x00000009) + b"recoverable firmware"
        )
        complete = build_manifest(image, image_version=9)
        flash_page = bytearray(b"\xFF" * MANIFEST_PAGE_SIZE)

        # BEGIN_IMAGE erases the page. A reset now must not expose an image.
        with self.assertRaises(ValueError):
            parse_manifest(flash_page)

        # COMMIT writes every non-magic word first. Power loss at any point in
        # this loop remains invalid because word zero is still erased.
        for offset in range(4, 32, 4):
            flash_page[offset : offset + 4] = complete[offset : offset + 4]
            with self.assertRaises(ValueError):
                parse_manifest(flash_page)

        flash_page[:4] = complete[:4]
        self.assertEqual(parse_manifest(flash_page).image_version, 9)


class BootloaderClientTests(unittest.TestCase):
    def test_command_skips_cut_through_echo(self):
        address = 3
        command = bytes([CMD_BOOT_BASE + address, BOOT_SUB_GET_INFO])
        reply = info_reply(address, flags=0, size=0x100, crc32=1, version=2)
        ring = FakeRing([command, reply])

        info = BootloaderClient(ring).get_info(address)

        self.assertEqual(info.address, address)
        self.assertEqual(info.uid, "111111112222222233333333")
        self.assertIsNone(info.reset_status)
        self.assertEqual(ring.sent, [command])

    def test_protocol_v2_reports_loader_entry_diagnostics(self):
        address = 0
        diagnostics = (0x00000020, 0x00000081, 0x00100000)
        reply = info_reply(
            address,
            flags=0,
            size=0x100,
            crc32=1,
            version=2,
            protocol_version=2,
            diagnostics=diagnostics,
        )

        info = BootloaderClient(FakeRing([reply])).get_info(address)

        self.assertEqual(info.protocol_version, 2)
        self.assertEqual(
            (info.reset_status, info.entry_isp_control, info.entry_isp_status),
            diagnostics,
        )

    def test_lost_commit_ack_is_resolved_from_manifest(self):
        address = 2
        image = prepare_image(
            struct.pack("<II", 0x20000FF0, 0x00000009) + b"firmware"
        )
        meta = image_metadata(image, image_version=7)
        timeout = RingTimeout("commit ACK lost")
        committed = info_reply(
            address,
            flags=BOOT_FLAG_COMMITTED_VALID,
            size=meta.image_size,
            crc32=meta.image_crc32,
            version=meta.image_version,
        )
        ring = FakeRing([timeout, committed])

        BootloaderClient(ring).commit_image(address, meta)

        self.assertEqual(ring.sent[0][1], BOOT_SUB_COMMIT_IMAGE)
        self.assertEqual(ring.sent[1][1], BOOT_SUB_GET_INFO)

    def test_chunk_retry_reuses_identical_idempotent_command(self):
        address = 1
        data = b"\x10\x20\x30\x40"
        command = bytes([CMD_BOOT_BASE + address, BOOT_SUB_WRITE_CHUNK]) + struct.pack(
            ">H", 0x20
        ) + data
        reply = bytes(
            [CMD_BOOT_REPLY_BASE + address, BOOT_SUB_WRITE_CHUNK, 0, 0, 0x20]
        )
        ring = FakeRing([RingTimeout("lost ACK"), command, reply])

        BootloaderClient(ring, retries=2).write_chunk(address, 0x20, data)

        self.assertEqual(ring.sent, [command, command])


if __name__ == "__main__":
    unittest.main()
