import struct
import sys
import unittest
import zlib
from pathlib import Path
from unittest.mock import patch


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
    BOOT_SUB_BEGIN_IMAGE,
    BOOT_SUB_COMMIT_IMAGE,
    BOOT_SUB_ERASE_PAGE,
    BOOT_SUB_GET_INFO,
    BOOT_SUB_VERIFY_IMAGE,
    BOOT_SUB_WRITE_CHUNK,
    CMD_BOOT_BASE,
    CMD_BOOT_BROADCAST_BASE,
    CMD_BOOT_REPLY_BASE,
    BootInfo,
    BootloaderClient,
    update_ring,
)
from ring_bus import RingError, RingTimeout  # noqa: E402


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


class BootloaderUpdateTests(unittest.TestCase):
    def test_broadcast_verifies_repairs_commits_and_restarts_individually(self):
        image = prepare_image(
            struct.pack("<II", 0x20000FF0, 0x00000009) + b"firmware"
        )
        meta = image_metadata(image, image_version=9)

        class OpenRing:
            def __init__(self):
                self.entered = []
                self.status_queries = []

            def enumerate(self):
                return 3

            def enter_bootloader(self, address):
                self.entered.append(address)

            def query_status(self, address):
                self.status_queries.append(address)

        class Loader:
            def __init__(self):
                self.begin = []
                self.writes = []
                self.verified = []
                self.repaired = []
                self.committed = []
                self.started = []

            @staticmethod
            def _info(address):
                return BootInfo(
                    protocol_version=3,
                    loader_version=3,
                    state_flags=0x02,
                    address=address,
                    page_size=0x200,
                    app_limit=APP_LIMIT,
                    image_size=0,
                    image_crc32=0,
                    image_version=0,
                    uid=f"{address:024X}",
                )

            def get_info(self, address):
                return self._info(address)

            def begin_image(self, address, update_meta, *, command_base):
                self.begin.append((address, update_meta, command_base))

            def write_image_pages(self, address, update_image, update_meta,
                                  progress, *, command_base):
                self.writes.append((address, update_image, update_meta, command_base))
                progress(54, 54)

            def verify_image(self, address, expected_crc32):
                self.verified.append((address, expected_crc32))
                if address == 1:
                    raise RingError("injected CRC mismatch")

            def program_image(self, address, update_image, update_meta, progress):
                self.repaired.append(address)
                progress(54, 54)

            def commit_image(self, address, update_meta):
                self.committed.append(address)

            def run_application(self, address):
                self.started.append(address)

        ring = OpenRing()
        loader = Loader()
        events = []
        with patch("ring_bootload.BootloaderClient", return_value=loader):
            result = update_ring(
                ring,
                image,
                meta,
                broadcast_all=True,
                progress=events.append,
            )

        self.assertEqual(ring.entered, [0, 1, 2])
        self.assertEqual(loader.begin[0][0::2], (2, CMD_BOOT_BROADCAST_BASE))
        self.assertEqual(loader.writes[0][0], 2)
        self.assertEqual(loader.writes[0][3], CMD_BOOT_BROADCAST_BASE)
        self.assertEqual([address for address, _ in loader.verified], [0, 1, 2])
        self.assertEqual(loader.repaired, [1])
        self.assertEqual(loader.committed, [0, 1, 2])
        self.assertEqual(loader.started, [0, 1, 2])
        self.assertEqual(ring.status_queries, [0, 1, 2])
        self.assertEqual(result["repaired"], [1])
        self.assertIn("writing", [event["phase"] for event in events])
        self.assertEqual(events[-1]["phase"], "complete")

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

    def test_broadcast_chunk_is_paced_by_tail_reply(self):
        tail_address = 5
        data = b"\x10\x20\x30\x40"
        command = bytes(
            [CMD_BOOT_BROADCAST_BASE + tail_address, BOOT_SUB_WRITE_CHUNK]
        ) + struct.pack(">H", 0x20) + data
        reply = bytes(
            [
                CMD_BOOT_REPLY_BASE + tail_address,
                BOOT_SUB_WRITE_CHUNK,
                0,
                0,
                0x20,
            ]
        )
        ring = FakeRing([command, reply])

        BootloaderClient(ring).write_chunk(
            tail_address,
            0x20,
            data,
            command_base=CMD_BOOT_BROADCAST_BASE,
        )

        self.assertEqual(ring.sent, [command])

    def test_program_image_stops_before_manifest_commit(self):
        address = 1
        image = prepare_image(
            struct.pack("<II", 0x20000FF0, 0x00000009) + b"firmware"
        )
        meta = image_metadata(image, image_version=8)
        begin = bytes([CMD_BOOT_REPLY_BASE + address, BOOT_SUB_BEGIN_IMAGE, 0])
        erase = bytes(
            [CMD_BOOT_REPLY_BASE + address, BOOT_SUB_ERASE_PAGE, 0, 0]
        )
        write = bytes(
            [CMD_BOOT_REPLY_BASE + address, BOOT_SUB_WRITE_CHUNK, 0, 0, 0]
        )
        verify = bytes(
            [CMD_BOOT_REPLY_BASE + address, BOOT_SUB_VERIFY_IMAGE, 0]
        ) + struct.pack(">I", meta.image_crc32)
        ring = FakeRing([begin, erase, write, verify])

        BootloaderClient(ring).program_image(address, image, meta)

        self.assertEqual(
            [payload[1] for payload in ring.sent],
            [
                BOOT_SUB_BEGIN_IMAGE,
                BOOT_SUB_ERASE_PAGE,
                BOOT_SUB_WRITE_CHUNK,
                BOOT_SUB_VERIFY_IMAGE,
            ],
        )


if __name__ == "__main__":
    unittest.main()
