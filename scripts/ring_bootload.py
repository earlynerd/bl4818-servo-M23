#!/usr/bin/env python3
"""Power-loss-safe APROM updater for the Gen1 M2003 UART ring."""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

from firmware_image import APP_LIMIT, ImageMetadata, image_metadata, prepare_image
from ring_bus import RingClientV2, RingError, RingTimeout, auto_detect_port


CMD_STAY_BOOT = 0x61
CMD_BOOT_BASE = 0x70
CMD_BOOT_REPLY_BASE = 0x80

BOOT_SUB_GET_INFO = 0x01
BOOT_SUB_BEGIN_IMAGE = 0x10
BOOT_SUB_ERASE_PAGE = 0x11
BOOT_SUB_WRITE_CHUNK = 0x12
BOOT_SUB_VERIFY_IMAGE = 0x13
BOOT_SUB_COMMIT_IMAGE = 0x14
BOOT_SUB_RUN_APROM = 0x15

BOOT_RESULT_OK = 0x00
BOOT_RESULT_NAMES = {
    0x00: "OK",
    0x01: "BAD_COMMAND",
    0x02: "BAD_STATE",
    0x03: "BAD_RANGE",
    0x04: "FLASH_FAILED",
    0x05: "CRC_MISMATCH",
}

BOOT_FLAG_COMMITTED_VALID = 0x01
BOOT_FLAG_UPDATE_ACTIVE = 0x02
PAGE_SIZE = 0x200
CHUNK_SIZE = 32


@dataclass(frozen=True)
class BootInfo:
    protocol_version: int
    loader_version: int
    state_flags: int
    address: int
    page_size: int
    app_limit: int
    image_size: int
    image_crc32: int
    image_version: int
    uid: str
    reset_status: Optional[int] = None
    entry_isp_control: Optional[int] = None
    entry_isp_status: Optional[int] = None

    def committed_matches(self, meta: ImageMetadata) -> bool:
        return bool(self.state_flags & BOOT_FLAG_COMMITTED_VALID) and (
            self.image_size == meta.image_size
            and self.image_crc32 == meta.image_crc32
            and self.image_version == meta.image_version
        )


class BootloaderClient:
    def __init__(self, ring: RingClientV2, retries: int = 3):
        self.ring = ring
        self.retries = retries

    def _command(
        self,
        address: int,
        subcommand: int,
        data: bytes = b"",
        *,
        retries: Optional[int] = None,
        timeout_ms: Optional[int] = None,
    ) -> bytes:
        payload = bytes([CMD_BOOT_BASE + address, subcommand]) + data
        attempts = retries if retries is not None else self.retries
        last_timeout: Optional[RingTimeout] = None

        for _ in range(attempts):
            self.ring._send_frame(payload)
            try:
                for _ in range(8):
                    reply = self.ring._recv_frame(timeout_ms=timeout_ms)
                    if reply == payload:
                        continue
                    if (
                        len(reply) >= 3
                        and reply[0] == CMD_BOOT_REPLY_BASE + address
                        and reply[1] == subcommand
                    ):
                        result = reply[2]
                        if result != BOOT_RESULT_OK:
                            name = BOOT_RESULT_NAMES.get(result, f"0x{result:02X}")
                            raise RingError(
                                f"loader {address} {subcommand:#04x} failed: {name}"
                            )
                        return reply[3:]
            except RingTimeout as exc:
                last_timeout = exc
                continue

        if last_timeout is not None:
            raise last_timeout
        raise RingError(f"loader {address} returned no matching reply")

    def get_info(self, address: int) -> BootInfo:
        detail = self._command(address, BOOT_SUB_GET_INFO)
        if len(detail) not in (33, 45):
            raise RingError(f"loader {address} GET_INFO returned {len(detail)} detail bytes")
        uid_words = struct.unpack_from(">III", detail, 21)
        return BootInfo(
            protocol_version=detail[0],
            loader_version=struct.unpack_from(">H", detail, 1)[0],
            state_flags=detail[3],
            address=detail[4],
            page_size=struct.unpack_from(">H", detail, 5)[0],
            app_limit=struct.unpack_from(">H", detail, 7)[0],
            image_size=struct.unpack_from(">I", detail, 9)[0],
            image_crc32=struct.unpack_from(">I", detail, 13)[0],
            image_version=struct.unpack_from(">I", detail, 17)[0],
            uid="".join(f"{word:08X}" for word in uid_words),
            reset_status=struct.unpack_from(">I", detail, 33)[0]
            if len(detail) >= 45
            else None,
            entry_isp_control=struct.unpack_from(">I", detail, 37)[0]
            if len(detail) >= 45
            else None,
            entry_isp_status=struct.unpack_from(">I", detail, 41)[0]
            if len(detail) >= 45
            else None,
        )

    def begin_image(self, address: int, meta: ImageMetadata) -> None:
        self._command(
            address,
            BOOT_SUB_BEGIN_IMAGE,
            struct.pack(">IIII", meta.image_size, meta.image_crc32, meta.image_version, meta.flags),
        )

    def erase_page(self, address: int, page_index: int) -> None:
        detail = self._command(address, BOOT_SUB_ERASE_PAGE, bytes([page_index]))
        if detail != bytes([page_index]):
            raise RingError(f"loader {address} acknowledged the wrong erased page")

    def write_chunk(self, address: int, offset: int, data: bytes) -> None:
        if not data or len(data) > CHUNK_SIZE or len(data) % 4 or offset % 4:
            raise ValueError("chunk offset/length must be word aligned and length <= 32")
        detail = self._command(
            address,
            BOOT_SUB_WRITE_CHUNK,
            struct.pack(">H", offset) + data,
        )
        if detail != struct.pack(">H", offset):
            raise RingError(f"loader {address} acknowledged the wrong chunk offset")

    def verify_image(self, address: int, expected_crc32: int) -> None:
        detail = self._command(
            address,
            BOOT_SUB_VERIFY_IMAGE,
            timeout_ms=2000,
        )
        if len(detail) != 4 or struct.unpack(">I", detail)[0] != expected_crc32:
            raise RingError(f"loader {address} returned an unexpected verification CRC")

    def commit_image(self, address: int, meta: ImageMetadata) -> None:
        try:
            self._command(
                address,
                BOOT_SUB_COMMIT_IMAGE,
                retries=1,
                timeout_ms=2500,
            )
        except RingTimeout:
            # COMMIT is intentionally not blindly retried: magic may already
            # have been written. Resolve an ACK loss by reading committed state.
            info = self.get_info(address)
            if not info.committed_matches(meta):
                raise

    def run_application(self, address: int) -> None:
        self._command(address, BOOT_SUB_RUN_APROM, retries=2, timeout_ms=1000)

    def update_image(
        self,
        address: int,
        image: bytes,
        meta: ImageMetadata,
        progress: Optional[Callable[[int, int], None]] = None,
    ) -> None:
        if len(image) != meta.image_size or len(image) > APP_LIMIT:
            raise ValueError("prepared image and metadata disagree")

        self.begin_image(address, meta)
        page_count = (len(image) + PAGE_SIZE - 1) // PAGE_SIZE
        for page_index in range(page_count):
            self.erase_page(address, page_index)
            page_start = page_index * PAGE_SIZE
            page_end = min(page_start + PAGE_SIZE, len(image))
            for offset in range(page_start, page_end, CHUNK_SIZE):
                chunk = image[offset : min(offset + CHUNK_SIZE, page_end)]
                if chunk != b"\xFF" * len(chunk):
                    self.write_chunk(address, offset, chunk)
            if progress is not None:
                progress(page_index + 1, page_count)

        self.verify_image(address, meta.image_crc32)
        self.commit_image(address, meta)


def hold_boot_window(ring: RingClientV2, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    payload = bytes([CMD_STAY_BOOT])
    while time.monotonic() < deadline:
        ring._send_frame(payload)
        time.sleep(0.020)
    ring._flush_rx()


def wait_for_loader(
    loader: BootloaderClient, address: int, timeout_seconds: float = 3.0
) -> BootInfo:
    deadline = time.monotonic() + timeout_seconds
    last_error: Optional[RingError] = None
    while time.monotonic() < deadline:
        try:
            return loader.get_info(address)
        except RingError as exc:
            last_error = exc
            time.sleep(0.050)
    raise RingTimeout(f"loader {address} did not become reachable: {last_error}")


def wait_for_application(
    ring: RingClientV2,
    address: int,
    expected_count: int,
    timeout_seconds: float = 3.0,
) -> None:
    deadline = time.monotonic() + timeout_seconds
    last_error: Optional[RingError] = None
    while time.monotonic() < deadline:
        try:
            count = ring.enumerate()
            if count != expected_count:
                raise RingError(
                    f"ring count changed after update: expected {expected_count}, got {count}"
                )
            ring.query_status(address)
            return
        except RingError as exc:
            last_error = exc
            time.sleep(0.050)
    raise RingTimeout(f"application {address} did not rejoin the ring: {last_error}")


def _int_auto(value: str) -> int:
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Update M2003 APROM over the Gen1 UART ring"
    )
    parser.add_argument("image", type=Path, help="build/m2003-motor.bin")
    parser.add_argument("-p", "--port", help="serial port (auto-detect if omitted)")
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--addr", type=int, action="append", dest="addresses")
    target.add_argument("--all", action="store_true")
    parser.add_argument("--image-version", type=_int_auto, default=0)
    parser.add_argument("--no-enter", action="store_true", help="targets are already in LDROM")
    parser.add_argument(
        "--recover-seconds",
        type=float,
        default=0.0,
        help="stream STAY_IN_BOOTLOADER while power is applied, then enumerate",
    )
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--timeout-ms", type=int, default=500)
    parser.add_argument("--trace", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="validate/package only")
    parser.add_argument(
        "--info-only",
        action="store_true",
        help="query loader identity and entry diagnostics without updating",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    image = prepare_image(args.image.read_bytes())
    meta = image_metadata(image, image_version=args.image_version)
    print(
        f"image {meta.image_size} bytes, CRC32 0x{meta.image_crc32:08X}, "
        f"version 0x{meta.image_version:08X}"
    )
    if args.dry_run:
        return 0

    port = args.port or auto_detect_port()
    if not port:
        raise RingError("no serial port found; pass --port")

    ring = RingClientV2(
        port=port,
        timeout_ms=args.timeout_ms,
        trace=args.trace,
    )
    ring.open()
    try:
        if args.recover_seconds > 0:
            print(f"streaming boot hold for {args.recover_seconds:.1f}s; apply power now")
            hold_boot_window(ring, args.recover_seconds)

        count = ring.enumerate()
        addresses = list(range(count)) if args.all else list(dict.fromkeys(args.addresses))
        for address in addresses:
            if not 0 <= address < count:
                raise RingError(f"address {address} is outside enumerated ring 0..{count - 1}")

        loader = BootloaderClient(ring, retries=args.retries)
        if not args.no_enter:
            for address in addresses:
                try:
                    ring.enter_bootloader(address)
                except RingTimeout:
                    # It may already be a loader; the identity probe below is
                    # authoritative and avoids treating that as an app failure.
                    pass
                wait_for_loader(loader, address)

        for address in addresses:
            info = loader.get_info(address)
            if info.protocol_version not in (1, 2) or info.page_size != PAGE_SIZE or info.app_limit != APP_LIMIT:
                raise RingError(f"loader {address} reported incompatible geometry/version: {info}")
            print(f"[{address}] loader v{info.loader_version}, UID {info.uid}")
            if info.reset_status is not None:
                print(
                    f"[{address}] entry RSTSTS=0x{info.reset_status:08X}, "
                    f"ISPCTL=0x{info.entry_isp_control:08X}, "
                    f"ISPSTS=0x{info.entry_isp_status:08X}"
                )
            if args.info_only:
                continue
            loader.update_image(
                address,
                image,
                meta,
                progress=lambda done, total, a=address: print(
                    f"\r[{a}] pages {done}/{total}", end="", flush=True
                ),
            )
            print(f"\n[{address}] verified and committed")
            loader.run_application(address)
            wait_for_application(ring, address, count)
            print(f"[{address}] application re-enumerated and answered status")
    finally:
        ring.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RingError) as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)
