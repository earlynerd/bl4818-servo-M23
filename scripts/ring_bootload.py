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
CMD_BOOT_BROADCAST_BASE = 0x90

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

ProgressCallback = Optional[Callable[[dict], None]]


def _report(progress: ProgressCallback, phase: str, message: str, **details) -> None:
    if progress is not None:
        progress({"phase": phase, "message": message, **details})


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
        command_base: int = CMD_BOOT_BASE,
        retries: Optional[int] = None,
        timeout_ms: Optional[int] = None,
    ) -> bytes:
        payload = bytes([command_base + address, subcommand]) + data
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

    def begin_image(
        self,
        address: int,
        meta: ImageMetadata,
        *,
        command_base: int = CMD_BOOT_BASE,
    ) -> None:
        self._command(
            address,
            BOOT_SUB_BEGIN_IMAGE,
            struct.pack(">IIII", meta.image_size, meta.image_crc32, meta.image_version, meta.flags),
            command_base=command_base,
        )

    def erase_page(
        self,
        address: int,
        page_index: int,
        *,
        command_base: int = CMD_BOOT_BASE,
    ) -> None:
        detail = self._command(
            address,
            BOOT_SUB_ERASE_PAGE,
            bytes([page_index]),
            command_base=command_base,
        )
        if detail != bytes([page_index]):
            raise RingError(f"loader {address} acknowledged the wrong erased page")

    def write_chunk(
        self,
        address: int,
        offset: int,
        data: bytes,
        *,
        command_base: int = CMD_BOOT_BASE,
    ) -> None:
        if not data or len(data) > CHUNK_SIZE or len(data) % 4 or offset % 4:
            raise ValueError("chunk offset/length must be word aligned and length <= 32")
        detail = self._command(
            address,
            BOOT_SUB_WRITE_CHUNK,
            struct.pack(">H", offset) + data,
            command_base=command_base,
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

    def write_image_pages(
        self,
        address: int,
        image: bytes,
        meta: ImageMetadata,
        progress: Optional[Callable[[int, int], None]] = None,
        *,
        command_base: int = CMD_BOOT_BASE,
    ) -> None:
        if len(image) != meta.image_size or len(image) > APP_LIMIT:
            raise ValueError("prepared image and metadata disagree")

        page_count = (len(image) + PAGE_SIZE - 1) // PAGE_SIZE
        for page_index in range(page_count):
            self.erase_page(
                address,
                page_index,
                command_base=command_base,
            )
            page_start = page_index * PAGE_SIZE
            page_end = min(page_start + PAGE_SIZE, len(image))
            for offset in range(page_start, page_end, CHUNK_SIZE):
                chunk = image[offset : min(offset + CHUNK_SIZE, page_end)]
                if chunk != b"\xFF" * len(chunk):
                    self.write_chunk(
                        address,
                        offset,
                        chunk,
                        command_base=command_base,
                    )
            if progress is not None:
                progress(page_index + 1, page_count)

    def program_image(
        self,
        address: int,
        image: bytes,
        meta: ImageMetadata,
        progress: Optional[Callable[[int, int], None]] = None,
    ) -> None:
        self.begin_image(address, meta)
        self.write_image_pages(address, image, meta, progress)
        self.verify_image(address, meta.image_crc32)

    def update_image(
        self,
        address: int,
        image: bytes,
        meta: ImageMetadata,
        progress: Optional[Callable[[int, int], None]] = None,
    ) -> None:
        self.program_image(address, image, meta, progress)
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


def inspect_loader_targets(
    ring: RingClientV2,
    addresses: Optional[list[int]] = None,
    *,
    enter: bool = True,
    retries: int = 3,
    require_protocol3: bool = False,
    progress: ProgressCallback = None,
) -> tuple[int, list[int], BootloaderClient, list[BootInfo]]:
    """Enumerate and validate loader targets on an already-open ring.

    ``addresses=None`` means every enumerated actuator. The returned client
    shares the caller's serial connection, which lets the MIDI server perform
    an update without competing with a second process for the COM port.
    """
    _report(progress, "enumerating", "Enumerating ring")
    count = ring.enumerate()
    targets = list(range(count)) if addresses is None else list(dict.fromkeys(addresses))
    if not targets:
        raise RingError("no actuators enumerated")
    for address in targets:
        if not 0 <= address < count:
            raise RingError(
                f"address {address} is outside enumerated ring 0..{count - 1}"
            )

    loader = BootloaderClient(ring, retries=retries)
    if enter:
        for index, address in enumerate(targets, 1):
            _report(
                progress,
                "entering_loader",
                f"Entering loader on actuator {address}",
                address=address,
                done=index - 1,
                total=len(targets),
            )
            try:
                ring.enter_bootloader(address)
            except RingTimeout:
                # It may already be a loader; the identity probe below is
                # authoritative and avoids treating that as an app failure.
                pass
            wait_for_loader(loader, address)

    infos: list[BootInfo] = []
    for index, address in enumerate(targets, 1):
        info = loader.get_info(address)
        if (
            info.protocol_version not in (1, 2, 3)
            or info.page_size != PAGE_SIZE
            or info.app_limit != APP_LIMIT
        ):
            raise RingError(
                f"loader {address} reported incompatible geometry/version: {info}"
            )
        if require_protocol3 and info.protocol_version < 3:
            raise RingError(
                f"loader {address} protocol {info.protocol_version} "
                "does not support broadcast transfer"
            )
        infos.append(info)
        _report(
            progress,
            "loader_ready",
            f"Actuator {address}: loader v{info.loader_version}, UID {info.uid}",
            address=address,
            done=index,
            total=len(targets),
            protocol_version=info.protocol_version,
            loader_version=info.loader_version,
            uid=info.uid,
            reset_status=info.reset_status,
            entry_isp_control=info.entry_isp_control,
            entry_isp_status=info.entry_isp_status,
        )
    return count, targets, loader, infos


def update_ring(
    ring: RingClientV2,
    image: bytes,
    meta: ImageMetadata,
    addresses: Optional[list[int]] = None,
    *,
    broadcast_all: bool = False,
    enter: bool = True,
    retries: int = 3,
    progress: ProgressCallback = None,
) -> dict:
    """Update targets using an already-open ring connection.

    The protocol-3 path broadcasts only the idempotent data phase. CRC verify,
    addressed repair, manifest commit, and APROM restart remain per-device.
    """
    if len(image) != meta.image_size or len(image) > APP_LIMIT:
        raise ValueError("prepared image and metadata disagree")

    count, targets, loader, infos = inspect_loader_targets(
        ring,
        addresses,
        enter=enter,
        retries=retries,
        require_protocol3=broadcast_all,
        progress=progress,
    )
    repaired: list[int] = []

    if broadcast_all:
        pace_address = targets[-1]
        _report(
            progress,
            "broadcast_begin",
            f"Starting broadcast through tail address {pace_address}",
            tail_address=pace_address,
            count=count,
        )
        loader.begin_image(
            pace_address,
            meta,
            command_base=CMD_BOOT_BROADCAST_BASE,
        )
        for address in targets:
            info = loader.get_info(address)
            if (
                info.state_flags
                & (BOOT_FLAG_COMMITTED_VALID | BOOT_FLAG_UPDATE_ACTIVE)
            ) != BOOT_FLAG_UPDATE_ACTIVE:
                raise RingError(
                    f"loader {address} did not enter broadcast update state"
                )

        loader.write_image_pages(
            pace_address,
            image,
            meta,
            progress=lambda done, total: _report(
                progress,
                "writing",
                f"Broadcasting firmware pages {done}/{total}",
                done=done,
                total=total,
                tail_address=pace_address,
            ),
            command_base=CMD_BOOT_BROADCAST_BASE,
        )

        for index, address in enumerate(targets, 1):
            _report(
                progress,
                "verifying",
                f"Verifying actuator {address}",
                address=address,
                done=index - 1,
                total=len(targets),
            )
            try:
                loader.verify_image(address, meta.image_crc32)
            except RingError:
                repaired.append(address)

        for repair_index, address in enumerate(repaired, 1):
            loader.program_image(
                address,
                image,
                meta,
                progress=lambda done, total, a=address: _report(
                    progress,
                    "repairing",
                    f"Repairing actuator {a}: pages {done}/{total}",
                    address=a,
                    done=done,
                    total=total,
                    repair_index=repair_index,
                    repair_total=len(repaired),
                ),
            )

        for index, address in enumerate(targets, 1):
            _report(
                progress,
                "committing",
                f"Committing actuator {address}",
                address=address,
                done=index - 1,
                total=len(targets),
            )
            loader.commit_image(address, meta)
    else:
        for index, address in enumerate(targets, 1):
            loader.update_image(
                address,
                image,
                meta,
                progress=lambda done, total, a=address: _report(
                    progress,
                    "writing",
                    f"Writing actuator {a}: pages {done}/{total}",
                    address=a,
                    done=done,
                    total=total,
                    target_index=index,
                    target_total=len(targets),
                ),
            )

    for index, address in enumerate(targets, 1):
        _report(
            progress,
            "restarting",
            f"Restarting actuator {address}",
            address=address,
            done=index - 1,
            total=len(targets),
        )
        loader.run_application(address)
        wait_for_application(ring, address, count)

    result = {
        "count": count,
        "addresses": targets,
        "repaired": repaired,
        "image_size": meta.image_size,
        "image_crc32": meta.image_crc32,
        "image_version": meta.image_version,
        "loader_versions": [info.loader_version for info in infos],
    }
    _report(
        progress,
        "complete",
        f"Firmware update complete on {len(targets)} actuator(s)",
        **result,
    )
    return result


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
    target.add_argument(
        "--broadcast-all",
        action="store_true",
        help="broadcast the data phase, then verify/commit each loader",
    )
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
        addresses = None if args.all or args.broadcast_all else args.addresses
        inline = False

        def show_progress(event: dict) -> None:
            nonlocal inline
            phase = event["phase"]
            if phase in ("writing", "repairing"):
                print(f"\r{event['message']}", end="", flush=True)
                inline = True
                return
            if inline:
                print()
                inline = False
            print(event["message"])
            if phase == "loader_ready" and event.get("reset_status") is not None:
                print(
                    f"[{event['address']}] entry "
                    f"RSTSTS=0x{event['reset_status']:08X}, "
                    f"ISPCTL=0x{event['entry_isp_control']:08X}, "
                    f"ISPSTS=0x{event['entry_isp_status']:08X}"
                )

        if args.info_only:
            inspect_loader_targets(
                ring,
                addresses,
                enter=not args.no_enter,
                retries=args.retries,
                require_protocol3=args.broadcast_all,
                progress=show_progress,
            )
        else:
            update_ring(
                ring,
                image,
                meta,
                addresses,
                broadcast_all=args.broadcast_all,
                enter=not args.no_enter,
                retries=args.retries,
                progress=show_progress,
            )
    finally:
        ring.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RingError) as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)
