#!/usr/bin/env python3
"""Build and inspect the APROM image/manifest pair used by the LDROM loader."""

from __future__ import annotations

import argparse
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path


APP_LIMIT = 0x7A00
SRAM_BASE = 0x20000000
BOOT_MAILBOX_BASE = 0x20000FF0
MANIFEST_PAGE_SIZE = 0x200
MANIFEST_MAGIC = 0x4633324D
MANIFEST_VERSION = 1
MANIFEST_SIZE = 32
MANIFEST_STRUCT = struct.Struct("<IHHIIIIII")


@dataclass(frozen=True)
class ImageMetadata:
    image_size: int
    image_crc32: int
    image_version: int
    flags: int = 0


def prepare_image(data: bytes) -> bytes:
    """Pad an application binary to the M2003's 32-bit program width."""
    if not data:
        raise ValueError("application image is empty")
    padding = (-len(data)) % 4
    if padding:
        data += b"\xFF" * padding
    if len(data) < 8:
        raise ValueError("application image must contain an initial stack and reset vector")
    if len(data) > APP_LIMIT:
        raise ValueError(
            f"application is {len(data)} bytes; limit is {APP_LIMIT} bytes"
        )
    initial_stack, reset_vector = struct.unpack_from("<II", data)
    reset_address = reset_vector & ~1
    if not SRAM_BASE <= initial_stack <= BOOT_MAILBOX_BASE or initial_stack % 4:
        raise ValueError("application initial stack is outside usable M2003 SRAM")
    if not reset_vector & 1 or reset_address < 8 or reset_address >= len(data):
        raise ValueError("application reset vector is outside the image or not Thumb")
    return data


def image_metadata(data: bytes, image_version: int = 0, flags: int = 0) -> ImageMetadata:
    data = prepare_image(data)
    if not 0 <= image_version <= 0xFFFFFFFF:
        raise ValueError("image version must fit uint32")
    if flags != 0:
        raise ValueError("manifest flags must be zero for format version 1")
    return ImageMetadata(
        image_size=len(data),
        image_crc32=zlib.crc32(data) & 0xFFFFFFFF,
        image_version=image_version,
        flags=flags,
    )


def build_manifest(data: bytes, image_version: int = 0, flags: int = 0) -> bytes:
    meta = image_metadata(data, image_version=image_version, flags=flags)
    prefix = struct.pack(
        "<IHHIIIII",
        MANIFEST_MAGIC,
        MANIFEST_VERSION,
        MANIFEST_SIZE,
        meta.image_size,
        meta.image_crc32,
        meta.image_version,
        meta.flags,
        0,
    )
    manifest_crc32 = zlib.crc32(prefix) & 0xFFFFFFFF
    header = prefix + struct.pack("<I", manifest_crc32)
    if len(header) != MANIFEST_STRUCT.size:
        raise AssertionError("manifest layout drifted from the firmware contract")
    return header + b"\xFF" * (MANIFEST_PAGE_SIZE - len(header))


def parse_manifest(page: bytes) -> ImageMetadata:
    if len(page) < MANIFEST_SIZE:
        raise ValueError("manifest is shorter than 32 bytes")
    fields = MANIFEST_STRUCT.unpack_from(page)
    magic, version, header_size = fields[:3]
    if magic != MANIFEST_MAGIC or version != MANIFEST_VERSION or header_size != MANIFEST_SIZE:
        raise ValueError("manifest magic/version/header size is invalid")
    if zlib.crc32(page[: MANIFEST_SIZE - 4]) & 0xFFFFFFFF != fields[-1]:
        raise ValueError("manifest CRC-32 does not match")
    if fields[3] < 8 or fields[3] > APP_LIMIT or fields[3] % 4:
        raise ValueError("manifest image size is outside the application region")
    if fields[6] != 0 or fields[7] != 0:
        raise ValueError("manifest flags/reserved fields are unsupported")
    return ImageMetadata(
        image_size=fields[3],
        image_crc32=fields[4],
        image_version=fields[5],
        flags=fields[6],
    )


def _int_auto(value: str) -> int:
    return int(value, 0)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create a word-padded APROM image and its commit manifest"
    )
    parser.add_argument("image", type=Path, help="application .bin")
    parser.add_argument("--image-version", type=_int_auto, default=0)
    parser.add_argument("--image-out", type=Path)
    parser.add_argument("--manifest-out", type=Path)
    args = parser.parse_args()

    image = prepare_image(args.image.read_bytes())
    manifest = build_manifest(image, args.image_version)
    meta = parse_manifest(manifest)

    image_out = args.image_out or args.image.with_name(args.image.stem + ".update.bin")
    manifest_out = args.manifest_out or args.image.with_name(
        args.image.stem + ".manifest.bin"
    )
    image_out.write_bytes(image)
    manifest_out.write_bytes(manifest)
    print(
        f"image={meta.image_size} bytes crc32=0x{meta.image_crc32:08X} "
        f"version=0x{meta.image_version:08X}"
    )
    print(f"wrote {image_out}")
    print(f"wrote {manifest_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
