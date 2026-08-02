#!/usr/bin/env python3
"""Generate M2003 firmware-stack and configuration-capture J-Link scripts."""

from __future__ import annotations

import argparse
import struct
import zlib
from pathlib import Path

from firmware_image import APP_LIMIT, MANIFEST_PAGE_SIZE, parse_manifest


SYS_REGLCTL = 0x40000100
CLK_AHBCLK = 0x40000204
FMC_ISPCTL = 0x4000C000
FMC_ISPADDR = 0x4000C004
FMC_ISPDAT = 0x4000C008
FMC_ISPCMD = 0x4000C00C
FMC_ISPTRG = 0x4000C010
FMC_ISPSTS = 0x4000C040

CONFIG_BASE = 0x00300000
LDROM_BASE = 0x00100000
LDROM_SIZE = 0x1000
MANIFEST_BASE = 0x00007A00
PAGE_SIZE = 0x200

CMD_READ = 0x00
CMD_PROGRAM = 0x21
CMD_PAGE_ERASE = 0x22
ISPCTL_ENABLE_APROM = 0x09
ISPCTL_ENABLE_LDROM = 0x21
ISPCTL_READ_ONLY = 0x01
ISPCTL_READ_ONLY_LDROM_BOOT = 0x03
AHBCLK_RESET_WITH_ISP = 0x00000103


def _padded_words(data: bytes) -> tuple[bytes, tuple[int, ...]]:
    if len(data) % 4:
        data += b"\xFF" * (-len(data) % 4)
    return data, struct.unpack(f"<{len(data) // 4}I", data)


def _slash(path: Path) -> str:
    return str(path.resolve()).replace("\\", "/")


def _append_unlock(lines: list[str]) -> None:
    lines.extend(
        [
            f"w4 0x{SYS_REGLCTL:08X} 0x00000059",
            f"w4 0x{SYS_REGLCTL:08X} 0x00000016",
            f"w4 0x{SYS_REGLCTL:08X} 0x00000088",
        ]
    )


def _append_region(
    lines: list[str],
    *,
    name: str,
    base: int,
    data: bytes,
    ispctl: int,
    erase_size: int | None = None,
    commit_word_last: bool = False,
) -> None:
    data, words = _padded_words(data)
    erase_length = erase_size if erase_size is not None else len(data)
    page_count = (erase_length + PAGE_SIZE - 1) // PAGE_SIZE
    lines.append(f"// {name}: {len(data)} bytes, erase {page_count} pages")
    lines.append(f"w4 0x{FMC_ISPCTL:08X} 0x{ispctl:08X}")
    for page in range(page_count):
        address = base + page * PAGE_SIZE
        lines.extend(
            [
                f"w4 0x{FMC_ISPADDR:08X} 0x{address:08X}",
                f"w4 0x{FMC_ISPCMD:08X} 0x{CMD_PAGE_ERASE:08X}",
                f"w4 0x{FMC_ISPTRG:08X} 0x00000001",
                "sleep 20",
                f"mem32 0x{FMC_ISPTRG:08X} 1",
            ]
        )
    indices = list(range(len(words)))
    if commit_word_last:
        indices = indices[1:] + indices[:1]
    for index in indices:
        word = words[index]
        if word == 0xFFFFFFFF:
            continue
        address = base + index * 4
        lines.extend(
            [
                f"w4 0x{FMC_ISPADDR:08X} 0x{address:08X}",
                f"w4 0x{FMC_ISPDAT:08X} 0x{word:08X}",
                f"w4 0x{FMC_ISPCMD:08X} 0x{CMD_PROGRAM:08X}",
                f"w4 0x{FMC_ISPTRG:08X} 0x00000001",
                f"mem32 0x{FMC_ISPTRG:08X} 1",
            ]
        )


def _append_physical_reads(lines: list[str], base: int, length: int) -> None:
    if length % 4:
        raise ValueError("physical FMC read length must be word aligned")
    for address in range(base, base + length, 4):
        lines.extend(
            [
                f"w4 0x{FMC_ISPADDR:08X} 0x{address:08X}",
                f"w4 0x{FMC_ISPCMD:08X} 0x{CMD_READ:08X}",
                f"w4 0x{FMC_ISPTRG:08X} 0x00000001",
                f"mem32 0x{FMC_ISPTRG:08X} 1",
                f"mem32 0x{FMC_ISPDAT:08X} 1",
            ]
        )


def generate_provision_script(
    app_path: Path,
    manifest_path: Path,
    ldrom_path: Path,
    output_path: Path,
) -> None:
    app = app_path.read_bytes()
    manifest = manifest_path.read_bytes()
    ldrom = ldrom_path.read_bytes()
    if not app or len(app) > APP_LIMIT or len(app) % 4:
        raise ValueError("packaged application must be nonempty, word aligned, and <= 0x7A00")
    if len(manifest) != MANIFEST_PAGE_SIZE:
        raise ValueError("manifest must be exactly one 512-byte page")
    meta = parse_manifest(manifest)
    if meta.image_size != len(app):
        raise ValueError("manifest size does not match packaged application")
    if meta.image_crc32 != zlib.crc32(app) & 0xFFFFFFFF:
        raise ValueError("manifest CRC does not match packaged application")
    if not ldrom or len(ldrom) > LDROM_SIZE:
        raise ValueError("LDROM image must be 1..4096 bytes")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "// M2003 one-time loader provisioning (NO CONFIG WRITES)",
        "r",
        "h",
    ]
    _append_unlock(lines)
    lines.append(f"w4 0x{CLK_AHBCLK:08X} 0x{AHBCLK_RESET_WITH_ISP:08X}")
    # Invalidate the old commit before touching APROM. The manifest region
    # helper erases this page again immediately before writing the new commit.
    lines.append(f"w4 0x{FMC_ISPCTL:08X} 0x{ISPCTL_ENABLE_APROM:08X}")
    lines.extend(
        [
            f"w4 0x{FMC_ISPADDR:08X} 0x{MANIFEST_BASE:08X}",
            f"w4 0x{FMC_ISPCMD:08X} 0x{CMD_PAGE_ERASE:08X}",
            f"w4 0x{FMC_ISPTRG:08X} 0x00000001",
            "sleep 20",
            f"mem32 0x{FMC_ISPTRG:08X} 1",
        ]
    )
    _append_region(
        lines,
        name="APROM application",
        base=0,
        data=app,
        ispctl=ISPCTL_ENABLE_APROM,
    )
    _append_region(
        lines,
        name="APROM committed-image manifest",
        base=MANIFEST_BASE,
        data=manifest,
        ispctl=ISPCTL_ENABLE_APROM,
        erase_size=MANIFEST_PAGE_SIZE,
        commit_word_last=True,
    )
    _append_region(
        lines,
        name="permanent LDROM loader",
        base=LDROM_BASE,
        data=ldrom,
        ispctl=ISPCTL_ENABLE_LDROM,
        erase_size=LDROM_SIZE,
    )
    lines.extend(
        [
            f"w4 0x{FMC_ISPCTL:08X} 0x{ISPCTL_READ_ONLY:08X}",
            f"w4 0x{SYS_REGLCTL:08X} 0x00000000",
            "r",
            "g",
            "exit",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="ascii")


def generate_verify_script(
    app_path: Path,
    manifest_path: Path,
    ldrom_path: Path,
    output_path: Path,
) -> None:
    app, _ = _padded_words(app_path.read_bytes())
    manifest = manifest_path.read_bytes()
    ldrom, _ = _padded_words(ldrom_path.read_bytes())
    if not app or len(app) > APP_LIMIT:
        raise ValueError("packaged application must be nonempty and <= 0x7A00")
    if len(manifest) != MANIFEST_PAGE_SIZE:
        raise ValueError("manifest must be exactly one 512-byte page")
    if not ldrom or len(ldrom) > LDROM_SIZE:
        raise ValueError("LDROM image must be 1..4096 bytes")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    prefix = output_path.with_suffix("")
    vector_length = min(len(app), PAGE_SIZE)
    tail_length = len(app) - vector_length
    lines = [
        "// M2003 physical firmware verification (READ ONLY)",
        "r",
        "h",
    ]
    _append_unlock(lines)
    lines.extend(
        [
            f"w4 0x{CLK_AHBCLK:08X} 0x{AHBCLK_RESET_WITH_ISP:08X}",
            f"w4 0x{FMC_ISPCTL:08X} 0x{ISPCTL_READ_ONLY_LDROM_BOOT:08X}",
        ]
    )
    # CPU address 0 aliases the active LDROM vector page. Read physical APROM
    # page zero through FMC, then use direct debugger reads above that window.
    _append_physical_reads(lines, 0, vector_length)
    if tail_length:
        lines.append(
            f"savebin {_slash(prefix.with_name(prefix.name + '-app-tail-readback.bin'))} "
            f"0x{PAGE_SIZE:08X} 0x{tail_length:X}"
        )
    lines.extend(
        [
            f"savebin {_slash(prefix.with_name(prefix.name + '-manifest-readback.bin'))} "
            f"0x{MANIFEST_BASE:08X} 0x{len(manifest):X}",
            f"savebin {_slash(prefix.with_name(prefix.name + '-ldrom-readback.bin'))} "
            f"0x{LDROM_BASE:08X} 0x{len(ldrom):X}",
            f"mem32 0x{FMC_ISPSTS:08X} 1",
            f"w4 0x{SYS_REGLCTL:08X} 0x00000000",
            "r",
            "g",
            "exit",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="ascii")


def generate_config_read_script(output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "// M2003 boot/configuration capture (READ ONLY)",
        "r",
        "h",
    ]
    _append_unlock(lines)
    lines.append(f"w4 0x{CLK_AHBCLK:08X} 0x{AHBCLK_RESET_WITH_ISP:08X}")
    lines.extend(
        [
            f"w4 0x{FMC_ISPCTL:08X} 0x{ISPCTL_READ_ONLY:08X}",
            f"mem32 0x{FMC_ISPCTL:08X} 1",
            f"mem32 0x{FMC_ISPSTS:08X} 1",
        ]
    )
    for index in range(3):
        address = CONFIG_BASE + index * 4
        lines.extend(
            [
                f"// CONFIG{index}",
                f"w4 0x{FMC_ISPADDR:08X} 0x{address:08X}",
                f"w4 0x{FMC_ISPCMD:08X} 0x{CMD_READ:08X}",
                f"w4 0x{FMC_ISPTRG:08X} 0x00000001",
                f"mem32 0x{FMC_ISPTRG:08X} 1",
                f"mem32 0x{FMC_ISPDAT:08X} 1",
            ]
        )
    lines.extend(
        [
            f"w4 0x{SYS_REGLCTL:08X} 0x00000000",
            "exit",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="ascii")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate M2003 loader provisioning and config-read J-Link files"
    )
    parser.add_argument("--app", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--ldrom", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--verify-output", type=Path, required=True)
    parser.add_argument("--config-read-output", type=Path, required=True)
    args = parser.parse_args()

    generate_provision_script(args.app, args.manifest, args.ldrom, args.output)
    generate_verify_script(args.app, args.manifest, args.ldrom, args.verify_output)
    generate_config_read_script(args.config_read_output)
    print(f"wrote {args.output} (APROM + manifest + LDROM)")
    print(f"wrote {args.verify_output} (physical read-only firmware verification)")
    print(f"wrote {args.config_read_output} (read-only CONFIG/FMC capture)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
