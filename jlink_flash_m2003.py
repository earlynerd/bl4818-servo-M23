#!/usr/bin/env python3
"""
Program and verify Nuvoton M2003 flash via J-Link Commander and FMC ISP registers.

Usage:
  python jlink_flash_m2003.py <firmware.bin> [output.jlink]

Generates a J-Link script, runs it (program + readback), then verifies.
"""

import sys
import struct
import subprocess
from pathlib import Path

# Nuvoton M2003 FMC register addresses
SYS_REGLCTL    = 0x40000100
FMC_ISPCON     = 0x4000C000
FMC_ISPADR     = 0x4000C004
FMC_ISPDAT     = 0x4000C008
FMC_ISPCMD     = 0x4000C00C
FMC_ISPTRG     = 0x4000C010
FMC_ISPSTS     = 0x4000C040

CMD_PAGE_ERASE = 0x00000022
CMD_PROGRAM    = 0x00000021

FLASH_BASE     = 0x00000000
PAGE_SIZE      = 512
ISPCTL_ENABLE_APROM = 0x00000009  # ISPEN | APUEN


def generate_script(bin_path: Path, out_path: Path, readback_path: Path):
    data = bin_path.read_bytes()

    if len(data) % 4:
        data += b'\xFF' * (4 - len(data) % 4)

    num_words = len(data) // 4
    num_pages = (len(data) + PAGE_SIZE - 1) // PAGE_SIZE

    lines = []
    lines.append(f"// Flash + verify: {bin_path.name} ({len(data)} bytes)")
    lines.append("r")
    lines.append("h")

    # Unlock
    lines.append(f"w4 0x{SYS_REGLCTL:08X} 0x00000059")
    lines.append(f"w4 0x{SYS_REGLCTL:08X} 0x00000016")
    lines.append(f"w4 0x{SYS_REGLCTL:08X} 0x00000088")
    lines.append(f"w4 0x{FMC_ISPCON:08X} 0x{ISPCTL_ENABLE_APROM:08X}")

    # Erase
    for page in range(num_pages):
        addr = FLASH_BASE + page * PAGE_SIZE
        lines.append(f"w4 0x{FMC_ISPADR:08X} 0x{addr:08X}")
        lines.append(f"w4 0x{FMC_ISPCMD:08X} 0x{CMD_PAGE_ERASE:08X}")
        lines.append(f"w4 0x{FMC_ISPTRG:08X} 0x00000001")
        lines.append("sleep 20")
        lines.append(f"mem32 0x{FMC_ISPTRG:08X} 1")

    # Program
    words = struct.unpack(f'<{num_words}I', data)
    programmed = 0
    for i, word in enumerate(words):
        if word == 0xFFFFFFFF:
            continue
        addr = FLASH_BASE + i * 4
        lines.append(f"w4 0x{FMC_ISPADR:08X} 0x{addr:08X}")
        lines.append(f"w4 0x{FMC_ISPDAT:08X} 0x{word:08X}")
        lines.append(f"w4 0x{FMC_ISPCMD:08X} 0x{CMD_PROGRAM:08X}")
        lines.append(f"w4 0x{FMC_ISPTRG:08X} 0x00000001")
        # Read back ISPTRG to force SWD bus synchronization —
        # the read stalls until the write completes on the target
        lines.append(f"mem32 0x{FMC_ISPTRG:08X} 1")
        programmed += 1

    # Readback for verify — use forward slashes, J-Link handles both
    lines.append(f"savebin {str(readback_path).replace(chr(92), '/')} 0x{FLASH_BASE:08X} 0x{len(data):X}")

    # Lock and run
    lines.append(f"w4 0x{SYS_REGLCTL:08X} 0x00000000")
    lines.append("r")
    lines.append("g")
    lines.append("exit")

    out_path.write_text('\n'.join(lines))
    print(f"{len(data)} bytes, {num_pages} pages, {programmed} words to program")
    return data


def verify(expected: bytes, readback_path: Path):
    if not readback_path.exists():
        print(f"VERIFY SKIP: {readback_path} not found (J-Link savebin may have failed)")
        return False

    actual = readback_path.read_bytes()
    if len(actual) < len(expected):
        print(f"VERIFY FAIL: readback {len(actual)} bytes, expected {len(expected)}")
        return False

    actual = actual[:len(expected)]
    errors = 0
    for i in range(0, len(expected), 4):
        exp = struct.unpack_from('<I', expected, i)[0]
        act = struct.unpack_from('<I', actual, i)[0]
        if exp != act:
            print(f"  MISMATCH @ 0x{FLASH_BASE + i:08X}: "
                  f"expected 0x{exp:08X}, got 0x{act:08X}")
            errors += 1
            if errors >= 20:
                print("  ... (stopping after 20)")
                break

    if errors == 0:
        print(f"VERIFY OK ({len(expected)} bytes)")
        return True
    else:
        print(f"VERIFY FAILED ({errors} words differ)")
        return False


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <firmware.bin> [output.jlink]")
        sys.exit(1)

    bin_path = Path(sys.argv[1])
    jlink_path = Path(sys.argv[2]) if len(sys.argv) > 2 else bin_path.with_suffix('.jlink')
    readback_path = bin_path.with_name(bin_path.stem + '_readback.bin')

    if not bin_path.exists():
        print(f"Error: {bin_path} not found")
        sys.exit(1)

    expected = generate_script(bin_path, jlink_path, readback_path)

    print(f"Flashing...")
    result = subprocess.run(
        ['JLink.exe', '-device', 'Cortex-M23', '-if', 'SWD',
         '-speed', '4000', '-CommandFile', str(jlink_path)],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        print(f"J-Link failed (exit {result.returncode})")
        print(result.stdout[-500:] if result.stdout else "")
        print(result.stderr[-500:] if result.stderr else "")
        sys.exit(1)

    print("Flash complete, verifying...")
    if verify(expected, readback_path):
        sys.exit(0)
    else:
        sys.exit(1)
