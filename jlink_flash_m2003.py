#!/usr/bin/env python3
"""
Generate a J-Link Commander script to program Nuvoton M2003 flash
via FMC ISP registers. Brute force but it works.

Usage:
  1. arm-none-eabi-objcopy -O binary m2003-motor.elf m2003-motor.bin
  2. python jlink_flash_m2003.py m2003-motor.bin flash.jlink
  3. JLink.exe -device Cortex-M23 -if SWD -speed 4000 -CommandFile flash.jlink
"""

import sys
import struct
from pathlib import Path

# Nuvoton M2003 FMC register addresses
SYS_REGLCTL    = 0x40000100   # Register lock control (unlock sequence target)
CLK_AHBCLK     = 0x40000204   # AHB clock enable (may need FMC clock bit)
FMC_ISPCON     = 0x4000C000   # ISP control
FMC_ISPADR     = 0x4000C004   # ISP address
FMC_ISPDAT     = 0x4000C008   # ISP data
FMC_ISPCMD     = 0x4000C00C   # ISP command
FMC_ISPTRG     = 0x4000C010   # ISP trigger
FMC_ISPSTS     = 0x4000C040   # ISP status

# ISP commands
CMD_PAGE_ERASE = 0x00000022   # Page erase (512 bytes typical)
CMD_PROGRAM    = 0x00000021   # Program one word
CMD_READ       = 0x00000000   # Read one word

FLASH_BASE     = 0x00000000
PAGE_SIZE      = 512          # Nuvoton typical page size, adjust if needed
ISPCTL_ENABLE_APROM = 0x00000009  # ISPEN | APUEN, boot from APROM


def generate_script(bin_path: Path, out_path: Path):
    data = bin_path.read_bytes()

    # Pad to 4-byte alignment
    if len(data) % 4:
        data += b'\xFF' * (4 - len(data) % 4)

    num_words = len(data) // 4
    num_pages = (len(data) + PAGE_SIZE - 1) // PAGE_SIZE

    print(f"Binary: {len(data)} bytes, {num_words} words, {num_pages} pages")

    lines = []
    lines.append("// Auto-generated M2003 flash programming script")
    lines.append(f"// Source: {bin_path.name} ({len(data)} bytes)")
    lines.append("")
    lines.append("r")  # Reset and halt
    lines.append("h")
    lines.append("")

    # Unlock protected registers (three-key sequence)
    lines.append("// Unlock register write protection")
    lines.append(f"w4 0x{SYS_REGLCTL:08X} 0x00000059")
    lines.append(f"w4 0x{SYS_REGLCTL:08X} 0x00000016")
    lines.append(f"w4 0x{SYS_REGLCTL:08X} 0x00000088")
    lines.append("")

    # Enable ISP with APROM updates allowed. Do not clear APUEN here.
    lines.append("// Enable FMC ISP and APROM update, booting from APROM")
    lines.append(f"w4 0x{FMC_ISPCON:08X} 0x{ISPCTL_ENABLE_APROM:08X}")
    lines.append("")

    # Erase pages
    lines.append(f"// Erase {num_pages} pages")
    for page in range(num_pages):
        addr = FLASH_BASE + page * PAGE_SIZE
        lines.append(f"w4 0x{FMC_ISPADR:08X} 0x{addr:08X}")
        lines.append(f"w4 0x{FMC_ISPCMD:08X} 0x{CMD_PAGE_ERASE:08X}")
        lines.append(f"w4 0x{FMC_ISPTRG:08X} 0x00000001")
        lines.append("sleep 50")  # Give erase time to complete
    lines.append("")

    # Program words
    lines.append(f"// Program {num_words} words")
    words = struct.unpack(f'<{num_words}I', data)
    for i, word in enumerate(words):
        if word == 0xFFFFFFFF:
            continue  # Skip blank words, already erased
        addr = FLASH_BASE + i * 4
        lines.append(f"w4 0x{FMC_ISPADR:08X} 0x{addr:08X}")
        lines.append(f"w4 0x{FMC_ISPDAT:08X} 0x{word:08X}")
        lines.append(f"w4 0x{FMC_ISPCMD:08X} 0x{CMD_PROGRAM:08X}")
        lines.append(f"w4 0x{FMC_ISPTRG:08X} 0x00000001")
        lines.append("sleep 5")

    lines.append("")

    # Verify: read back first few words
    lines.append("// Verify first 16 bytes and FMC status")
    lines.append(f"mem 0x{FLASH_BASE:08X} 16")
    lines.append(f"mem32 0x{FMC_ISPCON:08X} 1")
    lines.append(f"mem32 0x{FMC_ISPSTS:08X} 1")
    lines.append("")

    # Lock registers back and reset
    lines.append("// Lock registers and reset")
    lines.append(f"w4 0x{SYS_REGLCTL:08X} 0x00000000")
    lines.append("r")
    lines.append("g")
    lines.append("exit")

    out_path.write_text('\n'.join(lines))
    print(f"Written: {out_path} ({len(lines)} lines)")
    print()
    print("Run with:")
    print(f'  JLink.exe -device Cortex-M23 -if SWD -speed 4000 -CommandFile {out_path.name}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <firmware.bin> [output.jlink]")
        sys.exit(1)

    bin_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else bin_path.with_suffix('.jlink')

    if not bin_path.exists():
        print(f"Error: {bin_path} not found")
        sys.exit(1)

    generate_script(bin_path, out_path)
