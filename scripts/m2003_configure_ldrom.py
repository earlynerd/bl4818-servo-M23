#!/usr/bin/env python3
"""Safely plan and verify the M2003 CONFIG0.CBS LDROM-boot change."""

from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path


SYS_REGLCTL = 0x40000100
CLK_AHBCLK = 0x40000204
FMC_ISPCTL = 0x4000C000
FMC_ISPADDR = 0x4000C004
FMC_ISPDAT = 0x4000C008
FMC_ISPCMD = 0x4000C00C
FMC_ISPTRG = 0x4000C010
FMC_ISPSTS = 0x4000C040

CONFIG_BASE = 0x00300000
CONFIG_CBS_MASK = 1 << 7
CONFIG_LOCK_MASK = 1 << 1
CONFIG2_UNLOCKED = 0xFFFFFF5A
ERASED_CONFIG = (0xFFFFFFFF, 0xFFFFFFFF, CONFIG2_UNLOCKED)

AHBCLK_RESET_WITH_ISP = 0x00000103
ISPCTL_CONFIG_UPDATE = 0x00000011
ISPCTL_CLEAR_FAIL_CONFIG_UPDATE = 0x00000051
ISPCTL_READ_ONLY = 0x00000001

CMD_READ = 0x00
CMD_PROGRAM = 0x21
CMD_PAGE_ERASE = 0x22


@dataclass(frozen=True)
class ConfigPlan:
    original: tuple[int, int, int]
    desired: tuple[int, int, int]

    @property
    def needs_update(self) -> bool:
        return self.original != self.desired


def parse_config_capture(text: str) -> tuple[int, int, int]:
    matches = re.findall(
        rf"(?im)^\s*{FMC_ISPDAT:08X}\s*=\s*([0-9a-f]{{8}})\s*$", text
    )
    if len(matches) != 3:
        raise ValueError(
            f"expected exactly three CONFIG data reads, found {len(matches)}"
        )
    return tuple(int(value, 16) for value in matches)  # type: ignore[return-value]


def read_config_capture(path: Path) -> tuple[int, int, int]:
    data = path.read_bytes()
    if data.startswith((b"\xFF\xFE", b"\xFE\xFF")):
        text = data.decode("utf-16")
    else:
        text = data.decode("utf-8-sig")
    return parse_config_capture(text)


def make_config_plan(config: tuple[int, int, int]) -> ConfigPlan:
    config0, config1, config2 = config
    if (config0 & CONFIG_LOCK_MASK) == 0:
        raise ValueError("CONFIG0.LOCK is cleared; refusing automatic CONFIG update")
    if config2 != CONFIG2_UNLOCKED:
        raise ValueError(
            f"CONFIG2 is 0x{config2:08X}, not unlocked value 0x{CONFIG2_UNLOCKED:08X}"
        )
    desired = (config0 & ~CONFIG_CBS_MASK, config1, config2)
    return ConfigPlan(original=config, desired=desired)


def _append_unlock(lines: list[str]) -> None:
    lines.extend(
        [
            f"w4 0x{SYS_REGLCTL:08X} 0x00000059",
            f"w4 0x{SYS_REGLCTL:08X} 0x00000016",
            f"w4 0x{SYS_REGLCTL:08X} 0x00000088",
        ]
    )


def _append_isp_setup(lines: list[str], ispctl: int) -> None:
    _append_unlock(lines)
    lines.extend(
        [
            f"w4 0x{CLK_AHBCLK:08X} 0x{AHBCLK_RESET_WITH_ISP:08X}",
            f"w4 0x{FMC_ISPCTL:08X} 0x{ispctl:08X}",
        ]
    )


def _append_config_reads(lines: list[str]) -> None:
    lines.append(f"w4 0x{FMC_ISPCTL:08X} 0x{ISPCTL_READ_ONLY:08X}")
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


def generate_config_erase_script(output_path: Path) -> None:
    lines = [
        "// Erase M2003 CONFIG only after a preserved read/modify plan",
        "r",
        "h",
    ]
    _append_isp_setup(lines, ISPCTL_CLEAR_FAIL_CONFIG_UPDATE)
    lines.extend(
        [
            f"w4 0x{FMC_ISPCTL:08X} 0x{ISPCTL_CONFIG_UPDATE:08X}",
            f"w4 0x{FMC_ISPADDR:08X} 0x{CONFIG_BASE:08X}",
            f"w4 0x{FMC_ISPCMD:08X} 0x{CMD_PAGE_ERASE:08X}",
            f"w4 0x{FMC_ISPTRG:08X} 0x00000001",
            "sleep 20",
            f"mem32 0x{FMC_ISPTRG:08X} 1",
            f"mem32 0x{FMC_ISPSTS:08X} 1",
        ]
    )
    _append_config_reads(lines)
    lines.extend([f"w4 0x{SYS_REGLCTL:08X} 0x00000000", "exit"])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="ascii")


def generate_config_program_script(plan: ConfigPlan, output_path: Path) -> None:
    lines = [
        "// Restore preserved M2003 CONFIG and commit LDROM-first CBS last",
        "r",
        "h",
    ]
    _append_isp_setup(lines, ISPCTL_CLEAR_FAIL_CONFIG_UPDATE)
    lines.append(f"w4 0x{FMC_ISPCTL:08X} 0x{ISPCTL_CONFIG_UPDATE:08X}")

    # CONFIG1 is restored before CONFIG0 so CBS is the final committed change.
    for index in (1, 0):
        value = plan.desired[index]
        if value == 0xFFFFFFFF:
            continue
        address = CONFIG_BASE + index * 4
        lines.extend(
            [
                f"w4 0x{FMC_ISPADDR:08X} 0x{address:08X}",
                f"w4 0x{FMC_ISPDAT:08X} 0x{value:08X}",
                f"w4 0x{FMC_ISPCMD:08X} 0x{CMD_PROGRAM:08X}",
                f"w4 0x{FMC_ISPTRG:08X} 0x00000001",
                "sleep 1",
                f"mem32 0x{FMC_ISPTRG:08X} 1",
                f"mem32 0x{FMC_ISPSTS:08X} 1",
            ]
        )

    _append_config_reads(lines)
    lines.extend(
        [
            f"w4 0x{SYS_REGLCTL:08X} 0x00000000",
            "r",
            "g",
            "exit",
        ]
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="ascii")


def write_plan(plan: ConfigPlan, output_path: Path) -> None:
    document = {
        "original": [f"0x{value:08X}" for value in plan.original],
        "desired": [f"0x{value:08X}" for value in plan.desired],
        "needs_update": plan.needs_update,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(document, indent=2) + "\n", encoding="ascii")


def read_plan(path: Path) -> ConfigPlan:
    document = json.loads(path.read_text(encoding="ascii"))
    original = tuple(int(value, 0) for value in document["original"])
    desired = tuple(int(value, 0) for value in document["desired"])
    if len(original) != 3 or len(desired) != 3:
        raise ValueError("CONFIG plan must contain exactly three original and desired words")
    plan = make_config_plan(original)  # type: ignore[arg-type]
    if plan.desired != desired:
        raise ValueError("CONFIG plan desired values do not match its preserved original")
    return plan


def _prepare(args: argparse.Namespace) -> int:
    capture = read_config_capture(args.capture)
    plan = make_config_plan(capture)
    write_plan(plan, args.plan)
    generate_config_erase_script(args.erase_script)
    generate_config_program_script(plan, args.program_script)
    print(
        "CONFIG captured: "
        + " ".join(f"CONFIG{i}=0x{value:08X}" for i, value in enumerate(capture))
    )
    if plan.needs_update:
        print(f"CONFIG0.CBS plan: 0x{capture[0]:08X} -> 0x{plan.desired[0]:08X}")
    else:
        print("CONFIG0.CBS already selects LDROM; no CONFIG erase/write needed")
    return 0


def _verify(args: argparse.Namespace) -> int:
    actual = read_config_capture(args.capture)
    plan = read_plan(args.plan)
    expected = ERASED_CONFIG if args.phase == "erased" else plan.desired
    if actual != expected:
        raise ValueError(
            f"CONFIG {args.phase} verify failed: expected "
            f"{tuple(f'0x{x:08X}' for x in expected)}, got "
            f"{tuple(f'0x{x:08X}' for x in actual)}"
        )
    print(
        f"CONFIG {args.phase} verify OK: "
        + " ".join(f"CONFIG{i}=0x{value:08X}" for i, value in enumerate(actual))
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Preserve M2003 CONFIG while selecting LDROM-first boot"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    prepare = subparsers.add_parser("prepare")
    prepare.add_argument("--capture", type=Path, required=True)
    prepare.add_argument("--plan", type=Path, required=True)
    prepare.add_argument("--erase-script", type=Path, required=True)
    prepare.add_argument("--program-script", type=Path, required=True)
    prepare.set_defaults(func=_prepare)

    verify = subparsers.add_parser("verify")
    verify.add_argument("--capture", type=Path, required=True)
    verify.add_argument("--plan", type=Path, required=True)
    verify.add_argument("--phase", choices=("erased", "final"), required=True)
    verify.set_defaults(func=_verify)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
