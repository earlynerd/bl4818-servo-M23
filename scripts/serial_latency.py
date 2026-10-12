"""Check Windows FTDI latency before opening the configured ring ports.

Normal startup is read-only. The explicit --set-2ms command needs an
Administrator terminal and changes only the named, identity-verified adapters.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
import re
import sys

from serial.tools import list_ports

FTDI_LATENCY_MS = 2


class SerialLatencyError(RuntimeError):
    pass


@dataclass(frozen=True)
class PortLatency:
    port: str
    serial_number: str | None = None
    registry_key: str | None = None
    latency_ms: int | None = None
    note: str = ""

    def describe(self) -> str:
        if self.registry_key:
            return (f"{self.port}: FTDI {self.serial_number}, saved latency timer "
                    f"{self.latency_ms} ms (required 1..{FTDI_LATENCY_MS} ms)")
        return f"{self.port}: {self.note}"


def _port_name(port: str) -> str:
    name = port.upper()
    if name.startswith("\\\\.\\"):
        name = name[4:]
    return name


def inspect_serial_latency(ports: list[str]) -> list[PortLatency]:
    """Read selected present-device identities and their per-device settings."""
    names = list(dict.fromkeys(_port_name(p) for p in ports))
    if sys.platform != "win32":
        return [PortLatency(p, note="Windows FTDI latency check not applicable") for p in ports]
    import winreg

    present = list(list_ports.comports())
    results = []
    for name in names:
        matches = [p for p in present if _port_name(p.device) == name]
        if len(matches) != 1:
            raise SerialLatencyError(f"{name}: expected one present serial adapter, found {len(matches)}")
        device = matches[0]
        is_ftdi = (device.vid == 0x0403
                   or "ftdi" in (device.manufacturer or "").lower()
                   or "FTDIBUS" in (device.hwid or "").upper())
        if not is_ftdi:
            results.append(PortLatency(name, note="not an identified FTDI adapter; latency setting not checked"))
            continue
        serial_number = device.serial_number
        if (device.vid is None or device.pid is None or not serial_number
                or not re.fullmatch(r"[A-Za-z0-9_.-]+", serial_number)):
            raise SerialLatencyError(f"{name}: cannot verify FTDI adapter identity; inspect Windows port properties")
        identity = f"VID_{device.vid:04X}+PID_{device.pid:04X}+{serial_number}"
        path = "SYSTEM\\CurrentControlSet\\Enum\\FTDIBUS\\" + identity + "\\0000\\Device Parameters"
        try:
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path) as key:
                assigned_port = winreg.QueryValueEx(key, "PortName")[0]
                value, kind = winreg.QueryValueEx(key, "LatencyTimer")
        except OSError as exc:
            raise SerialLatencyError(f"{name}: cannot read the saved FTDI latency timer: {exc}") from exc
        if _port_name(assigned_port) != name:
            raise SerialLatencyError(f"{name}: FTDI registry identity belongs to {assigned_port}; refusing to guess")
        if kind != winreg.REG_DWORD or not isinstance(value, int) or not 1 <= value <= 255:
            raise SerialLatencyError(f"{name}: invalid FTDI LatencyTimer value {value!r}")
        results.append(PortLatency(name, serial_number, path, value))
    return results


def check_serial_latency(ports: list[str]) -> list[PortLatency]:
    settings = inspect_serial_latency(ports)
    slow = [s for s in settings if s.registry_key and s.latency_ms > FTDI_LATENCY_MS]
    if slow:
        details = "; ".join(s.describe() for s in slow)
        flags = " ".join(f"--port {s.port}" for s in slow)
        raise SerialLatencyError(
            f"{details}. Stop the drum server, then run in an Administrator terminal "
            f"from the repository root: py scripts/serial_latency.py {flags} --set-2ms . "
            "Restart the drum server afterward. Other ports are not changed."
        )
    return settings


def configure_serial_latency(ports: list[str]) -> list[PortLatency]:
    """Explicit maintenance operation; preflight every target before writing."""
    if sys.platform != "win32":
        raise SerialLatencyError("Setting FTDI latency is supported only on Windows")
    import winreg

    settings = inspect_serial_latency(ports)
    if not settings or any(not s.registry_key for s in settings):
        raise SerialLatencyError("--set-2ms requires only identified FTDI ports; no settings changed")
    with ExitStack() as stack:
        opened = []
        try:
            for setting in settings:
                key = stack.enter_context(winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                    setting.registry_key, 0, winreg.KEY_READ | winreg.KEY_SET_VALUE))
                assigned_port = winreg.QueryValueEx(key, "PortName")[0]
                value, kind = winreg.QueryValueEx(key, "LatencyTimer")
                if (_port_name(assigned_port) != setting.port or value != setting.latency_ms
                        or kind != winreg.REG_DWORD):
                    raise SerialLatencyError(f"{setting.port}: settings changed during preflight; retry")
                opened.append((setting, key))
        except OSError as exc:
            raise SerialLatencyError(
                f"Cannot open all selected FTDI settings for writing; use an Administrator terminal. "
                f"No settings changed: {exc}") from exc
        changed = []
        try:
            for setting, key in opened:
                if setting.latency_ms == FTDI_LATENCY_MS:
                    continue
                changed.append((setting, key))
                winreg.SetValueEx(key, "LatencyTimer", 0, winreg.REG_DWORD, FTDI_LATENCY_MS)
                if winreg.QueryValueEx(key, "LatencyTimer") != (FTDI_LATENCY_MS, winreg.REG_DWORD):
                    raise SerialLatencyError(f"{setting.port}: write readback failed")
        except (OSError, SerialLatencyError) as exc:
            rollback_errors = []
            for setting, key in reversed(changed):
                try:
                    winreg.SetValueEx(key, "LatencyTimer", 0, winreg.REG_DWORD, setting.latency_ms)
                    if winreg.QueryValueEx(key, "LatencyTimer") != (setting.latency_ms, winreg.REG_DWORD):
                        raise SerialLatencyError("restore readback failed")
                except (OSError, SerialLatencyError) as restore_exc:
                    rollback_errors.append(f"{setting.port}: {restore_exc}")
            state = ("Rollback failed: " + "; ".join(rollback_errors)
                     if rollback_errors else "Original settings restored.")
            raise SerialLatencyError(f"Latency update failed: {exc}. {state}") from exc
    return check_serial_latency(ports)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", "-p", action="append", required=True,
                        help="Configured ring port; repeat for additional rings")
    parser.add_argument("--set-2ms", action="store_true",
                        help="Explicitly set only these FTDI adapters to 2 ms (Administrator required)")
    args = parser.parse_args(argv)
    try:
        settings = (configure_serial_latency(args.port) if args.set_2ms
                    else check_serial_latency(args.port))
    except SerialLatencyError as exc:
        print(f"Serial latency check failed: {exc}", file=sys.stderr)
        return 1
    for setting in settings:
        print(setting.describe())
    if args.set_2ms:
        print("Reopen the configured drum ports before playback; saved values do not measure live latency.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
