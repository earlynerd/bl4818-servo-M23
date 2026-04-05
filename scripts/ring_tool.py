#!/usr/bin/env python3
"""
ring_tool.py - Ring Bus Protocol v2 client for M2003 motor boards

Examples:
    python ring_tool.py -p COM7 enumerate
    python ring_tool.py -p COM7 status 0
    python ring_tool.py -p COM7 set-duty 0 400
    python ring_tool.py -p COM7 torque 0 2500
    python ring_tool.py -p COM7 stop 0
    python ring_tool.py -p COM7 broadcast 200 0 0
    python ring_tool.py -p COM7 monitor 0 --hz 10

Requires: pyserial (pip install pyserial)
"""

from __future__ import annotations

import argparse
import dataclasses
import struct
import sys
import time
from typing import Iterable, Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial")
    sys.exit(1)


# ── Framing ──────────────────────────────────────────────────────────────────

PREAMBLE = b"\xA5\x5A"
MAX_PAYLOAD = 34
MAX_DEVICES = 16

# ── Command types ────────────────────────────────────────────────────────────

CMD_ENTER_SF        = 0x01
CMD_ENTER_CT        = 0x02
CMD_SET_ADDRESS     = 0x03
CMD_BROADCAST_DUTY  = 0x10
CMD_ADDR_BASE       = 0x20
CMD_ADDR_END        = 0x30
CMD_STATUS_BASE     = 0x40
CMD_STATUS_END      = 0x50

# ── Addressed sub-commands ───────────────────────────────────────────────────

SUBCMD_SET_DUTY     = 0x01
SUBCMD_SET_TORQUE   = 0x02
SUBCMD_STOP         = 0x03
SUBCMD_CLEAR_FAULT  = 0x04
SUBCMD_QUERY_STATUS = 0x10

# ── Timing defaults ─────────────────────────────────────────────────────────

DEFAULT_BAUD        = 250000
DEFAULT_TIMEOUT_MS  = 200
DEFAULT_SETTLE_MS   = 250


# ── CRC-16/CCITT ────────────────────────────────────────────────────────────

def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ── Data types ───────────────────────────────────────────────────────────────

MOTOR_STATES = {0: "IDLE", 1: "RUN", 2: "FAULT"}
FAULT_CODES  = {0: "NONE", 1: "OVERCURRENT", 2: "HALL_INVALID"}


@dataclasses.dataclass
class MotorStatus:
    address: int
    state: int
    fault: int
    current_ma: int
    hall: int
    angle: int          # 14-bit encoder angle (0-16383)

    @property
    def state_name(self) -> str:
        return MOTOR_STATES.get(self.state, f"UNKNOWN({self.state})")

    @property
    def fault_name(self) -> str:
        return FAULT_CODES.get(self.fault, f"UNKNOWN({self.fault})")

    @property
    def angle_deg(self) -> float:
        return self.angle * 360.0 / 16384.0


# ── Exceptions ───────────────────────────────────────────────────────────────

class RingError(Exception):
    pass

class RingTimeout(RingError):
    pass

class RingCRCError(RingError):
    pass


# ── Client ───────────────────────────────────────────────────────────────────

class RingClientV2:
    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUD,
        timeout_ms: int = DEFAULT_TIMEOUT_MS,
        settle_ms: int = DEFAULT_SETTLE_MS,
        trace: bool = False,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout_ms = timeout_ms
        self.settle_ms = settle_ms
        self.trace = trace
        self.ser: Optional[serial.Serial] = None
        self.device_count: Optional[int] = None

    # ── Port management ──────────────────────────────────────────────────

    def open(self) -> None:
        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=0.01,
            inter_byte_timeout=0.005,
            write_timeout=max(self.timeout_ms / 1000.0, 0.1),
        )
        self.ser.setRTS(False)
        self.ser.setDTR(True)
        if self.settle_ms > 0:
            time.sleep(self.settle_ms / 1000.0)
        self._flush_rx()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def _flush_rx(self) -> None:
        if not self.ser:
            return
        while True:
            waiting = self.ser.in_waiting
            if waiting <= 0:
                break
            self.ser.read(waiting)
            time.sleep(0.001)

    # ── Low-level framing ────────────────────────────────────────────────

    def _build_frame(self, payload: bytes) -> bytes:
        """Build a complete wire frame: preamble + LEN + payload + CRC16."""
        length = len(payload)
        body = bytes([length]) + payload
        crc = crc16_ccitt(body)
        return PREAMBLE + body + struct.pack(">H", crc)

    def _send_frame(self, payload: bytes) -> None:
        assert self.ser is not None
        frame = self._build_frame(payload)
        self._trace("tx", frame)
        self.ser.write(frame)
        self.ser.flush()

    def _recv_frame(self, timeout_ms: Optional[int] = None) -> bytes:
        """
        Firmware-style state-machine receiver.

        Mirrors the RX_SCAN_0 / RX_SCAN_1 / RX_BUFFERING phases in
        protocol.c.  On CRC failure the buffered bytes (minus the 0xA5
        that started the attempt) are re-scanned, matching the protocol
        spec's single-byte-slip resync rule.
        """
        assert self.ser is not None
        timeout_s = (timeout_ms or self.timeout_ms) / 1000.0
        deadline = time.monotonic() + timeout_s

        SCAN_0, SCAN_1, BUFFERING = 0, 1, 2
        phase = SCAN_0
        buf = bytearray()       # LEN + payload + CRC during BUFFERING
        expected = 0             # total bytes to collect in BUFFERING

        def next_byte() -> int:
            while True:
                if time.monotonic() >= deadline:
                    raise RingTimeout("timeout waiting for frame")
                b = self.ser.read(1)
                if b:
                    return b[0]

        while True:
            c = next_byte()

            if phase == SCAN_0:
                if c == 0xA5:
                    phase = SCAN_1

            elif phase == SCAN_1:
                if c == 0x5A:
                    phase = BUFFERING
                    buf.clear()
                    expected = 0
                elif c == 0xA5:
                    pass        # might be start of a new preamble
                else:
                    phase = SCAN_0

            elif phase == BUFFERING:
                buf.append(c)

                # First buffered byte is LEN
                if len(buf) == 1:
                    length = buf[0]
                    if length == 0 or length > MAX_PAYLOAD:
                        # Bad length — rescan from after the 0xA5
                        phase = SCAN_0
                        buf.clear()
                        continue
                    expected = 1 + length + 2   # LEN + payload + CRC16

                if expected and len(buf) >= expected:
                    length = buf[0]
                    payload = bytes(buf[1:1 + length])
                    rx_crc = struct.unpack(">H", buf[1 + length:1 + length + 2])[0]
                    calc_crc = crc16_ccitt(bytes(buf[:1 + length]))

                    if calc_crc == rx_crc:
                        self._trace("rx", PREAMBLE + bytes(buf))
                        return payload

                    # CRC fail — slip one byte past the 0xA5 and rescan
                    self._trace("rx-crc-fail", PREAMBLE + bytes(buf))
                    phase = SCAN_0
                    buf.clear()
                    expected = 0

    def _trace(self, label: str, data: bytes) -> None:
        if self.trace:
            print(f"  [{label}] {data.hex(' ').upper()}")

    # ── Protocol operations ──────────────────────────────────────────────

    def enumerate(self) -> int:
        """Run the enumeration sequence: SF mode, SET_ADDRESS, CT mode."""
        self._flush_rx()

        # Enter store-and-forward.
        # Device is in CT mode by default, so it echoes this frame back
        # before switching to S&F.  Wait for the echo and discard it.
        self._send_frame(bytes([CMD_ENTER_SF]))
        time.sleep(0.010)
        self._flush_rx()

        # SET_ADDRESS with counter=0 — device is now in S&F, no echo
        self._send_frame(bytes([CMD_SET_ADDRESS, 0x00]))

        # Read the forwarded SET_ADDRESS back — counter tells us device count
        payload = self._recv_frame()

        if len(payload) < 2 or payload[0] != CMD_SET_ADDRESS:
            raise RingError(f"unexpected enumerate reply: {payload.hex(' ')}")

        self.device_count = payload[1]

        # Enter cut-through
        time.sleep(0.002)
        self._send_frame(bytes([CMD_ENTER_CT]))
        time.sleep(0.010)
        self._flush_rx()

        return self.device_count

    def _recv_status_reply(self) -> MotorStatus:
        """Read frames until we get a STATUS_REPLY, skipping stale frames."""
        deadline = time.monotonic() + self.timeout_ms / 1000.0
        while True:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            payload = self._recv_frame(timeout_ms=remaining_ms)
            cmd = payload[0] if payload else 0
            if CMD_STATUS_BASE <= cmd < CMD_STATUS_END:
                return self._parse_status(payload)
            self._trace("skip", bytes([cmd]))

    def query_status(self, address: int) -> MotorStatus:
        self._check_address(address)
        self._flush_rx()
        self._send_frame(bytes([CMD_ADDR_BASE | address, SUBCMD_QUERY_STATUS]))
        return self._recv_status_reply()

    def set_duty(self, address: int, duty: int) -> MotorStatus:
        self._check_address(address)
        if duty < -32768 or duty > 32767:
            raise RingError("duty must fit in int16")
        self._flush_rx()
        self._send_frame(
            bytes([CMD_ADDR_BASE | address, SUBCMD_SET_DUTY])
            + struct.pack(">h", duty)
        )
        return self._recv_status_reply()

    def set_torque(self, address: int, torque_ma: int) -> MotorStatus:
        self._check_address(address)
        if torque_ma < 0 or torque_ma > 0xFFFF:
            raise RingError("torque must fit in uint16")
        self._flush_rx()
        self._send_frame(
            bytes([CMD_ADDR_BASE | address, SUBCMD_SET_TORQUE])
            + struct.pack(">H", torque_ma)
        )
        return self._recv_status_reply()

    def stop(self, address: int) -> MotorStatus:
        self._check_address(address)
        self._flush_rx()
        self._send_frame(bytes([CMD_ADDR_BASE | address, SUBCMD_STOP]))
        return self._recv_status_reply()

    def clear_fault(self, address: int) -> MotorStatus:
        self._check_address(address)
        self._flush_rx()
        self._send_frame(bytes([CMD_ADDR_BASE | address, SUBCMD_CLEAR_FAULT]))
        return self._recv_status_reply()

    def broadcast_duty(self, duties: Iterable[int]) -> None:
        duty_list = list(duties)
        if not duty_list or len(duty_list) > MAX_DEVICES:
            raise RingError(f"broadcast needs 1..{MAX_DEVICES} duty slots")
        payload = bytearray([CMD_BROADCAST_DUTY])
        for d in duty_list:
            payload.extend(struct.pack(">h", d))
        self._flush_rx()
        self._send_frame(bytes(payload))
        # Broadcast has no reply

    # ── Parsing ──────────────────────────────────────────────────────────

    def _parse_status(self, payload: bytes) -> MotorStatus:
        """Parse a STATUS_REPLY payload (8 bytes: cmd state fault cur_hi cur_lo hall angle_hi angle_lo)."""
        if len(payload) < 7:
            raise RingError(f"status reply too short ({len(payload)} bytes): {payload.hex(' ')}")

        cmd = payload[0]
        if cmd < CMD_STATUS_BASE or cmd >= CMD_STATUS_END:
            raise RingError(f"not a status reply (cmd=0x{cmd:02X}): {payload.hex(' ')}")

        address = cmd & 0x0F
        state = payload[1]
        fault = payload[2]
        current = struct.unpack(">H", payload[3:5])[0]
        hall = payload[5]

        # Encoder angle is present if payload is 8 bytes (new format)
        if len(payload) >= 8:
            angle = struct.unpack(">H", payload[6:8])[0]
        else:
            angle = 0

        return MotorStatus(
            address=address,
            state=state,
            fault=fault,
            current_ma=current,
            hall=hall,
            angle=angle,
        )

    def _check_address(self, address: int) -> None:
        if address < 0 or address >= MAX_DEVICES:
            raise RingError(f"address must be 0..{MAX_DEVICES - 1}")
        if self.device_count is not None and address >= self.device_count:
            raise RingError(f"address {address} out of range (enumerated {self.device_count})")


# ── Display helpers ──────────────────────────────────────────────────────────

def format_status(s: MotorStatus) -> str:
    return (
        f"addr={s.address} state={s.state_name} fault={s.fault_name} "
        f"current={s.current_ma}mA hall={s.hall} "
        f"angle={s.angle} ({s.angle_deg:.1f}°)"
    )


# ── Port detection ───────────────────────────────────────────────────────────

def auto_detect_port() -> Optional[str]:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    preferred = []
    fallback = []
    for p in ports:
        desc = (p.description or "").lower()
        mfr = (p.manufacturer or "").lower()
        if (
            p.vid in (0x2E8A, 0x1A86, 0x10C4, 0x0403)
            or "usb serial" in desc
            or "wch" in mfr
            or "silicon labs" in mfr
            or "ftdi" in mfr
            or "pico" in desc
            or "acm" in desc
        ):
            preferred.append(p.device)
        else:
            fallback.append(p.device)

    return preferred[0] if preferred else (fallback[0] if fallback else None)


def list_ports() -> int:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 1
    for p in ports:
        print(f"  {p.device}: {p.description}")
    return 0


# ── CLI ──────────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Ring Bus Protocol v2 client for M2003 motor boards"
    )
    parser.add_argument("-p", "--port", help="Serial port (auto-detect if omitted)")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--timeout-ms", type=int, default=DEFAULT_TIMEOUT_MS,
                        help=f"Response timeout in ms (default: {DEFAULT_TIMEOUT_MS})")
    parser.add_argument("--settle-ms", type=int, default=DEFAULT_SETTLE_MS,
                        help=f"Post-open settle delay in ms (default: {DEFAULT_SETTLE_MS})")
    parser.add_argument("--trace", action="store_true",
                        help="Print raw TX/RX hex for each frame")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("ports", help="List serial ports")
    sub.add_parser("enumerate", help="Enumerate devices on the ring")

    sp = sub.add_parser("status", help="Query status from a device")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("set-duty", help="Set signed duty on a device")
    sp.add_argument("address", type=int)
    sp.add_argument("duty", type=int, help="Signed int16 duty")

    sp = sub.add_parser("torque", help="Set torque limit (mA)")
    sp.add_argument("address", type=int)
    sp.add_argument("milliamps", type=int)

    sp = sub.add_parser("stop", help="Stop a device")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("clear-fault", help="Clear fault on a device")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("broadcast", help="Broadcast duty to all devices")
    sp.add_argument("duties", nargs="+", type=int, help="One signed duty per device")

    sp = sub.add_parser("monitor", help="Poll status continuously")
    sp.add_argument("address", type=int)
    sp.add_argument("--hz", type=float, default=10.0,
                    help="Poll rate in Hz (default: 10)")

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "ports":
        return list_ports()

    port = args.port or auto_detect_port()
    if not port:
        print("ERROR: no serial port found. Use -p to specify.")
        return 1
    if args.port is None:
        print(f"Auto-detected port: {port}")

    client = RingClientV2(
        port=port,
        baudrate=args.baud,
        timeout_ms=args.timeout_ms,
        settle_ms=args.settle_ms,
        trace=args.trace,
    )

    try:
        client.open()

        if args.command == "enumerate":
            count = client.enumerate()
            print(f"devices={count}")
            return 0

        # Try to enumerate, but don't bail if it fails —
        # the device may already be enumerated from a previous run.
        try:
            count = client.enumerate()
            print(f"devices={count}")
            if count == 0:
                print("WARNING: enumeration returned 0 devices")
        except RingError as exc:
            print(f"WARNING: enumerate failed ({exc}), proceeding anyway")

        if args.command == "status":
            print(format_status(client.query_status(args.address)))

        elif args.command == "set-duty":
            print(format_status(client.set_duty(args.address, args.duty)))

        elif args.command == "torque":
            print(format_status(client.set_torque(args.address, args.milliamps)))

        elif args.command == "stop":
            print(format_status(client.stop(args.address)))

        elif args.command == "clear-fault":
            print(format_status(client.clear_fault(args.address)))

        elif args.command == "broadcast":
            if client.device_count is not None and len(args.duties) != client.device_count:
                print(f"WARNING: sending {len(args.duties)} duties but {client.device_count} devices enumerated")
            client.broadcast_duty(args.duties)
            print("broadcast ok")

        elif args.command == "monitor":
            interval = 1.0 / args.hz if args.hz > 0 else 0.1
            print(f"Monitoring device {args.address} at {args.hz:.1f} Hz (Ctrl-C to stop)")
            while True:
                try:
                    s = client.query_status(args.address)
                    print(format_status(s))
                except RingTimeout:
                    print("  (timeout)")
                except RingCRCError as e:
                    print(f"  (crc error: {e})")
                time.sleep(interval)

        return 0

    except RingError as exc:
        print(f"ERROR: {exc}")
        return 1
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\nCancelled.")
        return 130
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
