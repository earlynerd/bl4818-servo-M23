#!/usr/bin/env python3
"""
ring_tool.py - Ring Bus Protocol v2 client for M2003 motor boards

Examples:
    python ring_tool.py -p COM7 enumerate
    python ring_tool.py -p COM7 status 0
    python ring_tool.py -p COM7 set-duty 0 400
    python ring_tool.py -p COM7 set-velocity 0 500
    python ring_tool.py -p COM7 zero-pos 0
    python ring_tool.py -p COM7 save-settings 0
    python ring_tool.py -p COM7 set-pid 0 128 16 0
    python ring_tool.py -p COM7 set-mode 0 1
    python ring_tool.py -p COM7 torque 0 2500
    python ring_tool.py -p COM7 stop 0
    python ring_tool.py -p COM7 broadcast 200 0 0
    python ring_tool.py -p COM7 monitor 0 --hz 10
    python ring_tool.py -p COM7 measure-strike-timing 0 --start 100 --stop 800 --step 100 --csv strike.csv
    python ring_tool.py -p COM7 measure-strike-timing 0 --sweep-param home-offset --start 1200 --stop 2200 --step 200 --strike-duty 500

Requires: pyserial (pip install pyserial)
"""

from __future__ import annotations

import argparse
import csv
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
CMD_ACK_BASE        = 0x50
CMD_ACK_END         = 0x60

# ── Addressed sub-commands ───────────────────────────────────────────────────

SUBCMD_SET_DUTY     = 0x01
SUBCMD_SET_TORQUE   = 0x02
SUBCMD_STOP         = 0x03
SUBCMD_CLEAR_FAULT  = 0x04
SUBCMD_SET_MODE     = 0x05
SUBCMD_SET_VELOCITY = 0x06
SUBCMD_SET_PID      = 0x07
SUBCMD_SET_FF       = 0x08
SUBCMD_SET_POSITION = 0x09
SUBCMD_SET_POS_PID  = 0x0A
SUBCMD_ZERO_POS     = 0x0B
SUBCMD_STRIKE       = 0x0C
SUBCMD_STRIKE_HOME  = 0x0D
SUBCMD_STRIKE_CANCEL = 0x0E
SUBCMD_SET_STRIKE_PARAM = 0x0F
SUBCMD_QUERY_STATUS = 0x10
SUBCMD_QUERY_STRIKE = 0x11
SUBCMD_SAVE_SETTINGS = 0x12
SUBCMD_CLEAR_SETTINGS = 0x13
SUBCMD_MASK         = 0x3F
SUBCMD_REPLY_FULL   = 0x00
SUBCMD_REPLY_ACK    = 0x40
SUBCMD_REPLY_NONE   = 0x80

REPLY_MODE_FULL = "full"
REPLY_MODE_ACK = "ack"
REPLY_MODE_NONE = "none"

ACK_RESULT_OK               = 0x00
ACK_RESULT_OK_RETRIGGERED   = 0x01
ACK_RESULT_REJECT_NOT_HOMED = 0x02
ACK_RESULT_REJECT_FAULT     = 0x03
ACK_RESULT_REJECT_ZERO      = 0x04
ACK_RESULT_INVALID_ARGUMENT = 0x05

# ── Strike parameter IDs ───────────────────────────────────────────────────

STRIKE_PARAM_HOME_OFFSET    = 0x01
STRIKE_PARAM_COAST_DISTANCE = 0x02
STRIKE_PARAM_HOMING_DUTY    = 0x03

STRIKE_TIMING_COAST_VALID     = 0x01
STRIKE_TIMING_REBOUND_VALID   = 0x02
STRIKE_TIMING_READY_VALID     = 0x04
STRIKE_TIMING_ACTIVE          = 0x08
STRIKE_TIMING_RETRIGGERED     = 0x10
STRIKE_TIMING_REBOUND_TIMEOUT = 0x20
STRIKE_TIMING_VELOCITY_VALID  = 0x40

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

MOTOR_STATES  = {0: "IDLE", 1: "RUN", 2: "FAULT"}
FAULT_CODES   = {0: "NONE", 1: "OVERCURRENT", 2: "HALL_INVALID"}
CTRL_MODES    = {0: "DUTY", 1: "VELOCITY", 2: "POSITION"}
STRIKE_STATES = {0: "IDLE", 1: "HOMING", 2: "DRIVING", 3: "COASTING", 4: "BRAKING", 5: "CATCHING"}
SUBCMD_NAMES  = {
    SUBCMD_SET_DUTY: "SET_DUTY",
    SUBCMD_SET_TORQUE: "SET_TORQUE",
    SUBCMD_STOP: "STOP",
    SUBCMD_CLEAR_FAULT: "CLEAR_FAULT",
    SUBCMD_SET_MODE: "SET_MODE",
    SUBCMD_SET_VELOCITY: "SET_VELOCITY",
    SUBCMD_SET_PID: "SET_PID",
    SUBCMD_SET_FF: "SET_FF",
    SUBCMD_SET_POSITION: "SET_POSITION",
    SUBCMD_SET_POS_PID: "SET_POS_PID",
    SUBCMD_ZERO_POS: "ZERO_POSITION",
    SUBCMD_STRIKE: "STRIKE",
    SUBCMD_STRIKE_HOME: "STRIKE_HOME",
    SUBCMD_STRIKE_CANCEL: "STRIKE_CANCEL",
    SUBCMD_SET_STRIKE_PARAM: "SET_STRIKE_PARAM",
    SUBCMD_QUERY_STATUS: "QUERY_STATUS",
    SUBCMD_QUERY_STRIKE: "QUERY_STRIKE",
    SUBCMD_SAVE_SETTINGS: "SAVE_SETTINGS",
    SUBCMD_CLEAR_SETTINGS: "CLEAR_SETTINGS",
}
ACK_RESULT_NAMES = {
    ACK_RESULT_OK: "OK",
    ACK_RESULT_OK_RETRIGGERED: "OK_RETRIGGERED",
    ACK_RESULT_REJECT_NOT_HOMED: "REJECT_NOT_HOMED",
    ACK_RESULT_REJECT_FAULT: "REJECT_FAULT",
    ACK_RESULT_REJECT_ZERO: "REJECT_ZERO",
    ACK_RESULT_INVALID_ARGUMENT: "INVALID_ARGUMENT",
}


@dataclasses.dataclass
class MotorStatus:
    address: int
    state: int
    fault: int
    mode: int           # ctrl_mode_t: 0=DUTY, 1=VELOCITY
    current_ma: int
    hall: int
    angle: int          # 14-bit encoder angle (0-16383)
    velocity: int       # measured RPM (signed)
    target: int         # active target: duty or RPM depending on mode
    position: int       # continuous encoder position (int32, multi-turn)

    @property
    def state_name(self) -> str:
        return MOTOR_STATES.get(self.state, f"UNKNOWN({self.state})")

    @property
    def fault_name(self) -> str:
        return FAULT_CODES.get(self.fault, f"UNKNOWN({self.fault})")

    @property
    def mode_name(self) -> str:
        return CTRL_MODES.get(self.mode, f"UNKNOWN({self.mode})")

    @property
    def angle_deg(self) -> float:
        return self.angle * 360.0 / 16384.0


@dataclasses.dataclass
class CommandAck:
    address: int
    subcmd: int
    result: int
    detail: int

    @property
    def subcmd_name(self) -> str:
        return SUBCMD_NAMES.get(self.subcmd, f"UNKNOWN(0x{self.subcmd:02X})")

    @property
    def result_name(self) -> str:
        return ACK_RESULT_NAMES.get(self.result, f"UNKNOWN(0x{self.result:02X})")

    @property
    def accepted(self) -> bool:
        return self.result in (ACK_RESULT_OK, ACK_RESULT_OK_RETRIGGERED)


@dataclasses.dataclass
class StrikeStatus:
    address: int
    state: int          # strike_state_t: 0=IDLE..5=CATCHING
    homed: int
    flags: int = 0
    sequence: int = 0
    last_duty: int = 0
    trigger_to_coast_ms: int = 0
    trigger_to_rebound_ms: int = 0
    trigger_to_ready_ms: int = 0
    estimated_strike_velocity_dps: Optional[int] = None
    drum_position: int = 0  # int32
    home_position: int = 0  # int32
    home_offset: Optional[int] = None
    coast_distance: Optional[int] = None
    homing_duty: Optional[int] = None

    @property
    def state_name(self) -> str:
        return STRIKE_STATES.get(self.state, f"UNKNOWN({self.state})")

    @property
    def active(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_ACTIVE)

    @property
    def retriggered(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_RETRIGGERED)

    @property
    def rebound_timeout(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_REBOUND_TIMEOUT)

    @property
    def coast_valid(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_COAST_VALID)

    @property
    def rebound_valid(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_REBOUND_VALID)

    @property
    def ready_valid(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_READY_VALID)

    @property
    def velocity_valid(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_VELOCITY_VALID)


@dataclasses.dataclass
class StrikeTimingSample:
    sweep_param: str
    sweep_value: int
    requested_duty: int
    applied_duty: int
    configured_home_offset: Optional[int]
    repeat_index: int
    sequence: int
    coast_ms: Optional[int]
    rebound_ms: Optional[int]
    ready_ms: Optional[int]
    estimated_strike_velocity_dps: Optional[int]
    retriggered: bool
    rebound_timeout: bool
    drum_position: int
    home_position: int


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

    def _encode_reply_mode(self, reply_mode: str) -> int:
        mode = reply_mode.lower()
        if mode == REPLY_MODE_FULL:
            return SUBCMD_REPLY_FULL
        if mode == REPLY_MODE_ACK:
            return SUBCMD_REPLY_ACK
        if mode == REPLY_MODE_NONE:
            return SUBCMD_REPLY_NONE
        raise RingError(f"reply_mode must be one of: {REPLY_MODE_FULL}, {REPLY_MODE_ACK}, {REPLY_MODE_NONE}")

    def _build_addressed_payload(
        self,
        address: int,
        subcmd: int,
        data: bytes = b"",
        reply_mode: str = REPLY_MODE_FULL,
    ) -> bytes:
        encoded_subcmd = (subcmd & SUBCMD_MASK) | self._encode_reply_mode(reply_mode)
        return bytes([CMD_ADDR_BASE | address, encoded_subcmd]) + data

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

    def _recv_ack_reply(self, address: int, subcmd: int) -> CommandAck:
        deadline = time.monotonic() + self.timeout_ms / 1000.0
        expected_cmd = CMD_ACK_BASE | address
        expected_subcmd = subcmd & SUBCMD_MASK

        while True:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            payload = self._recv_frame(timeout_ms=remaining_ms)
            cmd = payload[0] if payload else 0
            if cmd == expected_cmd and len(payload) == 5 and (payload[1] & SUBCMD_MASK) == expected_subcmd:
                return CommandAck(
                    address=address,
                    subcmd=payload[1] & SUBCMD_MASK,
                    result=payload[2],
                    detail=struct.unpack(">H", payload[3:5])[0],
                )
            self._trace("skip", bytes([cmd]))

    def _addressed_command(
        self,
        address: int,
        subcmd: int,
        data: bytes = b"",
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        # "none" suppresses the device-generated reply, but the addressed
        # frame still circulates through the cut-through ring and will return
        # to the master RX path. Callers that burst fire-and-forget commands
        # should keep draining RX between bursts instead of assuming silence.
        self._check_address(address)
        self._flush_rx()
        self._send_frame(self._build_addressed_payload(address, subcmd, data, reply_mode))

        encoded_reply_mode = self._encode_reply_mode(reply_mode)
        if encoded_reply_mode == SUBCMD_REPLY_NONE:
            return None
        if encoded_reply_mode == SUBCMD_REPLY_ACK:
            return self._recv_ack_reply(address, subcmd)
        return self._recv_status_reply()

    def query_status(self, address: int) -> MotorStatus:
        self._check_address(address)
        self._flush_rx()
        self._send_frame(self._build_addressed_payload(address, SUBCMD_QUERY_STATUS))
        return self._recv_status_reply()

    def set_duty(
        self,
        address: int,
        duty: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        if duty < -32768 or duty > 32767:
            raise RingError("duty must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_DUTY, struct.pack(">h", duty), reply_mode)

    def set_torque(
        self,
        address: int,
        torque_ma: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        if torque_ma < 0 or torque_ma > 0xFFFF:
            raise RingError("torque must fit in uint16")
        return self._addressed_command(address, SUBCMD_SET_TORQUE, struct.pack(">H", torque_ma), reply_mode)

    def stop(
        self,
        address: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        return self._addressed_command(address, SUBCMD_STOP, reply_mode=reply_mode)

    def clear_fault(
        self,
        address: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        return self._addressed_command(address, SUBCMD_CLEAR_FAULT, reply_mode=reply_mode)

    def set_mode(
        self,
        address: int,
        mode: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        return self._addressed_command(address, SUBCMD_SET_MODE, bytes([mode & 0xFF]), reply_mode)

    def set_velocity(
        self,
        address: int,
        rpm: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        if rpm < -32768 or rpm > 32767:
            raise RingError("velocity must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_VELOCITY, struct.pack(">h", rpm), reply_mode)

    def set_ff(
        self,
        address: int,
        gain: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        if gain < -32768 or gain > 32767:
            raise RingError("ff gain must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_FF, struct.pack(">h", gain), reply_mode)

    def set_position(
        self,
        address: int,
        counts: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        return self._addressed_command(address, SUBCMD_SET_POSITION, struct.pack(">i", counts), reply_mode)

    def set_pos_pid(
        self,
        address: int,
        kp: int,
        ki: int,
        kd: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        for name, val in [("kp", kp), ("ki", ki), ("kd", kd)]:
            if val < -32768 or val > 32767:
                raise RingError(f"{name} must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_POS_PID, struct.pack(">hhh", kp, ki, kd), reply_mode)

    def zero_position(
        self,
        address: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        return self._addressed_command(address, SUBCMD_ZERO_POS, reply_mode=reply_mode)

    def save_settings(
        self,
        address: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        return self._addressed_command(address, SUBCMD_SAVE_SETTINGS, reply_mode=reply_mode)

    def clear_settings(
        self,
        address: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        return self._addressed_command(address, SUBCMD_CLEAR_SETTINGS, reply_mode=reply_mode)

    def set_pid(
        self,
        address: int,
        kp: int,
        ki: int,
        kd: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        for name, val in [("kp", kp), ("ki", ki), ("kd", kd)]:
            if val < -32768 or val > 32767:
                raise RingError(f"{name} must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_PID, struct.pack(">hhh", kp, ki, kd), reply_mode)

    # ── Strike commands ────────────────────────────────────────────────

    def strike(
        self,
        address: int,
        duty: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        return self._addressed_command(address, SUBCMD_STRIKE, struct.pack(">h", duty), reply_mode)

    def strike_home(
        self,
        address: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        return self._addressed_command(address, SUBCMD_STRIKE_HOME, reply_mode=reply_mode)

    def strike_cancel(
        self,
        address: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        return self._addressed_command(address, SUBCMD_STRIKE_CANCEL, reply_mode=reply_mode)

    def set_strike_param(
        self,
        address: int,
        param_id: int,
        value: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> MotorStatus | CommandAck | None:
        return self._addressed_command(
            address,
            SUBCMD_SET_STRIKE_PARAM,
            bytes([param_id & 0xFF]) + struct.pack(">h", value),
            reply_mode,
        )

    def query_strike(self, address: int) -> StrikeStatus:
        self._check_address(address)
        self._flush_rx()
        self._send_frame(self._build_addressed_payload(address, SUBCMD_QUERY_STRIKE))
        deadline = time.monotonic() + self.timeout_ms / 1000.0
        while True:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            payload = self._recv_frame(timeout_ms=remaining_ms)
            cmd = payload[0] if payload else 0
            if CMD_STATUS_BASE <= cmd < CMD_STATUS_END and len(payload) in (11, 22, 24, 30):
                return self._parse_strike_status(payload)
            self._trace("skip", bytes([cmd]))

    def _parse_strike_status(self, payload: bytes) -> StrikeStatus:
        cmd = payload[0]
        address = cmd & 0x0F
        if len(payload) == 30:
            return StrikeStatus(
                address=address,
                state=payload[1],
                homed=payload[2],
                flags=payload[3],
                sequence=struct.unpack(">H", payload[4:6])[0],
                last_duty=struct.unpack(">h", payload[6:8])[0],
                trigger_to_coast_ms=struct.unpack(">H", payload[8:10])[0],
                trigger_to_rebound_ms=struct.unpack(">H", payload[10:12])[0],
                trigger_to_ready_ms=struct.unpack(">H", payload[12:14])[0],
                estimated_strike_velocity_dps=struct.unpack(">H", payload[14:16])[0],
                drum_position=struct.unpack(">i", payload[16:20])[0],
                home_position=struct.unpack(">i", payload[20:24])[0],
                home_offset=struct.unpack(">h", payload[24:26])[0],
                coast_distance=struct.unpack(">h", payload[26:28])[0],
                homing_duty=struct.unpack(">h", payload[28:30])[0],
            )

        if len(payload) == 24:
            return StrikeStatus(
                address=address,
                state=payload[1],
                homed=payload[2],
                flags=payload[3],
                sequence=struct.unpack(">H", payload[4:6])[0],
                last_duty=struct.unpack(">h", payload[6:8])[0],
                trigger_to_coast_ms=struct.unpack(">H", payload[8:10])[0],
                trigger_to_rebound_ms=struct.unpack(">H", payload[10:12])[0],
                trigger_to_ready_ms=struct.unpack(">H", payload[12:14])[0],
                estimated_strike_velocity_dps=struct.unpack(">H", payload[14:16])[0],
                drum_position=struct.unpack(">i", payload[16:20])[0],
                home_position=struct.unpack(">i", payload[20:24])[0],
            )

        if len(payload) == 22:
            return StrikeStatus(
                address=address,
                state=payload[1],
                homed=payload[2],
                flags=payload[3],
                sequence=struct.unpack(">H", payload[4:6])[0],
                last_duty=struct.unpack(">h", payload[6:8])[0],
                trigger_to_coast_ms=struct.unpack(">H", payload[8:10])[0],
                trigger_to_rebound_ms=struct.unpack(">H", payload[10:12])[0],
                trigger_to_ready_ms=struct.unpack(">H", payload[12:14])[0],
                estimated_strike_velocity_dps=None,
                drum_position=struct.unpack(">i", payload[14:18])[0],
                home_position=struct.unpack(">i", payload[18:22])[0],
            )

        return StrikeStatus(
            address=address,
            state=payload[1],
            homed=payload[2],
            drum_position=struct.unpack(">i", payload[3:7])[0],
            home_position=struct.unpack(">i", payload[7:11])[0],
        )

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
        """Parse a STATUS_REPLY payload.

        v3 format (17 bytes): cmd state fault mode cur_hi cur_lo hall
            angle_hi angle_lo vel_hi vel_lo tgt_hi tgt_lo pos_b3 pos_b2 pos_b1 pos_b0
        v2 format (13 bytes): same but without position
        """
        if len(payload) < 7:
            raise RingError(f"status reply too short ({len(payload)} bytes): {payload.hex(' ')}")

        cmd = payload[0]
        if cmd < CMD_STATUS_BASE or cmd >= CMD_STATUS_END:
            raise RingError(f"not a status reply (cmd=0x{cmd:02X}): {payload.hex(' ')}")

        address = cmd & 0x0F
        state = payload[1]
        fault = payload[2]

        if len(payload) >= 13:
            mode = payload[3]
            current = struct.unpack(">H", payload[4:6])[0]
            hall = payload[6]
            angle = struct.unpack(">H", payload[7:9])[0]
            velocity = struct.unpack(">h", payload[9:11])[0]
            target = struct.unpack(">h", payload[11:13])[0]
        else:
            mode = 0
            current = struct.unpack(">H", payload[3:5])[0]
            hall = payload[5]
            angle = struct.unpack(">H", payload[6:8])[0] if len(payload) >= 8 else 0
            velocity = 0
            target = 0

        # v3: continuous encoder position (int32)
        if len(payload) >= 17:
            position = struct.unpack(">i", payload[13:17])[0]
        else:
            position = 0

        return MotorStatus(
            address=address,
            state=state,
            fault=fault,
            mode=mode,
            current_ma=current,
            hall=hall,
            angle=angle,
            velocity=velocity,
            target=target,
            position=position,
        )

    def _check_address(self, address: int) -> None:
        if address < 0 or address >= MAX_DEVICES:
            raise RingError(f"address must be 0..{MAX_DEVICES - 1}")
        if self.device_count is not None and address >= self.device_count:
            raise RingError(f"address {address} out of range (enumerated {self.device_count})")


# ── Display helpers ──────────────────────────────────────────────────────────

def format_status(s: MotorStatus) -> str:
    target_label = "duty" if s.mode == 0 else "rpm"
    return (
        f"addr={s.address} state={s.state_name} fault={s.fault_name} "
        f"mode={s.mode_name} current={s.current_ma}mA hall={s.hall} "
        f"angle={s.angle} ({s.angle_deg:.1f}\u00b0) pos={s.position} "
        f"vel={s.velocity}rpm target_{target_label}={s.target}"
    )


def build_sweep_values(start: int, stop: int, step: int) -> list[int]:
    values: list[int] = []

    if step == 0:
        raise RingError("sweep step must be non-zero")

    if start != stop and (stop - start) * step < 0:
        raise RingError("sweep step sign does not move start toward stop")

    current = start
    if step > 0:
        while current <= stop:
            values.append(current)
            current += step
    else:
        while current >= stop:
            values.append(current)
            current += step

    if not values:
        raise RingError("no sweep values generated")

    return values


def measurement_sweep_label(sweep_param: str) -> str:
    if sweep_param == "home-offset":
        return "home_offset"
    return "duty"


def measurement_sweep_axis_label(sweep_param: str) -> str:
    if sweep_param == "home-offset":
        return "Home offset above drum (encoder counts)"
    return "Requested strike duty"


def resolve_measurement_sweep_values(args: argparse.Namespace) -> list[int]:
    if args.values and args.duties:
        raise RingError("--values and --duties cannot be used together")
    if args.sweep_param != "duty" and args.duties:
        raise RingError("--duties can only be used when --sweep-param=duty")

    values = list(args.values) if args.values else list(args.duties) if args.duties else build_sweep_values(args.start, args.stop, args.step)

    if args.sweep_param == "duty" and any(value == 0 for value in values):
        raise RingError("duty sweep must not include 0 because STRIKE 0 is ignored")

    return values


def wait_for_strike_idle(
    client: RingClientV2,
    address: int,
    timeout_ms: int,
    poll_ms: int,
) -> StrikeStatus:
    deadline = time.monotonic() + timeout_ms / 1000.0
    last_status: Optional[StrikeStatus] = None
    poll_s = max(poll_ms, 0) / 1000.0

    while True:
        last_status = client.query_strike(address)
        if last_status.state == 0 and not last_status.active:
            return last_status
        if time.monotonic() >= deadline:
            break
        if poll_s > 0:
            time.sleep(poll_s)

    state_name = last_status.state_name if last_status is not None else "UNKNOWN"
    raise RingTimeout(f"timed out waiting for strike idle (last strike state: {state_name})")


def collect_strike_timing_sample(
    client: RingClientV2,
    address: int,
    sweep_param: str,
    sweep_value: int,
    strike_duty: int,
    repeat_index: int,
    poll_ms: int,
    idle_timeout_ms: int,
    strike_timeout_ms: int,
) -> StrikeTimingSample:
    baseline = wait_for_strike_idle(client, address, idle_timeout_ms, poll_ms)
    poll_s = max(poll_ms, 0) / 1000.0

    if not baseline.homed:
        raise RingError("strike actuator is not homed; run strike-home first")

    if sweep_param == "home-offset":
        response = client.set_strike_param(address, STRIKE_PARAM_HOME_OFFSET, sweep_value)
        if response.fault != 0:
            raise RingError(f"set home-offset faulted the motor: {format_status(response)}")
        baseline = wait_for_strike_idle(client, address, idle_timeout_ms, poll_ms)
        if baseline.home_offset is not None and baseline.home_offset != sweep_value:
            raise RingError(
                f"home_offset verification failed: expected {sweep_value}, got {baseline.home_offset}"
            )
        requested_duty = strike_duty
    else:
        requested_duty = sweep_value

    prev_sequence = baseline.sequence
    response = client.strike(address, requested_duty)
    if response.fault != 0:
        raise RingError(f"strike command faulted the motor: {format_status(response)}")

    deadline = time.monotonic() + strike_timeout_ms / 1000.0
    accepted_status: Optional[StrikeStatus] = None

    while time.monotonic() < deadline:
        status = client.query_strike(address)
        if status.sequence != prev_sequence:
            accepted_status = status
            break
        if poll_s > 0:
            time.sleep(poll_s)

    if accepted_status is None:
        raise RingTimeout(
            "strike sequence did not advance; the strike was rejected or the firmware "
            "does not expose timing telemetry"
        )

    accepted_sequence = accepted_status.sequence
    final_status = accepted_status

    while time.monotonic() < deadline:
        if final_status.sequence != accepted_sequence:
            raise RingError(
                f"strike sequence changed from {accepted_sequence} to {final_status.sequence} "
                "while waiting for completion"
            )
        if not final_status.active and final_status.ready_valid:
            return StrikeTimingSample(
                sweep_param=sweep_param,
                sweep_value=sweep_value,
                requested_duty=requested_duty,
                applied_duty=final_status.last_duty,
                configured_home_offset=final_status.home_offset,
                repeat_index=repeat_index,
                sequence=final_status.sequence,
                coast_ms=final_status.trigger_to_coast_ms if final_status.coast_valid else None,
                rebound_ms=final_status.trigger_to_rebound_ms if final_status.rebound_valid else None,
                ready_ms=final_status.trigger_to_ready_ms if final_status.ready_valid else None,
                estimated_strike_velocity_dps=(
                    final_status.estimated_strike_velocity_dps if final_status.velocity_valid else None
                ),
                retriggered=final_status.retriggered,
                rebound_timeout=final_status.rebound_timeout,
                drum_position=final_status.drum_position,
                home_position=final_status.home_position,
            )
        if poll_s > 0:
            time.sleep(poll_s)
        final_status = client.query_strike(address)

    raise RingTimeout(
        f"timed out waiting for strike sequence {accepted_sequence} to finish and report ready timing"
    )


def write_strike_timing_csv(samples: list[StrikeTimingSample], path: str) -> None:
    fieldnames = [
        "sweep_param",
        "sweep_value",
        "requested_duty",
        "applied_duty",
        "configured_home_offset",
        "repeat_index",
        "sequence",
        "coast_ms",
        "rebound_ms",
        "ready_ms",
        "estimated_strike_velocity_dps",
        "retriggered",
        "rebound_timeout",
        "drum_position",
        "home_position",
    ]

    with open(path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(
                {
                    "sweep_param": sample.sweep_param,
                    "sweep_value": sample.sweep_value,
                    "requested_duty": sample.requested_duty,
                    "applied_duty": sample.applied_duty,
                    "configured_home_offset": (
                        "" if sample.configured_home_offset is None else sample.configured_home_offset
                    ),
                    "repeat_index": sample.repeat_index,
                    "sequence": sample.sequence,
                    "coast_ms": "" if sample.coast_ms is None else sample.coast_ms,
                    "rebound_ms": "" if sample.rebound_ms is None else sample.rebound_ms,
                    "ready_ms": "" if sample.ready_ms is None else sample.ready_ms,
                    "estimated_strike_velocity_dps": (
                        "" if sample.estimated_strike_velocity_dps is None else sample.estimated_strike_velocity_dps
                    ),
                    "retriggered": int(sample.retriggered),
                    "rebound_timeout": int(sample.rebound_timeout),
                    "drum_position": sample.drum_position,
                    "home_position": sample.home_position,
                }
            )


def mean_or_none(values: list[Optional[int]]) -> Optional[float]:
    filtered = [value for value in values if value is not None]
    if not filtered:
        return None
    return sum(filtered) / len(filtered)


def print_strike_timing_summary(samples: list[StrikeTimingSample]) -> None:
    grouped: dict[int, list[StrikeTimingSample]] = {}
    sweep_param = samples[0].sweep_param if samples else "duty"
    value_label = measurement_sweep_label(sweep_param)

    for sample in samples:
        grouped.setdefault(sample.sweep_value, []).append(sample)

    print("Strike timing summary:")
    for sweep_value, sweep_samples in grouped.items():
        coast_mean = mean_or_none([sample.coast_ms for sample in sweep_samples])
        rebound_mean = mean_or_none([sample.rebound_ms for sample in sweep_samples])
        ready_mean = mean_or_none([sample.ready_ms for sample in sweep_samples])
        velocity_mean = mean_or_none([sample.estimated_strike_velocity_dps for sample in sweep_samples])
        timeout_count = sum(1 for sample in sweep_samples if sample.rebound_timeout)
        retrigger_count = sum(1 for sample in sweep_samples if sample.retriggered)
        duty_mean = mean_or_none([sample.requested_duty for sample in sweep_samples])

        coast_text = "n/a" if coast_mean is None else f"{coast_mean:.1f}"
        rebound_text = "n/a" if rebound_mean is None else f"{rebound_mean:.1f}"
        ready_text = "n/a" if ready_mean is None else f"{ready_mean:.1f}"
        velocity_text = "n/a" if velocity_mean is None else f"{velocity_mean:.1f}"
        duty_text = "n/a" if duty_mean is None else f"{duty_mean:.1f}"

        print(
            f"  {value_label}={sweep_value} n={len(sweep_samples)} "
            f"strike_duty_mean={duty_text} "
            f"coast_mean_ms={coast_text} rebound_mean_ms={rebound_text} "
            f"ready_mean_ms={ready_text} strike_vel_mean_dps={velocity_text} "
            f"rebound_timeouts={timeout_count} "
            f"retriggered={retrigger_count}"
        )


def plot_strike_timing_samples(
    samples: list[StrikeTimingSample],
    title: str,
    plot_path: Optional[str],
    show_plot: bool,
) -> None:
    try:
        if not show_plot:
            import matplotlib
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RingError("matplotlib is required for strike timing plots. Install with: pip install matplotlib") from exc

    grouped: dict[int, list[StrikeTimingSample]] = {}
    sweep_param = samples[0].sweep_param if samples else "duty"
    for sample in samples:
        grouped.setdefault(sample.sweep_value, []).append(sample)

    duties = sorted(grouped.keys())
    time_series = [
        ("coast_ms", "Trigger to coast", "tab:blue"),
        ("rebound_ms", "Trigger to rebound", "tab:orange"),
        ("ready_ms", "Trigger to ready", "tab:green"),
    ]
    velocity_series = ("estimated_strike_velocity_dps", "Strike velocity", "tab:red")

    fig, ax = plt.subplots(figsize=(9, 5))

    for attr_name, label, color in time_series:
        raw_x: list[int] = []
        raw_y: list[int] = []
        mean_x: list[int] = []
        mean_y: list[float] = []

        for duty in duties:
            values = [
                getattr(sample, attr_name)
                for sample in grouped[duty]
                if getattr(sample, attr_name) is not None
            ]
            raw_x.extend([duty] * len(values))
            raw_y.extend(values)
            if values:
                mean_x.append(duty)
                mean_y.append(sum(values) / len(values))

        if raw_y:
            ax.scatter(raw_x, raw_y, s=24, alpha=0.35, color=color)
            ax.plot(mean_x, mean_y, marker="o", linewidth=2, label=label, color=color)

    velocity_attr_name, velocity_label, velocity_color = velocity_series
    velocity_raw_x: list[int] = []
    velocity_raw_y: list[int] = []
    velocity_mean_x: list[int] = []
    velocity_mean_y: list[float] = []

    for duty in duties:
        values = [
            getattr(sample, velocity_attr_name)
            for sample in grouped[duty]
            if getattr(sample, velocity_attr_name) is not None
        ]
        velocity_raw_x.extend([duty] * len(values))
        velocity_raw_y.extend(values)
        if values:
            velocity_mean_x.append(duty)
            velocity_mean_y.append(sum(values) / len(values))

    velocity_ax = None
    if velocity_raw_y:
        velocity_ax = ax.twinx()
        velocity_ax.scatter(velocity_raw_x, velocity_raw_y, s=24, alpha=0.35, color=velocity_color)
        velocity_ax.plot(
            velocity_mean_x,
            velocity_mean_y,
            marker="s",
            linewidth=2,
            label=velocity_label,
            color=velocity_color,
        )
        velocity_ax.set_ylabel("Degrees per second", color=velocity_color)
        velocity_ax.tick_params(axis="y", labelcolor=velocity_color)

    ax.set_title(title)
    ax.set_xlabel(measurement_sweep_axis_label(sweep_param))
    ax.set_ylabel("Milliseconds")
    ax.grid(True, alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    if velocity_ax is not None:
        velocity_handles, velocity_labels = velocity_ax.get_legend_handles_labels()
        handles += velocity_handles
        labels += velocity_labels
    ax.legend(handles, labels)
    fig.tight_layout()

    if plot_path:
        fig.savefig(plot_path, dpi=160)

    if show_plot:
        plt.show()
    else:
        plt.close(fig)


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

    sp = sub.add_parser("set-position", help="Set position target (encoder counts, auto-sets POSITION mode)")
    sp.add_argument("address", type=int)
    sp.add_argument("counts", type=int, help="Signed int32 target in encoder counts")

    sp = sub.add_parser("set-pos-pid", help="Set position PID gains (Q8, output=RPM)")
    sp.add_argument("address", type=int)
    sp.add_argument("kp", type=int, help="Proportional gain (Q8)")
    sp.add_argument("ki", type=int, help="Integral gain (Q8)")
    sp.add_argument("kd", type=int, help="Derivative gain (Q8)")

    sp = sub.add_parser("zero-pos", help="Set the current absolute encoder angle as logical zero and save it")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("save-settings", help="Save current runtime settings/calibration to flash")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("clear-settings", help="Erase persisted settings from flash for next boot")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("set-mode", help="Set control mode (0=DUTY, 1=VELOCITY, 2=POSITION)")
    sp.add_argument("address", type=int)
    sp.add_argument("mode", type=int, choices=[0, 1, 2],
                    help="0=DUTY, 1=VELOCITY, 2=POSITION")

    sp = sub.add_parser("set-velocity", help="Set velocity target (RPM, auto-sets VELOCITY mode)")
    sp.add_argument("address", type=int)
    sp.add_argument("rpm", type=int, help="Signed int16 target RPM")

    sp = sub.add_parser("set-pid", help="Set velocity PID gains (Q8 fixed-point)")
    sp.add_argument("address", type=int)
    sp.add_argument("kp", type=int, help="Proportional gain (Q8: 256 = 1.0)")
    sp.add_argument("ki", type=int, help="Integral gain (Q8)")
    sp.add_argument("kd", type=int, help="Derivative gain (Q8)")

    sp = sub.add_parser("set-ff", help="Set velocity feedforward gain (Q8)")
    sp.add_argument("address", type=int)
    sp.add_argument("gain", type=int,
                    help="Duty per RPM of back-EMF (Q8: 256 = 1.0)")

    sp = sub.add_parser("strike", help="Trigger a strike")
    sp.add_argument("address", type=int)
    sp.add_argument("duty", type=int, help="Signed strike duty (loudness)")

    sp = sub.add_parser("strike-home", help="Run strike homing sequence")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("strike-cancel", help="Cancel strike/homing")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("strike-status", help="Query strike status")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("set-strike-param", help="Set strike parameter")
    sp.add_argument("address", type=int)
    sp.add_argument("param", choices=["home-offset", "coast-distance", "homing-duty"])
    sp.add_argument("value", type=int)

    sp = sub.add_parser("measure-strike-timing", help="Sweep strike timing versus duty or home-offset")
    sp.add_argument("address", type=int)
    sp.add_argument("--sweep-param", choices=["duty", "home-offset"], default="duty",
                    help="Parameter to sweep (default: duty)")
    sp.add_argument("--values", nargs="+", type=int,
                    help="Explicit sweep values; overrides --start/--stop/--step")
    sp.add_argument("--duties", nargs="+", type=int,
                    help="Explicit duty sweep values; duty sweep only, overrides --start/--stop/--step")
    sp.add_argument("--start", type=int, default=100,
                    help="Start value for generated sweep (default: 100)")
    sp.add_argument("--stop", type=int, default=1000,
                    help="Stop value for generated sweep, inclusive (default: 1000)")
    sp.add_argument("--step", type=int, default=100,
                    help="Step for generated sweep (default: 100)")
    sp.add_argument("--strike-duty", type=int, default=400,
                    help="Strike duty to use when sweeping non-duty parameters (default: 400)")
    sp.add_argument("--repeats", type=int, default=3,
                    help="Samples to collect per sweep value (default: 3)")
    sp.add_argument("--poll-ms", type=int, default=10,
                    help="Strike-status polling interval in ms (default: 10)")
    sp.add_argument("--idle-timeout-ms", type=int, default=3000,
                    help="Timeout waiting for idle before each strike (default: 3000)")
    sp.add_argument("--strike-timeout-ms", type=int, default=3000,
                    help="Timeout waiting for each strike to finish (default: 3000)")
    sp.add_argument("--pause-ms", type=int, default=100,
                    help="Pause after each completed strike sample (default: 100)")
    sp.add_argument("--csv", help="Write raw per-strike timing samples to CSV")
    sp.add_argument("--plot-out", help="Save the generated plot image to this path")
    sp.add_argument("--no-show", action="store_true",
                    help="Do not open the plot window")
    sp.add_argument("--no-plot", action="store_true",
                    help="Collect data without plotting")
    sp.add_argument("--title", help="Custom plot title")

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

        elif args.command == "set-position":
            print(format_status(client.set_position(args.address, args.counts)))

        elif args.command == "set-pos-pid":
            print(format_status(client.set_pos_pid(args.address, args.kp, args.ki, args.kd)))

        elif args.command == "zero-pos":
            print(format_status(client.zero_position(args.address)))

        elif args.command == "save-settings":
            print(format_status(client.save_settings(args.address)))

        elif args.command == "clear-settings":
            print(format_status(client.clear_settings(args.address)))

        elif args.command == "set-mode":
            print(format_status(client.set_mode(args.address, args.mode)))

        elif args.command == "set-velocity":
            print(format_status(client.set_velocity(args.address, args.rpm)))

        elif args.command == "set-pid":
            print(format_status(client.set_pid(args.address, args.kp, args.ki, args.kd)))

        elif args.command == "set-ff":
            print(format_status(client.set_ff(args.address, args.gain)))

        elif args.command == "strike":
            print(format_status(client.strike(args.address, args.duty)))

        elif args.command == "strike-home":
            print(format_status(client.strike_home(args.address)))

        elif args.command == "strike-cancel":
            print(format_status(client.strike_cancel(args.address)))

        elif args.command == "strike-status":
            ss = client.query_strike(args.address)
            print(f"addr={ss.address} strike={ss.state_name} homed={ss.homed} "
                  f"seq={ss.sequence} flags=0x{ss.flags:02X} duty={ss.last_duty} "
                  f"coast_ms={ss.trigger_to_coast_ms if ss.coast_valid else -1} "
                  f"rebound_ms={ss.trigger_to_rebound_ms if ss.rebound_valid else -1} "
                  f"ready_ms={ss.trigger_to_ready_ms if ss.ready_valid else -1} "
                  f"strike_vel_dps={ss.estimated_strike_velocity_dps if ss.velocity_valid and ss.estimated_strike_velocity_dps is not None else -1} "
                  f"active={int(ss.active)} retriggered={int(ss.retriggered)} "
                  f"rebound_timeout={int(ss.rebound_timeout)} "
                  f"drum_pos={ss.drum_position} home_pos={ss.home_position} "
                  f"home_offset={ss.home_offset if ss.home_offset is not None else 'n/a'} "
                  f"coast_distance={ss.coast_distance if ss.coast_distance is not None else 'n/a'} "
                  f"homing_duty={ss.homing_duty if ss.homing_duty is not None else 'n/a'}")

        elif args.command == "set-strike-param":
            param_map = {
                "home-offset": STRIKE_PARAM_HOME_OFFSET,
                "coast-distance": STRIKE_PARAM_COAST_DISTANCE,
                "homing-duty": STRIKE_PARAM_HOMING_DUTY,
            }
            print(format_status(client.set_strike_param(
                args.address, param_map[args.param], args.value)))
            ss = client.query_strike(args.address)
            print(f"strike params: home_offset={ss.home_offset if ss.home_offset is not None else 'n/a'} "
                  f"coast_distance={ss.coast_distance if ss.coast_distance is not None else 'n/a'} "
                  f"homing_duty={ss.homing_duty if ss.homing_duty is not None else 'n/a'}")

        elif args.command == "measure-strike-timing":
            if args.repeats < 1:
                raise RingError("--repeats must be at least 1")
            if args.poll_ms < 0 or args.idle_timeout_ms < 1 or args.strike_timeout_ms < 1 or args.pause_ms < 0:
                raise RingError("poll/timeout/pause values must be non-negative, and timeouts must be at least 1 ms")
            if args.no_plot and args.plot_out:
                raise RingError("--plot-out cannot be used together with --no-plot")
            if args.sweep_param != "duty" and args.strike_duty == 0:
                raise RingError("--strike-duty must be non-zero when sweeping non-duty parameters")

            sweep_values = resolve_measurement_sweep_values(args)
            sweep_label = measurement_sweep_label(args.sweep_param)
            total_samples = len(sweep_values) * args.repeats
            samples: list[StrikeTimingSample] = []
            completed = 0

            print(
                f"Collecting strike timing for addr={args.address} "
                f"{sweep_label}s={sweep_values} repeats={args.repeats} "
                f"strike_duty={args.strike_duty if args.sweep_param != 'duty' else 'swept'}"
            )

            for sweep_value in sweep_values:
                for repeat_index in range(1, args.repeats + 1):
                    completed += 1
                    print(f"[{completed}/{total_samples}] {sweep_label}={sweep_value} repeat={repeat_index}")
                    sample = collect_strike_timing_sample(
                        client=client,
                        address=args.address,
                        sweep_param=args.sweep_param,
                        sweep_value=sweep_value,
                        strike_duty=args.strike_duty,
                        repeat_index=repeat_index,
                        poll_ms=args.poll_ms,
                        idle_timeout_ms=args.idle_timeout_ms,
                        strike_timeout_ms=args.strike_timeout_ms,
                    )
                    samples.append(sample)
                    coast_text = "n/a" if sample.coast_ms is None else str(sample.coast_ms)
                    rebound_text = "n/a" if sample.rebound_ms is None else str(sample.rebound_ms)
                    ready_text = "n/a" if sample.ready_ms is None else str(sample.ready_ms)
                    velocity_text = (
                        "n/a" if sample.estimated_strike_velocity_dps is None
                        else str(sample.estimated_strike_velocity_dps)
                    )
                    print(
                        f"  seq={sample.sequence} {sweep_label}={sample.sweep_value} "
                        f"requested_duty={sample.requested_duty} applied_duty={sample.applied_duty} "
                        f"home_offset={sample.configured_home_offset if sample.configured_home_offset is not None else 'n/a'} "
                        f"coast_ms={coast_text} rebound_ms={rebound_text} ready_ms={ready_text} "
                        f"strike_vel_dps={velocity_text} "
                        f"rebound_timeout={int(sample.rebound_timeout)} retriggered={int(sample.retriggered)}"
                    )
                    if args.pause_ms > 0:
                        time.sleep(args.pause_ms / 1000.0)

            print_strike_timing_summary(samples)

            if args.csv:
                write_strike_timing_csv(samples, args.csv)
                print(f"CSV saved to {args.csv}")

            if not args.no_plot:
                plot_strike_timing_samples(
                    samples=samples,
                    title=args.title or (
                        f"Strike timing sweep addr {args.address} vs {sweep_label}"
                    ),
                    plot_path=args.plot_out,
                    show_plot=not args.no_show,
                )
                if args.plot_out:
                    print(f"Plot saved to {args.plot_out}")

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
