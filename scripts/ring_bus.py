#!/usr/bin/env python3
"""
Reusable Ring Bus protocol client for M2003 motor boards.
"""

from __future__ import annotations

import dataclasses
import struct
import time
from collections import deque
from typing import Iterable, Optional

import serial
import serial.tools.list_ports


# ── Framing ──────────────────────────────────────────────────────────────────

PREAMBLE = b"\xA5\x5A"
MAX_PAYLOAD = 64
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

SUBCMD_SET_DUTY      = 0x01
SUBCMD_SET_TORQUE    = 0x02
SUBCMD_STOP          = 0x03
SUBCMD_CLEAR_FAULT   = 0x04
SUBCMD_SET_MODE      = 0x05
SUBCMD_SET_VELOCITY  = 0x06
SUBCMD_SET_PID       = 0x07
SUBCMD_SET_FF        = 0x08
SUBCMD_SET_POSITION  = 0x09
SUBCMD_SET_POS_PID   = 0x0A
SUBCMD_ZERO_POS      = 0x0B
SUBCMD_STRIKE        = 0x0C
SUBCMD_STRIKE_HOME   = 0x0D
SUBCMD_STRIKE_CANCEL = 0x0E
SUBCMD_SET_STRIKE_PARAM = 0x0F
SUBCMD_QUERY_STATUS  = 0x10
SUBCMD_QUERY_STRIKE  = 0x11
SUBCMD_SAVE_SETTINGS = 0x12
SUBCMD_CLEAR_SETTINGS = 0x13
SUBCMD_SET_CUR_PID   = 0x14
SUBCMD_SET_CURRENT   = 0x15
SUBCMD_QUERY_TIMING  = 0x16
SUBCMD_DETECT_CSN_POLARITY = 0x17
SUBCMD_SET_CSN_POLARITY    = 0x18
SUBCMD_QUERY_STRIKE_TIMING = 0x19
SUBCMD_STRIKE_EX     = 0x1A
SUBCMD_ENTER_BOOTLOADER = 0x1B
SUBCMD_MASK            = 0x3F
SUBCMD_REPLY_FULL      = 0x00
SUBCMD_REPLY_ACK       = 0x40
SUBCMD_REPLY_NONE      = 0x80
SUBCMD_REPLY_ACK_TIMED = 0xC0

REPLY_MODE_FULL = "full"
REPLY_MODE_ACK = "ack"
REPLY_MODE_NONE = "none"
REPLY_MODE_ACK_TIMED = "ack-timed"

ACK_RESULT_OK               = 0x00
ACK_RESULT_OK_RETRIGGERED   = 0x01
ACK_RESULT_REJECT_NOT_HOMED = 0x02
ACK_RESULT_REJECT_FAULT     = 0x03
ACK_RESULT_REJECT_ZERO      = 0x04
ACK_RESULT_REJECT_NOT_READY = 0x05
ACK_RESULT_INVALID_ARGUMENT = 0x06
ACK_RESULT_PERSIST_FAILED   = 0x07

# ── Strike parameter IDs ───────────────────────────────────────────────────

STRIKE_PARAM_HOME_OFFSET    = 0x01
STRIKE_PARAM_COAST_DISTANCE = 0x02
STRIKE_PARAM_HOMING_DUTY    = 0x03
STRIKE_PARAM_MUTE_BRAKE_MS  = 0x04
STRIKE_PARAM_MUTE_PRESS_MA  = 0x05
STRIKE_PARAM_MUTE_ENGAGE_OFFSET = 0x06

# ── Strike articulation types (STRIKE_EX) ──────────────────────────────────

STRIKE_TYPE_NORMAL = 0
STRIKE_TYPE_DEAD   = 1

STRIKE_TIMING_COAST_VALID     = 0x0001
STRIKE_TIMING_REBOUND_VALID   = 0x0002
STRIKE_TIMING_READY_VALID     = 0x0004
STRIKE_TIMING_ACTIVE          = 0x0008
STRIKE_TIMING_RETRIGGERED     = 0x0010
STRIKE_TIMING_REBOUND_TIMEOUT = 0x0020
STRIKE_TIMING_VELOCITY_VALID  = 0x0040
STRIKE_TIMING_RETRIGGER_READY_VALID = 0x0080
STRIKE_TIMING_IMPACT_VALID    = 0x0100
STRIKE_TIMING_DEAD            = 0x0200

# ── Timing defaults ─────────────────────────────────────────────────────────

DEFAULT_BAUD       = 250000
DEFAULT_TIMEOUT_MS = 200
DEFAULT_SETTLE_MS  = 250
RX_DEBUG_TAIL_LIMIT = 160
RX_DEBUG_EVENT_LIMIT = 4


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
CTRL_MODES    = {0: "DUTY", 1: "VELOCITY", 2: "POSITION", 3: "TORQUE"}
STRIKE_STATES = {0: "IDLE", 1: "HOMING", 2: "DRIVING", 3: "COASTING", 4: "LEGACY_RETURNING", 5: "CATCHING", 6: "MUTING"}
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
    SUBCMD_SET_CUR_PID: "SET_CUR_PID",
    SUBCMD_SET_CURRENT: "SET_CURRENT",
    SUBCMD_QUERY_TIMING: "QUERY_TIMING",
    SUBCMD_DETECT_CSN_POLARITY: "DETECT_CSN_POLARITY",
    SUBCMD_SET_CSN_POLARITY: "SET_CSN_POLARITY",
    SUBCMD_QUERY_STRIKE_TIMING: "QUERY_STRIKE_TIMING",
    SUBCMD_STRIKE_EX: "STRIKE_EX",
    SUBCMD_ENTER_BOOTLOADER: "ENTER_BOOTLOADER",
}
ACK_RESULT_NAMES = {
    ACK_RESULT_OK: "OK",
    ACK_RESULT_OK_RETRIGGERED: "OK_RETRIGGERED",
    ACK_RESULT_REJECT_NOT_HOMED: "REJECT_NOT_HOMED",
    ACK_RESULT_REJECT_FAULT: "REJECT_FAULT",
    ACK_RESULT_REJECT_ZERO: "REJECT_ZERO",
    ACK_RESULT_REJECT_NOT_READY: "REJECT_NOT_READY",
    ACK_RESULT_INVALID_ARGUMENT: "INVALID_ARGUMENT",
    ACK_RESULT_PERSIST_FAILED: "PERSIST_FAILED",
}


def _format_hex(data: bytes, limit: int = RX_DEBUG_TAIL_LIMIT) -> str:
    if not data:
        return "<none>"
    if len(data) <= limit:
        return data.hex(" ").upper()
    shown = data[-limit:].hex(" ").upper()
    return f"...(+{len(data) - limit} bytes) {shown}"


def _append_tail(buf: bytearray, data: bytes | bytearray | int) -> None:
    if isinstance(data, int):
        buf.append(data & 0xFF)
    else:
        buf.extend(data)
    if len(buf) > RX_DEBUG_TAIL_LIMIT:
        del buf[:len(buf) - RX_DEBUG_TAIL_LIMIT]


def _remember_event(events: list[bytes], data: bytes) -> None:
    events.append(data)
    if len(events) > RX_DEBUG_EVENT_LIMIT:
        del events[0]


def _cmd_name(cmd: int) -> str:
    if cmd == CMD_ENTER_SF:
        return "ENTER_SF"
    if cmd == CMD_ENTER_CT:
        return "ENTER_CT"
    if cmd == CMD_SET_ADDRESS:
        return "SET_ADDRESS"
    if cmd == CMD_BROADCAST_DUTY:
        return "BROADCAST_DUTY"
    if CMD_ADDR_BASE <= cmd < CMD_ADDR_END:
        return f"ADDR_CMD(addr={cmd & 0x0F})"
    if CMD_STATUS_BASE <= cmd < CMD_STATUS_END:
        return f"STATUS_REPLY(addr={cmd & 0x0F})"
    if CMD_ACK_BASE <= cmd < CMD_ACK_END:
        return f"ACK_REPLY(addr={cmd & 0x0F})"
    return f"UNKNOWN(0x{cmd:02X})"


def _subcmd_name(subcmd: int) -> str:
    return SUBCMD_NAMES.get(subcmd & SUBCMD_MASK, f"UNKNOWN(0x{subcmd & SUBCMD_MASK:02X})")


def _describe_payload(payload: bytes) -> str:
    if not payload:
        return "len=0 payload=<empty>"

    cmd = payload[0]
    parts = [f"len={len(payload)}", f"cmd=0x{cmd:02X}({_cmd_name(cmd)})"]

    if CMD_ADDR_BASE <= cmd < CMD_ADDR_END and len(payload) >= 2:
        raw_subcmd = payload[1]
        parts.append(f"subcmd=0x{raw_subcmd & SUBCMD_MASK:02X}({_subcmd_name(raw_subcmd)})")
        parts.append(f"reply_bits=0x{raw_subcmd & 0xC0:02X}")
    elif CMD_ACK_BASE <= cmd < CMD_ACK_END and len(payload) >= 2:
        parts.append(f"subcmd=0x{payload[1] & SUBCMD_MASK:02X}({_subcmd_name(payload[1])})")
        if len(payload) >= 5:
            result = payload[2]
            detail = struct.unpack(">H", payload[3:5])[0]
            result_name = ACK_RESULT_NAMES.get(result, f"UNKNOWN(0x{result:02X})")
            parts.append(f"result=0x{result:02X}({result_name})")
            parts.append(f"detail={detail}")

    parts.append(f"payload={_format_hex(payload)}")
    return " ".join(parts)


def _format_skipped(skipped: list[bytes]) -> str:
    if not skipped:
        return "skipped=no valid nonmatching frames"
    return "skipped=" + " | ".join(_describe_payload(payload) for payload in skipped)


def _format_timeout_context(context: str, skipped: list[bytes], exc: RingTimeout) -> str:
    return f"{context}; {_format_skipped(skipped)}; last_frame_read=({exc})"


@dataclasses.dataclass
class MotorStatus:
    address: int
    state: int
    fault: int
    mode: int
    current_ma: int
    hall: int
    angle: int
    velocity: int
    target: int
    position: int

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
class StrikeTiming:
    """Compact snapshot of the most recently completed strike's timing.

    Returned by QUERY_STRIKE_TIMING and piggybacked on REPLY_MODE_ACK_TIMED.
    `sequence` correlates the snapshot with which strike command produced
    it; if `sequence == 0` the device has not completed any strike yet."""

    address: int
    sequence: int
    flags: int
    trigger_to_coast_ms: Optional[int]
    trigger_to_impact_ms: Optional[int]
    trigger_to_rebound_ms: Optional[int]
    estimated_strike_velocity_dps: Optional[int]

    @property
    def coast_valid(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_COAST_VALID)

    @property
    def impact_valid(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_IMPACT_VALID)

    @property
    def rebound_valid(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_REBOUND_VALID)

    @property
    def velocity_valid(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_VELOCITY_VALID)

    @property
    def rebound_timeout(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_REBOUND_TIMEOUT)

    @property
    def has_data(self) -> bool:
        """True if any of the validity flags we care about are set. False
        means the device hasn't snapshotted a completed strike yet."""
        return bool(self.flags & (
            STRIKE_TIMING_COAST_VALID
            | STRIKE_TIMING_IMPACT_VALID
            | STRIKE_TIMING_REBOUND_VALID
        ))


@dataclasses.dataclass
class CommandAck:
    address: int
    subcmd: int
    result: int
    detail: int
    metrics: Optional[StrikeTiming] = None

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
    state: int
    homed: int
    flags: int = 0
    sequence: int = 0
    last_current_ma: int = 0
    trigger_to_coast_ms: int = 0
    trigger_to_impact_ms: Optional[int] = None
    trigger_to_rebound_ms: int = 0
    trigger_to_retrigger_ready_ms: int = 0
    trigger_to_ready_ms: int = 0
    estimated_strike_velocity_dps: Optional[int] = None
    drum_position: int = 0
    home_position: int = 0
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

    @property
    def retrigger_ready_valid(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_RETRIGGER_READY_VALID)

    @property
    def impact_valid(self) -> bool:
        return bool(self.flags & STRIKE_TIMING_IMPACT_VALID)


@dataclasses.dataclass
class TimingStatus:
    address: int
    control_budget_us: int
    control_last_us: int
    control_max_us: int
    control_overrun_count: int
    velocity_drop_count: int
    position_drop_count: int
    strike_drop_count: int
    protocol_drop_count: int
    hall_last_us: int
    hall_max_us: int
    uart_last_us: int
    uart_max_us: int
    adc_last_us: int
    adc_max_us: int
    protocol_poll_last_us: int
    protocol_poll_max_us: int
    protocol_backlog_max: int
    uptime_ms: int
    uart_rx_overflow_count: int = 0
    adc_overrun_count: int = 0
    proto_dbg_sequence: int = 0
    proto_dbg_cmd_type: int = 0
    proto_dbg_len: int = 0
    proto_dbg_target: int = 0
    proto_dbg_raw_subcmd: int = 0
    proto_dbg_subcmd: int = 0
    proto_dbg_reply_mode_initial: int = 0
    proto_dbg_reply_mode_final: int = 0
    proto_dbg_full_reply_kind: int = 0
    proto_dbg_reply_branch: int = 0
    proto_dbg_ack_result: int = 0
    proto_dbg_ack_detail: int = 0
    proto_dbg_frame_start_mode: int = 0
    proto_dbg_fwd_mode: int = 0

    @property
    def control_last_pct(self) -> float:
        if self.control_budget_us <= 0:
            return 0.0
        return 100.0 * self.control_last_us / self.control_budget_us

    @property
    def control_max_pct(self) -> float:
        if self.control_budget_us <= 0:
            return 0.0
        return 100.0 * self.control_max_us / self.control_budget_us


AddressedReply = MotorStatus | CommandAck | TimingStatus | None


def _parse_strike_timing_payload(address: int, body: bytes) -> StrikeTiming:
    """Parse the 12-byte timing tail shared by REPLY_MODE_ACK_TIMED and
    QUERY_STRIKE_TIMING. Layout:
        seq:u16 | flags:u16 | t_coast:u16 | t_impact:u16 | t_rebound:u16 | vel:u16
    Each value's "valid" bit decides whether the corresponding field on
    the returned StrikeTiming is reported or left as ``None``.
    """
    if len(body) != 12:
        raise RingError(f"strike timing payload must be 12 bytes, got {len(body)}")
    seq = struct.unpack(">H", body[0:2])[0]
    flags = struct.unpack(">H", body[2:4])[0]
    t_coast = struct.unpack(">H", body[4:6])[0]
    t_impact = struct.unpack(">H", body[6:8])[0]
    t_rebound = struct.unpack(">H", body[8:10])[0]
    vel = struct.unpack(">H", body[10:12])[0]
    return StrikeTiming(
        address=address,
        sequence=seq,
        flags=flags,
        trigger_to_coast_ms=t_coast if (flags & STRIKE_TIMING_COAST_VALID) else None,
        trigger_to_impact_ms=t_impact if (flags & STRIKE_TIMING_IMPACT_VALID) else None,
        trigger_to_rebound_ms=t_rebound if (flags & STRIKE_TIMING_REBOUND_VALID) else None,
        estimated_strike_velocity_dps=vel if (flags & STRIKE_TIMING_VELOCITY_VALID) else None,
    )


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
        self.last_strike_status_payload_len: Optional[int] = None
        self._last_tx_frame: bytes = b""
        self._last_tx_payload: bytes = b""

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

    def _build_frame(self, payload: bytes) -> bytes:
        length = len(payload)
        body = bytes([length]) + payload
        crc = crc16_ccitt(body)
        return PREAMBLE + body + struct.pack(">H", crc)

    def _send_frame(self, payload: bytes) -> None:
        assert self.ser is not None
        frame = self._build_frame(payload)
        self._last_tx_frame = frame
        self._last_tx_payload = payload
        self._trace("tx", frame)
        self.ser.write(frame)
        self.ser.flush()

    def _recv_frame(self, timeout_ms: Optional[int] = None) -> bytes:
        assert self.ser is not None
        timeout_s = (timeout_ms or self.timeout_ms) / 1000.0
        deadline = time.monotonic() + timeout_s

        scan_0, scan_1, buffering = 0, 1, 2
        phase = scan_0
        buf = bytearray()
        expected = 0
        raw_tail = bytearray()
        crc_failures: list[bytes] = []
        invalid_lengths: list[bytes] = []
        replay: deque[int] = deque()

        def timeout_debug() -> str:
            pending_error = None
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    pending = self.ser.read(waiting)
                    _append_tail(raw_tail, pending)
            except Exception as exc:  # pyserial diagnostics must not mask timeout
                pending_error = exc

            phase_names = {
                scan_0: "scan_0",
                scan_1: "scan_1",
                buffering: "buffering",
            }
            parts = [
                f"phase={phase_names.get(phase, str(phase))}",
                f"rx_tail={_format_hex(bytes(raw_tail))}",
            ]
            if phase == scan_1:
                parts.append("partial_preamble=A5")
            elif phase == buffering and buf:
                partial = PREAMBLE + bytes(buf)
                expected_desc = str(expected) if expected else "unknown"
                parts.append(f"partial_frame={_format_hex(partial)}")
                parts.append(f"partial_len={len(partial)}")
                parts.append(f"expected_body_len={expected_desc}")
            if invalid_lengths:
                parts.append(
                    "invalid_len_frames="
                    + " | ".join(_format_hex(frame) for frame in invalid_lengths)
                )
            if crc_failures:
                parts.append(
                    "crc_fail_frames="
                    + " | ".join(_format_hex(frame) for frame in crc_failures)
                )
            if pending_error is not None:
                parts.append(f"pending_read_error={pending_error}")
            return "; ".join(parts)

        def next_byte() -> int:
            while True:
                if replay:
                    return replay.popleft()
                if time.monotonic() >= deadline:
                    raise RingTimeout(f"timeout waiting for frame; {timeout_debug()}")
                byte = self.ser.read(1)
                if byte:
                    _append_tail(raw_tail, byte[0])
                    return byte[0]

        while True:
            c = next_byte()

            if phase == scan_0:
                if c == 0xA5:
                    phase = scan_1

            elif phase == scan_1:
                if c == 0x5A:
                    phase = buffering
                    buf.clear()
                    expected = 0
                elif c == 0xA5:
                    pass
                else:
                    phase = scan_0

            elif phase == buffering:
                buf.append(c)

                if len(buf) == 1:
                    length = buf[0]
                    if length == 0 or length > MAX_PAYLOAD:
                        bad_frame = PREAMBLE + bytes(buf)
                        _remember_event(invalid_lengths, bad_frame)
                        # Restart one byte after the candidate preamble. This
                        # preserves a valid frame whose A5 5A appeared inside
                        # a corrupt length/body instead of discarding it.
                        replay.extend(bad_frame[1:])
                        phase = scan_0
                        buf.clear()
                        expected = 0
                        continue
                    expected = 1 + length + 2

                if expected and len(buf) >= expected:
                    length = buf[0]
                    payload = bytes(buf[1:1 + length])
                    rx_crc = struct.unpack(">H", buf[1 + length:1 + length + 2])[0]
                    calc_crc = crc16_ccitt(bytes(buf[:1 + length]))

                    if calc_crc == rx_crc:
                        self._trace("rx", PREAMBLE + bytes(buf))
                        return payload

                    bad_frame = PREAMBLE + bytes(buf)
                    _remember_event(crc_failures, bad_frame)
                    self._trace("rx-crc-fail", bad_frame)
                    replay.extend(bad_frame[1:])
                    phase = scan_0
                    buf.clear()
                    expected = 0

    def _trace(self, label: str, data: bytes) -> None:
        if self.trace:
            print(f"  [{label}] {data.hex(' ').upper()}")

    def _print_unexpected_reply(self, context: str, payload: bytes) -> None:
        print(
            "  [unexpected-reply] "
            f"{context}; "
            f"last_tx_frame={_format_hex(self._last_tx_frame)}; "
            f"last_tx_payload={_format_hex(self._last_tx_payload)}; "
            f"rx_payload={_describe_payload(payload)}"
        )

    def enumerate(self) -> int:
        def receive_matching(predicate, description: str) -> bytes:
            deadline = time.monotonic() + self.timeout_ms / 1000.0
            skipped: list[bytes] = []
            while True:
                remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
                try:
                    payload = self._recv_frame(timeout_ms=remaining_ms)
                except RingTimeout as exc:
                    skipped_text = ", ".join(item.hex(" ") for item in skipped)
                    raise RingTimeout(
                        f"timeout waiting for {description}; skipped=[{skipped_text}]"
                    ) from exc
                if predicate(payload):
                    return payload
                _remember_event(skipped, payload)
                self._trace("skip", payload)

        self._flush_rx()
        self._send_frame(bytes([CMD_ENTER_SF]))
        receive_matching(
            lambda payload: payload == bytes([CMD_ENTER_SF]),
            "ENTER_STORE_FORWARD echo",
        )

        seed = bytes([CMD_SET_ADDRESS, 0x00])
        self._send_frame(seed)
        payload = receive_matching(
            lambda reply: (
                len(reply) == 2
                and reply[0] == CMD_SET_ADDRESS
                and 0 < reply[1] <= MAX_DEVICES
            ),
            "SET_ADDRESS count",
        )

        self.device_count = payload[1]

        self._send_frame(bytes([CMD_ENTER_CT]))
        receive_matching(
            lambda reply: reply == bytes([CMD_ENTER_CT]),
            "ENTER_CUT_THROUGH echo",
        )
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
        if mode == REPLY_MODE_ACK_TIMED:
            return SUBCMD_REPLY_ACK_TIMED
        raise RingError(
            f"reply_mode must be one of: {REPLY_MODE_FULL}, {REPLY_MODE_ACK}, "
            f"{REPLY_MODE_ACK_TIMED}, {REPLY_MODE_NONE}"
        )

    def _build_addressed_payload(
        self,
        address: int,
        subcmd: int,
        data: bytes = b"",
        reply_mode: str = REPLY_MODE_FULL,
    ) -> bytes:
        encoded_subcmd = (subcmd & SUBCMD_MASK) | self._encode_reply_mode(reply_mode)
        return bytes([CMD_ADDR_BASE | address, encoded_subcmd]) + data

    def _recv_status_reply(self, address: Optional[int] = None) -> MotorStatus:
        deadline = time.monotonic() + self.timeout_ms / 1000.0
        expected_cmd = None if address is None else (CMD_STATUS_BASE | address)
        skipped: list[bytes] = []
        while True:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            try:
                payload = self._recv_frame(timeout_ms=remaining_ms)
            except RingTimeout as exc:
                expected = (
                    "any status reply"
                    if expected_cmd is None
                    else f"cmd=0x{expected_cmd:02X}({_cmd_name(expected_cmd)})"
                )
                raise RingTimeout(_format_timeout_context(
                    f"timeout waiting for status reply {expected}",
                    skipped,
                    exc,
                )) from exc
            cmd = payload[0] if payload else 0
            if (
                CMD_STATUS_BASE <= cmd < CMD_STATUS_END
                and len(payload) in (13, 17)
                and (expected_cmd is None or cmd == expected_cmd)
            ):
                return self._parse_status(payload)
            _remember_event(skipped, payload)
            self._trace("skip", payload)

    def _recv_ack_reply(self, address: int, subcmd: int) -> CommandAck:
        deadline = time.monotonic() + self.timeout_ms / 1000.0
        expected_cmd = CMD_ACK_BASE | address
        expected_subcmd = subcmd & SUBCMD_MASK
        skipped: list[bytes] = []

        while True:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            try:
                payload = self._recv_frame(timeout_ms=remaining_ms)
            except RingTimeout as exc:
                raise RingTimeout(_format_timeout_context(
                    f"timeout waiting for ACK reply addr={address} "
                    f"expected_cmd=0x{expected_cmd:02X}({_cmd_name(expected_cmd)}) "
                    f"expected_subcmd=0x{expected_subcmd:02X}({_subcmd_name(expected_subcmd)})",
                    skipped,
                    exc,
                )) from exc
            cmd = payload[0] if payload else 0
            payload_subcmd = payload[1] & SUBCMD_MASK if len(payload) >= 2 else 0
            if cmd == expected_cmd and payload_subcmd == expected_subcmd:
                ack = self._parse_ack_payload(address, expected_subcmd, payload)
                if ack is not None:
                    return ack
            elif cmd == (CMD_STATUS_BASE | address):
                self._print_unexpected_reply(
                    f"expected ACK reply addr={address} "
                    f"expected_cmd=0x{expected_cmd:02X}({_cmd_name(expected_cmd)}) "
                    f"expected_subcmd=0x{expected_subcmd:02X}({_subcmd_name(expected_subcmd)})",
                    payload,
                )
            _remember_event(skipped, payload)
            self._trace("skip", payload)

    @staticmethod
    def _parse_ack_payload(address: int, subcmd: int, payload: bytes) -> Optional[CommandAck]:
        if len(payload) not in (5, 17):
            return None
        if payload[0] != (CMD_ACK_BASE | address):
            return None
        payload_subcmd = payload[1] & SUBCMD_MASK
        if payload_subcmd != (subcmd & SUBCMD_MASK):
            return None
        metrics = None
        if len(payload) == 17:
            metrics = _parse_strike_timing_payload(address, payload[5:17])
        return CommandAck(
            address=address,
            subcmd=payload_subcmd,
            result=payload[2],
            detail=struct.unpack(">H", payload[3:5])[0],
            metrics=metrics,
        )

    def _addressed_command(
        self,
        address: int,
        subcmd: int,
        data: bytes = b"",
        reply_mode: str = REPLY_MODE_FULL,
    ) -> AddressedReply:
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
        # Both ACK and ACK_TIMED come back as type 0x50+addr — they only
        # differ in payload length (5 vs 17). _recv_ack_reply already
        # handles both lengths and attaches the piggyback metrics from the
        # 17-byte form.
        if encoded_reply_mode == SUBCMD_REPLY_ACK or encoded_reply_mode == SUBCMD_REPLY_ACK_TIMED:
            return self._recv_ack_reply(address, subcmd)
        return self._recv_status_reply(address)

    def query_status(self, address: int) -> MotorStatus:
        self._check_address(address)
        self._flush_rx()
        self._send_frame(self._build_addressed_payload(address, SUBCMD_QUERY_STATUS))
        return self._recv_status_reply(address)

    def set_duty(self, address: int, duty: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        if duty < -32768 or duty > 32767:
            raise RingError("duty must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_DUTY, struct.pack(">h", duty), reply_mode)

    def set_torque(self, address: int, torque_ma: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        if torque_ma < 0 or torque_ma > 0xFFFF:
            raise RingError("torque must fit in uint16")
        return self._addressed_command(address, SUBCMD_SET_TORQUE, struct.pack(">H", torque_ma), reply_mode)

    def stop(self, address: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_STOP, reply_mode=reply_mode)

    def clear_fault(self, address: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_CLEAR_FAULT, reply_mode=reply_mode)

    def set_mode(self, address: int, mode: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_SET_MODE, bytes([mode & 0xFF]), reply_mode)

    def set_velocity(self, address: int, rpm: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        if rpm < -32768 or rpm > 32767:
            raise RingError("velocity must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_VELOCITY, struct.pack(">h", rpm), reply_mode)

    def set_ff(self, address: int, gain: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        if gain < -32768 or gain > 32767:
            raise RingError("ff gain must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_FF, struct.pack(">h", gain), reply_mode)

    def set_position(self, address: int, counts: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_SET_POSITION, struct.pack(">i", counts), reply_mode)

    def set_pos_pid(
        self,
        address: int,
        kp: int,
        ki: int,
        kd: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> AddressedReply:
        for name, value in [("kp", kp), ("ki", ki), ("kd", kd)]:
            if value < -32768 or value > 32767:
                raise RingError(f"{name} must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_POS_PID, struct.pack(">hhh", kp, ki, kd), reply_mode)

    def set_current(self, address: int, current_ma: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        if current_ma < -32768 or current_ma > 32767:
            raise RingError("current must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_CURRENT, struct.pack(">h", current_ma), reply_mode)

    def set_cur_pid(
        self,
        address: int,
        kp: int,
        ki: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> AddressedReply:
        for name, value in [("kp", kp), ("ki", ki)]:
            if value < -32768 or value > 32767:
                raise RingError(f"{name} must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_CUR_PID, struct.pack(">hh", kp, ki), reply_mode)

    def zero_position(self, address: int, reply_mode: str = REPLY_MODE_ACK) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_ZERO_POS, reply_mode=reply_mode)

    def save_settings(self, address: int, reply_mode: str = REPLY_MODE_ACK) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_SAVE_SETTINGS, reply_mode=reply_mode)

    def clear_settings(self, address: int, reply_mode: str = REPLY_MODE_ACK) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_CLEAR_SETTINGS, reply_mode=reply_mode)

    def enter_bootloader(self, address: int) -> AddressedReply:
        """Stop, ACK, and reset one application into its permanent LDROM loader."""
        return self._addressed_command(
            address,
            SUBCMD_ENTER_BOOTLOADER,
            reply_mode=REPLY_MODE_ACK,
        )

    def detect_csn_polarity(self, address: int, reply_mode: str = REPLY_MODE_ACK) -> AddressedReply:
        """Probe both SSI CSn polarities, keep the one whose MT6701 frame passes CRC, persist on success.
        ACK detail field carries the chosen polarity (0 or 1).  Rejected with NOT_READY while motor is running."""
        return self._addressed_command(address, SUBCMD_DETECT_CSN_POLARITY, reply_mode=reply_mode)

    def set_csn_polarity(self, address: int, level: int, reply_mode: str = REPLY_MODE_ACK) -> AddressedReply:
        """Manually override SSI CSn polarity (0 = FET-inverting variant, 1 = modchip variant) and persist.
        Rejected with NOT_READY while motor is running, INVALID_ARGUMENT for any value other than 0 or 1."""
        if level not in (0, 1):
            raise RingError("level must be 0 or 1")
        return self._addressed_command(address, SUBCMD_SET_CSN_POLARITY, bytes([level]), reply_mode)

    def set_pid(
        self,
        address: int,
        kp: int,
        ki: int,
        kd: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> AddressedReply:
        for name, value in [("kp", kp), ("ki", ki), ("kd", kd)]:
            if value < -32768 or value > 32767:
                raise RingError(f"{name} must fit in int16")
        return self._addressed_command(address, SUBCMD_SET_PID, struct.pack(">hhh", kp, ki, kd), reply_mode)

    def strike(self, address: int, current_ma: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_STRIKE, struct.pack(">h", current_ma), reply_mode)

    def strike_burst(
        self,
        strikes: Iterable[tuple[int, int]],
        reply_mode: str = REPLY_MODE_ACK,
    ) -> list[Optional[CommandAck]]:
        """Transmit a same-deadline chord without waiting between commands.

        Frames are put on the wire farthest-address first so each reply is
        inserted downstream of every later target. Replies are collected only
        after the complete command train has been written. The returned list
        follows the caller's input order; a missing ACK is represented by None.
        """
        assert self.ser is not None
        items = [(int(address), int(current_ma)) for address, current_ma in strikes]
        if not items:
            return []
        addresses = [address for address, _ in items]
        if len(set(addresses)) != len(addresses):
            raise RingError("strike burst requires unique addresses")
        encoded_reply_mode = self._encode_reply_mode(reply_mode)
        if encoded_reply_mode not in (
            SUBCMD_REPLY_ACK,
            SUBCMD_REPLY_ACK_TIMED,
            SUBCMD_REPLY_NONE,
        ):
            raise RingError("strike burst reply mode must be ack, ack-timed, or none")

        indexed = list(enumerate(items))
        ordered = sorted(indexed, key=lambda entry: entry[1][0], reverse=True)
        frames: list[tuple[int, int, bytes, bytes]] = []
        for original_index, (address, current_ma) in ordered:
            self._check_address(address)
            if current_ma < -32768 or current_ma > 32767:
                raise RingError("strike current must fit in int16")
            payload = self._build_addressed_payload(
                address,
                SUBCMD_STRIKE,
                struct.pack(">h", current_ma),
                reply_mode,
            )
            frames.append((original_index, address, payload, self._build_frame(payload)))

        self._flush_rx()
        wire = b"".join(frame for _, _, _, frame in frames)
        for _, _, _, frame in frames:
            self._trace("tx-burst", frame)
        written = self.ser.write(wire)
        self.ser.flush()
        if written != len(wire):
            raise RingError(f"short serial burst write: {written}/{len(wire)} bytes")
        self._last_tx_payload = frames[-1][2]
        self._last_tx_frame = frames[-1][3]

        results: list[Optional[CommandAck]] = [None] * len(items)
        if encoded_reply_mode == SUBCMD_REPLY_NONE:
            return results

        index_by_address = {address: original_index for original_index, address, _, _ in frames}
        pending = set(index_by_address)
        deadline = time.monotonic() + self.timeout_ms / 1000.0
        while pending:
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0:
                break
            remaining_ms = max(1, int(remaining_s * 1000))
            try:
                payload = self._recv_frame(timeout_ms=remaining_ms)
            except RingTimeout:
                break
            cmd = payload[0] if payload else 0
            if CMD_ACK_BASE <= cmd < CMD_ACK_END:
                address = cmd & 0x0F
                if address in pending:
                    ack = self._parse_ack_payload(address, SUBCMD_STRIKE, payload)
                    if ack is not None:
                        results[index_by_address[address]] = ack
                        pending.remove(address)
                        continue
            self._trace("skip-burst", payload)
        return results

    def strike_ex(
        self,
        address: int,
        current_ma: int,
        strike_type: int = STRIKE_TYPE_NORMAL,
        type_param: int = 0,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> AddressedReply:
        """Strike with explicit articulation. For STRIKE_TYPE_DEAD, type_param
        is the total mute dwell in ms (0 = firmware default)."""
        return self._addressed_command(
            address,
            SUBCMD_STRIKE_EX,
            bytes([strike_type & 0xFF]) + struct.pack(">hH", current_ma, type_param),
            reply_mode,
        )

    def strike_home(self, address: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_STRIKE_HOME, reply_mode=reply_mode)

    def strike_cancel(self, address: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_STRIKE_CANCEL, reply_mode=reply_mode)

    def set_strike_param(
        self,
        address: int,
        param_id: int,
        value: int,
        reply_mode: str = REPLY_MODE_FULL,
    ) -> AddressedReply:
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
        expected_cmd = CMD_STATUS_BASE | address
        skipped: list[bytes] = []
        while True:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            try:
                payload = self._recv_frame(timeout_ms=remaining_ms)
            except RingTimeout as exc:
                raise RingTimeout(_format_timeout_context(
                    f"timeout waiting for strike status addr={address} "
                    f"expected_cmd=0x{expected_cmd:02X}({_cmd_name(expected_cmd)})",
                    skipped,
                    exc,
                )) from exc
            cmd = payload[0] if payload else 0
            if cmd == expected_cmd and len(payload) in (11, 22, 24, 30, 32, 35):
                self.last_strike_status_payload_len = len(payload)
                return self._parse_strike_status(payload)
            _remember_event(skipped, payload)
            self._trace("skip", payload)

    def query_strike_timing(self, address: int) -> StrikeTiming:
        """Compact 13-byte poll of the last completed strike's metrics. ~70%
        less wire traffic than ``query_strike``; useful for harvesting timing
        without paying for the full strike-status payload."""
        self._check_address(address)
        self._flush_rx()
        self._send_frame(self._build_addressed_payload(address, SUBCMD_QUERY_STRIKE_TIMING))
        deadline = time.monotonic() + self.timeout_ms / 1000.0
        expected_cmd = CMD_STATUS_BASE | address
        skipped: list[bytes] = []
        while True:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            try:
                payload = self._recv_frame(timeout_ms=remaining_ms)
            except RingTimeout as exc:
                raise RingTimeout(_format_timeout_context(
                    f"timeout waiting for strike timing addr={address} "
                    f"expected_cmd=0x{expected_cmd:02X}({_cmd_name(expected_cmd)}) "
                    f"expected_len=13",
                    skipped,
                    exc,
                )) from exc
            cmd = payload[0] if payload else 0
            if cmd == expected_cmd and len(payload) == 13:
                return _parse_strike_timing_payload(address, payload[1:13])
            if cmd == expected_cmd:
                self._print_unexpected_reply(
                    f"expected strike timing reply addr={address} "
                    f"expected_cmd=0x{expected_cmd:02X}({_cmd_name(expected_cmd)}) "
                    f"expected_len=13",
                    payload,
                )
            _remember_event(skipped, payload)
            self._trace("skip", payload)

    def query_timing(self, address: int) -> TimingStatus:
        self._check_address(address)
        self._flush_rx()
        self._send_frame(self._build_addressed_payload(address, SUBCMD_QUERY_TIMING))
        deadline = time.monotonic() + self.timeout_ms / 1000.0
        expected_cmd = CMD_STATUS_BASE | address
        skipped: list[bytes] = []
        while True:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            try:
                payload = self._recv_frame(timeout_ms=remaining_ms)
            except RingTimeout as exc:
                raise RingTimeout(_format_timeout_context(
                    f"timeout waiting for timing status addr={address} "
                    f"expected_cmd=0x{expected_cmd:02X}({_cmd_name(expected_cmd)})",
                    skipped,
                    exc,
                )) from exc
            cmd = payload[0] if payload else 0
            if cmd == expected_cmd and len(payload) in (33, 49, 53, 57, 64):
                return self._parse_timing_status(payload)
            _remember_event(skipped, payload)
            self._trace("skip", payload)

    def _parse_strike_status(self, payload: bytes) -> StrikeStatus:
        cmd = payload[0]
        address = cmd & 0x0F
        if len(payload) == 35:
            # Newer firmware: same first 32 bytes as before, plus
            # flags-hi @ offset 32 and trigger_to_impact_ms @ 33..34.
            return StrikeStatus(
                address=address,
                state=payload[1],
                homed=payload[2],
                flags=payload[3] | (payload[32] << 8),
                sequence=struct.unpack(">H", payload[4:6])[0],
                last_current_ma=struct.unpack(">h", payload[6:8])[0],
                trigger_to_coast_ms=struct.unpack(">H", payload[8:10])[0],
                trigger_to_rebound_ms=struct.unpack(">H", payload[10:12])[0],
                trigger_to_retrigger_ready_ms=struct.unpack(">H", payload[12:14])[0],
                trigger_to_ready_ms=struct.unpack(">H", payload[14:16])[0],
                estimated_strike_velocity_dps=struct.unpack(">H", payload[16:18])[0],
                drum_position=struct.unpack(">i", payload[18:22])[0],
                home_position=struct.unpack(">i", payload[22:26])[0],
                home_offset=struct.unpack(">h", payload[26:28])[0],
                coast_distance=struct.unpack(">h", payload[28:30])[0],
                homing_duty=struct.unpack(">h", payload[30:32])[0],
                trigger_to_impact_ms=struct.unpack(">H", payload[33:35])[0],
            )

        if len(payload) == 32:
            return StrikeStatus(
                address=address,
                state=payload[1],
                homed=payload[2],
                flags=payload[3],
                sequence=struct.unpack(">H", payload[4:6])[0],
                last_current_ma=struct.unpack(">h", payload[6:8])[0],
                trigger_to_coast_ms=struct.unpack(">H", payload[8:10])[0],
                trigger_to_rebound_ms=struct.unpack(">H", payload[10:12])[0],
                trigger_to_retrigger_ready_ms=struct.unpack(">H", payload[12:14])[0],
                trigger_to_ready_ms=struct.unpack(">H", payload[14:16])[0],
                estimated_strike_velocity_dps=struct.unpack(">H", payload[16:18])[0],
                drum_position=struct.unpack(">i", payload[18:22])[0],
                home_position=struct.unpack(">i", payload[22:26])[0],
                home_offset=struct.unpack(">h", payload[26:28])[0],
                coast_distance=struct.unpack(">h", payload[28:30])[0],
                homing_duty=struct.unpack(">h", payload[30:32])[0],
            )

        if len(payload) == 30:
            return StrikeStatus(
                address=address,
                state=payload[1],
                homed=payload[2],
                flags=payload[3],
                sequence=struct.unpack(">H", payload[4:6])[0],
                last_current_ma=struct.unpack(">h", payload[6:8])[0],
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
                last_current_ma=struct.unpack(">h", payload[6:8])[0],
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
                last_current_ma=struct.unpack(">h", payload[6:8])[0],
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

    def _parse_timing_status(self, payload: bytes) -> TimingStatus:
        cmd = payload[0]
        address = cmd & 0x0F
        if len(payload) == 33:
            return TimingStatus(
                address=address,
                control_budget_us=struct.unpack(">H", payload[1:3])[0],
                control_last_us=struct.unpack(">H", payload[3:5])[0],
                control_max_us=struct.unpack(">H", payload[5:7])[0],
                control_overrun_count=struct.unpack(">I", payload[7:11])[0],
                velocity_drop_count=0,
                position_drop_count=0,
                strike_drop_count=0,
                protocol_drop_count=0,
                hall_last_us=struct.unpack(">H", payload[11:13])[0],
                hall_max_us=struct.unpack(">H", payload[13:15])[0],
                uart_last_us=struct.unpack(">H", payload[15:17])[0],
                uart_max_us=struct.unpack(">H", payload[17:19])[0],
                adc_last_us=struct.unpack(">H", payload[19:21])[0],
                adc_max_us=struct.unpack(">H", payload[21:23])[0],
                protocol_poll_last_us=struct.unpack(">H", payload[23:25])[0],
                protocol_poll_max_us=struct.unpack(">H", payload[25:27])[0],
                protocol_backlog_max=struct.unpack(">H", payload[27:29])[0],
                uptime_ms=struct.unpack(">I", payload[29:33])[0],
            )

        if len(payload) not in (49, 53, 57, 64):
            raise RingError(f"timing reply wrong size ({len(payload)} bytes): {payload.hex(' ')}")

        uart_rx_overflow_count = (
            struct.unpack(">I", payload[49:53])[0] if len(payload) >= 53 else 0
        )
        adc_overrun_count = (
            struct.unpack(">I", payload[53:57])[0] if len(payload) >= 57 else 0
        )
        proto_dbg = {}
        if len(payload) >= 64:
            raw_subcmd = payload[60]
            proto_dbg = {
                "proto_dbg_sequence": payload[57],
                "proto_dbg_cmd_type": payload[58],
                "proto_dbg_len": payload[59],
                "proto_dbg_target": payload[58] & 0x0F,
                "proto_dbg_raw_subcmd": raw_subcmd,
                "proto_dbg_subcmd": raw_subcmd & SUBCMD_MASK,
                "proto_dbg_reply_mode_initial": payload[61],
                "proto_dbg_reply_mode_final": payload[62],
                "proto_dbg_reply_branch": payload[63],
            }

        return TimingStatus(
            address=address,
            control_budget_us=struct.unpack(">H", payload[1:3])[0],
            control_last_us=struct.unpack(">H", payload[3:5])[0],
            control_max_us=struct.unpack(">H", payload[5:7])[0],
            control_overrun_count=struct.unpack(">I", payload[7:11])[0],
            velocity_drop_count=struct.unpack(">I", payload[11:15])[0],
            position_drop_count=struct.unpack(">I", payload[15:19])[0],
            strike_drop_count=struct.unpack(">I", payload[19:23])[0],
            protocol_drop_count=struct.unpack(">I", payload[23:27])[0],
            hall_last_us=struct.unpack(">H", payload[27:29])[0],
            hall_max_us=struct.unpack(">H", payload[29:31])[0],
            uart_last_us=struct.unpack(">H", payload[31:33])[0],
            uart_max_us=struct.unpack(">H", payload[33:35])[0],
            adc_last_us=struct.unpack(">H", payload[35:37])[0],
            adc_max_us=struct.unpack(">H", payload[37:39])[0],
            protocol_poll_last_us=struct.unpack(">H", payload[39:41])[0],
            protocol_poll_max_us=struct.unpack(">H", payload[41:43])[0],
            protocol_backlog_max=struct.unpack(">H", payload[43:45])[0],
            uptime_ms=struct.unpack(">I", payload[45:49])[0],
            uart_rx_overflow_count=uart_rx_overflow_count,
            adc_overrun_count=adc_overrun_count,
            **proto_dbg,
        )

    def broadcast_duty(self, duties: Iterable[int]) -> None:
        duty_list = list(duties)
        if not duty_list or len(duty_list) > MAX_DEVICES:
            raise RingError(f"broadcast needs 1..{MAX_DEVICES} duty slots")
        payload = bytearray([CMD_BROADCAST_DUTY])
        for duty in duty_list:
            payload.extend(struct.pack(">h", duty))
        self._flush_rx()
        self._send_frame(bytes(payload))

    def _parse_status(self, payload: bytes) -> MotorStatus:
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


def format_status(status: MotorStatus) -> str:
    target_label = {0: "duty", 3: "mA"}.get(status.mode, "rpm")
    return (
        f"addr={status.address} state={status.state_name} fault={status.fault_name} "
        f"mode={status.mode_name} current={status.current_ma}mA hall={status.hall} "
        f"angle={status.angle} ({status.angle_deg:.1f}\u00b0) pos={status.position} "
        f"vel={status.velocity}rpm target_{target_label}={status.target}"
    )


def format_ack(ack: CommandAck) -> str:
    return (
        f"addr={ack.address} subcmd={ack.subcmd_name} "
        f"result={ack.result_name} detail={ack.detail}"
    )


def format_timing_status(status: TimingStatus) -> str:
    text = (
        f"addr={status.address} control={status.control_last_us}/{status.control_budget_us}us "
        f"({status.control_last_pct:.1f}%) control_max={status.control_max_us}us "
        f"({status.control_max_pct:.1f}%) control_overruns={status.control_overrun_count} "
        f"vel_drops={status.velocity_drop_count} pos_drops={status.position_drop_count} "
        f"strike_drops={status.strike_drop_count} proto_drops={status.protocol_drop_count} "
        f"hall={status.hall_last_us}us hall_max={status.hall_max_us}us "
        f"uart={status.uart_last_us}us uart_max={status.uart_max_us}us "
        f"adc={status.adc_last_us}us adc_max={status.adc_max_us}us "
        f"proto_poll={status.protocol_poll_last_us}us proto_poll_max={status.protocol_poll_max_us}us "
        f"proto_backlog_max={status.protocol_backlog_max} uptime_ms={status.uptime_ms} "
        f"uart_rx_overflow={status.uart_rx_overflow_count} "
        f"adc_overrun={status.adc_overrun_count}"
    )
    if status.proto_dbg_sequence:
        branch_names = {
            0: "PENDING",
            1: "NONE",
            2: "ACK",
            3: "ACK_TIMED",
            4: "STATUS",
            5: "STRIKE_STATUS",
            6: "TIMING_STATUS",
            7: "STRIKE_TIMING",
        }
        text += (
            f" proto_dbg_seq={status.proto_dbg_sequence} "
            f"cmd=0x{status.proto_dbg_cmd_type:02X} target={status.proto_dbg_target} "
            f"len={status.proto_dbg_len} raw_subcmd=0x{status.proto_dbg_raw_subcmd:02X} "
            f"subcmd=0x{status.proto_dbg_subcmd:02X} "
            f"reply_initial=0x{status.proto_dbg_reply_mode_initial:02X} "
            f"reply_final=0x{status.proto_dbg_reply_mode_final:02X} "
            f"branch={branch_names.get(status.proto_dbg_reply_branch, status.proto_dbg_reply_branch)}"
        )
    return text


def auto_detect_port() -> Optional[str]:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    preferred: list[str] = []
    fallback: list[str] = []
    for port in ports:
        desc = (port.description or "").lower()
        manufacturer = (port.manufacturer or "").lower()
        if (
            port.vid in (0x2E8A, 0x1A86, 0x10C4, 0x0403)
            or "usb serial" in desc
            or "wch" in manufacturer
            or "silicon labs" in manufacturer
            or "ftdi" in manufacturer
            or "pico" in desc
            or "acm" in desc
        ):
            preferred.append(port.device)
        else:
            fallback.append(port.device)

    return preferred[0] if preferred else (fallback[0] if fallback else None)


def list_ports() -> int:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 1
    for port in ports:
        print(f"  {port.device}: {port.description}")
    return 0


__all__ = [
    "ACK_RESULT_INVALID_ARGUMENT",
    "ACK_RESULT_NAMES",
    "ACK_RESULT_OK",
    "ACK_RESULT_OK_RETRIGGERED",
    "ACK_RESULT_PERSIST_FAILED",
    "ACK_RESULT_REJECT_FAULT",
    "ACK_RESULT_REJECT_NOT_HOMED",
    "ACK_RESULT_REJECT_NOT_READY",
    "ACK_RESULT_REJECT_ZERO",
    "AddressedReply",
    "CMD_ACK_BASE",
    "CMD_ACK_END",
    "CMD_ADDR_BASE",
    "CMD_ADDR_END",
    "CMD_BROADCAST_DUTY",
    "CMD_ENTER_CT",
    "CMD_ENTER_SF",
    "CMD_SET_ADDRESS",
    "CMD_STATUS_BASE",
    "CMD_STATUS_END",
    "CommandAck",
    "DEFAULT_BAUD",
    "DEFAULT_SETTLE_MS",
    "DEFAULT_TIMEOUT_MS",
    "FAULT_CODES",
    "MAX_DEVICES",
    "MAX_PAYLOAD",
    "MOTOR_STATES",
    "MotorStatus",
    "PREAMBLE",
    "REPLY_MODE_ACK",
    "REPLY_MODE_ACK_TIMED",
    "REPLY_MODE_FULL",
    "REPLY_MODE_NONE",
    "RingCRCError",
    "RingClientV2",
    "RingError",
    "RingTimeout",
    "STRIKE_PARAM_COAST_DISTANCE",
    "STRIKE_PARAM_HOME_OFFSET",
    "STRIKE_PARAM_HOMING_DUTY",
    "STRIKE_PARAM_MUTE_BRAKE_MS",
    "STRIKE_PARAM_MUTE_ENGAGE_OFFSET",
    "STRIKE_PARAM_MUTE_PRESS_MA",
    "STRIKE_STATES",
    "STRIKE_TIMING_ACTIVE",
    "STRIKE_TIMING_COAST_VALID",
    "STRIKE_TIMING_DEAD",
    "STRIKE_TIMING_IMPACT_VALID",
    "STRIKE_TIMING_REBOUND_TIMEOUT",
    "STRIKE_TIMING_REBOUND_VALID",
    "STRIKE_TIMING_READY_VALID",
    "STRIKE_TIMING_RETRIGGER_READY_VALID",
    "STRIKE_TIMING_RETRIGGERED",
    "STRIKE_TIMING_VELOCITY_VALID",
    "SUBCMD_CLEAR_FAULT",
    "SUBCMD_CLEAR_SETTINGS",
    "SUBCMD_MASK",
    "SUBCMD_NAMES",
    "SUBCMD_QUERY_STATUS",
    "SUBCMD_QUERY_STRIKE",
    "SUBCMD_QUERY_STRIKE_TIMING",
    "SUBCMD_QUERY_TIMING",
    "SUBCMD_DETECT_CSN_POLARITY",
    "SUBCMD_ENTER_BOOTLOADER",
    "SUBCMD_SET_CSN_POLARITY",
    "SUBCMD_REPLY_ACK",
    "SUBCMD_REPLY_ACK_TIMED",
    "SUBCMD_REPLY_FULL",
    "SUBCMD_REPLY_NONE",
    "SUBCMD_SAVE_SETTINGS",
    "SUBCMD_SET_CUR_PID",
    "SUBCMD_SET_CURRENT",
    "SUBCMD_SET_DUTY",
    "SUBCMD_SET_FF",
    "SUBCMD_SET_MODE",
    "SUBCMD_SET_PID",
    "SUBCMD_SET_POSITION",
    "SUBCMD_SET_POS_PID",
    "SUBCMD_SET_STRIKE_PARAM",
    "SUBCMD_SET_TORQUE",
    "SUBCMD_SET_VELOCITY",
    "SUBCMD_STOP",
    "SUBCMD_STRIKE",
    "SUBCMD_STRIKE_CANCEL",
    "SUBCMD_STRIKE_EX",
    "SUBCMD_STRIKE_HOME",
    "SUBCMD_ZERO_POS",
    "STRIKE_TYPE_DEAD",
    "STRIKE_TYPE_NORMAL",
    "StrikeStatus",
    "StrikeTiming",
    "TimingStatus",
    "auto_detect_port",
    "crc16_ccitt",
    "format_status",
    "format_ack",
    "format_timing_status",
    "list_ports",
]
