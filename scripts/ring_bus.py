#!/usr/bin/env python3
"""
Reusable Ring Bus protocol client for M2003 motor boards.
"""

from __future__ import annotations

import dataclasses
import struct
import time
from typing import Iterable, Optional

import serial
import serial.tools.list_ports


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
SUBCMD_MASK          = 0x3F
SUBCMD_REPLY_FULL    = 0x00
SUBCMD_REPLY_ACK     = 0x40
SUBCMD_REPLY_NONE    = 0x80

REPLY_MODE_FULL = "full"
REPLY_MODE_ACK = "ack"
REPLY_MODE_NONE = "none"

ACK_RESULT_OK               = 0x00
ACK_RESULT_OK_RETRIGGERED   = 0x01
ACK_RESULT_REJECT_NOT_HOMED = 0x02
ACK_RESULT_REJECT_FAULT     = 0x03
ACK_RESULT_REJECT_ZERO      = 0x04
ACK_RESULT_REJECT_NOT_READY = 0x05
ACK_RESULT_INVALID_ARGUMENT = 0x06

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
STRIKE_TIMING_RETRIGGER_READY_VALID = 0x80

# ── Timing defaults ─────────────────────────────────────────────────────────

DEFAULT_BAUD       = 250000
DEFAULT_TIMEOUT_MS = 200
DEFAULT_SETTLE_MS  = 250


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
STRIKE_STATES = {0: "IDLE", 1: "HOMING", 2: "DRIVING", 3: "COASTING", 4: "RETURNING", 5: "CATCHING"}
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
}
ACK_RESULT_NAMES = {
    ACK_RESULT_OK: "OK",
    ACK_RESULT_OK_RETRIGGERED: "OK_RETRIGGERED",
    ACK_RESULT_REJECT_NOT_HOMED: "REJECT_NOT_HOMED",
    ACK_RESULT_REJECT_FAULT: "REJECT_FAULT",
    ACK_RESULT_REJECT_ZERO: "REJECT_ZERO",
    ACK_RESULT_REJECT_NOT_READY: "REJECT_NOT_READY",
    ACK_RESULT_INVALID_ARGUMENT: "INVALID_ARGUMENT",
}


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
    state: int
    homed: int
    flags: int = 0
    sequence: int = 0
    last_duty: int = 0
    trigger_to_coast_ms: int = 0
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


AddressedReply = MotorStatus | CommandAck | None


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

        def next_byte() -> int:
            while True:
                if time.monotonic() >= deadline:
                    raise RingTimeout("timeout waiting for frame")
                byte = self.ser.read(1)
                if byte:
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
                        phase = scan_0
                        buf.clear()
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

                    self._trace("rx-crc-fail", PREAMBLE + bytes(buf))
                    phase = scan_0
                    buf.clear()
                    expected = 0

    def _trace(self, label: str, data: bytes) -> None:
        if self.trace:
            print(f"  [{label}] {data.hex(' ').upper()}")

    def enumerate(self) -> int:
        self._flush_rx()
        self._send_frame(bytes([CMD_ENTER_SF]))
        time.sleep(0.010)
        self._flush_rx()

        self._send_frame(bytes([CMD_SET_ADDRESS, 0x00]))
        payload = self._recv_frame()

        if len(payload) < 2 or payload[0] != CMD_SET_ADDRESS:
            raise RingError(f"unexpected enumerate reply: {payload.hex(' ')}")

        self.device_count = payload[1]

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
        raise RingError(
            f"reply_mode must be one of: {REPLY_MODE_FULL}, {REPLY_MODE_ACK}, {REPLY_MODE_NONE}"
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
        while True:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            payload = self._recv_frame(timeout_ms=remaining_ms)
            cmd = payload[0] if payload else 0
            if (
                CMD_STATUS_BASE <= cmd < CMD_STATUS_END
                and len(payload) in (13, 17)
                and (expected_cmd is None or cmd == expected_cmd)
            ):
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
        if encoded_reply_mode == SUBCMD_REPLY_ACK:
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

    def zero_position(self, address: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_ZERO_POS, reply_mode=reply_mode)

    def save_settings(self, address: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_SAVE_SETTINGS, reply_mode=reply_mode)

    def clear_settings(self, address: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_CLEAR_SETTINGS, reply_mode=reply_mode)

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

    def strike(self, address: int, duty: int, reply_mode: str = REPLY_MODE_FULL) -> AddressedReply:
        return self._addressed_command(address, SUBCMD_STRIKE, struct.pack(">h", duty), reply_mode)

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
        while True:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            payload = self._recv_frame(timeout_ms=remaining_ms)
            cmd = payload[0] if payload else 0
            if cmd == expected_cmd and len(payload) in (11, 22, 24, 30, 32):
                return self._parse_strike_status(payload)
            self._trace("skip", bytes([cmd]))

    def _parse_strike_status(self, payload: bytes) -> StrikeStatus:
        cmd = payload[0]
        address = cmd & 0x0F
        if len(payload) == 32:
            return StrikeStatus(
                address=address,
                state=payload[1],
                homed=payload[2],
                flags=payload[3],
                sequence=struct.unpack(">H", payload[4:6])[0],
                last_duty=struct.unpack(">h", payload[6:8])[0],
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
    "REPLY_MODE_FULL",
    "REPLY_MODE_NONE",
    "RingCRCError",
    "RingClientV2",
    "RingError",
    "RingTimeout",
    "STRIKE_PARAM_COAST_DISTANCE",
    "STRIKE_PARAM_HOME_OFFSET",
    "STRIKE_PARAM_HOMING_DUTY",
    "STRIKE_STATES",
    "STRIKE_TIMING_ACTIVE",
    "STRIKE_TIMING_COAST_VALID",
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
    "SUBCMD_REPLY_ACK",
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
    "SUBCMD_STRIKE_HOME",
    "SUBCMD_ZERO_POS",
    "StrikeStatus",
    "auto_detect_port",
    "crc16_ccitt",
    "format_status",
    "list_ports",
]
