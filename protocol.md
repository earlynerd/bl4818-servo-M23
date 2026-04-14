# Ring Bus Protocol v2

Binary framing protocol for the BL4818 motor driver ring bus.
Up to 16 devices, single master, 250 kbaud UART.

## Packet Structure

```
 0       1       2       3        3+LEN   4+LEN
+-------+-------+-------+--------+-------+-------+
| 0xA5  | 0x5A  |  LEN  | payload|CRC_HI |CRC_LO |
+-------+-------+-------+--------+-------+-------+
 preamble        ^                 ^
                 |                 |
                 +-- LEN bytes ----+
```

- **Preamble** (2 bytes): `0xA5 0x5A` -- sync marker, not included in CRC
- **LEN** (1 byte): number of payload bytes (max 34)
- **Payload** (LEN bytes): type byte followed by command-specific data
- **CRC-16/CCITT** (2 bytes): over LEN + payload, polynomial 0x1021, init 0xFFFF, big-endian

Minimum packet: 6 bytes (preamble + LEN + 1-byte payload + CRC16).
Maximum packet: 39 bytes (preamble + LEN + 34-byte payload + CRC16).

## Forwarding Modes

Devices operate in one of two forwarding modes:

### Cut-Through (default)

Every byte received on RX is immediately echoed to TX.  A local copy is
buffered in parallel.  After the complete frame is received, the device
validates CRC and acts on the contents.  Corrupted frames propagate
through the ring but are independently rejected by every node.

### Store-and-Forward (enumeration only)

The entire frame is buffered, CRC-validated, processed (possibly
modified), CRC recomputed, and then forwarded.  Used during enumeration
where each device must modify the frame before passing it on.

## Receiver State Machine

1. Scan byte stream for `0xA5` followed by `0x5A`.
2. Read LEN.  If LEN > MAX_PAYLOAD or LEN == 0, reject; advance one byte
   past the `0xA5` and re-scan.
3. Buffer LEN payload bytes + 2 CRC bytes.
4. Verify CRC-16 over [LEN, payload].
5. **CRC pass:** process payload, advance past frame.
6. **CRC fail:** rewind to one byte after the `0xA5` that started this
   attempt and re-scan (single-byte slip for re-sync).

## Command Table

The first payload byte is the command type.

| Type | Name | Payload (after type byte) | Payload LEN | Direction |
|------|------|---------------------------|-------------|-----------|
| 0x01 | ENTER_STORE_FORWARD | -- | 1 | master -> ring |
| 0x02 | ENTER_CUT_THROUGH | -- | 1 | master -> ring |
| 0x03 | SET_ADDRESS | `[counter]` | 2 | master -> ring (S&F) |
| 0x10 | BROADCAST_DUTY | `[duty_hi duty_lo] x N` | 1 + 2*N | master -> ring |
| 0x20+addr | ADDRESSED_CMD | `[subcmd_flags] [data...]` | 2..8 | master -> ring |
| 0x40+addr | STATUS_REPLY | query-dependent status payload | 11, 17, 32, or 49 | device -> master |
| 0x50+addr | ACK_REPLY | `[subcmd] [result] [detail_hi] [detail_lo]` | 5 | device -> master |

### Addressed Sub-Commands

Used with type `0x20 + device_addr`. The low 6 bits of `subcmd_flags` select the
sub-command ID; the top 2 bits select reply policy.

| Sub-Cmd | Name | Data | Data LEN |
|---------|------|------|----------|
| 0x01 | SET_DUTY | `[duty_hi] [duty_lo]` | 2 |
| 0x02 | SET_TORQUE | `[ma_hi] [ma_lo]` | 2 |
| 0x03 | STOP | -- | 0 |
| 0x04 | CLEAR_FAULT | -- | 0 |
| 0x05 | SET_MODE | `[mode]` | 1 |
| 0x06 | SET_VELOCITY | `[rpm_hi] [rpm_lo]` | 2 |
| 0x07 | SET_PID | `[kp_hi] [kp_lo] [ki_hi] [ki_lo] [kd_hi] [kd_lo]` | 6 |
| 0x08 | SET_FF | `[gain_hi] [gain_lo]` | 2 |
| 0x09 | SET_POSITION | `[pos_b3] [pos_b2] [pos_b1] [pos_b0]` | 4 |
| 0x0A | SET_POS_PID | `[kp_hi] [kp_lo] [ki_hi] [ki_lo] [kd_hi] [kd_lo]` | 6 |
| 0x0B | ZERO_POSITION | -- | 0 |
| 0x0C | STRIKE | `[current_hi] [current_lo]` | 2 |
| 0x0D | STRIKE_HOME | -- | 0 |
| 0x0E | STRIKE_CANCEL | -- | 0 |
| 0x0F | SET_STRIKE_PARAM | `[param_id] [value_hi] [value_lo]` | 3 |
| 0x10 | QUERY_STATUS | -- | 0 |
| 0x11 | QUERY_STRIKE | -- | 0 |
| 0x12 | SAVE_SETTINGS | -- | 0 |
| 0x13 | CLEAR_SETTINGS | -- | 0 |
| 0x16 | QUERY_TIMING | -- | 0 |

### Addressed Reply Modes

`subcmd_flags` layout:

```text
bits 7:6 = reply mode
bits 5:0 = sub-command ID
```

| Bits 7:6 | Mode | Behavior |
|----------|------|----------|
| `00` | FULL | Current/default behavior. Device emits the normal full reply. |
| `01` | ACK | Device emits a compact `ACK_REPLY`. |
| `10` | NONE | Device suppresses the device-generated reply. |
| `11` | Reserved | Must not be sent. Current firmware treats it as `FULL`. |

Notes:

- `QUERY_STATUS` and `QUERY_STRIKE` always return their full reply payloads.
- `QUERY_TIMING` also always returns its full reply payload.
- `NONE` only suppresses the device-generated reply. In cut-through mode the
  command frame itself still propagates around the ring and returns to the
  master RX path.

### ACK Replies

Addressed commands sent with reply mode `ACK` reply with type `0x50 + addr` and
5 payload bytes:

```text
[type] [subcmd] [result] [detail_hi] [detail_lo]
```

`subcmd` is the low-6-bit sub-command ID without the reply-mode bits.

`result` codes currently used by firmware:

| Result | Meaning |
|--------|---------|
| `0x00` | OK |
| `0x01` | OK_RETRIGGERED |
| `0x02` | REJECT_NOT_HOMED |
| `0x03` | REJECT_FAULT |
| `0x04` | REJECT_ZERO |
| `0x05` | REJECT_NOT_READY |
| `0x06` | INVALID_ARGUMENT |
| `0x07` | PERSIST_FAILED |

`detail` is command-specific. For `STRIKE`, `STRIKE_HOME`, `STRIKE_CANCEL`, and
`SET_STRIKE_PARAM`, it reports the current 16-bit strike sequence. Other
commands currently return `detail = 0`. Flash-backed maintenance commands
(`ZERO_POSITION`, `SAVE_SETTINGS`, `CLEAR_SETTINGS`) return `PERSIST_FAILED`
when the underlying erase/write/verify operation fails.

### Status Replies

`QUERY_STATUS` replies with type `0x40 + addr` and 17 payload bytes:

```
[type] [state] [fault] [mode] [current_hi] [current_lo]
[hall] [angle_hi] [angle_lo] [velocity_hi] [velocity_lo]
[target_hi] [target_lo] [position_b3] [position_b2] [position_b1] [position_b0]
```

`current` is the filtered control-window average current in mA. Firmware uses a
separate sustained peak-current detector plus a higher instantaneous ceiling
for `FAULT_OVERCURRENT`; the raw peak sample is not reported in this reply.

`QUERY_STRIKE` replies with the same type `0x40 + addr` and 32 payload bytes:

```
[type] [strike_state] [homed] [timing_flags]
[seq_hi] [seq_lo] [current_hi] [current_lo]
[t_coast_hi] [t_coast_lo] [t_rebound_hi] [t_rebound_lo]
[t_retrigger_ready_hi] [t_retrigger_ready_lo]
[t_ready_hi] [t_ready_lo] [vel_dps_hi] [vel_dps_lo]
[drum_pos_b3] [drum_pos_b2] [drum_pos_b1] [drum_pos_b0]
[home_pos_b3] [home_pos_b2] [home_pos_b1] [home_pos_b0]
[home_offset_hi] [home_offset_lo]
[coast_distance_hi] [coast_distance_lo]
[homing_duty_hi] [homing_duty_lo]
```

`timing_flags` bits:

- `0x01` trigger-to-coast time valid
- `0x02` trigger-to-rebound time valid
- `0x04` trigger-to-ready time valid
- `0x08` strike currently active
- `0x10` most recent strike was accepted as a retrigger while already active
- `0x20` rebound timing was produced by coast timeout rather than detected reversal
- `0x40` strike velocity estimate valid
- `0x80` trigger-to-retrigger-ready time valid

Timing fields are reported in milliseconds for the most recently accepted
strike. `t_rebound` is an approximate impact proxy derived from rebound
detection or coast timeout, not a direct drum-contact sensor.

The `STRIKE` payload is a signed 16-bit current command in mA. Firmware
orients that current toward the learned drum direction for the outbound hit.
The `current` field in `QUERY_STRIKE` reports the last accepted strike current
command in mA.

`t_retrigger_ready` is earlier than `t_ready`. It marks the first moment when
the mallet is both near the configured home position and moving slowly enough
that a new strike can be accepted with predictable timing. Firmware rejects a
retrigger attempt before this point with `REJECT_NOT_READY`.

`vel_dps` is an estimated strike approach speed in degrees per second,
computed as the peak toward-drum angular velocity observed during the
DRIVING and COASTING phases. It is a useful proxy for strike intensity,
not a direct velocity measurement at the exact instant of contact.

The final three signed 16-bit fields report the current live strike
configuration: `home_offset`, `coast_distance`, and `homing_duty`.
When `home_offset` is changed while the actuator is homed and not in the
strike approach, the parked home target is updated immediately.

If a `STRIKE` command arrives while the actuator is already in DRIVING,
COASTING, or CATCHING, firmware only accepts it once
`t_retrigger_ready` has become valid. Before that point the compact
`ACK_REPLY` returns `REJECT_NOT_READY`. Once accepted, the current recovery is
aborted and a new strike attempt starts immediately, and the `ACK_REPLY`
reports `OK_RETRIGGERED`.

`QUERY_TIMING` replies with the same type `0x40 + addr` and 49 payload bytes:

```
[type]
[control_budget_hi] [control_budget_lo]
[control_last_hi] [control_last_lo]
[control_max_hi] [control_max_lo]
[control_overrun_b3] [control_overrun_b2] [control_overrun_b1] [control_overrun_b0]
[vel_drop_b3] [vel_drop_b2] [vel_drop_b1] [vel_drop_b0]
[pos_drop_b3] [pos_drop_b2] [pos_drop_b1] [pos_drop_b0]
[strike_drop_b3] [strike_drop_b2] [strike_drop_b1] [strike_drop_b0]
[proto_drop_b3] [proto_drop_b2] [proto_drop_b1] [proto_drop_b0]
[hall_last_hi] [hall_last_lo] [hall_max_hi] [hall_max_lo]
[uart_last_hi] [uart_last_lo] [uart_max_hi] [uart_max_lo]
[adc_last_hi] [adc_last_lo] [adc_max_hi] [adc_max_lo]
[proto_poll_last_hi] [proto_poll_last_lo]
[proto_poll_max_hi] [proto_poll_max_lo]
[proto_backlog_hi] [proto_backlog_lo]
[uptime_b3] [uptime_b2] [uptime_b1] [uptime_b0]
```

All timing fields are in microseconds except `uptime`, which is milliseconds
since boot. `control_*` measures the full `SysTick` control service time against
the configured control-period budget. `vel_drop`, `pos_drop`, `strike_drop`,
and `proto_drop` count scheduler events that were skipped because the lower-rate
task had fallen behind. `hall_*` measures GPIO hall IRQ service time. `uart_*`
and `adc_*` measure the corresponding ISR wall times. `proto_poll_*` measures
foreground `protocol_poll()` wall time. The `proto_backlog` field is the
maximum number of `protocol_tick()` periods that were pending before the main
loop caught up, which is a direct signal that foreground work is starting to
miss its schedule.

## Persistent Settings

The firmware reserves the last 512-byte APROM page for a CRC-protected
settings block that is loaded at boot.

Persisted items:

- encoder zero reference, stored as the absolute 14-bit encoder angle
- strike tuning (`home_offset`, `coast_distance`, `homing_duty`)
- strike learned calibration (`drum_position`, `home_position`) when homed
- motor torque limit, velocity PID, velocity feedforward, and position PID

`ZERO_POSITION` updates the logical zero point immediately and also saves the
new absolute zero reference to flash. The other tunables are only committed
when `SAVE_SETTINGS` is issued. `CLEAR_SETTINGS` erases the persisted block for
the next boot; it does not change the current live runtime parameters. Hosts
that need the flash result should request an `ACK_REPLY` for these commands and
check for `PERSIST_FAILED`.
These flash operations should be treated as at-rest maintenance commands, not
high-rate control traffic.

## Enumeration Sequence

```
Master                          Device 0          Device 1        ...
  |                                |                 |
  |-- ENTER_STORE_FORWARD -------->|----store+fwd--->|---->
  |                                |                 |
  |-- SET_ADDRESS [0] ------------>|                 |
  |                                | claim addr=0    |
  |                                |-- SET_ADDRESS [1]-->|
  |                                                  | claim addr=1
  |                                                  |-- SET_ADDRESS [2] -->
  |                                                  |
  |<---------- SET_ADDRESS [N] (master sees final count)
  |                                |                 |
  |-- ENTER_CUT_THROUGH --------->|----cut-thru---->|---->
  |                                |                 |
  v  normal operation              v                 v
```

## Broadcast Duty Frame Example

4-device ring, master sets duties to 100, 200, 300, 400:

```
A5 5A  09  10  00 64  00 C8  01 2C  01 90  [CRC_HI] [CRC_LO]
^^^^^ ^^^  ^^  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^  ^^^^^^^^^^^^^^^^^
pream LEN type dev0   dev1   dev2   dev3     CRC-16/CCITT
```

Each device forwards every byte immediately as it arrives (cut-through).
After the full frame is received, device N validates CRC and extracts
`payload[1 + 2*N]` and `payload[2 + 2*N]` as its signed 16-bit duty.

## CRC-16/CCITT

- Polynomial: 0x1021
- Initial value: 0xFFFF
- Computed over: LEN byte + all payload bytes (not preamble, not CRC itself)
- Transmitted big-endian (high byte first)
