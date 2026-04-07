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
| 0x20+addr | ADDRESSED_CMD | `[subcmd] [data...]` | 2..8 | master -> ring |
| 0x40+addr | STATUS_REPLY | query-dependent status payload | 11 or 17 | device -> master |

### Addressed Sub-Commands

Used with type `0x20 + device_addr`:

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
| 0x0C | STRIKE | `[duty_hi] [duty_lo]` | 2 |
| 0x0D | STRIKE_HOME | -- | 0 |
| 0x0E | STRIKE_CANCEL | -- | 0 |
| 0x0F | SET_STRIKE_PARAM | `[param_id] [value_hi] [value_lo]` | 3 |
| 0x10 | QUERY_STATUS | -- | 0 |
| 0x11 | QUERY_STRIKE | -- | 0 |
| 0x12 | SAVE_SETTINGS | -- | 0 |
| 0x13 | CLEAR_SETTINGS | -- | 0 |

### Status Replies

`QUERY_STATUS` replies with type `0x40 + addr` and 17 payload bytes:

```
[type] [state] [fault] [mode] [current_hi] [current_lo]
[hall] [angle_hi] [angle_lo] [velocity_hi] [velocity_lo]
[target_hi] [target_lo] [position_b3] [position_b2] [position_b1] [position_b0]
```

`QUERY_STRIKE` replies with the same type `0x40 + addr` and 11 payload bytes:

```
[type] [strike_state] [homed]
[drum_pos_b3] [drum_pos_b2] [drum_pos_b1] [drum_pos_b0]
[home_pos_b3] [home_pos_b2] [home_pos_b1] [home_pos_b0]
```

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
the next boot; it does not change the current live runtime parameters.
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
