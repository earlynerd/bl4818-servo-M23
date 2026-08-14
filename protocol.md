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
- **LEN** (1 byte): number of payload bytes (max 64)
- **Payload** (LEN bytes): type byte followed by command-specific data
- **CRC-16/CCITT** (2 bytes): over LEN + payload, polynomial 0x1021, init 0xFFFF, big-endian

Minimum packet: 6 bytes (preamble + LEN + 1-byte payload + CRC16).
Maximum packet: 69 bytes (preamble + LEN + 64-byte payload + CRC16).

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
| 0x40+addr | STATUS_REPLY | query-dependent status payload | 13, 17, 32, 33, 35, 49, 53, 57, or 64 | device -> master |
| 0x50+addr | ACK_REPLY | `[subcmd] [result] [detail_hi] [detail_lo]` (bare) or +12 timing bytes (timed) | 5 or 17 | device -> master |
| 0x61 | STAY_IN_BOOTLOADER | -- | 1 | master -> ring during LDROM boot window |
| 0x70+addr | BOOT_CMD | `[boot_subcmd] [data...]` | 2..36 | master -> LDROM loader |
| 0x80+addr | BOOT_REPLY | `[boot_subcmd] [result] [detail...]` | 3..36 | LDROM loader -> master |
| 0x90+tail_addr | BOOT_BULK_CMD | `[boot_subcmd] [data...]` | 2..36 | master -> all assigned protocol-3 LDROM loaders |

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
| 0x19 | QUERY_STRIKE_TIMING | -- | 0 |
| 0x1A | STRIKE_EX | `[type] [current_hi] [current_lo] [param_hi] [param_lo]` | 5 |
| 0x1B | ENTER_BOOTLOADER | -- | 0 |
| 0x1C | QUERY_CONFIG | -- | 0 |

`ENTER_BOOTLOADER` is a maintenance transition for LDROM-provisioned Gen1
devices. The application performs the same coordinated stop as `STOP`, sends
the requested reply, drains UART TX, records its current ring address in a
fixed soft-reset mailbox, selects LDROM for the next system reset, and resets.
Use compact ACK mode so the host has an explicit handoff boundary. A device
without the permanent loader/configuration provisioning must not be sent this
command.

`STOP` immediately disables motor drive, aborts any active strike/homing
sequence, and clears the learned-homed runtime flag. A successful `STRIKE_HOME`
is therefore required before the next strike. With ACK reply mode,
`STRIKE_HOME` returns `REJECT_NOT_READY` while another sequence is active,
`REJECT_FAULT` on motor fault, and `INVALID_ARGUMENT` when homing is disabled
(`homing_duty == 0`). `OK` means homing started, not that it has finished.

Homing is bounded by both cumulative encoder travel and elapsed time: one
encoder revolution (16,384 counts) or 5 seconds, whichever comes first. An
active strike sequence is bounded by the same one-revolution cumulative-travel
ceiling but has no separate time limit; low-current strikes may legitimately
take longer, while high-current runaway reaches the encoder ceiling quickly.
Crossing either bound immediately disables PWM, latches `HOMING_LIMIT` or
`STRIKE_LIMIT`, clears `homed`, and requires `CLEAR_FAULT` followed by a fresh
`STRIKE_HOME`.

On a successful re-home, firmware compares the new drum position with the
previous known calibration modulo one absolute-encoder revolution. A change of
1,024 counts (22.5 degrees) or more sets the non-faulting
`STRIKE_WARNING_HOME_SHIFT` status bit. A subsequent re-home within that
threshold clears the warning. `SAVE_SETTINGS` can persist the calibration for
comparison after reboot; when invoked during an idle position hold, firmware
briefly disables that hold around the blocking flash write and resumes it
afterward.

### STRIKE_EX Articulation Types

`STRIKE_EX` is `STRIKE` with an explicit articulation type and a type-specific
parameter. `type = 0x00` (NORMAL) behaves exactly like `STRIKE`; `param` is
ignored. Unknown types are rejected with `INVALID_ARGUMENT`.

| Type | Name | `param` meaning |
|------|------|-----------------|
| 0x00 | NORMAL | ignored |
| 0x01 | DEAD | total mute dwell in ms (0 = firmware default, currently 150; clamped to 1000) |

## LDROM update protocol

The permanent Gen1 LDROM loader uses the same preamble, length, CRC-16, UART
rate, forwarding modes, and `SET_ADDRESS` enumeration as the application. It
continues to forward each received byte immediately in cut-through mode.
Loader command and reply type ranges are separate from application status and
ACK ranges, so a host can distinguish the running image from its first reply.
On the current Gen1 bench retrofit, LDROM also holds the shared PB1 CSn/status
LED high and solid on as a local indication that the loader is resident.

An application-requested loader inherits the application's address through a
soft-reset mailbox. A loader that stayed resident after cold reset starts
unassigned and participates in normal store-and-forward enumeration alongside
any running application nodes.

`STAY_IN_BOOTLOADER` (`0x61`) is a framed, no-reply broadcast used only during
the loader's 1 second initial boot-intercept window. Repeatedly sending it while
power is applied holds otherwise-valid nodes in LDROM. An invalid or
uncommitted application remains in LDROM without this command.

### Loader commands

Loader multibyte command fields are big-endian, like existing protocol fields.
Application image bytes inside `WRITE_CHUNK` retain their binary byte order.

| Sub-Cmd | Name | Command data | Successful reply detail |
| --- | --- | --- | --- |
| `0x01` | GET_INFO | -- | protocol/version, state, geometry, committed image fields, UID, loader-entry diagnostics |
| `0x10` | BEGIN_IMAGE | `[size_b3..b0] [crc_b3..b0] [version_b3..b0] [flags_b3..b0]` | -- |
| `0x11` | ERASE_PAGE | `[page_index]` | `[page_index]` |
| `0x12` | WRITE_CHUNK | `[offset_hi] [offset_lo] [4..32 data bytes]` | `[offset_hi] [offset_lo]` |
| `0x13` | VERIFY_IMAGE | -- | `[calculated_crc_b3..b0]` |
| `0x14` | COMMIT_IMAGE | -- | -- |
| `0x15` | RUN_APROM | -- | --; loader remaps APROM vectors, selects APROM, and system-resets after reply drains |

Protocol 3 adds `BOOT_BULK_CMD` for the idempotent data phase only:
`BEGIN_IMAGE`, `ERASE_PAGE`, and `WRITE_CHUNK`. Every assigned loader executes
the command, but only the loader whose address is encoded in the type byte
returns the normal `BOOT_REPLY`. The host uses the farthest physical address
(`count - 1`) as this tail pacekeeper because it receives each cut-through
frame last. Commands outside the three data-phase subcommands produce no bulk
reply and are not executed.

The tail reply is a flow-control boundary, not proof that every flash operation
succeeded. After the shared transfer, the host individually runs
`VERIFY_IMAGE` on every loader, repeats the addressed data phase for any failed
device, and only then individually issues `COMMIT_IMAGE` and `RUN_APROM`.
Allowing only one reply prevents multiple loaders from injecting overlapping
reply frames into the one-direction ring.

LDROM never calls the application reset vector directly. A system reset gives
APROM normal reset-state CPU and peripheral state. Each handoff first issues and
verifies the M2003 `VECMAP` command for the destination, selects and verifies the
matching runtime boot source, then requests the reset. APROM then re-arms LDROM
as the destination of the next reset before normal initialization without
changing the vector map while APROM is running.

Protocol 2 and 3 `GET_INFO` reply detail is 45 bytes:

```text
[protocol_version]
[loader_version_hi] [loader_version_lo]
[state_flags] [address]
[page_size_hi] [page_size_lo]
[app_limit_hi] [app_limit_lo]
[committed_size_b3] [committed_size_b2] [committed_size_b1] [committed_size_b0]
[committed_crc_b3] [committed_crc_b2] [committed_crc_b1] [committed_crc_b0]
[committed_version_b3] [committed_version_b2] [committed_version_b1] [committed_version_b0]
[uid_word_0_b3..b0] [uid_word_1_b3..b0] [uid_word_2_b3..b0]
[reset_status_b3..b0]
[entry_isp_control_b3..b0]
[entry_isp_status_b3..b0]
```

State flags are `0x01 = committed image valid` and `0x02 = update active`;
other bits are reserved. Loader and protocol version are both 3 in the current
implementation. The final three words are raw `SYS.RSTSTS`, `FMC.ISPCTL`, and
`FMC.ISPSTS` snapshots taken before the loader changes its FMC state. They make
system-reset, watchdog-reset, selected boot source, and entry vector mapping
observable over UART when SWD cannot be connected. Protocol 1 loaders return
the original 33-byte prefix; the current host accepts both reply layouts and
uses addressed updates for loaders older than protocol 3. Page size is 512 and
application limit is `0x7A00`.

`BEGIN_IMAGE` accepts only a word-aligned size from 8 through `0x7A00` and
flags 0. It erases the manifest page before marking the RAM update state
active. `ERASE_PAGE` is then limited to pages intersecting that exact proposed
image. `WRITE_CHUNK` requires a word-aligned offset and length and rejects any
write beyond the proposed image; the loader also rejects all application
writes at or above `0x7A00`. Repeating a chunk with identical contents is
idempotent. A different value at an already-programmed word fails, forcing the
host to erase and restart that page.

`VERIFY_IMAGE` calculates CRC-32/ISO-HDLC across exactly the proposed byte
length. `COMMIT_IMAGE` recalculates it, constructs the 32-byte manifest, writes
the non-magic words first, and writes the manifest magic last. Reset or power
loss before that final verified word leaves the application unbootable and the
loader resident. See `docs/firmware-update.md` for the flash map and recovery
sequence.

### Loader results

Every `BOOT_REPLY` starts `[0x80+addr] [subcmd] [result]`.

| Result | Meaning |
| --- | --- |
| `0x00` | OK |
| `0x01` | BAD_COMMAND: wrong payload length or unknown subcommand |
| `0x02` | BAD_STATE: command requires an active update or committed image |
| `0x03` | BAD_RANGE: address, size, alignment, page, or flags rejected |
| `0x04` | FLASH_FAILED: erase/program/read/map verification failed |
| `0x05` | CRC_MISMATCH: programmed APROM differs from `BEGIN_IMAGE` |

A dead strike drives and coasts identically to a normal strike at the same
current, so the attack is unchanged. The device arms a second position
trigger at the learned drum surface (offset by `MUTE_ENGAGE_OFFSET`): on the
raw encoder crossing of that point — checked at the encoder sampling
cadence, with no velocity-filter or strike-tick lag — it enters a MUTING
state instead of catching the rebound: first a velocity-0 brake phase (the
velocity servo fights the rebound with current proportional to rebound
speed, so braking self-scales to strike energy and engages while the ball
tip is still compressing), then a small steady toward-drum current presses
the mallet into the surface for the remainder of the dwell, muting the
note. The mallet then returns home through the normal CATCHING path. A
mallet that stalls short of the trigger point falls back to the
filtered-velocity zero-cross for mute entry.

Dead strikes report `strike_state = 6` (MUTING) while in contact and set
timing flag `0x0200` in the 16-bit `timing_flags`. The rebound timing field
never becomes valid for a dead strike (the rebound is deliberately
suppressed); `t_impact`, `t_coast`, and the velocity estimate are reported
as usual. If impact is never detected before the coast timeout, the device
still enters MUTING (gentle press into the surface) and sets the
rebound-timeout flag `0x0020`.

A `STRIKE` or `STRIKE_EX` received during MUTING is rejected with
`REJECT_NOT_READY` (the mallet is at the surface, not at home);
`STRIKE_CANCEL` aborts the mute early and returns the mallet home.

### Strike Parameter IDs (SET_STRIKE_PARAM)

| ID | Name | Units | Notes |
|----|------|-------|-------|
| 0x01 | HOME_OFFSET | encoder counts | parked height above drum |
| 0x02 | COAST_DISTANCE | encoder counts | power-cut distance from drum |
| 0x03 | HOMING_DUTY | signed duty | sign selects toward-drum direction |
| 0x04 | MUTE_BRAKE_MS | ms | dead strike velocity-0 brake phase (default 30); value must be >= 0 |
| 0x05 | MUTE_PRESS_MA | mA | dead strike contact-press current (default 250); 0 = hold velocity-0 for the whole dwell; value must be >= 0 |
| 0x06 | MUTE_ENGAGE_OFFSET | encoder counts (signed) | dead strike brake trip point, this far before the learned drum surface (default 0 = at the surface; negative = past it, requiring compression travel) |

All six strike parameters are included by `SAVE_SETTINGS` and restored at
boot. Version-3 settings records remain readable; their three dead-strike
fields use compiled defaults until the next save migrates the record to v4.

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
| `11` | ACK_TIMED | Device emits an `ACK_REPLY` with the last completed strike's compact metrics piggybacked on the end. 17 payload bytes total. |

Notes:

- `QUERY_STATUS`, `QUERY_STRIKE`, `QUERY_STRIKE_TIMING`, and `QUERY_CONFIG`
  always return their full reply payloads regardless of requested reply mode.
- `QUERY_TIMING` also always returns its full reply payload.
- `NONE` only suppresses the device-generated reply. In cut-through mode the
  command frame itself still propagates around the ring and returns to the
  master RX path.
- `ACK_TIMED` is the recommended reply mode for interactive and isolated
  `STRIKE` commands: it costs 12 extra payload bytes (~480 µs at 250 kbaud) but
  eliminates the need for a separate `QUERY_STRIKE` round trip to harvest the
  previous strike's timing. The server's real-time MIDI chord trains use
  `NONE`; reply loss must not become scheduler flow control during playback.

### ACK Replies

Addressed commands sent with reply mode `ACK` reply with type `0x50 + addr` and
5 payload bytes:

```text
[type] [subcmd] [result] [detail_hi] [detail_lo]
```

`subcmd` is the low-6-bit sub-command ID without the reply-mode bits.

Reply mode `ACK_TIMED` returns the same 5-byte ACK header with 12 additional
bytes describing the most recently completed strike (read from the firmware's
`prev_compact` snapshot — never the in-flight strike whose ACK these bytes
ride on; correlate by sequence number):

```text
[type] [subcmd] [result] [detail_hi] [detail_lo]
[seq_hi] [seq_lo] [flags_hi] [flags_lo]
[t_coast_hi] [t_coast_lo] [t_impact_hi] [t_impact_lo]
[t_rebound_hi] [t_rebound_lo] [vel_hi] [vel_lo]
```

Total payload is 17 bytes. If the device has not yet completed any strike
since boot, `seq` is 0 and `flags` has no validity bits set; hosts should
treat the timing fields as missing in that case.

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
The `fault` byte values are `0 = NONE`, `1 = OVERCURRENT`, `2 = HALL_INVALID`,
`3 = ENCODER_TIMEOUT`, `4 = ADC_TIMEOUT`, `5 = HOMING_LIMIT`, and
`6 = STRIKE_LIMIT`. `CLEAR_FAULT` is a no-op when no fault is latched; after a
real fault it clears the latch but deliberately leaves `homed = 0`.

`QUERY_CONFIG` (`0x1C`) is an on-demand read used by the server/UI tuning
panel. It returns type `0x40 + addr` and 36 payload bytes and is intentionally
separate from the high-frequency status and strike replies:

```text
[type]
[vel_kp:i16] [vel_ki:i16] [vel_kd:i16] [vel_ff:i16]
[pos_kp:i16] [pos_ki:i16] [pos_kd:i16]
[cur_kp:i16] [cur_ki:i16] [torque_limit_ma:u16]
[home_offset:i16] [coast_distance:i16] [homing_duty:i16]
[mute_brake_ms:i16] [mute_press_ma:i16] [mute_engage_offset:i16]
[flags:u16] [encoder_csn_assert_level:u8]
```

All multi-byte fields are big-endian. Flag bits are `0x0001 = valid persisted
record loaded`, `0x0002 = encoder zero valid`, `0x0004 = CS/LED polarity
valid`, and `0x0008 = learned strike calibration valid`.

`QUERY_STRIKE` replies with the same type `0x40 + addr` and 35 payload bytes
(legacy firmware emits 32 bytes — older hosts should accept either length).
The first 32 bytes match the legacy layout exactly; the new fields are
appended:

```
[type] [strike_state] [homed] [timing_flags_lo]
[seq_hi] [seq_lo] [current_hi] [current_lo]
[t_coast_hi] [t_coast_lo] [t_rebound_hi] [t_rebound_lo]
[t_retrigger_ready_hi] [t_retrigger_ready_lo]
[t_ready_hi] [t_ready_lo] [vel_dps_hi] [vel_dps_lo]
[drum_pos_b3] [drum_pos_b2] [drum_pos_b1] [drum_pos_b0]
[home_pos_b3] [home_pos_b2] [home_pos_b1] [home_pos_b0]
[home_offset_hi] [home_offset_lo]
[coast_distance_hi] [coast_distance_lo]
[homing_duty_hi] [homing_duty_lo]
[timing_flags_hi]
[t_impact_hi] [t_impact_lo]
```

`timing_flags` is a 16-bit field carried as the byte at offset 3 (low byte)
and the byte at offset 32 (high byte). Bits:

- `0x0001` trigger-to-coast time valid
- `0x0002` trigger-to-rebound time valid
- `0x0004` trigger-to-ready time valid
- `0x0008` strike currently active
- `0x0010` most recent strike was accepted as a retrigger while already active
- `0x0020` rebound timing was produced by coast timeout rather than detected reversal
- `0x0040` strike velocity estimate valid
- `0x0080` trigger-to-retrigger-ready time valid
- `0x0100` trigger-to-impact time valid
- `0x0200` strike was a dead strike (`STRIKE_EX` type 0x01); rebound fields never become valid
- `0x0400` warning: latest home shifted at least 1,024 counts from the prior known calibration

Timing fields are reported in milliseconds for the most recently accepted
strike. `t_impact` is the firmware's measurement of mallet-velocity zero-cross
during coast — a direct lower bound on drum contact, suitable for scheduling
strikes ahead of their musical time. `t_rebound` is the older "velocity
reversed past threshold" proxy, slightly later than impact.

`QUERY_STRIKE_TIMING` replies with type `0x40 + addr` and 13 payload bytes —
a compact subset of the strike status focused on the timing fields the host
needs to schedule ahead. Reads from the firmware's `prev_compact` snapshot,
so the values describe the last completed strike, never an in-progress one:

```
[type] [seq_hi] [seq_lo] [flags_hi] [flags_lo]
[t_coast_hi] [t_coast_lo] [t_impact_hi] [t_impact_lo]
[t_rebound_hi] [t_rebound_lo] [vel_hi] [vel_lo]
```

Use this rather than `QUERY_STRIKE` when only the timing fields are needed —
saves ~70% of the round-trip byte cost (16 wire bytes vs 38).

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

`QUERY_TIMING` replies with the same type `0x40 + addr` and 64 payload bytes
(older firmware revisions may stop after the 49-, 53-, or 57-byte prefix):

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
[uart_rx_overflow_b3] [uart_rx_overflow_b2] [uart_rx_overflow_b1] [uart_rx_overflow_b0]
[adc_overrun_b3] [adc_overrun_b2] [adc_overrun_b1] [adc_overrun_b0]
[proto_dbg_seq]
[proto_dbg_cmd_type] [proto_dbg_len] [proto_dbg_raw_subcmd]
[proto_dbg_reply_initial] [proto_dbg_reply_final] [proto_dbg_reply_branch]
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
miss its schedule. `uart_rx_overflow` counts every UART1 buffer-overflow event
(hardware FIFO or software ring); nonzero values mean the ring cable, baud
setting, or UART ISR latency is on the edge of what the link can sustain.
`adc_overrun` counts every ADC conversion whose previous sample had not yet
been consumed when it landed; nonzero means ADC ISR latency is starving the
pipeline and current/voltage samples are being silently dropped.
The final seven `proto_dbg_*` bytes describe the most recent addressed command
and reply branch; `QUERY_TIMING` itself deliberately does not overwrite this
snapshot.

## Persistent Settings

The firmware reserves the last two 512-byte APROM pages for a CRC-protected
settings block that is loaded at boot. The two pages are used as a
ping-pong pair with a monotonic sequence number in each record: on save,
firmware writes the record to whichever page is not currently live, then
adopts the new page on success. If power is lost mid-write, the previously
live page is untouched and the device boots on the earlier record. This
also doubles the flash-write endurance of the persist region.

Persisted items:

- encoder zero reference, stored as the absolute 14-bit encoder angle
- encoder CS/status-LED assert polarity when detected or explicitly set
- strike tuning (`home_offset`, `coast_distance`, `homing_duty`)
- dead-strike tuning (`mute_brake_ms`, `mute_press_ma`, `mute_engage_offset`)
- strike learned calibration (`drum_position`, `home_position`) when homed
- motor torque limit, velocity PID, velocity feedforward, position PID, and
  current PI

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

The host treats both forwarding-mode changes as acknowledged transitions: it
waits for the corresponding command frame to return around the ring before
advancing. While waiting for the final nonzero `SET_ADDRESS [N]`, it ignores
delayed mode-command frames and the original `SET_ADDRESS [0]` cut-through
echo. This avoids depending on USB-UART latency or an arbitrary settling delay
and ensures every node is in the expected forwarding mode when enumeration
returns.

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
