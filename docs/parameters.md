# Firmware and Operator Parameter Catalog

This catalog distinguishes values that are safe to tune during ordinary
operation from board calibration and compile-time safety policy. The primary
browser player's actuator configuration panel is the normal tuning path. Every
ordinary tuning value can target either the selected actuator or all currently
enumerated actuators; a fleet-applied value can still be changed later on any
individual actuator.

`Save to drive` follows the same selected/all target and commits the complete
live settings snapshot, not only the field that was most recently changed.

## Runtime tuning saved by `SAVE_SETTINGS`

| Parameter | Default | Units / encoding | What it changes | UI scope |
|---|---:|---|---|---|
| `velocity_kp` | 1536 | Q8 coefficient | Proportional response of the velocity loop; its output is requested motor current. | one or all |
| `velocity_ki` | 10 | Q8 coefficient | Removes sustained velocity error. Too much can wind up or oscillate. | one or all |
| `velocity_kd` | 20 | Q8 coefficient | Damps changes in measured velocity; derivative input is filtered. | one or all |
| `velocity_ff` | 220 | Q8 feedforward | Adds current in proportion to requested velocity before PID correction. | one or all |
| `position_kp` | 1200 | Q8 with 16:1 error prescale | Converts position error into a velocity request. | one or all |
| `position_ki` | 10 | Q8 with 16:1 error prescale | Removes steady position error in the outer loop. | one or all |
| `position_kd` | 20 | Q8 with 16:1 error prescale | Adds position-loop damping. | one or all |
| `current_kp` | 96 | Q8 coefficient | Immediate PWM response to current error in the 6 kHz inner PI loop. | one or all |
| `current_ki` | 5 | Q8 coefficient | Removes steady current error. | one or all |
| `torque_limit_ma` | 3200 | mA, 0..3800 effective | Clamps commanded motor current below the firmware's hard sustained-current ceiling. | one or all |
| `home_offset` | 1024 | encoder counts | Resting distance away from the learned drum surface. A live change moves an idle homed mallet's hold target. | one or all |
| `coast_distance` | 300 | encoder counts | Distance before the learned drum surface at which powered acceleration ends. | one or all |
| `homing_duty` | 100 | signed PWM counts | Low drive used to seek the drum. Its sign selects the toward-drum direction; zero disables homing. There is no separate persisted motor-direction parameter. | one or all |
| `mute_brake_ms` | 30 | ms | Dead-strike velocity-zero braking phase before the contact press. | one or all |
| `mute_press_ma` | 250 | mA | Dead-strike current that holds the ball against the drum after braking; zero keeps velocity-zero hold. | one or all |
| `mute_engage_offset` | 0 | signed encoder counts | Dead-strike brake trigger relative to the learned surface; negative values require travel past that reference. | one or all |

All PID and strike tuning wire values are signed 16-bit. The torque limit is
unsigned 16-bit on the wire and is clamped to the 3800 mA compiled safety
ceiling by firmware. Settings records use two flash pages, CRC protection, and
ping-pong updates. Version 5 adds the raw drum-contact encoder angle used by
cross-boot home-shift detection while retaining read compatibility with
versions 3 and 4. Older records intentionally provide no warning baseline
until a successful home is saved in the new format.

## Learned and board-specific calibration

| Value | How it is established | Persistence | Operator treatment |
|---|---|---|---|
| Drum position | Homing stall detection | Logical position and raw 14-bit drum-contact angle are saved only when the actuator is homed | Re-home after mechanical changes; the warning compares raw absolute angles, independent of the rotor position at power-up. A shift of at least 1024 counts (22.5°) from the saved or previous same-boot home produces a warning. |
| Home position | Derived from drum position and `home_offset` | Saved with drum position | Automatically recalculated after homing or a live home-offset change. |
| Drum direction | Sign of `homing_duty`, captured while homing | Reconstructed from saved calibration and homing duty | Tune per actuator if a motor/encoder is mechanically reversed; it may also be fleet-applied when the hardware is uniform. |
| Encoder zero reference | `ZERO_POSITION` | Saved immediately | Maintenance calibration; changing it shifts logical position references. |
| Encoder CS / status-LED assert level | Automatic detection or explicit CS-polarity command | Saved immediately on successful detection/set | Per-board electrical calibration. The application LED shares the CS pin, so this is also its effective polarity; there is no independent LED-polarity parameter. The browser shows it but does not offer a fleet-wide setter. |
| Ring address | Enumeration order | Not persisted | Assigned from 0 upward each time the host enumerates the physical ring. |

Motor commutation `drive_dir` is internal runtime state chosen from the sign of
the active duty/current/velocity request. It is not a user parameter. The
compile-time `COMMUTATION_OFFSET` selects hall-sector advance for this hardware
design and should not be confused with actuator direction.

## Per-command values (not saved as tuning)

| Value | Range / units | Lifetime |
|---|---|---|
| Strike current | signed mA; browser clamps ordinary musical strikes to 0..3000 mA | One strike; firmware orients it toward the learned drum direction. |
| Dead-strike dwell | 0..1000 ms | One `STRIKE_EX`; zero selects the 150 ms default. |
| Duty target | signed 16-bit PWM counts | Until another control command or stop. |
| Velocity target | signed 16-bit RPM | Until another control command or stop. |
| Position target | signed 32-bit encoder counts | Until another control command or stop. |
| Current target | signed 16-bit mA | Until another control command or stop. |
| Control mode | duty, velocity, position, or torque/current | Runtime only. Strike sequencing changes modes internally. |

## Browser/server performance parameters

These live on the host rather than in each motor drive: slot-to-MIDI mapping,
per-slot current trim, master strike current, MIDI velocity floor, playback
tempo, latency-compensation enable/default, and learned per-slot/current-band
impact latency. The server owns the serial port and provides status, recovery,
single/fleet tuning, saving, playback, and firmware-update operations.

## Compile-time control and safety constants

These are firmware policy, not operator tuning. Changing them requires a new
firmware build and fleet update.

| Area | Current values | Purpose |
|---|---|---|
| Control rates | PWM/ADC 20 kHz; current 6 kHz; encoder/velocity 1.5 kHz; position 1 kHz; strike 500 Hz | Establishes CPU and control bandwidth. Work added to the fast loops must be budgeted carefully at 24 MHz. |
| Current protection | sustained 3800 mA for 8 ms; peak 4500 mA for 300 µs | Hard protection above the tunable torque limit. |
| Motion bounds | one cumulative encoder revolution for homing and for a full strike sequence | Stops a loose or displaced mallet from running indefinitely. |
| Homing wall time | 5000 ms | Generous independent backstop in addition to the encoder travel bound. |
| Home-shift warning | 1024 counts / 22.5° | Advisory mechanical-change detection; does not fault. |
| Encoder health | 1000 ms without motion while meaningful duty is applied | Detects a failed/stale encoder path. |
| ADC health | 20 missed 6 kHz fast ticks (about 3.33 ms) | Detects loss of the current-sampling pipeline. |
| Position clamps | 2000 RPM output, 4000 RPM integral clamp | Bounds outer-loop requests. |
| Strike recovery | 500 ms coast timeout; 512-count overshoot/retrigger windows; 20 ms settle | Defines rebound recognition and safe return behavior. |
| Dead-strike ceiling | 1000 ms dwell | Limits heating while the mallet is intentionally held at the surface. |

The authoritative numeric definitions are in `include/m2003_config.h`; the
wire commands and reply layouts are in `protocol.md`.
