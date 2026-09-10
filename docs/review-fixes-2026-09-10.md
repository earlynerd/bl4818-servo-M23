# Review fixes and validation — 2026-09-10

## Changes

- STOP removes drive while preserving a latched motor fault. CLEAR_FAULT is
  still required before another motion command can start the motor.
- Saved calibration uses the absolute drum-contact encoder angle and the home
  clearance (`home_offset`). Startup reconstructs runtime coordinates from the
  current encoder reading rather than restoring coordinates from an earlier
  boot. Restoring calibration does not start motion.
- The tuning tool attempts STOP when a run fails, including when a motion
  command's reply is lost, and closes the connection even if STOP also fails.
- Bus reply deadlines now expire even while unrelated frames keep arriving.

## Settings compatibility

New saves use settings version 6 with the existing flash-page locations and
record size. Versions 3 through 5 can be migrated when they contain an absolute
contact angle or enough information to recover it. Otherwise tuning is loaded
and a fresh home is required. Older firmware cannot read version 6 records.
Wire commands are unchanged.

The absolute encoder identifies position within one revolution. Startup selects
the nearest equivalent home; it cannot recover a previous boot's turn count.

## Validation

- `make -B images`: passed a forced firmware and loader rebuild. Application
  size: 29,572 bytes of text, 12 bytes of data, and 1,360 bytes of BSS.
- `python -m unittest discover -s tests -v` on Windows: 55 passed; the one native
  C test was skipped because native GCC is unavailable on that PC.
- `python3 tests/test_firmware_native.py` in an isolated copy on `music-pi`:
  passed using the Pi's native GCC. It executed production motor, strike, PID,
  and settings-record code with simulated peripherals, covering 50 reboot
  geometries, encoder wrap, both directions, legacy migration, and all six
  motor-fault latches.
- `python3 tests/test_review_regressions.py -q` in that same isolated copy:
  eight tests passed, covering bus deadlines and tuning-tool cleanup.
- `git diff --check`: passed. New and modified functions were reviewed for
  implementation completeness.

The update payload is `build/m2003-motor.update.bin`: 29,584 bytes, CRC32
`0xEBFDF9BC`. The raw application and manifest are also generated in `build/`.

## Instrument checks

Read-only status checks on the existing instrument reported 14 homed actuators
and inactive playback. Its installed firmware, live application files, and
running services were left unchanged. No motor-motion commands were sent.

Flashing is reserved for the owner. The native tests validate software behavior
with simulated peripherals; flash persistence, real encoder communication,
power-cycle restoration, and motor behavior with the new firmware still need
physical validation after flashing.
