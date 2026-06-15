# Debug Log

Bench-observed bugs and their resolutions. Structural decisions live in DECISIONS.md; this file is the bug breadcrumb trail.

## 2026-06-10 — Dead strike produced a double hit instead of one clean hit

- **Observation:** Dead strikes nearly worked with tuning, but the mallet audibly hit twice. User hypothesis (confirmed by code reading): the mallet has already begun rebounding before MUTING engages.
- **Root cause:** `src/strike.c` mute entry keyed off `maybe_record_impact()` — a zero-cross of the *filtered* velocity estimate (`vel_filt_q8` EMA in `src/motor.c`), sampled in `strike_tick` at 500 Hz. Filter group delay + 2 ms tick quantization meant the velocity-0 brake engaged after the ball tip had decompressed and launched; the brake stopped the mallet hovering off the surface and the press phase re-seated it from a gap → second tap.
- **Fix:** Two-stage position-armed trigger in `src/motor.c` (`motor_arm_coast_then_brake`): coast at the coast point, then engage the velocity-0 brake on the raw encoder crossing of the learned drum surface (± new `MUTE_ENGAGE_OFFSET` tunable, param 0x06), at the encoder sampling cadence — same proven path as coast arming, no filter or strike-tick lag. Filtered zero-cross retained only as the stalled-short fallback.
- **Class:** filtered-estimate-latency (control action keyed to a lagging derived signal instead of the raw measurement)
- **Recently-touched?** yes — mute entry written earlier the same session it was bench-tested.
- **Time to fix:** ~1 session (diagnosis was user-assisted from bench observation).
