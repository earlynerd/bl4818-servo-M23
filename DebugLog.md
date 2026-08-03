# Debug Log

Bench-observed bugs and their resolutions. Structural decisions live in DECISIONS.md; this file is the bug breadcrumb trail.

## 2026-06-10 — Dead strike produced a double hit instead of one clean hit

- **Observation:** Dead strikes nearly worked with tuning, but the mallet audibly hit twice. User hypothesis (confirmed by code reading): the mallet has already begun rebounding before MUTING engages.
- **Root cause:** `src/strike.c` mute entry keyed off `maybe_record_impact()` — a zero-cross of the *filtered* velocity estimate (`vel_filt_q8` EMA in `src/motor.c`), sampled in `strike_tick` at 500 Hz. Filter group delay + 2 ms tick quantization meant the velocity-0 brake engaged after the ball tip had decompressed and launched; the brake stopped the mallet hovering off the surface and the press phase re-seated it from a gap → second tap.
- **Fix:** Two-stage position-armed trigger in `src/motor.c` (`motor_arm_coast_then_brake`): coast at the coast point, then engage the velocity-0 brake on the raw encoder crossing of the learned drum surface (± new `MUTE_ENGAGE_OFFSET` tunable, param 0x06), at the encoder sampling cadence — same proven path as coast arming, no filter or strike-tick lag. Filtered zero-cross retained only as the stalled-short fallback.
- **Class:** filtered-estimate-latency (control action keyed to a lagging derived signal instead of the raw measurement)
- **Recently-touched?** yes — mute entry written earlier the same session it was bench-tested.
- **Time to fix:** ~1 session (diagnosis was user-assisted from bench observation).

## 2026-08-03 — Chords clumped or dropped following MIDI notes

- **Observation:** A freshly updated 14-actuator handpan ring played isolated-note songs normally, but chord passages sometimes pulled a later note into the chord or lost several following notes. One captured play scheduled 278 events but attempted only 212, with 13 transport failures, one `REJECT_NOT_READY`, and 508.7 ms maximum serial wait.
- **Root cause:** In commit `c62435e`, `scripts/ring_midi_server.py:1088` grouped by compensated transmit time without requiring the same MIDI impact time, while `scripts/ring_midi_server.py:1142` synchronously collected chord replies; missing replies stalled the absolute-time worker. `player/midi_player.html:1983` then canceled the still-running worker 500 ms after nominal song duration.
- **Fix:** Require equal MIDI `t_ms` for chord membership, transmit playback chords with reply mode `NONE`, account for them as unacknowledged rather than accepted, and let server state exclusively determine natural completion.
- **Class:** blocking-reply-in-realtime-scheduler
- **Recently-touched?** yes — all three paths were introduced or changed with the chord synchronization work.
- **Time to fix:** one bench feedback cycle.
