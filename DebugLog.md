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
- **Hardware validation:** **PASS 2026-08-03** on the 14-actuator handpan; chords were nearly perfectly synchronized and rapid layered passages retained following notes cleanly.

## 2026-08-13 — Loose mallet mount allowed a runaway strike

- **Observation:** If a mallet mounting loosened and pushed the ball out of the drum path, a strike could spin the motor near top speed and sometimes throw the mallet or the ball. Homing also had no explicit travel/time ceiling, although its low duty was not energetic enough to throw hardware.
- **Root cause:** `src/strike.c` had no terminal condition based on total encoder travel in `STRIKE_DRIVING`, `STRIKE_COASTING`, `STRIKE_MUTING`, or `STRIKE_CATCHING`; those states assumed the learned drum contact/rebound would occur. `STRIKE_HOMING` likewise relied only on stall and settle detection. A displaced mechanical stop violated both assumptions.
- **Fix:** Accumulate absolute encoder travel for each active sequence at the existing 500 Hz strike cadence. Fault and disable PWM at one revolution for either homing or strike; add a 5-second homing backstop; invalidate homing on every motor fault and fault clear. Re-home position changes of at least 1,024 counts are now exposed as a warning, and an idle position hold can be paused safely while `SAVE_SETTINGS` persists the reference.
- **Class:** unbounded-mechanical-state-machine
- **Recently-touched?** no — the missing termination contract was longstanding; the bench observation exposed it.
- **Time to fix:** one session.
- **Hardware validation:** pending a restrained-actuator test of both limit faults, recovery/re-home, and the shifted-home warning.

## 2026-08-22 — First home after power-up falsely warned of a shifted drum

- **Observation:** Most actuators reported a home-shift warning on the first touch-off after power-up even though the drum surface had not moved; later homes in the same boot were generally consistent.
- **Root cause:** `src/encoder.c:163` seeds continuous position at zero from the arbitrary power-up rotor angle when no logical zero exists, while pre-fix `src/persist.c:185` saved that boot-relative drum position and pre-fix `src/strike.c:889` compared it after the next boot as though both values shared an origin.
- **Fix:** Capture and compare the raw 14-bit encoder angle at drum contact, persist it in record v5, and make v3/v4 migration establish a new baseline without warning.
- **Class:** boot-relative-coordinate-persisted-as-absolute
- **Recently-touched?** yes — the warning comparison was introduced in the latest firmware commit.

## 2026-09-20 — Microphone scan stalled on noise and skipped cached status

- **Observation:** Mobile scanning waited too long for ringdown in background city noise, skipped mallets for unavailable fresh state, and later stopped with a generic “Stopped before completion” row when no usable note was heard.
- **Root cause:** The newly added browser scan required return to the initial noise floor, treated every cached observation as unusable, and distinguished silence from unclear audio using onset alone. Background noise can satisfy onset without yielding a pitch. Final cleanup replaced the affected row's reason with a generic stopped label.
- **Fix:** Bound settling to 0.6–1.5 seconds, remeasure the pre-strike background, accept healthy recent cache observations and recheck stale status on the same mallet. Retry silence or unclear audio once, then continue unresolved. Bound extra current to 20% / 200 mA and 3000 mA total; overload retries at original current. Preserve actual command/readiness errors in the row and never retry an unacknowledged command.
- **Recently-touched?** yes — browser scan implementation in this session; firmware and bus protocol unchanged.
- **Validation:** 31 pitch/audio/workflow tests, 19 MIDI tests and 22 bridge/playback tests passed; forced firmware rebuild passed. Isolated browser simulation recovered a noise-only first attempt, exhausted two silent attempts on another slot, and finished with all nine audible notes detected. Physical validation of these revised behaviors remains pending.

## 2026-09-20 — Scan appeared stuck after hearing a strike

- **Observation:** The user intermittently saw “Heard the strike” without progress.
- **Root cause reproduced:** In `player/pitch_detector.js`, the fresh-audio guard returned before normal timeout checks when the AudioContext was interrupted or its clock froze. That left the last progress message visible until the 25-second fallback. Timer callback exceptions also escaped the capture promise.
- **Fix:** Add a wall-clock watchdog before that guard, attempt one audio resume at 0.75 seconds, and report stalled audio by three seconds. Do not dispatch further strikes without recent running audio. Route analyser exceptions into capture failure and preserve the existing retry behavior for ordinary unclear notes.
- **Recently-touched?** yes — the browser capture loop was edited in this session.
- **Validation:** Added regression cases for frozen audio immediately after onset, successful recovery with running/interrupted AudioContext states, and analyser exceptions. The two stall/recovery tests failed against the prior implementation and pass with the fix. All 34 pitch/audio/workflow tests pass. The physical cause on the user's phone is not yet confirmed.
