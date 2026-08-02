# Decision Log

Forward-facing, append-only record of architectural and behavioral decisions for this project. Read this file at the start of each session — it's more reliable than memory and supersedes any conflicting claims in CLAUDE.md, AGENTS.md, or README.

When a decision is reversed or superseded, append a new entry rather than rewriting the old one.

## 2026-05-14 — Rev2 PCB: switch bus from UART ring to RS485 multi-drop (tentative)

- **Decision:** Rev2 driver PCBA replaces the 5V UART ring (250 kbaud, cut-through forwarding) with RS485 half-duplex multi-drop. XT30 2+2 connectors retained; daisy-chain power + data through devices with a terminator plug on the last open connector. Addresses pre-programmed per device before bus placement and persisted to flash, replacing the ring-walk `ENTER_STORE_FORWARD` / `SET_ADDRESS` enumeration for rev2 nodes. Protocol shifts to master-polls with broadcast packets for command-rate traffic (extends the existing `BROADCAST_DUTY` slot-extraction pattern); feedback is lower rate. Baud target TBD, watch `uart_max` / `uart_rx_overflow_count` telemetry as it goes up from 250 kbaud.
- **Why:** Biggest concrete win is eliminating the cut-through echo write in `UART1_IRQHandler` (`src/app_uart.c:62-66`) — frees CPU on the 24 MHz M2003, which is already at the deadline edge. Differential signaling adds noise headroom (margin, not a current pain point) and removes the ring's downstream-break failure mode. CAN was considered but rejected: M2003 has no CAN peripheral, and rev2 keeps the M2003 because real-world firmware performance has been excellent.
- **Status:** Tentative. Rev2 schematic in progress. Gen1 ring-bus hardware (24 actuators) is in active use today and the gen1 protocol is unchanged on those units.
- **Supersedes:** (initial)
- **Affects:** rev2 PCB design; future `src/app_uart.c`, `src/protocol.c`, `protocol.md` updates for rev2 firmware path.

## 2026-05-14 — Rev2 PCB: RS485 transceiver control on a single GPIO (PB.14)

- **Decision:** PB.14 is the only spare GPIO and is tied to both DE and /RE for half-duplex control. Biased HIGH externally so during MCU reset/SWD the /RE input is deasserted and RO is tri-stated — frees PF.1 (shared with ICE_CLK) for the programmer. DE is also asserted in that state as a consequence. A ~10 kΩ pulldown on the PF.0 → DI net parks the driver at a defined recessive idle when PF.0 is high-Z, so a device under SWD drives a steady idle rather than spraying SWD edges or random levels onto the bus.
- **Why:** PF.0/PF.1 are shared between UART1 and ICE_DAT/ICE_CLK, so the transceiver's RO output cannot fight the programmer on PF.1. Biasing PB.14 high satisfies that constraint with the one available pin. Full tri-state of the transceiver during reset would need a second GPIO (none available) or a transceiver with a dedicated enable pin — not pursued.
- **Accepted tradeoff:** A device under SWD pauses bus traffic for the duration of the programming session (its transceiver dominates the differential pair driving recessive idle). Acceptable because (a) not safety-critical, (b) workflow programs devices off-bus before placement; no live-bus reflash use case.
- **Status:** Tentative, follows from the rev2 bus topology decision above.
- **Supersedes:** (initial)
- **Affects:** rev2 PCB schematic — RS485 transceiver wiring, PB.14 net, PF.0/DI pulldown.

## 2026-05-28 — Server-side playback dispatch; one `Player` for chimes + songs

- **Decision:** Scheduled playback is dispatched from the Python server, not the browser. Browser POSTs the full schedule once at play-start to `POST /api/play`; a single worker thread inside `Player` (`scripts/ring_midi_server.py`) fires each strike from `time.monotonic()` + `Event.wait()`. Live tweaks (master mA, mute set) go through `POST /api/play/update` and apply on the next strike. `POST /api/play/stop` stops the worker without disabling motors. The previous `MotifPlayer` (chime integrations) and the never-shipped browser-side song dispatcher are unified into one `Player` class — the `/api/play` handler auto-detects pitch-style vs canonical event input and translates the former via `motif_events_to_canonical()`. `/api/motif` is removed.
- **Why:** Browser-side per-event `setTimeout` + per-strike `fetch /api/strike` sounded fine over localhost (~3 ms network, ~2.5 ms scheduler jitter) but failed audibly over WiFi (57 ms mean network, 1990 ms max → 2 seconds of silence followed by a flurry of catch-up strikes). Any browser tab hiccup or WiFi stall queued per-strike POSTs that arrived in a clump. Server-side dispatch puts the dispatcher next to the hardware so the variable-latency link can't affect beat timing — the schedule is already in the server's hands.
- **Accepted tradeoffs:** (1) Per-slot trim and velocity-floor are now frozen at schedule build; only master mA and the mute set survive as live tweaks (matches the user's stated need; full live parity would require recomputing currents server-side from raw velocity, duplicating the browser's pipeline). (2) The browser popup's pipeline-timing metrics (sched jitter, network, server total, lock wait, serial RTT) and the per-slot residual stats no longer update during server-dispatched playback — those measurements live in `strikeAsync` which is bypassed on this path. They still work for keyboard test taps. Closing that gap (server exposes per-slot residual; browser polls during playback) is deferred.
- **Supersedes:** the implicit browser-dispatch architecture in `play()` and `MotifPlayer` (now replaced by `Player`). Builds on [[feedback_timing_compensation_minimal]] — that was the *prediction* problem; this is the *dispatch* problem.
- **Affects:** `scripts/ring_midi_server.py` (new `Player`; deleted `MotifPlayer`/`SongPlayer`; new endpoints `/api/play`, `/api/play/stop`, `/api/play/update`; removed `/api/motif`); `player/midi_player.html` (`play()` and `stopPlayback()` rewritten; master-slider live-update hook); `scripts/chime_demo.py` (point at `/api/play`); `midi_server_api.md` (endpoint docs).

## 2026-05-29 — Player exposes residual + timing stats; popup polls during playback

- **Decision:** `Player.status()` now includes `residual_stats` (per-address: `last`/`sum`/`sumSq`/`n`/`maxAbs`) and `timing_stats` (`lock_wait_ms` + `serial_rtt_ms`, same bag shape). Player tracks residual = `trigger_to_impact_ms − comp_ms_at_strike_time`, holding the previous prediction per address so the next ACK's metric attributes to the right strike (same one-strike lag pattern as `LatencyTracker._fold_metrics_into_ema`). Browser polls `GET /api/play` every 250 ms while `S.playing` and routes the popup's Residual, lock_wait, and serial_rtt rows through the server's view; sched_jitter and network rows hold their pre-playback session totals (those metrics don't exist on the server-dispatch path — no setTimeout, no per-strike fetch).
- **Why:** Closes the "deferred" gap explicitly noted in the previous entry. Without this, the per-strike metrics we'd added one session earlier went silent during server-dispatched playback — invisible exactly when they were most useful. Stats accumulate across plays; only a server restart resets them, so the popup's `n=` count keeps growing as a useful sample size.
- **Status:** Completes the deferred item in the 2026-05-28 entry. Does not supersede that decision — the architecture is unchanged; this is the visibility layer on top of it.
- **Supersedes:** (none — completes deferred work from 2026-05-28).
- **Affects:** `scripts/ring_midi_server.py` (`Player.residual_stats` / `timing_stats` / `_last_prediction_ms` / `_fold_stat`; `status()` payload includes the new bags); `player/midi_player.html` (`startPlayPolling` / `stopPlayPolling`; `refreshCfgTiming` reads server bags when `S.playing`).

## 2026-06-04 — Status LED polarity tracks autodetected CSn polarity

- **Decision:** The indicator no longer assumes "PB1 high = LED lit". It maps each pattern's logical lit/dark through the autodetected CSn assert level (`encoder_get_csn_polarity()`) via a new `pin_level_for()` helper: lit → assert level, dark → de-assert level. The LED lights and CSn asserts on the same electrical event, so "lit" must follow the same per-board polarity the encoder already learns. `indicator_get_csn_level()` now publishes the *physical* PB1 level (was logical).
- **Why:** PB1 is shared between SSI CSn and the status LED, and the LED lights at whichever level asserts CSn. The old fixed `PIN_SSI_CSN = level` only matched the modchip variant (assert=1); on FET-inverting / CSn-active-low boards (assert=0) every pattern read inverted. Behavior is byte-identical on assert=1 boards (identity map) and inverted-as-intended on assert=0 boards.
- **Caveat:** Mapping assumes the LED lights at the *assert* level. Consistent with the symptom (active-low boards read inverted) but unverified on the bench at time of writing — if still inverted on an assert=0 unit, flip the ternary in `pin_level_for()`.
- **Supersedes:** (initial) — first polarity-aware behavior for the indicator; builds on the existing CSn autodetect (`encoder_autodetect_csn_polarity`).
- **Affects:** `src/indicator.c` (`pin_level_for`, `indicator_init`, `indicator_tick`), `include/indicator.h` (module doc + `indicator_get_csn_level` contract).

## 2026-06-04 — Ring bus link-health instrumentation + browser panel

- **Decision:** The bridge now accumulates per-address ring-link health (`BusHealth`: RTT sum/sumSq/max/last in µs over successful transactions, plus `timeouts` / `crc_errors` / `errors` counts and `last_error`). Fed at the bridge chokepoints that touch the bus — `strike()` (wrapped to record the drop on the exception it already lets propagate) and the per-address `query_strike` path shared by `status()` / `pitches()` / the new `probe_bus()`. New endpoints: `GET /api/bus-health` (snapshot), `POST /api/bus-health/probe` (one active query round, returns snapshot), `POST /api/bus-health/reset`. The browser player gained a live "Bus Health" sidebar panel that polls every 1 s and, while idle, actively probes once a second (auto-suspends during playback — live strikes feed the same counters then); rows show per-device `mean ± σ` / max RTT and a drop-rate, colored red on CRC/≥5% drops, amber on occasional drops.
- **Why:** Diagnosing intermittent wiring (flaky connectors on the daisy-chain) had no observable signal — dropped strike replies were caught in the playback worker and silently skipped (`Player._run`), and `serial_rtt` was only folded during song playback. This surfaces "packets disappear" (timeouts), "come back mangled" (CRC = bus noise), and "take longer than expected" (RTT spread) per device, live.
- **Accepted tradeoffs:** (1) Counts are cumulative since reset (mirrors the existing `timing_stats`/`residual_stats` bags) — no rolling window; the reset button covers the wiggle-a-wire workflow. (2) Active probe holds the ring lock per-address (not around the whole sweep) to cap the latency a concurrent keyboard tap can incur to one query. (3) Latency spread is shown numerically but not colored — sub-ms RTTs see large *relative* OS/serial jitter on a healthy bus, so coloring on σ would cry wolf.
- **Supersedes:** (initial) — first link-health visibility. Builds on the server-side dispatch + stats-polling pattern from the 2026-05-28/05-29 entries.
- **Affects:** `scripts/ring_midi_server.py` (`BusHealth`; `Bridge.health` / `_record_bus_exc` / `_query_strike_timed` / `probe_bus`; instrumented `strike` / `status` / `pitches`; 3 endpoints; imports `RingCRCError`), `player/midi_player.html` (Bus Health panel + `renderBusHealth` / `busHealthTick` / monitor wiring), `midi_server_api.md` (endpoint docs).

## 2026-06-09 — Dead-strike articulation: STRIKE_EX command + MUTING state

- **Decision:** New `STRIKE_EX` (0x1A): `[type] [current_hi] [current_lo] [param_hi] [param_lo]`. Type 0 = normal (identical to `STRIKE`), type 1 = dead strike — drive/coast unchanged, then at detected impact the firmware enters a new `STRIKE_MUTING` state (wire value 6): velocity mode setpoint 0 brakes the rebound (braking current self-scales via the 1500 Hz velocity PI; bumpless because `motor_set_mode` seeds the PI at zero error), then a small toward-drum torque presses the mallet into the surface for the commanded dwell (`param`, ms; 0 = 150 default, clamped to 1000), then the normal catch. Articulation is per-command, not device state. New `SET_STRIKE_PARAM` tunables: `MUTE_BRAKE_MS` 0x04 (default 30) and `MUTE_PRESS_MA` 0x05 (default 250; 0 = velocity-hold for whole dwell). New timing flag 0x0200 = dead strike; rebound fields never become valid for dead strikes. Coast-timeout without detected impact still enters MUTING (gentle press = damp-like) with the 0x0020 timeout flag. Retrigger during MUTING rejects `NOT_READY`; `STRIKE_CANCEL` aborts the mute early.
- **Why:** Mute articulation ("dead strikes") needs the mallet to stay in contact past impact, overcoming the rubber tip's rebound to prevent bounce-chatter. Velocity-0 braking was chosen over a fixed torque surge because it scales braking current to actual rebound energy with no per-strike tunable. Per-command type (vs a sticky mode) follows the per-request-over-global-state rule. New subcmd (vs widening `STRIKE`) so gen1 firmware rejects it loudly with `INVALID_ARGUMENT` instead of silently playing the wrong articulation.
- **Accepted tradeoffs:** (1) `MUTE_BRAKE_MS`/`MUTE_PRESS_MA` are not persisted by `SAVE_SETTINGS` (reset to defaults at boot) — defer flash-layout change until bench tuning settles values. (2) `QUERY_STRIKE` reply does not report the new tunables (wire layout unchanged). (3) Brake/press handoff quality on the rubber tip is unverified on the bench at time of writing; if the velocity loop chatters against the compliant contact, fall back to torque-surge entry or retune brake duration.
- **Supersedes:** (initial) — first articulation beyond the single strike type.
- **Affects:** `src/strike.c` / `include/strike.h` (`STRIKE_MUTING`, `strike_trigger_ex`, mute tunables, `STRIKE_TIMING_DEAD`), `src/protocol.c` (`SUBCMD_STRIKE_EX`, param ids 0x04/0x05), `include/m2003_config.h` (`STRIKE_MUTE_*` defaults), `protocol.md`, `scripts/ring_bus.py` (`strike_ex`, consts), `scripts/ring_tool.py` (`strike --dead --mute-ms`, new `set-strike-param` choices).

## 2026-06-10 — Dead-strike mute entry is position-armed, not velocity-zero-cross

- **Decision:** The velocity-0 brake for dead strikes now trips inside the motor layer on the raw encoder position crossing of the learned drum surface, via a two-stage armed trigger (`motor_arm_coast_then_brake`: coast at the coast point, brake at `drum_position − drum_dir × MUTE_ENGAGE_OFFSET`), checked at the encoder sampling cadence like the existing coast arm. New `SET_STRIKE_PARAM` id `MUTE_ENGAGE_OFFSET` 0x06 (signed counts; default 0 = at the surface; positive = before it, negative = past it). The filtered-velocity zero-cross remains only as the stalled-short fallback; the coast-timeout damp path is unchanged. `strike_tick` records coast/impact timestamps best-effort (≤ one 500 Hz tick late) when the brake trips between ticks — the brake action itself is not delayed.
- **Why:** Bench showed double hits: the filtered-velocity zero-cross (EMA group delay) plus the 500 Hz strike-tick quantization engaged the brake after the rubber tip had already launched the rebound, so the press phase re-seated the mallet from a hover gap — an audible second tap. Raw-position arming engages the brake within one encoder sample of contact, while the ball is still compressing. See DebugLog.md 2026-06-10.
- **Supersedes:** the "at the firmware's impact detection (velocity zero-cross)" entry mechanism in the 2026-06-09 dead-strike entry; the rest of that decision stands.
- **Affects:** `src/motor.c` / `include/motor.h` (`motor_arm_coast_then_brake`, `motor_is_brake_engaged`, brake stage in the coast-trip handler, `brake_trip_pos` in `motor_shift_position_reference`), `src/strike.c` (`on_brake_engaged` / `enter_muting` split, dead-strike arming in `begin_strike`, `mute_engage_offset`), `src/protocol.c` (param 0x06), `include/m2003_config.h`, `protocol.md`, `scripts/ring_bus.py`, `scripts/ring_tool.py`.

## 2026-06-12 — Library browser: subfolders, favorites/ratings, jukebox controls

- **Decision:** The MIDI library browser is now a drill-down file browser over a *recursive* `--library-dir`. `GET /api/library` recurses (`rglob`) and returns each file as `{path, name, dir, size, mtime}` (POSIX-relative) plus a bundled `meta` map; the old flat `{name, size}` is superseded (the `name` field is retained, so the change is additive for any other reader). `GET`/`POST /api/library/<path>` now accept nested subpaths — the `name == Path(name).name` rejection is replaced by `Handler._resolve_library_path()` (rejects absolute / `..`, then a `resolve().relative_to(lib)` containment check is the authoritative escape guard); POST creates parent dirs so "Save current" lands in the folder being browsed. Favorites + 0–5 ratings (plus `play_count` / `last_played`) persist in **`.midi_library.json` inside the library folder** (keyed by path), read/merged/written under `Handler._meta_lock` since the server is a `ThreadingHTTPServer`; new `GET`/`POST /api/library-meta`. Browser side (`player/midi_player.html`): folder tree + breadcrumb derived from the flat list, a search box that flattens across all folders, a ★-only filter, a sort dropdown (name/rating/recent/plays), per-row ♥ + ★ controls, and jukebox controls — Shuffle, Autoplay-next, and Prev/Next buttons. The current view *is* the play queue.
- **Why:** A flat list doesn't scale as the collection grows, and there was no way to favorite, rate, or shuffle. Storing metadata in the library folder (the user's explicit choice) makes it shared across browsers/machines and travels with the collection — the same rationale as `mapping.json` being server-side, not per-browser localStorage. Only `midi_player.html` consumes `/api/library` (`looper.html` has no browser), so the response-shape change is low-risk.
- **Accepted tradeoffs:** (1) Autoplay **wraps** at end-of-list (jukebox "repeat all") rather than stopping — toggle autoplay off to halt. (2) Stale meta keys for deleted files are left in place (harmless; no prune step). (3) Play-count bumps in the client `play()` start path, so it counts real plays, not previews — but that path needs a live bridge, so it's the one piece not exercised by the headless browser test (the server-side `bump_play` merge itself is covered). (4) Autoplay-next distinguishes natural end from manual stop via a new `reason` arg on `stopPlayback("ended"|"user")` — only the teardown timer and the poll's `!playing` branch pass `"ended"`.
- **Supersedes:** the flat, single-folder library listing and the no-path-separator filename guard. Builds on the server-side dispatch architecture (2026-05-28) — autoplay hooks the existing `stopPlayback` natural-end signal.
- **Affects:** `scripts/ring_midi_server.py` (recursive `/api/library`; `_resolve_library_path` / `_library_meta_path` / `load_library_meta` / `save_library_meta` / `LIBRARY_META_NAME` / `_meta_lock`; new `/api/library-meta` GET+POST; subpath-aware library GET/POST; `PurePosixPath` import; module docstring + `--library-dir` help), `player/midi_player.html` (library panel markup + CSS, the rewritten library closure, Prev/Next buttons, `stopPlayback(reason)` + the two natural-end call sites, `libraryNotePlayed` hook in `play()`).

## 2026-07-31 — Document the proven actuator as a manual BL4818 retrofit

- **Decision:** The current reproducible hardware target is the proven manual BL4818 drive-board retrofit; a drop-in replacement for the complete integrated drive PCB remains a future hardware revision and must be documented separately.
- **Why:** The working actuators replace the stock MCU, add an INA180B2 shunt amplifier, modify resistor links, add a cable assembly, and connect both cable assemblies to a custom MT6701/power/interface board inside the printed shell. Replication documentation must describe the hardware that has actually been built before presenting the cleaner future board.
- **Supersedes:** (initial)
- **Affects:** `docs/hardware.md`, `docs/replication-guide.md`, `docs/replication-status.md`, `pcb/`, and `mechanical/`.

## 2026-07-31 — Gen1 STOP resets strike state; housekeeping follows SysTick

- **Decision:** Gen1 `STOP` is a coordinated actuator stop, not only a motor-mode change. It disarms coast/brake triggers, disables motor drive, returns the strike state machine to idle, and clears the runtime `homed` flag so a new `STRIKE_HOME` must complete before another strike. `STRIKE_HOME` now gives a truthful compact ACK (`OK`, busy/not-ready, fault, or disabled/invalid) when ACK mode is requested. Strike, indicator, and protocol-timeout housekeeping are scheduled from the fixed SysTick/control cadence rather than elapsed PWM/ADC samples, so they continue to run while PWM and ADC triggering are stopped.
- **Why:** A motor-only stop could leave the strike state machine active while also freezing the sample-derived housekeeping that would unwind it. That produced stale state and misleading command success after a stop. The coordinated contract makes the disabled state explicit and recoverable through re-homing without imposing tighter validation on tuning parameters.
- **Supersedes:** the implicit behavior where `STOP` and zero-duty only called `motor_stop()` while preserving strike/homed state; no prior documented protocol contract is otherwise changed.
- **Affects:** `src/main.c`, `include/sched_util.h`, `src/protocol.c`, `src/strike.c`, `include/strike.h`, `protocol.md`, `scripts/ring_midi_server.py`, `midi_server_api.md`.

## 2026-07-31 — Gen1 chord dispatch pipelines addressed strikes far-to-near

- **Decision:** The server groups playback events whose compensated trigger deadlines are within 1 ms and target unique actuators, then writes their existing addressed `STRIKE` frames as one farthest-address-first serial train. It waits for compact ACKs only after every command has been written and matches replies by address. This adds no broadcast command and does not change Gen1's one-direction, byte-level cut-through forwarding; normal isolated strikes retain `ACK_TIMED` timing feedback.
- **Why:** Sequential request/ACK round trips add host latency between chord notes even though the point-to-point links themselves forward each byte immediately. Farthest-first ordering ensures a farther device's downstream reply cannot get in front of a command still intended for a nearer device. The result removes avoidable millisecond-scale host skew while keeping rejection visibility; remaining on-wire skew is only the short frame serialization/cut-through interval and was already below the audible problem threshold.
- **Accepted tradeoff:** Chord bursts use plain ACKs, so those strikes do not return the previous strike's timing snapshot inline. Timing learning continues through isolated `ACK_TIMED` strikes and explicit timing polls.
- **Supersedes:** only the per-note wait-for-ACK dispatch detail of the 2026-05-28 server-side playback decision; the single `Player`, server-side schedule ownership, and latency-compensation architecture remain in force. The tentative Rev2 RS485 direction is unchanged.
- **Affects:** `scripts/ring_bus.py`, `scripts/ring_midi_server.py`, `player/midi_player.html`, `midi_server_api.md`, `tests/test_ring_playback.py`.

## 2026-08-01 — Gen1 firmware updates use a permanent LDROM recovery loader

- **Decision:** Existing Gen1 actuators will receive a one-time SWD provisioned, 4 KB LDROM loader that runs before APROM, keeps byte-level UART cut-through forwarding, and updates one addressed device at a time. APROM is divided into application `0x0000..0x79FF`, a commit-last manifest page at `0x7A00..0x7BFF`, and the unchanged two-page settings journal at `0x7C00..0x7FFF`. The loader erases the manifest before any application page and writes it only after exact-length CRC-32 verification; an invalid or interrupted image therefore remains recoverable in LDROM. Version 1 cannot update LDROM, configuration words, or persistent settings over the ring.
- **Why:** Manually connecting a programmer to every installed actuator is costly, while the M2003 provides enough independent LDROM for a recovery anchor. A manifest-last transaction gives power-loss recovery without an A/B application, which cannot fit in 32 KB. Keeping updates sequential preserves clear per-device retry and failure attribution on the one-direction point-to-point ring.
- **Bench gate:** Do not encode or write CONFIG0/CBS yet. First read the fleet configuration and verify LDROM-first IAP plus system-reset behavior on one restrained actuator; the exact value remains a measured provisioning input until then.
- **Supersedes:** the APROM-only/manual J-Link update workflow for provisioned Gen1 devices. Initial loader installation still requires SWD. The tentative Rev2 RS485 decisions are unchanged.
- **Affects:** `m2003.ld`, `include/firmware_image.h`, `ldrom/`, `src/protocol.c`, `scripts/ring_bootload.py`, provisioning scripts, `protocol.md`, and `docs/firmware-update.md`.

## 2026-08-01 — Standard J-Link flashing always installs the complete firmware stack

- **Decision:** `build-jlink.ps1` and `flash-jlink.ps1` are the sole supported J-Link path and always build, program, and verify APROM, the commit manifest, and LDROM; they never write CONFIG. There is no separate app-only or provisioning entry point.
- **Why:** The temporary split left the familiar flash command installing a loader-aware application without the loader, which was surprising and produced an actuator that could not safely accept `ENTER_BOOTLOADER`.
- **Supersedes:** the temporary app-only `flash-jlink.ps1` plus separately named LDROM provisioning workflow created during the 2026-08-01 loader implementation.
- **Affects:** `Makefile`, `scripts/build-jlink.ps1`, `scripts/flash-jlink.ps1`, J-Link artifacts, `README.md`, `docs/firmware.md`, and `docs/firmware-update.md`.

## 2026-08-01 — Gen1 SWD and ring UART are mutually exclusive bench connections

- **Decision:** Gen1 bootloader diagnostics must not assume simultaneous SWD and UART access: the J-Link SWD connection and UART ring adapter use the same two MCU pins and can only be connected one at a time.
- **Why:** Bench testing confirmed that attaching J-Link while observing the UART boot handoff is physically impossible on the existing actuator hardware. Diagnostics must use UART-visible state, retained evidence, or a sequential cable handoff.
- **Supersedes:** the simultaneous J-Link/UART diagnostic proposed during the first LDROM bench test; no firmware architecture decision is otherwise changed.
- **Affects:** Gen1 bootloader bench procedure, `docs/firmware-update.md`, and future diagnostic instrumentation.

## 2026-08-01 — LDROM residence is indicated by a solid onboard LED

- **Decision:** The Gen1 LDROM loader configures shared PB1 as GPIO output and holds it low for its entire residency, lighting the onboard LED solid on the current N-FET retrofit; the loader never accesses the encoder while holding its shared CSn pin.
- **Why:** SWD and UART cannot be attached together, so loader entry needs an immediate local indication that is independent of either diagnostic connection.
- **Supersedes:** the proposed fast-blink loader indicator; the application LED patterns are unchanged.
- **Affects:** `ldrom/platform.c`, `docs/firmware-update.md`, and `protocol.md`.

## 2026-08-01 — LDROM boot-intercept window is one second

- **Decision:** A valid application reached through LDROM without a valid retained boot request remains in the loader for one second before returning to APROM. PB1 remains solid for that full interval, and `STAY_IN_BOOTLOADER` may hold it resident.
- **Why:** The former 100 ms interval was too brief to distinguish from the application's several normal LED flashes during UART-only bench diagnosis. One second makes loader execution unambiguous and gives the recovery host a practical acquisition window without adding another diagnostic-only code path.
- **Supersedes:** the initial 100 ms provisional boot-intercept value.
- **Affects:** `ldrom/bl_internal.h`, `protocol.md`, and `docs/firmware-update.md`.

## 2026-08-01 — Standard Gen1 provisioning selects LDROM-first boot

- **Decision:** The sole supported J-Link command reads and records CONFIG0..2 before any write, programs and verifies APROM/manifest/LDROM, then clears only CONFIG0.CBS (bit 7) so reset and cold power enter LDROM. The wrapper refuses a locked or unexpected CONFIG2 security state, uses the vendor CONFIG erase procedure, verifies the erased state, restores captured CONFIG1 first, commits the preserved CONFIG0 with CBS cleared last, and verifies all three words. Configuration remains inaccessible over the ring.
- **Why:** The restrained actuator measured `CONFIG0=FFFFFFFF`, so CBS selected direct APROM boot. Bench tests proved that the runtime BS write plus system reset still returned to APROM. LDROM-first CBS is therefore required for the permanent recovery invariant: an invalid or interrupted APROM image must reach the independent loader without SWD.
- **Supersedes:** the CONFIG-write prohibition and open CONFIG/CBS bench gate in the earlier permanent-loader decision, plus the “never writes CONFIG” clause of the standard J-Link-path decision. The one public flash path and its APROM/manifest/LDROM verification contract remain unchanged.
- **Affects:** `scripts/flash-jlink.ps1`, `scripts/m2003_configure_ldrom.py`, `scripts/make_provision_jlink.py`, provisioning tests, `README.md`, `docs/firmware.md`, and `docs/firmware-update.md`.

## 2026-08-01 — LDROM starts APROM only through system reset

- **Decision:** The loader never directly calls an APROM reset vector. After validation and UART drain it selects APROM in the runtime BS bit and requests a system reset; APROM's `Reset_Handler` immediately re-arms LDROM as the next-reset destination before normal initialization.
- **Why:** A direct IAP branch must reconstruct reset-state CPU and peripheral context and the first implementation left interrupts masked, producing a repeat watchdog/reset cycle. The M2003 system reset preserves the software BS selection, so a clean APROM reset followed by immediate LDROM re-arming is both simpler and recovery-safe.
- **Supersedes:** the resetless vector-remap-and-jump handoff in “Gen1 firmware updates use a permanent LDROM recovery loader.”
- **Affects:** `ldrom/main.c`, `src/startup_m2003.c`, `src/boot_control.c`, `protocol.md`, and `docs/firmware-update.md`.

## 2026-08-01 — LDROM indicator drives PB1 high on the bench retrofit

- **Decision:** While resident, the Gen1 LDROM loader holds shared PB1 high to light the onboard status LED solid; it still never accesses the encoder.
- **Why:** A UART-confirmed loader hold left the LED dark with PB1 low, while the application's brief high-level indication was visible, proving the assumed polarity was reversed on the restrained actuator.
- **Supersedes:** the PB1-low polarity in “LDROM residence is indicated by a solid onboard LED.”
- **Affects:** `ldrom/platform.c`, `protocol.md`, and `docs/firmware-update.md`.

## 2026-08-02 — APROM/LDROM handoffs verify both VECMAP and boot source

- **Decision:** Both software handoffs follow the M2003 vendor ISP sequence: with interrupts masked, issue and verify `VECMAP` for the destination, select and verify the matching runtime boot source, then request a system reset. The application still re-arms LDROM early by changing only the next-reset boot source; it does not remap vectors while APROM is running. Loader protocol 2 appends the entry `SYS.RSTSTS`, `FMC.ISPCTL`, and `FMC.ISPSTS` snapshots to `GET_INFO` so the result can be diagnosed over UART.
- **Why:** Bench behavior showed that changing the boot-source bit alone did not produce a reliable, observable transition, while the manufacturer's M2003 ISP examples explicitly pair destination `VECMAP`, boot selection, and system reset. Gen1 shares its SWD and UART pins, so retained UART-readable evidence is required to distinguish a clean system-reset handoff from a watchdog return or vector-map failure.
- **Accepted tradeoff:** Protocol 2 leaves only 16 bytes free in the 4 KB LDROM image. Further loader features must recover code space before being added.
- **Supersedes:** the handoff mechanism detail in “LDROM starts APROM only through system reset”; the no-direct-branch rule, true reset entry, and early LDROM re-arm remain unchanged.
- **Affects:** `ldrom/main.c`, `ldrom/flash.c`, `ldrom/protocol.c`, `src/boot_control.c`, `scripts/ring_bootload.py`, `protocol.md`, `docs/firmware.md`, and `docs/firmware-update.md`.

## 2026-08-02 — J-Link verifies the APROM vector page through physical FMC reads

- **Decision:** The standard `flash-jlink.ps1` path programs once, then runs a separate read-only verification script. It reconstructs APROM bytes `0x0000..0x01FF` from physical FMC read commands and joins them with a direct debugger read of APROM above `0x0200`; the manifest and LDROM remain direct physical reads. All three images are compared byte-for-byte before any CONFIG update is committed.
- **Why:** In provisioned LDROM-first mode, the debugger's CPU-address read at zero observes the active LDROM vector alias rather than physical APROM. The prior direct `savebin 0` check therefore reported a repeatable false APROM mismatch at reset-vector offset `+0x4` even though programming succeeded.
- **Supersedes:** direct debugger verification of the full APROM image beginning at CPU address zero. The one public flash command and byte-for-byte verification requirement remain unchanged.
- **Affects:** `scripts/make_provision_jlink.py`, `scripts/flash-jlink.ps1`, `tests/test_provision_jlink.py`, `docs/firmware.md`, and `docs/firmware-update.md`.

## 2026-08-02 — Ring enumeration waits for returned transition frames

- **Decision:** The host no longer uses fixed sleeps to change ring forwarding modes. It waits for the returned `ENTER_STORE_FORWARD` frame, accepts only a nonzero bounded `SET_ADDRESS` result while skipping delayed/seed echoes, then waits for the returned `ENTER_CUT_THROUGH` frame before declaring enumeration complete.
- **Why:** Immediate re-enumeration after `RUN_APROM` exposed USB-UART latency beyond the old 10 ms flush window; the delayed `0x01` echo was mistaken for the address result even though the application had started correctly.
- **Supersedes:** fixed 10 ms/2 ms enumeration settling delays and single-frame `SET_ADDRESS` reply handling.
- **Affects:** `scripts/ring_bus.py`, all host tools using `RingClientV2.enumerate()`, `tests/test_ring_playback.py`, and `protocol.md`.
