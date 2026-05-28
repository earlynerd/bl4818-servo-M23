# Ring MIDI Server HTTP API

`scripts/ring_midi_server.py` is the host-side bridge between the ring bus
and anything that wants to make the actuators play music or notification
chimes. It owns the serial port, exposes a small REST surface over HTTP,
and serves the browser-based player UI as a side benefit.

This document is the integration reference for that REST surface.

If you're writing a new application that wants to ping the actuators
("play a chime when CI fails", "ring at 9am", "play a melody from a
calendar event"), use the **Friendly API** section. The lower-level
endpoints are documented for completeness and for the browser player.

---

## Running the server

```
python scripts/ring_midi_server.py
python scripts/ring_midi_server.py -p COM7 --http-port 8765
```

Common flags:

| Flag                  | Default       | Purpose                                                                |
|-----------------------|---------------|------------------------------------------------------------------------|
| `-p`, `--port`        | auto-detect   | Serial port for the ring bus                                           |
| `--baud`              | 250000        | Ring baud rate                                                         |
| `--host`              | `127.0.0.1`   | HTTP bind host. Set to `0.0.0.0` to expose on the LAN                  |
| `--http-port`         | 8765          | HTTP bind port                                                         |
| `--timeout-ms`        | 1000          | Per-frame RX timeout                                                   |
| `--library-dir`       | (none)        | Folder of `.mid`/`.midi` files exposed at `/api/library`               |
| `--motif-current-ma`  | 800           | Default master current for pitch-style /api/play requests at velocity 127. 0-3000 mA     |
| `--motif-vel-floor`   | 0.5           | Default velocity scale floor. See "Velocity curve" below               |

Base URL throughout this document is `http://<host>:<http-port>`.
All endpoints accept any `Origin` (CORS is open by default), so calling
from a browser on another port is fine.

> **Windows gotcha:** the server binds IPv4 only by default. Python's
> `urllib` (and a few other clients) resolve `localhost` to `::1` first
> and eat a ~2 s IPv6 connect timeout before falling back to IPv4. Use
> `http://127.0.0.1:8765` from Python clients on Windows to skip that
> entirely. curl and browsers do not hit this.

---

## Quickstart

Three calls to play a chime:

```bash
# 1. Discover what pitches are currently mapped to actuators.
curl http://localhost:8765/api/pitches

# 2. Build a motif (chord on the first three pitches, t_ms is absolute).
curl -X POST http://localhost:8765/api/play \
  -H 'Content-Type: application/json' \
  -d '{
    "name": "ci-passed",
    "master_current_ma": 1000,
    "events": [
      {"t_ms": 0,   "pitch": 60, "velocity": 100},
      {"t_ms": 0,   "pitch": 64, "velocity": 100},
      {"t_ms": 0,   "pitch": 67, "velocity": 100},
      {"t_ms": 300, "pitch": 72, "velocity": 110}
    ]
  }'

# 3. (Only if you need to interrupt before duration_ms is up.)
curl -X POST http://localhost:8765/api/cancel
```

The motif POST returns immediately with a motif id, the scheduled
duration, and the list of any events that had to be skipped (e.g.
because the pitch isn't mapped). Playback is fire-and-forget; you do
not need to keep the connection open.

---

## Concepts

### Mapping: slot ↔ pitch

The physical ring has N actuators ("slots", addressed 0..N-1). The
mapping is a list of `slots.length` entries where each entry is either
a MIDI pitch number (0-127) or `null` (slot disabled / not used in this
song). It's persisted to `mapping.json` at the project root, so it
survives server restarts and is shared across every browser that
connects to the bridge.

Integrations talk in **MIDI pitch numbers**, not slot indexes. The
bridge looks each pitch up in the mapping and routes the strike to the
matching slot. Unmapped pitches are silently skipped (and reported back
in the response so the caller can tell).

You can fetch the human-friendly form via `GET /api/pitches`, which
already filters out null slots and sorts by pitch ascending. Use that
for any "what's available" discovery.

### Velocity curve

Every motif event carries a MIDI velocity (0-127). The bridge maps it
to a strike current in mA using the same curve as the browser player:

```
scale      = vel_floor + (1 - vel_floor) * (velocity / 127)
current_ma = master_current_ma * scale          # clamped to [0, 3000]
```

- `master_current_ma` is the max current — what you'd get at velocity 127.
- `vel_floor` is the floor of the scale. `vel_floor = 1.0` ignores
  velocity entirely (every note plays at master). `vel_floor = 0.5`
  (default) makes velocity 1 play at half master. `vel_floor = 0.0`
  makes soft notes barely strike.

Server-wide defaults come from `--motif-current-ma` and
`--motif-vel-floor`. Any individual `POST /api/play` can override
them with `master_current_ma` and `vel_floor` fields, applied only to
that motif. Out-of-range values are clamped; the response echoes back
the values actually used.

### Latency compensation

Sending a strike command and waiting for the mallet to physically hit
the tine isn't instantaneous — it's tens of milliseconds, varies with
strike current, and varies slot-to-slot. The bridge keeps a per-slot
per-current-bucket EMA of `trigger_to_impact_ms` (the
`LatencyTracker`), fed by every strike that flows through it (browser
or motif). Motif playback uses that table to fire each strike command
*early* enough that the mallet impact lands on the intended beat. You
don't need to do anything to opt in — just send `t_ms` values that
reflect when you want the *impact* to happen, and the bridge handles
the offset.

Beats stay flush even for chords (multiple events at the same `t_ms`).
The EMA needs a handful of strikes to converge on a slot if the bridge
has just started; the first few strikes use a 50 ms default.

### Preemption

There is exactly one motif slot. Posting a new motif preempts whatever
was playing — the previous worker's stop event is set, the worker is
joined (~2 s timeout), and the new motif starts. `POST /api/cancel`
also stops any running motif.

---

## Friendly API

The integration surface. Use these endpoints unless you specifically
need direct ring access.

### `GET /api/status`

What's on the ring right now.

**Response (200):**
```json
{
  "count": 5,
  "homed": [true, true, false, true, true],
  "slots": [
    {
      "homed": true,
      "home_offset": 120,
      "coast_distance": 80,
      "homing_duty": 600,
      "last_timing": {
        "address": 0, "sequence": 41, "flags": 1,
        "trigger_to_coast_ms": 22.5, "trigger_to_impact_ms": 38.1,
        "trigger_to_rebound_ms": 52.0,
        "estimated_strike_velocity_dps": 1450,
        "rebound_timeout": false
      }
    }
  ]
}
```

`last_timing` is only present if at least one strike has completed on
that slot since the bridge started. `homed: false` slots have null
strike parameters.

### `GET /api/pitches`

Currently mapped pitches with note names. The recommended discovery
endpoint for integrations.

**Response (200):**
```json
{
  "pitches": [
    {"pitch": 60, "name": "C4",  "slot": 0, "homed": true},
    {"pitch": 64, "name": "E4",  "slot": 1, "homed": true},
    {"pitch": 67, "name": "G4",  "slot": 2, "homed": true},
    {"pitch": 72, "name": "C5",  "slot": 3, "homed": true},
    {"pitch": 76, "name": "E5",  "slot": 4, "homed": true}
  ]
}
```

Sorted ascending by pitch. `homed: false` means the actuator hasn't
self-homed yet — strikes will likely fail or be inaccurate. Empty list
means no mapping has been saved yet.

### `GET /api/mapping`

The raw slot-indexed mapping. Lower-level than `/api/pitches`; integrations
usually want `/api/pitches` instead.

**Response (200):**
```json
{ "mapping": [60, 64, 67, 72, 76] }
```

If no mapping has been saved, `"mapping": null`.

### `POST /api/play`

Schedule and play a motif. Fire-and-forget. Preempts any in-flight motif.

**Request body:**
```json
{
  "name": "ci-passed",
  "master_current_ma": 1000,
  "vel_floor": 0.5,
  "events": [
    {"t_ms": 0,   "pitch": 60, "velocity": 100},
    {"t_ms": 250, "pitch": 64, "velocity": 100},
    {"t_ms": 500, "pitch": 67, "velocity": 100}
  ]
}
```

| Field               | Type      | Required | Notes                                                                              |
|---------------------|-----------|----------|------------------------------------------------------------------------------------|
| `events`            | array     | yes      | One element per note. Order doesn't matter; the server sorts by `t_ms`             |
| `events[].t_ms`     | int >= 0  | yes      | Absolute time from motif start. Multiple events at the same `t_ms` form a chord    |
| `events[].pitch`    | int 0-127 | yes      | MIDI pitch. Unmapped pitches are skipped (reported in response)                    |
| `events[].velocity` | int 0-127 | no       | Default 100. Maps to current_ma via the velocity curve                             |
| `name`              | string    | no       | Free-form label. Echoed in the response and in `Player.status()`              |
| `master_current_ma` | number    | no       | Per-motif master current. Defaults to `--motif-current-ma`. Clamped to [0, 3000]   |
| `vel_floor`         | number    | no       | Per-motif velocity floor. Defaults to `--motif-vel-floor`. Clamped to [0.0, 1.0]   |

**Response (200):**
```json
{
  "ok": true,
  "id": "a1b2c3d4",
  "name": "ci-passed",
  "duration_ms": 500,
  "scheduled": 3,
  "skipped": [
    {"t_ms": 750, "pitch": 96, "reason": "unmapped"}
  ],
  "master_current_ma": 1000,
  "vel_floor": 0.5
}
```

- `id` is a short hex token; useful for logging which motif is playing.
- `duration_ms` is the `t_ms` of the last scheduled event — i.e. when
  the final mallet impact is expected. The server does *not* wait that
  long; the call returns as soon as the worker thread is started.
- `scheduled` is the count of events that will actually fire.
- `skipped` lists events that won't fire and why. Today the only reason
  is `"unmapped"`. Skipped events still parse — they fail open so a
  partially-mapped ring can still play what it can.

**Error responses (400):**

```json
{ "error": "expected {'events': [...]}" }
{ "error": "'name' must be a string" }
{ "error": "'master_current_ma' must be a number" }
{ "error": "'vel_floor' must be a number" }
{ "error": "motif event t_ms must be >= 0, got -5" }
{ "error": "motif event pitch out of range: 200" }
{ "error": "bad motif event {...}: ..." }
```

### `GET /api/play`

Snapshot of the currently running playback. Useful for polling — the
`POST /api/play` call is fire-and-forget, so this is how callers learn
when playback is about to finish without having to track `duration_ms`
themselves.

The shape is the same whether playback was started via the pitch-style
or canonical-form `POST /api/play`. Anything stashed in metadata at
play-time (`name`, `master_current_ma`, `vel_floor` from pitch-style;
`schedule_master_ma` from canonical) is spread into the response.

**Response (200), playing pitch-style:**
```json
{
  "playing": true,
  "id": "a1b2c3d4",
  "scheduled": 3,
  "cursor": 1,
  "duration_ms": 500,
  "elapsed_ms": 213,
  "remaining_ms": 287,
  "master_scale": 1.0,
  "muted": [],
  "name": "ci-passed",
  "master_current_ma": 1000,
  "vel_floor": 0.5
}
```

**Response (200), idle:**
```json
{
  "playing": false,
  "id": null,
  "name": null,
  "scheduled": 0,
  "cursor": 0,
  "duration_ms": null,
  "elapsed_ms": null,
  "remaining_ms": null,
  "master_scale": 1.0,
  "muted": [],
  "master_current_ma": null,
  "vel_floor": null
}
```

- `cursor` is the index of the next event the worker will dispatch.
  `cursor == scheduled` once playback finishes.
- `master_scale` and `muted` reflect any live tweaks from
  `/api/play/update`. They reset on each `/api/play`.
- `elapsed_ms` and `remaining_ms` are computed from the server's
  monotonic clock at the time of the request. They're clamped so that
  `elapsed_ms + remaining_ms == duration_ms` once the schedule is
  complete; the worker thread may keep `playing: true` for a brief
  moment after `remaining_ms` reaches 0 while it tears down. Treat
  `playing: false` as the authoritative "done" signal.

### `POST /api/play` (canonical form)

The browser player uses a second event shape that bypasses pitch resolution
and the velocity curve — the events are already tied to ring addresses and
specific currents:

```json
{
  "master_ma": 800,
  "events": [
    {"t_ms": 0,    "address": 0, "nominal_current_ma": 600},
    {"t_ms": 250,  "address": 2, "nominal_current_ma": 750}
  ]
}
```

`master_ma` is informational — it's the master the caller used when
computing each `nominal_current_ma`. `POST /api/play/update` uses it as
the denominator when computing the live scale ratio. The server detects
this form vs the pitch-style form by looking at the first event's keys.

### `POST /api/play/stop`

Stop the in-flight playback worker. Does NOT disable motors (use
`/api/stop` for that, or `/api/cancel` for a full panic stop).

**Request body:** none.

**Response (200):** `{"ok": true}`

### `POST /api/play/update`

Apply live tweaks to in-flight playback. Both fields are optional and
independent. Takes effect on the next strike.

**Request body:**
```json
{
  "master_scale": 1.25,
  "muted": [3, 7]
}
```

Or send a new master in mA along with the schedule's original master and
let the server divide:

```json
{
  "master_ma": 1000,
  "schedule_master_ma": 800,
  "muted": []
}
```

| Field                 | Type        | Notes                                                                          |
|-----------------------|-------------|--------------------------------------------------------------------------------|
| `master_scale`        | number      | Direct multiplier on every event's nominal current. Wins if both forms set     |
| `master_ma`           | number      | New master mA. Combined with `schedule_master_ma` → ratio                      |
| `schedule_master_ma`  | number      | Original master used when computing the schedule's nominal currents            |
| `muted`               | int[]       | Addresses to skip until the next update. `[]` clears the mute set              |

**Response (200):** echoes the new state. Example:
```json
{"ok": true, "master_scale": 1.25, "muted": [3, 7]}
```

### `POST /api/cancel`

Stop the running playback (if any) and cancel all in-flight strikes
across the ring. Panic stop.

**Request body:** none.

**Response (200):**
```json
{ "ok": true }
```

---

## Building good chimes

A few patterns that work well in practice:

- **Acknowledgement / success** — a short ascending arpeggio over
  ~300-500 ms. Three notes at velocity 90-110.
- **Failure / error** — a descending two-note interval (e.g.
  perfect fourth or tritone) at higher velocity, ~250 ms gap.
- **Attention / notification** — a single high-pitched strike at
  high velocity, optionally followed by a quieter echo at half master.
- **Doorbell** — two pitches a perfect fourth apart with ~400 ms gap,
  e.g. G4 then D4.
- **Top-of-hour** — single low pitch repeated N times, ~500 ms apart,
  N = number of the hour (12-hour clock).

Keep total duration short (under ~2 s for chimes) and leave at least
~150 ms between strikes on the *same* slot — the firmware needs time to
home the mallet back to its rest position before another strike will
register cleanly. Multiple events at the same `t_ms` on *different*
slots are fine and form a chord.

If you're playing actual music, run it through the browser player at
`http://localhost:8765/` first — it shares the same mapping and the
same latency table, so it's the fastest way to validate that a piece
sounds right on the current ring layout.

---

## Lower-level endpoints

These exist primarily for the browser player and bench tooling. Most
integrations won't need them, but they're available.

### `POST /api/enumerate`

Re-walk the ring and rebuild the slot list. Use after physically adding
or removing devices.

**Response (200):** `{"count": N}`

### `POST /api/home`

Send a homing command to each addressed slot. Fire-and-forget — the
device homes asynchronously; poll `/api/status` to see the `homed` flag
flip.

**Request body:** `{"addresses": [0, 1, 2]?}` (omit to home all)

**Response (200):** `{"ok": true, "addresses": [0, 1, 2]}`

### `POST /api/strike`

Single direct strike, bypassing the motif scheduler. No latency
compensation. Returns the firmware ACK and the timing block for the
*previous* completed strike on that slot.

**Request body:** `{"address": 0, "current_ma": 1200}`

**Response (200):**
```json
{
  "address": 0,
  "accepted": true,
  "result": 0,
  "result_name": "OK",
  "detail": 0,
  "metrics": { "address": 0, "sequence": 42, "trigger_to_impact_ms": 38.1, ... }
}
```

### `POST /api/strikes`

Batch direct strikes. Same semantics as `/api/strike`, executed
sequentially under the ring lock.

**Request body:**
```json
{
  "strikes": [
    {"address": 0, "current_ma": 1000},
    {"address": 2, "current_ma": 1200}
  ]
}
```

**Response (200):** `{"results": [<ack>, <ack>]}`

### `POST /api/strike-param`

Set a strike-related parameter on a single slot. Used by the tuning UI.

**Request body:**
```json
{ "address": 0, "param": "home_offset", "value": 120 }
```

Valid `param` values: `"home_offset"`, `"coast_distance"`, `"homing_duty"`.

### `POST /api/stop`

Send `motor_stop` to each addressed slot — disables PWM, motor goes
idle. Different from `/api/cancel`, which is a panic stop of in-flight
strike commands.

**Request body:** `{"addresses": [0, 1]?}` (omit for all)

**Response (200):** `{"results": [<ack>, <ack>]}`

### `POST /api/save-settings`

Persist the current strike-param block on each addressed slot to that
device's NVM. Slot must not be running a strike or firmware returns
NOT_READY.

**Request body:** `{"addresses": [0, 1]?}` (omit for all)

**Response (200):** `{"results": [<ack>, <ack>]}`

### `GET /api/strike-timing`

Cached per-slot strike-timing snapshots, harvested from every strike
that flows through the bridge.

**Response (200):**
```json
{
  "slots": [
    { "address": 0, "sequence": 41, "trigger_to_impact_ms": 38.1, ... },
    null,
    { "address": 2, ... }
  ]
}
```

Slots with no completed strikes yet are `null`.

### `GET /api/strike-timing?address=N`

Synchronous fresh poll of one slot's strike-timing block. Returns the
same shape as one entry of `/api/strike-timing.slots`, or
`{"address": N, "sequence": 0, "flags": 0}` if the slot hasn't
completed any strike yet.

### `POST /api/mapping`

Replace the persisted slot→pitch mapping.

**Request body:** `{"mapping": [60, 64, 67, null, 72]}`

Length should match the slot count. Entries must be `null` or a valid
MIDI pitch (int 0-127). Invalid entries → 400.

**Response (200):** `{"ok": true, "mapping": [60, 64, 67, null, 72]}`

### `GET /api/library`

List `.mid`/`.midi` files in the `--library-dir` folder (flat, no
recursion). Returns `{"configured": false, "files": []}` if no library
dir was passed at startup.

**Response (200):**
```json
{
  "configured": true,
  "files": [
    {"name": "twinkle.mid", "size": 4321},
    {"name": "frere-jacques.mid", "size": 2210}
  ]
}
```

### `GET /api/library/<name>`

Raw `.mid` bytes from the library dir. Returns `audio/midi`.
404 if the library isn't configured or the file doesn't exist.
400 for invalid filenames (path traversal, wrong extension).

### `POST /api/library/<name>`

Upload a `.mid` file to the library dir.

- Body is raw MIDI bytes (must start with `MThd`).
- 409 if the file already exists, unless `?overwrite=1` is set.
- 413 if the file is larger than 8 MB.
- 400 for invalid filenames or non-MIDI uploads.

**Response (201):**
```json
{ "ok": true, "name": "twinkle.mid", "size": 4321, "overwritten": false }
```

---

## Error responses

All endpoints return JSON errors of the form `{"error": "..."}` for
expected failures (400 / 404 / 409 / 413) and `{"error": "<exception>"}`
for unexpected failures (500). The motif endpoint can also return 400
with specific validation messages — see the table under `POST /api/play`.

The server does not currently distinguish between "ring disconnected"
and "device returned an error" in the HTTP response. Inspect
`/api/status` if a strike or motif behaves unexpectedly.

---

## CORS

All endpoints set:
```
Access-Control-Allow-Origin: *
Access-Control-Allow-Methods: GET, POST, OPTIONS
Access-Control-Allow-Headers: Content-Type
```

`OPTIONS` requests return 204 with no body. This is intentional — the
bridge is expected to be reachable from arbitrary local-network
integrations, including browser-based ones served from a different
port.

---

## Worked example: chime client

A complete integration example (Python) lives at
`scripts/chime_demo.py`. It hits `/api/pitches`, picks a few pitches
to build chimes out of whatever's actually mapped, and exposes named
chimes ("success", "error", "attention", "doorbell") as one-line calls.

See that script for a copy-pasteable pattern for new integrations.
