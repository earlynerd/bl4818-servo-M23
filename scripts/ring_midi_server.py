#!/usr/bin/env python3
"""
HTTP bridge between a browser-based MIDI player and the ring bus.

Owns the serial port, exposes a tiny REST surface, and serves the static
player HTML. The browser handles MIDI parsing, scheduling, and the
pitch -> address mapping; this process just forwards strike commands to
the ring at the moment they arrive.

Endpoints
---------
GET  /                  -> redirect to /player.html
GET  /player.html       -> serves player/midi_player.html
GET  /looper.html       -> serves player/looper.html (layer/loop builder)
GET  /api/status              -> {"count": N, "homed": [bool * N], "slots": [{...}]}
GET  /api/strike-timing       -> cached compact timing per slot
GET  /api/strike-timing?addr=N -> fresh compact poll for one slot
GET  /api/mapping             -> {"mapping": [int|null, ...] | null}; persisted slot->pitch map
GET  /api/pitches             -> {"pitches": [{"pitch","name","slot","homed"}, ...]};
                                 currently mapped pitches, sorted ascending
GET  /api/library             -> {"configured": bool, "files": [{"name", "size"}, ...]}
GET  /api/library/<name>      -> raw .mid bytes from the configured --library-dir
POST /api/library/<name>      -> writes raw .mid bytes to --library-dir; 409 on
                                 collision unless ?overwrite=1
POST /api/enumerate           -> re-enumerate; returns same shape as /api/status
POST /api/home                -> {"addresses": [int]?}; homes them, returns ok/error
POST /api/strike              -> {"address": int, "current_ma": int}; ACK + last-strike timing
POST /api/strikes             -> {"strikes": [{"address","current_ma"}, ...]}; batch
POST /api/strike-param        -> {"address": int, "param": str, "value": int}; ACK
POST /api/cancel              -> cancels all active strikes (panic stop, also stops motif)
POST /api/stop                -> {"addresses": [int]?}; motor_stop on each (disables PWM)
POST /api/save-settings       -> {"addresses": [int]?}; persists strike params to NVM
POST /api/mapping             -> {"mapping": [int|null, ...]}; persists slot->pitch map to disk
POST /api/motif               -> {"name": str?, "events": [{"t_ms","pitch","velocity"}, ...]};
                                 fire-and-forget motif playback. Preempts any in-flight motif.
                                 Pitches not present in /api/mapping are skipped and listed
                                 in the response. Velocity (0-127) is mapped to current_ma
                                 using the same curve as the browser player.

Usage
-----
    python scripts/ring_midi_server.py
    python scripts/ring_midi_server.py -p COM7 --http-port 8765

Then open http://localhost:8765/ in Chrome.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import traceback
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

from ring_bus import (
    CommandAck,
    RingClientV2,
    RingError,
    RingTimeout,
    StrikeTiming,
    auto_detect_port,
    DEFAULT_BAUD,
    REPLY_MODE_ACK,
    REPLY_MODE_ACK_TIMED,
    STRIKE_PARAM_HOME_OFFSET,
    STRIKE_PARAM_COAST_DISTANCE,
    STRIKE_PARAM_HOMING_DUTY,
)


STRIKE_PARAM_BY_NAME = {
    "home_offset": STRIKE_PARAM_HOME_OFFSET,
    "coast_distance": STRIKE_PARAM_COAST_DISTANCE,
    "homing_duty": STRIKE_PARAM_HOMING_DUTY,
}

NOTE_NAMES = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")


def midi_name(pitch: int) -> str:
    return f"{NOTE_NAMES[pitch % 12]}{pitch // 12 - 1}"


class LatencyTracker:
    """Per-slot, per-current-bucket EMA of trigger_to_impact_ms.

    Mirrors the playback compensation logic in player/midi_player.html so
    motif playback fires each strike command early enough that the mallet's
    physical impact lands on the intended beat. The browser maintains its
    own EMA in JS; this is the bridge's parallel copy, fed by every strike
    that flows through the bridge (browser or motif).

    Lookup falls through: this slot+bucket -> cross-slot+bucket ->
    this slot nearest bucket -> cross-slot nearest bucket -> DEFAULT_MS.
    """

    EMA_ALPHA = 0.3
    BUCKET_WIDTH_MA = 500
    BUCKET_COUNT = 6
    DEFAULT_MS = 50.0

    def __init__(self) -> None:
        self._per_slot: dict[int, dict[int, float]] = {}
        self._global: dict[int, float] = {}
        # Tracks the strike current most recently SENT to each slot. The
        # firmware reports timing for the *previous* completed strike on
        # each ACK, so we attribute incoming metrics to the bucket of the
        # prior strike, not the one whose ACK delivered them.
        self._last_sent_current: dict[int, int] = {}
        self._lock = threading.Lock()

    @classmethod
    def _bucket(cls, current_ma: int) -> int:
        if current_ma is None or current_ma < 0:
            return 0
        idx = current_ma // cls.BUCKET_WIDTH_MA
        return max(0, min(cls.BUCKET_COUNT - 1, int(idx)))

    def record_sent(self, slot: int, current_ma: int) -> int | None:
        """Mark that a strike at `current_ma` was just sent to `slot`. Returns
        the previous current value (or None if this is the first strike on
        the slot since the bridge started)."""
        with self._lock:
            prev = self._last_sent_current.get(slot)
            self._last_sent_current[slot] = current_ma
            return prev

    def update(self, slot: int, current_ma: int, impact_ms: float) -> None:
        if impact_ms is None:
            return
        idx = self._bucket(current_ma)
        with self._lock:
            buckets = self._per_slot.setdefault(slot, {})
            prev = buckets.get(idx)
            buckets[idx] = (
                impact_ms if prev is None
                else (1 - self.EMA_ALPHA) * prev + self.EMA_ALPHA * impact_ms
            )
            prev_g = self._global.get(idx)
            self._global[idx] = (
                impact_ms if prev_g is None
                else (1 - self.EMA_ALPHA) * prev_g + self.EMA_ALPHA * impact_ms
            )

    def compensation_ms(self, slot: int, current_ma: int) -> float:
        idx = self._bucket(current_ma)
        with self._lock:
            buckets = self._per_slot.get(slot)
            if buckets is not None and idx in buckets:
                return buckets[idx]
            if idx in self._global:
                return self._global[idx]
            if buckets is not None:
                for d in range(1, self.BUCKET_COUNT):
                    if (idx - d) in buckets:
                        return buckets[idx - d]
                    if (idx + d) in buckets:
                        return buckets[idx + d]
            for d in range(1, self.BUCKET_COUNT):
                if (idx - d) in self._global:
                    return self._global[idx - d]
                if (idx + d) in self._global:
                    return self._global[idx + d]
            return self.DEFAULT_MS

ROOT = Path(__file__).resolve().parent.parent
PLAYER_HTML = ROOT / "player" / "midi_player.html"
LOOPER_HTML = ROOT / "player" / "looper.html"
# Persisted slot->pitch mapping. Lives at the project root so it's shared
# across every browser that connects to the bridge. The previous behavior
# was per-browser localStorage, which reset the mapping every time someone
# opened the page from a fresh machine.
MAPPING_FILE = ROOT / "mapping.json"


class Bridge:
    """Thread-safe wrapper around RingClientV2."""

    def __init__(self, port: str, baud: int, timeout_ms: int = 1000, trace: bool = False):
        # The default in ring_bus is 200 ms, which is enough for a quiet
        # bus but tight when many devices are running motor work in
        # parallel (homing especially). Default the bridge a little
        # higher and let the caller override for very busy setups.
        self.client = RingClientV2(port=port, baudrate=baud, timeout_ms=timeout_ms, trace=trace)
        self.client.open()
        self.lock = threading.Lock()
        self.count: int = 0
        # Per-address cache of the most recent compact strike-timing snapshot
        # we've seen, keyed by ring address. Populated by ACK_TIMED piggyback
        # on strike replies and by explicit /api/strike-timing polls. Each
        # entry is the dict shape produced by _strike_timing_to_dict.
        self.strike_timing: dict[int, dict] = {}
        # Latency EMA used by motif playback for trigger-to-impact compensation.
        # Fed by every strike that goes through this bridge (browser too), so
        # the player's arpeggio warm-up also warms the bridge's table.
        self.latency = LatencyTracker()
        # Motif scheduler. Lazily attached after construction so it can hold a
        # reference back to this Bridge for self.strike() / self.latency.
        self.motif_player: "MotifPlayer | None" = None

    def enumerate(self) -> int:
        with self.lock:
            self.count = self.client.enumerate()
        return self.count

    def status(self) -> dict:
        with self.lock:
            count = self.count
            homed: list[bool] = []
            slots: list[dict] = []
            for addr in range(count):
                try:
                    s = self.client.query_strike(addr)
                    homed.append(bool(s.homed))
                    slot = {
                        "homed": bool(s.homed),
                        "home_offset": s.home_offset,
                        "coast_distance": s.coast_distance,
                        "homing_duty": s.homing_duty,
                    }
                    last_timing = self.strike_timing.get(addr)
                    if last_timing is not None:
                        slot["last_timing"] = last_timing
                    slots.append(slot)
                except Exception:
                    homed.append(False)
                    slots.append({
                        "homed": False,
                        "home_offset": None,
                        "coast_distance": None,
                        "homing_duty": None,
                    })
        return {"count": count, "homed": homed, "slots": slots}

    def strike(self, address: int, current_ma: int) -> dict:
        # ACK_TIMED costs 12 extra payload bytes (~480 us at 250 kbaud) and
        # replaces the otherwise-needed query_strike round trip. Net: less
        # ring traffic per strike, plus the host gets timing for free.
        prev_current = self.latency.record_sent(address, current_ma)
        with self.lock:
            reply = self.client.strike(
                address, current_ma, reply_mode=REPLY_MODE_ACK_TIMED
            )
        result = self._ack_to_dict(address, reply, cache_metrics=True)
        self._fold_metrics_into_ema(address, prev_current, result.get("metrics"))
        return result

    def strikes(self, items: list[dict]) -> list[dict]:
        results: list[dict] = []
        prev_currents: list[int | None] = []
        with self.lock:
            for item in items:
                addr = int(item["address"])
                cur = int(item["current_ma"])
                prev_currents.append(self.latency.record_sent(addr, cur))
                reply = self.client.strike(
                    addr, cur, reply_mode=REPLY_MODE_ACK_TIMED
                )
                results.append(self._ack_to_dict(addr, reply, cache_metrics=True))
        for item, result, prev in zip(items, results, prev_currents):
            self._fold_metrics_into_ema(int(item["address"]), prev, result.get("metrics"))
        return results

    def _fold_metrics_into_ema(self, address: int, prev_current: int | None,
                                metrics: dict | None) -> None:
        """Attribute incoming impact-time metrics to the prior strike's current
        bucket. Skips if we have no record of the prior strike yet (first
        strike on this slot since bridge launch) or if the impact flag was
        invalid (in which case the bridge already nulled the field)."""
        if metrics is None or prev_current is None:
            return
        impact = metrics.get("trigger_to_impact_ms")
        if impact is None:
            return
        self.latency.update(address, prev_current, float(impact))

    def query_strike_timing(self, address: int) -> dict:
        """Compact poll. Use this to harvest the very last strike's timing
        (e.g. end of a song where there's no follow-up strike to piggyback
        on, or when callers want a synchronous snapshot)."""
        with self.lock:
            timing = self.client.query_strike_timing(address)
        snap = self._strike_timing_to_dict(timing)
        if snap is not None:
            self.strike_timing[address] = snap
        return snap or {"address": address, "sequence": 0, "flags": 0}

    def set_strike_param(self, address: int, param_id: int, value: int) -> dict:
        with self.lock:
            reply = self.client.set_strike_param(
                address, param_id, value, reply_mode=REPLY_MODE_ACK
            )
        return self._ack_to_dict(address, reply)

    def home(self, addresses: list[int], timeout_ms: int = 8000) -> None:
        """Fire-and-forget homing per address — mirrors ring_tool's
        strike-home behavior. Sends each command with the default
        REPLY_MODE_FULL and on RingTimeout re-enumerates and tries once
        more (same shape as ring_tool's retry_after_enumerate). No host-side
        polling: the device homes asynchronously, and callers that need to
        know when each slot has settled can poll /api/status — the
        `homed` field flips to true per slot once the device finishes."""
        del timeout_ms  # retained for API compat; no longer relevant
        with self.lock:
            for addr in addresses:
                self._strike_home_with_retry(int(addr))

    def _strike_home_with_retry(self, addr: int) -> None:
        try:
            self.client.strike_home(addr)
            return
        except RingTimeout as first_exc:
            # Single retry path — match ring_tool: re-enumerate, then try
            # the command once more. Re-enumeration is a quick re-sync
            # that often clears whatever transient parser state caused
            # the missed reply.
            try:
                self.count = self.client.enumerate()
            except RingError:
                raise first_exc
            self.client.strike_home(addr)

    def cancel_all(self) -> None:
        # Stop the motif worker first so it can't fire a new strike between
        # our cancel calls. motif_player.cancel() blocks on the worker's join,
        # which only takes one strike RTT to return (the worker checks the
        # stop event between strikes and at every wait).
        if self.motif_player is not None:
            self.motif_player.cancel()
        with self.lock:
            for addr in range(self.count):
                try:
                    self.client.strike_cancel(addr, reply_mode=REPLY_MODE_ACK)
                except Exception:
                    pass

    def pitches(self) -> list[dict]:
        """Currently mapped pitches, with note name + slot + homed flag,
        sorted ascending by pitch. Used by remote integrations to discover
        what's playable without having to interpret the raw slot->pitch list."""
        mapping = self.load_mapping()
        if not mapping:
            return []
        homed_by_slot: dict[int, bool] = {}
        with self.lock:
            for addr in range(self.count):
                try:
                    s = self.client.query_strike(addr)
                    homed_by_slot[addr] = bool(s.homed)
                except Exception:
                    homed_by_slot[addr] = False
        out: list[dict] = []
        for slot, pitch in enumerate(mapping):
            if pitch is None:
                continue
            out.append({
                "pitch": int(pitch),
                "name": midi_name(int(pitch)),
                "slot": slot,
                "homed": homed_by_slot.get(slot, False),
            })
        out.sort(key=lambda x: x["pitch"])
        return out

    def stop(self, addresses: list[int]) -> list[dict]:
        """Send SUBCMD_STOP to each address — disables PWM and drops target
        velocity/current/duty to zero, motor goes to MOTOR_IDLE. Returns a
        per-slot ack list so the UI can surface partial failures."""
        results: list[dict] = []
        with self.lock:
            for addr in addresses:
                a = int(addr)
                try:
                    reply = self.client.stop(a, reply_mode=REPLY_MODE_ACK)
                    results.append(self._ack_to_dict(a, reply))
                except Exception as exc:
                    results.append({"address": a, "accepted": False, "error": str(exc)})
        return results

    def save_settings(self, addresses: list[int]) -> list[dict]:
        """Persist the current strike-param block on each address to the
        device's NVM. Slot must not be running a strike — firmware rejects
        with NOT_READY otherwise."""
        results: list[dict] = []
        with self.lock:
            for addr in addresses:
                a = int(addr)
                try:
                    reply = self.client.save_settings(a, reply_mode=REPLY_MODE_ACK)
                    results.append(self._ack_to_dict(a, reply))
                except Exception as exc:
                    results.append({"address": a, "accepted": False, "error": str(exc)})
        return results

    def load_mapping(self) -> list | None:
        """Return the persisted slot->pitch mapping, or None if no file exists
        yet. Entries are int (MIDI pitch 0-127) or null (slot disabled)."""
        try:
            raw = MAPPING_FILE.read_text(encoding="utf-8")
        except FileNotFoundError:
            return None
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError:
            return None
        if isinstance(obj, dict):
            obj = obj.get("mapping")
        if not isinstance(obj, list):
            return None
        cleaned: list = []
        for v in obj:
            if v is None:
                cleaned.append(None)
            elif isinstance(v, int) and 0 <= v <= 127:
                cleaned.append(v)
            else:
                cleaned.append(None)
        return cleaned

    def save_mapping(self, mapping: list) -> list:
        """Write the slot->pitch mapping to disk. Returns the normalized list
        that was written."""
        cleaned: list = []
        for v in mapping:
            if v is None:
                cleaned.append(None)
            elif isinstance(v, int) and 0 <= v <= 127:
                cleaned.append(v)
            else:
                # Reject anything that isn't a valid pitch or null — keeps
                # the file readable and matches the browser's own validation.
                raise ValueError(f"mapping entry {v!r} is not null or int 0-127")
        tmp = MAPPING_FILE.with_suffix(MAPPING_FILE.suffix + ".tmp")
        tmp.write_text(json.dumps({"mapping": cleaned}, indent=2), encoding="utf-8")
        tmp.replace(MAPPING_FILE)
        return cleaned

    def close(self) -> None:
        try:
            self.client.close()
        except Exception:
            pass

    def _ack_to_dict(self, address: int, reply, *, cache_metrics: bool = False) -> dict:
        if isinstance(reply, CommandAck):
            payload = {
                "address": address,
                "accepted": reply.accepted,
                "result": reply.result,
                "result_name": reply.result_name,
                "detail": reply.detail,
            }
            metrics_dict = self._strike_timing_to_dict(reply.metrics)
            if metrics_dict is not None:
                payload["metrics"] = metrics_dict
                if cache_metrics:
                    self.strike_timing[address] = metrics_dict
            return payload
        return {
            "address": address,
            "accepted": False,
            "result": -1,
            "result_name": "NO-ACK",
            "detail": 0,
        }

    @staticmethod
    def _strike_timing_to_dict(timing) -> dict | None:
        if not isinstance(timing, StrikeTiming):
            return None
        if not timing.has_data:
            # An "empty" snapshot (no strike has completed yet on this device).
            # Don't pollute the cache with it; return None so callers know.
            return None
        return {
            "address": timing.address,
            "sequence": timing.sequence,
            "flags": timing.flags,
            "trigger_to_coast_ms": timing.trigger_to_coast_ms,
            "trigger_to_impact_ms": timing.trigger_to_impact_ms,
            "trigger_to_rebound_ms": timing.trigger_to_rebound_ms,
            "estimated_strike_velocity_dps": timing.estimated_strike_velocity_dps,
            "rebound_timeout": timing.rebound_timeout,
        }


STRIKE_MA_MIN = 0
STRIKE_MA_MAX = 3000


class MotifPlayer:
    """Plays short motifs scheduled by remote integrations (calendar, Slack,
    etc). Owns a single worker thread — a new motif preempts whatever was
    playing. Pitches are resolved via the bridge's persisted mapping, so
    callers speak in MIDI semantics and don't need to know the physical
    slot layout."""

    def __init__(self, bridge: Bridge, master_current_ma: int, vel_floor: float) -> None:
        self.bridge = bridge
        self.master_current_ma = max(STRIKE_MA_MIN, min(STRIKE_MA_MAX, int(master_current_ma)))
        self.vel_floor = max(0.0, min(1.0, float(vel_floor)))
        # _play_lock serializes whole play()/cancel() calls so two motif POSTs
        # arriving at the same time can't leave two worker threads running.
        # _control_lock guards just the small mutable state (thread handle,
        # stop event, id) and is held only briefly.
        self._play_lock = threading.Lock()
        self._control_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._current_id: str | None = None
        self._current_name: str | None = None

    def velocity_to_current(self, velocity: int) -> int:
        """Same curve as the browser: master * (floor + (1-floor) * v/127),
        clamped to the firmware-supported current range."""
        v = max(0, min(127, int(velocity)))
        scale = self.vel_floor + (1 - self.vel_floor) * (v / 127.0)
        ma = int(round(self.master_current_ma * scale))
        return max(STRIKE_MA_MIN, min(STRIKE_MA_MAX, ma))

    def status(self) -> dict:
        with self._control_lock:
            running = self._thread is not None and self._thread.is_alive()
            return {
                "playing": running,
                "id": self._current_id if running else None,
                "name": self._current_name if running else None,
            }

    def play(self, events: list[dict], name: str | None = None) -> dict:
        """Resolve, schedule, and start playback. Returns immediately with a
        motif id, the scheduled duration, and the list of events that had to
        be skipped (e.g. unmapped pitches)."""
        mapping = self.bridge.load_mapping() or []
        pitch_to_addr: dict[int, int] = {}
        for addr, pitch in enumerate(mapping):
            if pitch is None:
                continue
            # Mapping can in principle assign the same pitch to multiple slots
            # (chorus). Keep the first; everything else is fine to ignore for
            # motif playback — integrations don't care which slot rings.
            pitch_to_addr.setdefault(int(pitch), addr)

        resolved: list[dict] = []
        skipped: list[dict] = []
        for ev in events:
            try:
                t_ms = int(ev["t_ms"])
                pitch = int(ev["pitch"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"bad motif event {ev!r}: {exc}") from exc
            if t_ms < 0:
                raise ValueError(f"motif event t_ms must be >= 0, got {t_ms}")
            if not 0 <= pitch <= 127:
                raise ValueError(f"motif event pitch out of range: {pitch}")
            velocity = int(ev.get("velocity", 100))
            addr = pitch_to_addr.get(pitch)
            if addr is None:
                skipped.append({"t_ms": t_ms, "pitch": pitch, "reason": "unmapped"})
                continue
            resolved.append({
                "t_ms": t_ms,
                "address": addr,
                "current_ma": self.velocity_to_current(velocity),
                "pitch": pitch,
                "velocity": velocity,
            })
        resolved.sort(key=lambda e: e["t_ms"])
        duration_ms = resolved[-1]["t_ms"] if resolved else 0

        motif_id = uuid.uuid4().hex[:8]
        with self._play_lock:
            self._stop_and_join()
            with self._control_lock:
                self._stop_event = threading.Event()
                self._current_id = motif_id
                self._current_name = name
                self._thread = threading.Thread(
                    target=self._run,
                    args=(resolved, self._stop_event, motif_id),
                    name=f"motif-{motif_id}",
                    daemon=True,
                )
                self._thread.start()
        return {
            "id": motif_id,
            "name": name,
            "duration_ms": duration_ms,
            "scheduled": len(resolved),
            "skipped": skipped,
        }

    def cancel(self) -> None:
        with self._play_lock:
            self._stop_and_join()

    def _stop_and_join(self) -> None:
        with self._control_lock:
            self._stop_event.set()
            thread = self._thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            # 2s is generous: the worker only blocks on serial round-trips
            # (~1-2 ms each) and on its own stop_event.wait().
            thread.join(timeout=2.0)
        with self._control_lock:
            # Clear the "currently playing" labels only if the thread we just
            # joined is still the recorded one (a new play() could have raced
            # in here and installed its own thread).
            if self._thread is thread:
                self._thread = None
                self._current_id = None
                self._current_name = None

    def _run(self, events: list[dict], stop: threading.Event, motif_id: str) -> None:
        t0 = time.monotonic()
        for ev in events:
            comp_ms = self.bridge.latency.compensation_ms(ev["address"], ev["current_ma"])
            target = t0 + (ev["t_ms"] - comp_ms) / 1000.0
            remaining = target - time.monotonic()
            if remaining > 0:
                # Event.wait returns True when set, False on timeout. Either
                # way we re-check stop before striking — a wait that fell
                # through to timeout might still race with a concurrent stop.
                if stop.wait(remaining):
                    return
            if stop.is_set():
                return
            try:
                self.bridge.strike(ev["address"], ev["current_ma"])
            except Exception:
                # One bad strike shouldn't tank the whole motif. Log and move
                # on — the next event has its own scheduled time.
                traceback.print_exc()


class Handler(BaseHTTPRequestHandler):
    bridge: Bridge | None = None
    # Optional on-disk MIDI library. When set by --library-dir, /api/library
    # lists .mid/.midi files in this folder (flat, no recursion) and
    # /api/library/<name> serves the raw bytes.
    library_dir: Path | None = None

    def log_message(self, fmt, *args):
        # Quieter than the default per-request stderr spam.
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    # ---- helpers ---------------------------------------------------------

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _json(self, code: int, payload) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        n = int(self.headers.get("Content-Length", "0") or "0")
        if n <= 0:
            return {}
        raw = self.rfile.read(n)
        if not raw:
            return {}
        return json.loads(raw)

    # ---- methods ---------------------------------------------------------

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(302)
            self.send_header("Location", "/player.html")
            self._cors()
            self.end_headers()
            return

        if self.path in ("/player.html", "/looper.html"):
            html_path = PLAYER_HTML if self.path == "/player.html" else LOOPER_HTML
            try:
                data = html_path.read_bytes()
            except FileNotFoundError:
                self._json(500, {"error": f"Missing {html_path}"})
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self._cors()
            self.end_headers()
            self.wfile.write(data)
            return

        if self.path == "/api/status":
            try:
                self._json(200, self.bridge.status())
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        if self.path == "/api/mapping":
            try:
                self._json(200, {"mapping": self.bridge.load_mapping()})
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        if self.path == "/api/pitches":
            try:
                self._json(200, {"pitches": self.bridge.pitches()})
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        if self.path.startswith("/api/strike-timing"):
            try:
                # /api/strike-timing?address=N triggers a fresh poll;
                # /api/strike-timing alone returns the cached map.
                from urllib.parse import parse_qs
                qs = parse_qs(urlparse(self.path).query)
                if "address" in qs:
                    addr = int(qs["address"][0])
                    self._json(200, self.bridge.query_strike_timing(addr))
                else:
                    self._json(200, {
                        "slots": [
                            self.bridge.strike_timing.get(a)
                            for a in range(self.bridge.count)
                        ]
                    })
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        if self.path == "/api/library":
            try:
                lib = Handler.library_dir
                if lib is None:
                    self._json(200, {"configured": False, "files": []})
                    return
                files = []
                for p in sorted(lib.iterdir(), key=lambda x: x.name.lower()):
                    if p.is_file() and p.suffix.lower() in (".mid", ".midi"):
                        try:
                            size = p.stat().st_size
                        except OSError:
                            continue
                        files.append({"name": p.name, "size": size})
                self._json(200, {"configured": True, "files": files})
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        if self.path.startswith("/api/library/"):
            try:
                lib = Handler.library_dir
                if lib is None:
                    self._json(404, {"error": "No library configured"})
                    return
                name = unquote(urlparse(self.path).path[len("/api/library/"):])
                # Reject anything that tries to escape the library dir.
                # Path.name strips directory parts; comparing to the raw
                # request catches "../foo" and "sub/foo" before we touch
                # the filesystem.
                if not name or name != Path(name).name:
                    self._json(400, {"error": "Invalid filename"})
                    return
                if Path(name).suffix.lower() not in (".mid", ".midi"):
                    self._json(400, {"error": "Not a MIDI file"})
                    return
                target = (lib / name).resolve()
                try:
                    target.relative_to(lib.resolve())
                except ValueError:
                    self._json(400, {"error": "Invalid path"})
                    return
                if not target.is_file():
                    self._json(404, {"error": "Not found"})
                    return
                data = target.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "audio/midi")
                self.send_header("Content-Length", str(len(data)))
                self._cors()
                self.end_headers()
                self.wfile.write(data)
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        self.send_error(404, "Not Found")

    def do_POST(self):
        try:
            if self.path == "/api/enumerate":
                count = self.bridge.enumerate()
                self._json(200, {"count": count})
                return

            if self.path == "/api/home":
                data = self._read_json()
                count = self.bridge.count
                addrs = data.get("addresses")
                if not addrs:
                    addrs = list(range(count))
                addrs = [int(a) for a in addrs]
                self.bridge.home(addrs, timeout_ms=int(data.get("timeout_ms", 8000)))
                self._json(200, {"ok": True, "addresses": addrs})
                return

            if self.path == "/api/strike":
                data = self._read_json()
                addr = int(data["address"])
                cur = int(data["current_ma"])
                self._json(200, self.bridge.strike(addr, cur))
                return

            if self.path == "/api/strikes":
                data = self._read_json()
                items = data.get("strikes") or []
                self._json(200, {"results": self.bridge.strikes(items)})
                return

            if self.path == "/api/strike-param":
                data = self._read_json()
                addr = int(data["address"])
                name = str(data["param"])
                param_id = STRIKE_PARAM_BY_NAME.get(name)
                if param_id is None:
                    self._json(400, {"error": f"unknown strike param '{name}'"})
                    return
                value = int(data["value"])
                if value < -32768 or value > 32767:
                    self._json(400, {"error": "value out of int16 range"})
                    return
                self._json(200, self.bridge.set_strike_param(addr, param_id, value))
                return

            if self.path == "/api/cancel":
                self.bridge.cancel_all()
                self._json(200, {"ok": True})
                return

            if self.path == "/api/stop":
                data = self._read_json()
                count = self.bridge.count
                addrs = data.get("addresses")
                if not addrs:
                    addrs = list(range(count))
                results = self.bridge.stop([int(a) for a in addrs])
                self._json(200, {"results": results})
                return

            if self.path == "/api/mapping":
                data = self._read_json()
                mapping = data.get("mapping")
                if not isinstance(mapping, list):
                    self._json(400, {"error": "expected {'mapping': [...]}"})
                    return
                try:
                    cleaned = self.bridge.save_mapping(mapping)
                except ValueError as exc:
                    self._json(400, {"error": str(exc)})
                    return
                self._json(200, {"ok": True, "mapping": cleaned})
                return

            if self.path == "/api/motif":
                data = self._read_json()
                events = data.get("events")
                if not isinstance(events, list):
                    self._json(400, {"error": "expected {'events': [...]}"})
                    return
                name = data.get("name")
                if name is not None and not isinstance(name, str):
                    self._json(400, {"error": "'name' must be a string"})
                    return
                try:
                    result = self.bridge.motif_player.play(events, name=name)
                except ValueError as exc:
                    self._json(400, {"error": str(exc)})
                    return
                self._json(200, {"ok": True, **result})
                return

            if self.path == "/api/save-settings":
                data = self._read_json()
                count = self.bridge.count
                addrs = data.get("addresses")
                if not addrs:
                    addrs = list(range(count))
                results = self.bridge.save_settings([int(a) for a in addrs])
                self._json(200, {"results": results})
                return

            if self.path.startswith("/api/library/"):
                from urllib.parse import parse_qs
                lib = Handler.library_dir
                if lib is None:
                    self._json(404, {"error": "No library configured"})
                    return
                parsed = urlparse(self.path)
                name = unquote(parsed.path[len("/api/library/"):])
                qs = parse_qs(parsed.query)
                overwrite = (qs.get("overwrite", [""])[0] or "").lower() in ("1", "true", "yes")
                if not name or name != Path(name).name:
                    self._json(400, {"error": "Invalid filename"})
                    return
                if Path(name).suffix.lower() not in (".mid", ".midi"):
                    self._json(400, {"error": "Not a MIDI file"})
                    return
                target = (lib / name).resolve()
                try:
                    target.relative_to(lib.resolve())
                except ValueError:
                    self._json(400, {"error": "Invalid path"})
                    return
                existed = target.exists()
                if existed and not overwrite:
                    self._json(409, {"error": "File exists", "name": name})
                    return
                n = int(self.headers.get("Content-Length", "0") or "0")
                if n <= 0:
                    self._json(400, {"error": "Empty body"})
                    return
                # Soft cap. A normal MIDI file is well under a megabyte; this
                # is just here to keep a misuse from chewing up disk.
                MAX_BYTES = 8 * 1024 * 1024
                if n > MAX_BYTES:
                    self._json(413, {"error": "File too large"})
                    return
                body = self.rfile.read(n)
                # Sanity check the header so we don't litter the library with
                # random uploads that aren't even MIDI.
                if not body.startswith(b"MThd"):
                    self._json(400, {"error": "Not a MIDI file (missing MThd header)"})
                    return
                target.write_bytes(body)
                self._json(201, {"ok": True, "name": name, "size": len(body),
                                 "overwritten": existed})
                return

            self.send_error(404, "Not Found")
        except KeyError as exc:
            self._json(400, {"error": f"missing field {exc}"})
        except json.JSONDecodeError as exc:
            self._json(400, {"error": f"bad json: {exc}"})
        except Exception as exc:
            traceback.print_exc()
            self._json(500, {"error": str(exc)})


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="Serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    ap.add_argument("--host", default="127.0.0.1", help="HTTP bind host (default: 127.0.0.1)")
    ap.add_argument("--http-port", type=int, default=8765, help="HTTP bind port (default: 8765)")
    ap.add_argument("--timeout-ms", type=int, default=1000,
                    help="Per-frame RX timeout in ms (default: 1000). Bump higher if you see"
                         " transient 'timeout waiting for frame' errors during homing.")
    ap.add_argument("--trace", action="store_true", help="Print raw ring TX/RX frames")
    ap.add_argument("--library-dir", default=None,
                    help="Folder of .mid/.midi files exposed at /api/library. "
                         "Flat listing only — no recursion.")
    ap.add_argument("--motif-current-ma", type=int, default=800,
                    help="Master strike current used by /api/motif when converting "
                         "MIDI velocity to current_ma (default: 800). Matches the "
                         "browser player's default master.")
    ap.add_argument("--motif-vel-floor", type=float, default=0.5,
                    help="Fraction of master current applied at velocity=1 "
                         "(default: 0.5). 1.0 = velocity ignored; 0.0 = soft notes "
                         "barely strike. Matches the browser's DEFAULT_VEL_FLOOR.")
    args = ap.parse_args(argv)

    library_dir: Path | None = None
    if args.library_dir:
        library_dir = Path(args.library_dir).expanduser().resolve()
        if not library_dir.is_dir():
            print(f"--library-dir {library_dir} is not a directory", file=sys.stderr)
            return 1

    port = args.port or auto_detect_port()
    if not port:
        print("No serial port found; pass -p/--port", file=sys.stderr)
        return 1

    print(f"Opening {port} at {args.baud} baud (rx timeout {args.timeout_ms} ms)")
    bridge = Bridge(port=port, baud=args.baud, timeout_ms=args.timeout_ms, trace=args.trace)
    bridge.motif_player = MotifPlayer(
        bridge,
        master_current_ma=args.motif_current_ma,
        vel_floor=args.motif_vel_floor,
    )
    try:
        count = bridge.enumerate()
        print(f"Enumerated {count} device(s) on the ring")
    except Exception as exc:
        print(f"Enumeration failed: {exc}", file=sys.stderr)
        bridge.close()
        return 2

    Handler.bridge = bridge
    Handler.library_dir = library_dir
    server = ThreadingHTTPServer((args.host, args.http_port), Handler)
    url = f"http://{args.host}:{args.http_port}/"
    print(f"Serving {PLAYER_HTML.name} at {url}")
    print(f"Looper UI available at {url}looper.html")
    if library_dir is not None:
        print(f"MIDI library: {library_dir}")
    print(f"Motif playback: {args.motif_current_ma} mA master, "
          f"velocity floor {args.motif_vel_floor:.2f}")
    print("Open that URL in Chrome to play.  Ctrl-C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down")
    finally:
        server.server_close()
        if bridge.motif_player is not None:
            bridge.motif_player.cancel()
        bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
