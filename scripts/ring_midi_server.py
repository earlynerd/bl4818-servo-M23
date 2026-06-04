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
GET  /api/play                -> {"playing", "id", "scheduled", "cursor", "duration_ms",
                                  "elapsed_ms", "remaining_ms", "master_scale", "muted", ...};
                                 snapshot of the running playback (whatever started it).
                                 Anything stashed in metadata at play-time (name,
                                 master_current_ma, vel_floor, schedule_master_ma) is
                                 spread into the response.
GET  /api/library             -> {"configured": bool, "files": [{"name", "size"}, ...]}
GET  /api/library/<name>      -> raw .mid bytes from the configured --library-dir
POST /api/library/<name>      -> writes raw .mid bytes to --library-dir; 409 on
                                 collision unless ?overwrite=1
POST /api/enumerate           -> re-enumerate; returns same shape as /api/status
POST /api/home                -> {"addresses": [int]?}; homes them, returns ok/error
POST /api/strike              -> {"address": int, "current_ma": int}; ACK + last-strike timing
POST /api/strikes             -> {"strikes": [{"address","current_ma"}, ...]}; batch
POST /api/strike-param        -> {"address": int, "param": str, "value": int}; ACK
POST /api/cancel              -> cancels all active strikes (panic stop, also stops playback)
POST /api/stop                -> {"addresses": [int]?}; motor_stop on each (disables PWM)
POST /api/save-settings       -> {"addresses": [int]?}; persists strike params to NVM
POST /api/mapping             -> {"mapping": [int|null, ...]}; persists slot->pitch map to disk
POST /api/play                -> Start scheduled playback. Accepts two event shapes:
                                  (a) canonical (browser path):
                                      {"events": [{"t_ms","address","nominal_current_ma"}, ...],
                                       "master_ma": int?}
                                  (b) pitch-style (chime/motif integrations):
                                      {"events": [{"t_ms","pitch","velocity"}, ...],
                                       "name": str?, "master_current_ma": int?,
                                       "vel_floor": float?}
                                 In (b) the server resolves pitch->address via the persisted
                                 mapping and computes current = master * (floor + (1-floor)*v/127).
                                 Pitches not in /api/mapping are returned in "skipped".
                                 Preempts any in-flight playback.
POST /api/play/stop           -> stop in-flight playback (does NOT disable motors).
POST /api/play/update         -> {"master_scale": float?, "master_ma": int?,
                                  "schedule_master_ma": int?, "muted": [int]?};
                                 live tweaks. master_scale wins if set; otherwise
                                 master_ma/schedule_master_ma form the ratio. Applied
                                 to the next strike.

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
    RingCRCError,
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


class BusHealth:
    """Per-address ring-transaction health: round-trip latency distribution
    plus drop / CRC / error counts.

    Fed by the Bridge on every transaction that actually touches the bus
    (strikes and the per-address query_strike used by status/pitches/probe),
    so it reflects real link behavior whether or not a song is playing. The
    counts are cumulative since the last reset() — the browser's bus-health
    panel renders mean / stddev / max latency and a drop-rate from these, and
    the user resets when they wiggle a connector to get a clean before/after.

    Has its own lock so the Bridge can update it from inside the bus lock and
    the HTTP thread can snapshot it without contending on the bus lock. Never
    acquires the bus lock, so there's no lock-ordering cycle."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._addr: dict[int, dict] = {}
        self._since = time.monotonic()

    @staticmethod
    def _new_bag() -> dict:
        # rtt_* accumulate only over successful (ok) transactions, in
        # microseconds; the drop-rate denominator is `n` (every attempt).
        return {
            "n": 0, "ok": 0, "timeouts": 0, "crc_errors": 0, "errors": 0,
            "rtt_sum_us": 0.0, "rtt_sumsq_us": 0.0,
            "rtt_max_us": 0.0, "rtt_last_us": 0.0,
            "last_error": None,
        }

    def record_ok(self, address: int, rtt_us: float) -> None:
        with self._lock:
            bag = self._addr.get(address) or self._addr.setdefault(address, self._new_bag())
            bag["n"] += 1
            bag["ok"] += 1
            bag["rtt_sum_us"] += rtt_us
            bag["rtt_sumsq_us"] += rtt_us * rtt_us
            bag["rtt_last_us"] = rtt_us
            if rtt_us > bag["rtt_max_us"]:
                bag["rtt_max_us"] = rtt_us

    def record_fail(self, address: int, kind: str, error: str) -> None:
        # kind is "timeouts", "crc_errors", or "errors".
        with self._lock:
            bag = self._addr.get(address) or self._addr.setdefault(address, self._new_bag())
            bag["n"] += 1
            bag[kind] = bag.get(kind, 0) + 1
            bag["last_error"] = error

    def reset(self) -> None:
        with self._lock:
            self._addr.clear()
            self._since = time.monotonic()

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "since_s": round(time.monotonic() - self._since, 1),
                "addresses": {str(a): dict(b) for a, b in sorted(self._addr.items())},
            }


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
        # Per-address bus link health (RTT distribution + drop/CRC/error
        # counts). Fed by strike() and the per-address query_strike path.
        self.health = BusHealth()
        # Playback scheduler — server-side dispatch for both chime motifs
        # (HTTP integrations) and browser song playback. Lazily attached
        # after construction so it can hold a reference back to this Bridge
        # for self.strike() / self.latency.
        self.player: "Player | None" = None

    def enumerate(self) -> int:
        with self.lock:
            self.count = self.client.enumerate()
        return self.count

    def _record_bus_exc(self, address: int, exc: Exception) -> None:
        """Classify a failed bus transaction into the BusHealth buckets.
        Timeout = the device never answered (the 'packet disappeared' case);
        CRC = a frame came back mangled (bus noise / marginal wiring); other =
        anything else (framing, serial layer)."""
        if isinstance(exc, RingTimeout):
            self.health.record_fail(address, "timeouts", str(exc))
        elif isinstance(exc, RingCRCError):
            self.health.record_fail(address, "crc_errors", str(exc))
        else:
            self.health.record_fail(address, "errors", str(exc))

    def _query_strike_timed(self, address: int):
        """query_strike with BusHealth accounting. Caller must hold self.lock.
        Records the round-trip on success and the failure kind on exception,
        then re-raises so existing error handling is unchanged."""
        t0 = time.monotonic()
        try:
            s = self.client.query_strike(address)
        except Exception as exc:
            self._record_bus_exc(address, exc)
            raise
        self.health.record_ok(address, (time.monotonic() - t0) * 1_000_000)
        return s

    def status(self) -> dict:
        with self.lock:
            count = self.count
            homed: list[bool] = []
            slots: list[dict] = []
            for addr in range(count):
                try:
                    s = self._query_strike_timed(addr)
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

    def probe_bus(self) -> dict:
        """Run one query_strike round across every enumerated address purely to
        exercise the link and feed BusHealth, then return the health snapshot.
        Used by the browser's bus-health monitor to keep a steady per-address
        sampling cadence while the ring is otherwise idle (organic traffic —
        strikes, status polls — feeds the same counters; this just guarantees
        coverage when nothing else is happening). Failures are swallowed: the
        point is to *measure* drops, not propagate them.

        Takes the ring lock per-address rather than around the whole sweep, so
        a concurrent strike or keyboard tap only ever waits behind a single
        query — not the full round, which matters on a ring with many devices."""
        for addr in range(self.count):
            with self.lock:
                try:
                    self._query_strike_timed(addr)
                except Exception:
                    pass
        return self.health.snapshot()

    def strike(self, address: int, current_ma: int) -> dict:
        # ACK_TIMED costs 12 extra payload bytes (~480 us at 250 kbaud) and
        # replaces the otherwise-needed query_strike round trip. Net: less
        # ring traffic per strike, plus the host gets timing for free.
        # Stamps below let the browser decompose total beat error into
        # scheduler jitter / network / lock / serial-RTT / impact.
        t_recv = time.monotonic()
        prev_current = self.latency.record_sent(address, current_ma)
        with self.lock:
            t_lock = time.monotonic()
            try:
                reply = self.client.strike(
                    address, current_ma, reply_mode=REPLY_MODE_ACK_TIMED
                )
            except Exception as exc:
                # A dropped/mangled strike reply is exactly the bus trouble the
                # health panel exists to surface — record it before propagating
                # (the playback worker catches and skips, /api/strike → 500).
                self._record_bus_exc(address, exc)
                raise
            t_post_ack = time.monotonic()
        self.health.record_ok(address, (t_post_ack - t_lock) * 1_000_000)
        result = self._ack_to_dict(address, reply, cache_metrics=True)
        self._fold_metrics_into_ema(address, prev_current, result.get("metrics"))
        result["server_timing_us"] = {
            "lock_wait": int((t_lock - t_recv) * 1_000_000),
            "serial_rtt": int((t_post_ack - t_lock) * 1_000_000),
            "handler_total": int((time.monotonic() - t_recv) * 1_000_000),
        }
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
        # Stop the playback worker first so it can't fire a new strike
        # between our cancel calls. player.cancel() blocks on the worker's
        # join, which only takes one strike RTT to return (the worker
        # checks the stop event between strikes and at every wait).
        if self.player is not None:
            self.player.cancel()
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
                    s = self._query_strike_timed(addr)
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


def velocity_to_current(velocity: int, master_ma: int, vel_floor: float) -> int:
    """Same curve as the browser: master * (floor + (1-floor) * v/127),
    clamped to the firmware-supported current range. Pure function so both
    the /api/motif handler and any tests can reuse it without instantiating
    a player."""
    v = max(0, min(127, int(velocity)))
    scale = vel_floor + (1 - vel_floor) * (v / 127.0)
    ma = int(round(master_ma * scale))
    return max(STRIKE_MA_MIN, min(STRIKE_MA_MAX, ma))


def motif_events_to_canonical(
    events: list[dict],
    mapping: list,
    master_ma: int,
    vel_floor: float,
) -> tuple[list[dict], list[dict]]:
    """Translate motif-style events ({t_ms, pitch, velocity}) into the
    canonical Player shape ({t_ms, address, nominal_current_ma}). Returns
    (resolved, skipped). Pitches not in the mapping are reported in
    `skipped` with a reason — the motif HTTP layer echoes them back so
    callers can see what was dropped."""
    pitch_to_addr: dict[int, int] = {}
    for addr, pitch in enumerate(mapping or []):
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
            "nominal_current_ma": velocity_to_current(velocity, master_ma, vel_floor),
        })
    return resolved, skipped


class Player:
    """Single playback engine: schedule of canonical events fired by one
    worker thread off the bridge's monotonic clock. Both the motif HTTP API
    (calendar/Slack-style notification chimes) and the browser song player
    route through this — they differ only in how they shape their input
    before calling play(), not in how dispatch works.

    Each new play() preempts whatever was running. Live tweaks via update()
    apply on the next strike: master_scale multiplies every event's nominal
    current, muted is a set of addresses to skip. Metadata passed to play()
    is opaque to the engine — it's stored and echoed in status(), so the
    motif endpoint can carry name/master_current_ma/vel_floor for its
    consumers without the engine knowing what those mean.
    """

    def __init__(self, bridge: Bridge) -> None:
        self.bridge = bridge
        # _play_lock serializes whole play()/cancel() calls so two POSTs
        # arriving at the same time can't leave two worker threads running.
        # _control_lock guards just the small mutable state read by the
        # worker and by update()/status() and is held only briefly.
        self._play_lock = threading.Lock()
        self._control_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        # Mutable per-play state. Reset on each play().
        self._master_scale = 1.0
        self._muted: set[int] = set()
        self._started_monotonic = 0.0
        self._cursor = 0
        self._total = 0
        self._duration_ms = 0
        self._id: str | None = None
        self._metadata: dict = {}
        # Per-slot residual stats (actual trigger_to_impact_ms minus what the
        # EMA predicted at strike time). Browser shows these in its popup
        # during server-dispatched playback, since strikeAsync — which used
        # to compute residual client-side — is bypassed on this code path.
        # Accumulated across plays; resets only on server restart.
        self.residual_stats: dict[int, dict] = {}
        # Per-strike server-side timing decomposition. lock_wait and
        # serial_rtt come from bridge.strike()'s server_timing_us; sched
        # jitter and network don't exist here (no setTimeout, no fetch),
        # so the browser leaves those rows showing pre-playback data.
        self.timing_stats: dict[str, dict] = {
            "lock_wait_ms": self._new_stats_bag(),
            "serial_rtt_ms": self._new_stats_bag(),
        }
        # ACK metrics describe the *previous* strike on each address, so
        # we hold the prediction we made for THIS strike here, and pair
        # it with the NEXT strike's returned metric.
        self._last_prediction_ms: dict[int, float] = {}

    @staticmethod
    def _new_stats_bag() -> dict:
        return {"last": 0.0, "sum": 0.0, "sumSq": 0.0, "n": 0, "maxAbs": 0.0}

    @staticmethod
    def _fold_stat(bag: dict, value: float) -> None:
        bag["last"] = value
        bag["sum"] += value
        bag["sumSq"] += value * value
        bag["n"] += 1
        a = abs(value)
        if a > bag["maxAbs"]:
            bag["maxAbs"] = a

    def status(self) -> dict:
        """Snapshot of what the worker is currently playing. `elapsed_ms` and
        `remaining_ms` are computed against the monotonic clock at status()
        time, so motif/chime callers can poll until `playing` is false
        without tracking duration themselves. Anything the caller stored in
        metadata at play() time (e.g. name, master_current_ma, vel_floor)
        gets spread into the response.

        Includes `residual_stats` (per-address) and `timing_stats` (global)
        so the browser popup can show meaningful numbers during
        server-dispatched playback. Stats accumulate across plays — only a
        server restart resets them."""
        with self._control_lock:
            thread = self._thread
            running = thread is not None and thread.is_alive()
            residual = {str(k): dict(v) for k, v in self.residual_stats.items()}
            timing = {k: dict(v) for k, v in self.timing_stats.items()}
            if not running:
                # Keep the documented motif keys (name/master_current_ma/
                # vel_floor) present-but-null so chime_demo and friends see
                # the same shape as the old MotifPlayer when idle.
                return {
                    "playing": False,
                    "id": None,
                    "name": None,
                    "scheduled": 0,
                    "cursor": 0,
                    "duration_ms": None,
                    "elapsed_ms": None,
                    "remaining_ms": None,
                    "master_scale": self._master_scale,
                    "muted": sorted(self._muted),
                    "master_current_ma": None,
                    "vel_floor": None,
                    "residual_stats": residual,
                    "timing_stats": timing,
                }
            elapsed_ms = int((time.monotonic() - self._started_monotonic) * 1000)
            elapsed_ms = max(0, min(self._duration_ms, elapsed_ms))
            return {
                "playing": True,
                "id": self._id,
                "scheduled": self._total,
                "cursor": self._cursor,
                "duration_ms": self._duration_ms,
                "elapsed_ms": elapsed_ms,
                "remaining_ms": self._duration_ms - elapsed_ms,
                "master_scale": self._master_scale,
                "muted": sorted(self._muted),
                "residual_stats": residual,
                "timing_stats": timing,
                **self._metadata,
            }

    def play(self, events: list[dict], **metadata) -> dict:
        """Start playback of canonical events: a list of
        {t_ms, address, nominal_current_ma}. Returns immediately with an id,
        scheduled count, duration, and an echo of metadata. Preempts any
        playback already running.

        metadata is opaque to the engine — callers attach whatever they want
        echoed in status() (e.g. the motif endpoint stores name +
        master_current_ma + vel_floor; the song endpoint stores
        schedule_master_ma). Stored verbatim under _metadata and spread back
        into status()."""
        resolved: list[dict] = []
        for ev in events:
            try:
                t_ms = int(ev["t_ms"])
                addr = int(ev["address"])
                nominal = int(ev["nominal_current_ma"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"bad player event {ev!r}: {exc}") from exc
            if t_ms < 0:
                raise ValueError(f"player event t_ms must be >= 0, got {t_ms}")
            resolved.append({
                "t_ms": t_ms,
                "address": addr,
                "nominal_current_ma": max(
                    STRIKE_MA_MIN, min(STRIKE_MA_MAX, nominal)
                ),
            })
        resolved.sort(key=lambda e: e["t_ms"])
        duration_ms = resolved[-1]["t_ms"] if resolved else 0
        play_id = uuid.uuid4().hex[:8]

        with self._play_lock:
            self._stop_and_join()
            with self._control_lock:
                self._stop_event = threading.Event()
                self._master_scale = 1.0
                self._muted = set()
                self._cursor = 0
                self._total = len(resolved)
                self._duration_ms = duration_ms
                self._id = play_id
                self._metadata = dict(metadata)
                self._started_monotonic = time.monotonic()
                self._thread = threading.Thread(
                    target=self._run,
                    args=(resolved, self._stop_event, play_id),
                    name=f"player-{play_id}",
                    daemon=True,
                )
                self._thread.start()
        return {
            "id": play_id,
            "scheduled": len(resolved),
            "duration_ms": duration_ms,
            **dict(metadata),
        }

    def update(self, master_scale: float | None = None,
               muted: list | None = None) -> dict:
        """Apply live tweaks to in-flight playback. Both args optional and
        independent. Returns the post-update state for echo/debug. Tweaks
        take effect on the next strike — the worker re-reads both values
        at the top of each iteration."""
        with self._control_lock:
            if master_scale is not None:
                # Cap at 10x so an errant slider can't silently clamp every
                # strike to STRIKE_MA_MAX. Zero is fine — silent playback.
                self._master_scale = max(0.0, min(10.0, float(master_scale)))
            if muted is not None:
                self._muted = {int(a) for a in muted}
            return {
                "master_scale": self._master_scale,
                "muted": sorted(self._muted),
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
            # Clear identity only if the thread we just joined is still the
            # recorded one (a new play() could have raced in here and
            # installed its own thread).
            if self._thread is thread:
                self._thread = None
                self._id = None

    def _run(self, events: list[dict], stop: threading.Event, play_id: str) -> None:
        t0 = self._started_monotonic
        for i, ev in enumerate(events):
            # Snapshot live state for this iteration. master_scale and muted
            # take effect on the next strike after the slider/UI changes.
            # Don't hold _control_lock across the wait — update() needs it.
            with self._control_lock:
                scale = self._master_scale
                muted = self._muted
                self._cursor = i

            addr = ev["address"]
            nominal = ev["nominal_current_ma"]
            live_ma = max(
                STRIKE_MA_MIN,
                min(STRIKE_MA_MAX, int(round(nominal * scale))),
            )
            # Comp uses the post-scale current so we look up the right bucket.
            comp_ms = self.bridge.latency.compensation_ms(addr, live_ma)
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
            if addr in muted or live_ma <= 0:
                # Muted strikes don't reach the bus, so no metric comes
                # back — wipe the stashed prediction so a future strike on
                # this slot doesn't get attributed across a gap.
                self._last_prediction_ms.pop(addr, None)
                continue
            # Stash our prediction BEFORE striking. The ACK we're about to
            # get carries the metric for the previous strike on this addr,
            # which we'll attribute to the prediction we stashed for THAT
            # one (held in prev_prediction).
            prev_prediction = self._last_prediction_ms.get(addr)
            self._last_prediction_ms[addr] = comp_ms
            try:
                result = self.bridge.strike(addr, live_ma)
            except Exception:
                # One bad strike shouldn't tank the whole playback.
                traceback.print_exc()
                continue
            # Fold server-side timing components.
            st = result.get("server_timing_us") or {}
            metrics = result.get("metrics") or {}
            impact = metrics.get("trigger_to_impact_ms")
            with self._control_lock:
                if "lock_wait" in st:
                    self._fold_stat(self.timing_stats["lock_wait_ms"],
                                    st["lock_wait"] / 1000.0)
                if "serial_rtt" in st:
                    self._fold_stat(self.timing_stats["serial_rtt_ms"],
                                    st["serial_rtt"] / 1000.0)
                if impact is not None and prev_prediction is not None:
                    bag = self.residual_stats.setdefault(
                        addr, self._new_stats_bag()
                    )
                    self._fold_stat(bag, float(impact) - prev_prediction)
        with self._control_lock:
            self._cursor = len(events)


class Handler(BaseHTTPRequestHandler):
    bridge: Bridge | None = None
    # Optional on-disk MIDI library. When set by --library-dir, /api/library
    # lists .mid/.midi files in this folder (flat, no recursion) and
    # /api/library/<name> serves the raw bytes.
    library_dir: Path | None = None
    # Defaults used by /api/play when the request is pitch-style and doesn't
    # carry its own master_current_ma / vel_floor. Set from the
    # --motif-current-ma / --motif-vel-floor CLI flags in main().
    default_master_ma: int = 800
    default_vel_floor: float = 0.5

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

        if self.path == "/api/play":
            try:
                self._json(200, self.bridge.player.status())
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        if self.path == "/api/bus-health":
            try:
                self._json(200, self.bridge.health.snapshot())
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

            if self.path == "/api/bus-health/probe":
                # Active per-address sampling round; returns the fresh snapshot.
                self._json(200, self.bridge.probe_bus())
                return

            if self.path == "/api/bus-health/reset":
                self.bridge.health.reset()
                self._json(200, {"ok": True, **self.bridge.health.snapshot()})
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

            if self.path == "/api/play":
                # Unified playback start. Two accepted event shapes:
                #
                #   canonical (browser): {t_ms, address, nominal_current_ma}
                #     Plus optional top-level "master_ma" so /api/play/update
                #     can scale relative to it.
                #
                #   pitch-style (motif/chime integrations):
                #     {t_ms, pitch, velocity}
                #     Plus optional top-level "name", "master_current_ma",
                #     "vel_floor". The server resolves pitch->address via the
                #     persisted mapping and computes current via the master *
                #     (floor + (1-floor)*v/127) curve. Defaults come from the
                #     --motif-current-ma / --motif-vel-floor CLI flags.
                #
                # Detection: presence of "pitch" on the first event picks the
                # pitch-style branch. An empty events list is allowed and
                # produces an immediately-done playback.
                data = self._read_json()
                events = data.get("events")
                if not isinstance(events, list):
                    self._json(400, {"error": "expected {'events': [...]}"})
                    return

                def _is_num(x):
                    return x is None or (not isinstance(x, bool) and isinstance(x, (int, float)))

                pitch_style = bool(events) and "pitch" in events[0]
                metadata: dict = {}
                try:
                    if pitch_style:
                        name = data.get("name")
                        if name is not None and not isinstance(name, str):
                            self._json(400, {"error": "'name' must be a string"})
                            return
                        master = data.get("master_current_ma", self.default_master_ma)
                        floor = data.get("vel_floor", self.default_vel_floor)
                        if not _is_num(master):
                            self._json(400, {"error": "'master_current_ma' must be a number"})
                            return
                        if not _is_num(floor):
                            self._json(400, {"error": "'vel_floor' must be a number"})
                            return
                        master = max(STRIKE_MA_MIN, min(STRIKE_MA_MAX, int(master)))
                        floor = max(0.0, min(1.0, float(floor)))
                        mapping = self.bridge.load_mapping() or []
                        canonical, skipped = motif_events_to_canonical(
                            events, mapping, master, floor
                        )
                        metadata = {
                            "name": name,
                            "master_current_ma": master,
                            "vel_floor": floor,
                            "skipped": skipped,
                        }
                    else:
                        master = data.get("master_ma")
                        if not _is_num(master):
                            self._json(400, {"error": "'master_ma' must be a number"})
                            return
                        canonical = events
                        if master is not None:
                            metadata["schedule_master_ma"] = int(master)
                    result = self.bridge.player.play(canonical, **metadata)
                except ValueError as exc:
                    self._json(400, {"error": str(exc)})
                    return
                self._json(200, {"ok": True, **result})
                return

            if self.path == "/api/play/stop":
                self.bridge.player.cancel()
                self._json(200, {"ok": True})
                return

            if self.path == "/api/play/update":
                data = self._read_json()
                # Accept master_scale directly OR master_ma + the original
                # schedule_master_ma so the browser doesn't have to divide.
                # master_scale wins if both are present.
                master_scale = data.get("master_scale")
                if master_scale is None:
                    new_master = data.get("master_ma")
                    orig_master = data.get("schedule_master_ma")
                    if new_master is not None and orig_master:
                        try:
                            master_scale = float(new_master) / float(orig_master)
                        except (TypeError, ValueError, ZeroDivisionError):
                            master_scale = None
                if master_scale is not None and (isinstance(master_scale, bool)
                                                 or not isinstance(master_scale, (int, float))):
                    self._json(400, {"error": "'master_scale' must be a number"})
                    return
                muted = data.get("muted")
                if muted is not None and not isinstance(muted, list):
                    self._json(400, {"error": "'muted' must be a list of addresses"})
                    return
                result = self.bridge.player.update(
                    master_scale=float(master_scale) if master_scale is not None else None,
                    muted=muted,
                )
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
                    help="Default master current for pitch-style /api/play requests "
                         "(default: 800). The request body can override per call. "
                         "Matches the browser player's default master.")
    ap.add_argument("--motif-vel-floor", type=float, default=0.5,
                    help="Default fraction of master current applied at velocity=1 "
                         "for pitch-style /api/play requests (default: 0.5). "
                         "1.0 = velocity ignored; 0.0 = soft notes barely strike. "
                         "Matches the browser's DEFAULT_VEL_FLOOR.")
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
    bridge.player = Player(bridge)
    try:
        count = bridge.enumerate()
        print(f"Enumerated {count} device(s) on the ring")
    except Exception as exc:
        print(f"Enumeration failed: {exc}", file=sys.stderr)
        bridge.close()
        return 2

    Handler.bridge = bridge
    Handler.library_dir = library_dir
    Handler.default_master_ma = max(
        STRIKE_MA_MIN, min(STRIKE_MA_MAX, int(args.motif_current_ma))
    )
    Handler.default_vel_floor = max(0.0, min(1.0, float(args.motif_vel_floor)))
    server = ThreadingHTTPServer((args.host, args.http_port), Handler)
    url = f"http://{args.host}:{args.http_port}/"
    print(f"Serving {PLAYER_HTML.name} at {url}")
    print(f"Looper UI available at {url}looper.html")
    if library_dir is not None:
        print(f"MIDI library: {library_dir}")
    print(f"Pitch-style /api/play defaults: {Handler.default_master_ma} mA master, "
          f"velocity floor {Handler.default_vel_floor:.2f}")
    print("Open that URL in Chrome to play.  Ctrl-C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down")
    finally:
        server.server_close()
        if bridge.player is not None:
            bridge.player.cancel()
        bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
