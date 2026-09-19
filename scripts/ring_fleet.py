"""Host-only routing of fixed global slots to independent serial rings."""

from __future__ import annotations

import hashlib
import json
import re
import threading
from concurrent.futures import ThreadPoolExecutor
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path

from ring_bus import MAX_DEVICES


@dataclass(frozen=True)
class RingSpec:
    name: str
    port: str
    count: int


def parse_ring_specs(values: list[str]) -> list[RingSpec]:
    specs = []
    for value in values:
        try:
            name, endpoint = value.split("=", 1)
            port, count = endpoint.rsplit(",", 1)
            spec = RingSpec(name.strip(), port.strip(), int(count))
        except ValueError as exc:
            raise ValueError("use --ring NAME=PORT,COUNT (for example handpan=COM7,14)") from exc
        if not re.fullmatch(r"[A-Za-z0-9_-]+", spec.name) or not spec.port:
            raise ValueError("ring names must contain only letters, digits, _ or -; port is required")
        if not 1 <= spec.count <= MAX_DEVICES:
            raise ValueError(f"ring {spec.name}: count must be 1..{MAX_DEVICES}")
        if any(s.name.casefold() == spec.name.casefold() for s in specs):
            raise ValueError(f"duplicate ring name: {spec.name}")
        if any(s.port.casefold() == spec.port.casefold() for s in specs):
            raise ValueError(f"duplicate serial port: {spec.port}")
        specs.append(spec)
    return specs


class FleetHealth:
    def __init__(self, fleet):
        self.fleet = fleet

    def reset(self):
        for ring in self.fleet.rings:
            ring.health.reset()

    def snapshot(self):
        snapshots = [ring.health.snapshot() for ring in self.fleet.rings]
        return {
            "since_s": max((s["since_s"] for s in snapshots), default=0),
            "addresses": {
                str(offset + int(address)): bag
                for offset, snap in zip(self.fleet.offsets, snapshots)
                for address, bag in snap["addresses"].items()
            },
        }


class FleetLatency:
    def __init__(self, fleet):
        self.fleet = fleet

    def compensation_ms(self, address, current_ma):
        lane, local = self.fleet.locate(address)
        return self.fleet.rings[lane].latency.compensation_ms(local, current_ma)


class RingFleet:
    """Bridge-compatible facade. Addresses remain fixed even on discovery failure.

    A ring's expected size is part of its identity. A missing board must never
    shift every later ring's pitch mapping onto different actuators.
    """

    firmware_update_supported = False

    def __init__(self, specs, bridge_factory, *, baud, timeout_ms=1000,
                 trace=False, mapping_file: Path):
        if not specs:
            raise ValueError("at least one ring is required")
        self.specs = specs
        self.offsets = []
        self.count = 0
        self.rings = []
        self.maintenance = threading.Event()
        self._player = None
        self._mapping_lock = threading.Lock()
        self.mapping_file = mapping_file
        try:
            for spec in specs:
                self.offsets.append(self.count)
                self.count += spec.count
                ring = bridge_factory(port=spec.port, baud=baud,
                                      timeout_ms=timeout_ms, trace=trace)
                ring.expected_count = spec.count
                ring.maintenance = self.maintenance
                self.rings.append(ring)
        except Exception:
            self.close()
            raise
        self.health = FleetHealth(self)
        self.latency = FleetLatency(self)

    @property
    def player(self):
        return self._player

    @player.setter
    def player(self, player):
        self._player = player
        for ring in self.rings:
            ring.player = player

    @property
    def strike_timing(self):
        return {offset + address: self._translate(value, lane)
                for lane, (offset, ring) in enumerate(zip(self.offsets, self.rings))
                for address, value in list(ring.strike_timing.items())}

    def locate(self, address):
        if not isinstance(address, int) or not 0 <= address < self.count:
            raise ValueError(f"slot {address!r} is outside the configured rings")
        for lane in reversed(range(len(self.offsets))):
            if address >= self.offsets[lane]:
                return lane, address - self.offsets[lane]
        raise ValueError("invalid slot")

    def playback_lane(self, address):
        return self.locate(address)[0]

    def validate_playback(self):
        for lane in range(len(self.rings)):
            self._ready(lane)

    def _ready(self, lane):
        if self.rings[lane].count != self.specs[lane].count:
            raise RuntimeError(f"ring {self.specs[lane].name} is not enumerated at its expected count")

    def _translate(self, value, lane):
        if isinstance(value, list):
            return [self._translate(item, lane) for item in value]
        if not isinstance(value, dict):
            return value
        result = {key: self._translate(item, lane) for key, item in value.items()}
        if "address" in result:
            result["local_address"] = result["address"]
            result["address"] += self.offsets[lane]
            result["ring"] = self.specs[lane].name
        return result

    def _parallel(self, fn):
        with ThreadPoolExecutor(max_workers=len(self.rings)) as pool:
            return list(pool.map(fn, range(len(self.rings))))

    def enumerate(self):
        # The player lock excludes new schedules until discovery is complete.
        if self.player is None:
            return self._enumerate()
        with self.player._play_lock:
            self.player._stop_and_join()
            return self._enumerate()

    def _enumerate(self):
        self.maintenance.set()
        try:
            def discover(lane):
                try:
                    self.rings[lane].enumerate()
                    self._ready(lane)
                    return None
                except Exception as exc:
                    self.rings[lane].reset_status_cache(0)
                    return f"{self.specs[lane].name}: {exc}"
            errors = [error for error in self._parallel(discover) if error]
            if errors:
                raise RuntimeError("; ".join(errors))
            return self.count
        finally:
            self.maintenance.clear()

    def status(self):
        snapshots = self._parallel(lambda lane: self.rings[lane].status())
        slots, rings = [], []
        for lane, (spec, snap) in enumerate(zip(self.specs, snapshots)):
            ready = snap["count"] == spec.count
            rings.append({"name": spec.name, "port": spec.port,
                          "offset": self.offsets[lane], "count": snap["count"],
                          "expected_count": spec.count, "ready": ready})
            for local in range(spec.count):
                slot = snap["slots"][local] if ready else {
                    "homed": False, "status_cached": True,
                    "status_error": "ring must be enumerated at its expected count",
                }
                slots.append({**self._translate(slot, lane), "ring": spec.name,
                              "port": spec.port, "local_address": local,
                              "address": self.offsets[lane] + local})
        return {"count": self.count, "homed": [s["homed"] for s in slots],
                "slots": slots, "rings": rings,
                "status_deferred": any(s.get("status_cached") for s in slots)}

    def probe_bus(self):
        self._parallel(lambda lane: self.rings[lane].probe_bus())
        return self.health.snapshot()

    def _one(self, method, address, *args):
        lane, local = self.locate(address)
        ring = self.rings[lane]
        # The Bridge lock is intentionally acquired by the called method.
        self._ready(lane)
        return self._translate(getattr(ring, method)(local, *args), lane)

    def strike(self, address, current_ma):
        return self._one("strike", address, current_ma)

    def query_config(self, address):
        return self._one("query_config", address)

    def query_strike_timing(self, address):
        return self._one("query_strike_timing", address)

    def set_strike_param(self, address, param_id, value):
        return self._one("set_strike_param", address, param_id, value)

    def _many(self, method, addresses, *args, **kwargs):
        groups = [[] for _ in self.rings]
        for address in addresses:
            lane, local = self.locate(address)
            groups[lane].append(local)
        def invoke(lane):
            if not groups[lane]:
                return []
            try:
                self._ready(lane)
                result = getattr(self.rings[lane], method)(groups[lane], *args, **kwargs)
            except Exception as exc:
                result = [{"address": a, "accepted": False, "error": str(exc)}
                          for a in groups[lane]]
            return self._translate(result, lane)
        return [item for group in self._parallel(invoke) for item in group]

    def home(self, addresses, timeout_ms=8000):
        return self._many("home", addresses, timeout_ms=timeout_ms)

    def recover(self, addresses):
        return self._many("recover", addresses)

    def stop(self, addresses):
        with self.player._play_lock if self.player is not None else nullcontext():
            if self.player is not None:
                self.player._stop_and_join()
            return self._many("stop", addresses)

    def save_settings(self, addresses):
        return self._many("save_settings", addresses)

    def apply_config(self, addresses, group, values):
        return self._many("apply_config", addresses, group, values)

    def _strikes(self, method, items):
        groups = [[] for _ in self.rings]
        for item in items:
            lane, local = self.locate(int(item["address"]))
            groups[lane].append({**item, "address": local})
        def invoke(lane):
            if not groups[lane]:
                return []
            self._ready(lane)
            return self._translate(getattr(self.rings[lane], method)(groups[lane]), lane)
        # Playback already owns one worker per ring: avoid extra thread hops.
        active = [lane for lane, group in enumerate(groups) if group]
        if len(active) == 1:
            return invoke(active[0])
        results = [iter(group) for group in self._parallel(invoke)]
        return [next(results[self.playback_lane(int(item["address"]))]) for item in items]

    def strikes(self, items):
        return self._strikes("strikes", items)

    def strike_chord(self, items):
        return self._strikes("strike_chord", items)

    def cancel_all(self):
        with self.player._play_lock if self.player is not None else nullcontext():
            if self.player is not None:
                self.player._stop_and_join()
            groups = self._parallel(lambda lane: self.rings[lane].cancel_all(stop_playback=False))
        results = [item for lane, group in enumerate(groups)
                   for item in self._translate(group["results"], lane)]
        # A ring with failed discovery still has configured slots to report.
        for lane, ring in enumerate(self.rings):
            if ring.count != self.specs[lane].count:
                results.extend({"address": self.offsets[lane] + a,
                                "accepted": False, "error": "ring unavailable"}
                               for a in range(self.specs[lane].count))
        failed = [r["address"] for r in results if not r.get("accepted")]
        return {"ok": not failed, "failed": failed, "results": results}

    def _layout(self):
        return [vars(spec) for spec in self.specs]

    @property
    def mapping_context(self):
        return hashlib.sha256(json.dumps(self._layout(), sort_keys=True).encode()).hexdigest()[:16]

    def load_mapping(self):
        try:
            obj = json.loads(self.mapping_file.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            return [None] * self.count
        if not isinstance(obj, dict) or obj.get("rings") != self._layout():
            return [None] * self.count
        mapping = obj.get("mapping")
        if not isinstance(mapping, list) or len(mapping) != self.count:
            return [None] * self.count
        return [v if type(v) is int and 0 <= v <= 127 else None for v in mapping]

    def save_mapping(self, mapping):
        if len(mapping) != self.count:
            raise ValueError(f"mapping needs exactly {self.count} slots")
        if any(v is not None and (type(v) is not int or not 0 <= v <= 127) for v in mapping):
            raise ValueError("mapping entries must be null or MIDI pitches 0..127")
        with self._mapping_lock:
            tmp = self.mapping_file.with_suffix(".json.tmp")
            tmp.write_text(json.dumps({"rings": self._layout(), "mapping": mapping}, indent=2),
                           encoding="utf-8")
            tmp.replace(self.mapping_file)
        return list(mapping)

    def pitches(self):
        # Lazy import avoids a module cycle and shares the existing note naming.
        from ring_midi_server import midi_name
        slots = self.status()["slots"]
        return sorted([
            {"pitch": pitch, "name": midi_name(pitch), "slot": address,
             "homed": slots[address]["homed"], "ring": slots[address]["ring"],
             "local_address": slots[address]["local_address"]}
            for address, pitch in enumerate(self.load_mapping()) if pitch is not None
        ], key=lambda item: item["pitch"])

    def close(self):
        for ring in self.rings:
            ring.close()
