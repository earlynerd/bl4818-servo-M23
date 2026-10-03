"""Select existing drum profiles per ring while sharing player-level controls."""

import hashlib
import json
import threading

from instrument_profiles import clean_mapping, clean_settings


class FleetInstrumentProfiles:
    SHARED = ("current_ma", "vel_floor", "comp_enabled", "comp_default_ms")

    def __init__(self, path, legacy_path, layout, source):
        self.path = path
        self.legacy_path = legacy_path
        self.layout = layout
        self.source = source
        self.count = sum(ring["count"] for ring in layout)
        self.lock = threading.RLock()

    def _read(self):
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            mapping = [None] * self.count
            try:
                legacy = json.loads(self.legacy_path.read_text(encoding="utf-8"))
                if isinstance(legacy, dict) and legacy.get("rings") == self.layout:
                    mapping = self._check_mapping(legacy["mapping"])
            except (FileNotFoundError, ValueError, TypeError, KeyError):
                pass  # An absent or incompatible old layout starts unassigned.
            return {"rings": self.layout, "assignments": {}, "mapping": mapping,
                    "settings": {}, "fallbacks": {}}
        if not isinstance(data, dict) or data.get("rings") != self.layout:
            raise ValueError("saved ring settings do not match this layout")
        self._check_mapping(data["mapping"])
        return data

    def _write(self, data):
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        tmp.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        tmp.replace(self.path)

    def _check_mapping(self, mapping):
        cleaned = clean_mapping(mapping)
        if len(cleaned) != self.count:
            raise ValueError(f"mapping needs exactly {self.count} slots")
        return cleaned

    def _identity(self, data):
        raw = json.dumps([self.layout, data["assignments"]], sort_keys=True)
        return "rings-" + hashlib.sha256(raw.encode()).hexdigest()[:16]

    def _selected(self, data, source):
        offset = 0
        for ring in self.layout:
            profile_id = data["assignments"].get(ring["name"])
            profile = next((p for p in source["profiles"] if p["id"] == profile_id), None)
            if profile_id is not None and profile is None:
                raise ValueError(f"profile for ring {ring['name']} is missing; restore it before use")
            yield ring, offset, profile
            offset += ring["count"]

    def _snapshot(self, data, source):
        mapping = data["mapping"][:]
        settings = {"current_ma": 800, "vel_floor": 0.5, "comp_enabled": True,
                    "comp_default_ms": 50, **data["settings"]}
        trim = (settings.get("trim", []) + [1.0] * self.count)[:self.count]
        fallback, rings = {}, []
        for ring, offset, profile in self._selected(data, source):
            count = ring["count"]
            if profile is not None:
                mapping[offset:offset + count] = (profile["mapping"] + [None] * count)[:count]
                local = clean_settings(profile.get("settings", {}))
                trim[offset:offset + count] = (local.get("trim", []) + [1.0] * count)[:count]
                for pitch, slot in local.get("fallback", {}).items():
                    if slot < count:
                        fallback.setdefault(pitch, offset + slot)
            rings.append({**ring, "offset": offset,
                          "profile_id": profile["id"] if profile else None})
        active = self._identity(data)
        settings.update(trim=trim, fallback=data["fallbacks"].get(active, fallback))
        return {"active": active, "mapping": mapping, "settings": settings,
                "rings": rings, "profiles": [
                    {"id": p["id"], "name": p["name"], "count": len(p["mapping"])}
                    for p in source["profiles"]]}

    def snapshot(self):
        with self.lock, self.source.lock:
            return self._snapshot(self._read(), self.source._read())

    def mapping(self):
        return self.snapshot()["mapping"]

    def _expect(self, data, expected_id):
        if expected_id is not None and expected_id != self._identity(data):
            raise ValueError("selected instruments changed; reload before saving")

    def save_mapping(self, mapping, *, expected_id=None):
        mapping = self._check_mapping(mapping)
        with self.lock, self.source.lock:
            data, source = self._read(), self.source._read()
            self._expect(data, expected_id)
            changed = False
            for ring, offset, profile in self._selected(data, source):
                if profile is not None:
                    count = ring["count"]
                    profile["mapping"][:count] = mapping[offset:offset + count]
                    changed = True
            if changed:
                self.source._write(source)
            data["mapping"] = mapping
            self._write(data)
            return mapping

    def change(self, action, *, profile_id=None, ring=None, expected_id=None,
               settings=None, name=None, mapping=None):
        with self.lock, self.source.lock:
            data, source = self._read(), self.source._read()
            if action == "select_ring":
                self._expect(data, expected_id)
                if ring not in {r["name"] for r in self.layout}:
                    raise ValueError("unknown ring")
                selected = next((p for p in source["profiles"] if p["id"] == profile_id), None)
                if selected is None:
                    raise ValueError("instrument not found")
                if any(r != ring and pid == profile_id for r, pid in data["assignments"].items()):
                    raise ValueError("choose a different saved instrument for each ring")
                if not data["assignments"]:
                    # The first selected drum seeds the common controls once.
                    saved = clean_settings(selected.get("settings", {}))
                    data["settings"].update({key: saved[key] for key in self.SHARED if key in saved})
                data["assignments"][ring] = profile_id
            elif action == "update_settings":
                if profile_id is None:
                    raise ValueError("selected instruments token is required")
                self._expect(data, profile_id)
                cleaned = clean_settings(settings)
                if "trim" in cleaned and len(cleaned["trim"]) != self.count:
                    raise ValueError(f"trim needs exactly {self.count} entries")
                if "fallback" in cleaned and any(slot >= self.count for slot in cleaned["fallback"].values()):
                    raise ValueError("fallback targets an unavailable slot")
                # Pitch and trim changes belong to the selected drums. Shared
                # controls and combined fallback edits never overwrite their
                # original single-instrument current/velocity/routing settings.
                selected = list(self._selected(data, source))
                if "trim" in cleaned:
                    changed = False
                    for item, offset, profile in selected:
                        if profile is not None:
                            count = item["count"]
                            local = profile.setdefault("settings", {})
                            trims = (local.get("trim", []) + [1.0] * count)
                            keep = max(count, len(local.get("trim", [])))
                            trims = trims[:keep]
                            trims[:count] = cleaned["trim"][offset:offset + count]
                            local["trim"] = trims
                            changed = True
                    if changed:
                        self.source._write(source)
                    data["settings"]["trim"] = cleaned["trim"]
                if "fallback" in cleaned:
                    data["fallbacks"][self._identity(data)] = cleaned["fallback"]
                data["settings"].update({key: cleaned[key] for key in self.SHARED if key in cleaned})
            else:
                raise ValueError("select profiles per ring; create/rename/delete profiles in single-ring mode")
            self._write(data)
            return self._snapshot(data, source)
