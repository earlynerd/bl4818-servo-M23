"""Persist named actuator pitch maps, migrating the original mapping.json."""

import json
import threading
import uuid
from pathlib import Path


def clean_mapping(mapping):
    if not isinstance(mapping, list):
        raise ValueError("mapping must be a list")
    cleaned = []
    for value in mapping:
        if value is None:
            cleaned.append(None)
        elif type(value) is int and 0 <= value <= 127:
            cleaned.append(value)
        else:
            raise ValueError(f"mapping entry {value!r} is not null or int 0-127")
    return cleaned


def clean_settings(settings):
    if not isinstance(settings, dict):
        raise ValueError("instrument settings must be an object")
    cleaned = {}
    if "trim" in settings:
        values = settings["trim"]
        if not isinstance(values, list) or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not 0 <= v <= 2
            for v in values
        ):
            raise ValueError("trim must be a list of multipliers from 0 to 2")
        cleaned["trim"] = values[:]
    if "fallback" in settings:
        values = settings["fallback"]
        if not isinstance(values, dict) or any(
            not str(p).isdigit() or not 0 <= int(p) <= 127 or
            type(slot) is not int or slot < 0
            for p, slot in values.items()
        ):
            raise ValueError("fallback must map MIDI pitches to slots")
        cleaned["fallback"] = values.copy()
    for key, lower, upper in (("vel_floor", 0, 1),
                              ("comp_default_ms", 0, 150),
                              ("current_ma", 0, 3000)):
        if key in settings:
            value = settings[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not lower <= value <= upper:
                raise ValueError(f"{key} is outside {lower}..{upper}")
            cleaned[key] = value
    if "comp_enabled" in settings:
        if type(settings["comp_enabled"]) is not bool:
            raise ValueError("comp_enabled must be true or false")
        cleaned["comp_enabled"] = settings["comp_enabled"]
    if set(settings) != set(cleaned):
        raise ValueError("unknown instrument setting")
    return cleaned


class InstrumentProfiles:
    def __init__(self, path: Path, legacy_path: Path):
        self.path = path
        self.legacy_path = legacy_path
        self.lock = threading.RLock()

    def _read(self):
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            try:
                legacy = json.loads(self.legacy_path.read_text(encoding="utf-8"))
                if isinstance(legacy, dict):
                    legacy = legacy.get("mapping")
                mapping = clean_mapping(legacy)
            except (FileNotFoundError, ValueError, TypeError):
                mapping = []
            return {"active": "default", "profiles": [
                {"id": "default", "name": "Default", "mapping": mapping,
                 "settings": {}}
            ]}
        if not isinstance(data, dict) or not isinstance(data.get("profiles"), list):
            raise ValueError("invalid instruments file")
        if not any(p.get("id") == data.get("active") for p in data["profiles"]):
            raise ValueError("active instrument is missing")
        return data

    def _write(self, data):
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        tmp.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        tmp.replace(self.path)

    def snapshot(self):
        with self.lock:
            data = self._read()
            active = next(p for p in data["profiles"] if p["id"] == data["active"])
            return {"active": data["active"], "settings": active.get("settings", {}), "profiles": [
                {"id": p["id"], "name": p["name"], "count": len(p["mapping"])}
                for p in data["profiles"]
            ]}

    def mapping(self):
        with self.lock:
            if not self.path.exists() and not self.legacy_path.exists():
                return None
            data = self._read()
            return next(p["mapping"][:] for p in data["profiles"]
                        if p["id"] == data["active"])

    def save_mapping(self, mapping, *, expected_id=None):
        cleaned = clean_mapping(mapping)
        with self.lock:
            data = self._read()
            if expected_id is not None and expected_id != data["active"]:
                raise ValueError("selected instrument changed; refresh before saving")
            for profile in data["profiles"]:
                if profile["id"] == data["active"]:
                    profile["mapping"] = cleaned
                    break
            self._write(data)
            return cleaned

    def change(self, action, *, profile_id=None, name=None, mapping=None, settings=None):
        with self.lock:
            data = self._read()
            profiles = data["profiles"]
            selected = next((p for p in profiles if p["id"] == profile_id), None)
            if action in ("create", "rename"):
                if not isinstance(name, str) or not name.strip() or len(name.strip()) > 60:
                    raise ValueError("instrument name must be 1-60 characters")
                name = name.strip()
                if any(p["name"].casefold() == name.casefold() and p is not selected
                       for p in profiles):
                    raise ValueError("instrument name already exists")
            if action == "create":
                source = (self.mapping() or []) if mapping is None else clean_mapping(mapping)
                profile_id = uuid.uuid4().hex
                profiles.append({"id": profile_id, "name": name, "mapping": source,
                                 "settings": clean_settings(settings or {})})
                data["active"] = profile_id
            elif action == "update_settings":
                if selected is None or data["active"] != profile_id:
                    raise ValueError("selected instrument changed; refresh before saving")
                selected["settings"] = {
                    **selected.get("settings", {}), **clean_settings(settings)
                }
            elif action == "select":
                if selected is None:
                    raise ValueError("instrument not found")
                data["active"] = profile_id
            elif action == "rename":
                if selected is None:
                    raise ValueError("instrument not found")
                selected["name"] = name
            elif action == "delete":
                if selected is None:
                    raise ValueError("instrument not found")
                if len(profiles) == 1:
                    raise ValueError("keep at least one instrument")
                profiles.remove(selected)
                if data["active"] == profile_id:
                    data["active"] = profiles[0]["id"]
            else:
                raise ValueError("unknown instrument action")
            self._write(data)
            return self.snapshot()
