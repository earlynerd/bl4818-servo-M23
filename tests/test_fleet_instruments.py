"""Profile selection and persistence across rings, without serial hardware."""
import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from fleet_instruments import FleetInstrumentProfiles
from instrument_profiles import InstrumentProfiles


class FleetInstrumentTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.layout = [{"name": "jameson", "port": "COM29", "count": 10},
                       {"name": "retuned", "port": "COM48", "count": 10}]
        self.source = InstrumentProfiles(self.root / "instruments.json", self.root / "mapping.json")
        self.first = self.source.change("create", name="Jameson Drum", mapping=list(range(50, 60)),
                                        settings={"trim": [1.25] * 10, "fallback": {"40": 2},
                                                  "current_ma": 250, "vel_floor": 0.1,
                                                  "comp_default_ms": 37, "comp_enabled": True})["active"]
        self.second = self.source.change("create", name="retuned_drum", mapping=list(range(60, 70)),
                                         settings={"trim": [0.75] * 10, "fallback": {"40": 1, "41": 9},
                                                   "current_ma": 450, "vel_floor": 0.2})["active"]
        self.store = FleetInstrumentProfiles(self.root / "fleet.json", self.root / "mapping-rings.json",
                                             self.layout, self.source)
        self.original = self.source.path.read_bytes()

    def select(self, ring, profile):
        return self.store.change("select_ring", ring=ring, profile_id=profile,
                                 expected_id=self.store.snapshot()["active"])

    def select_both(self):
        self.select("jameson", self.first)
        return self.select("retuned", self.second)

    def test_select_two_existing_profiles_without_creating_or_changing_either(self):
        result = self.select_both()
        self.assertEqual(result["mapping"], list(range(50, 70)))
        self.assertEqual(result["settings"]["trim"], [1.25] * 10 + [0.75] * 10)
        self.assertEqual(result["settings"]["fallback"], {"40": 2, "41": 19})
        self.assertEqual(result["settings"]["current_ma"], 250)
        self.assertEqual(result["settings"]["vel_floor"], 0.1)
        self.assertEqual(result["settings"]["comp_default_ms"], 37)
        self.assertEqual(self.source.path.read_bytes(), self.original)

    def test_restart_preserves_selections_and_shared_controls(self):
        selected = self.select_both()
        self.store.change("update_settings", profile_id=selected["active"],
                          settings={"current_ma": 350, "vel_floor": 0.15})
        restarted = FleetInstrumentProfiles(self.store.path, self.store.legacy_path,
                                            self.layout, self.source)
        state = restarted.snapshot()
        self.assertEqual(state["active"], selected["active"])
        self.assertEqual(state["mapping"], selected["mapping"])
        self.assertEqual(state["settings"]["current_ma"], 350)
        self.assertEqual(self.source.path.read_bytes(), self.original)

    def test_mapping_and_trim_edits_write_back_to_correct_profile(self):
        selected = self.select_both()
        mapping = selected["mapping"][:]
        mapping[0], mapping[19] = 48, 72
        self.store.save_mapping(mapping, expected_id=selected["active"])
        self.store.change("update_settings", profile_id=selected["active"],
                          settings={"trim": [1.1] * 10 + [0.9] * 10,
                                    "current_ma": 300, "fallback": {"40": 19}})
        source = self.source._read()
        a = next(p for p in source["profiles"] if p["id"] == self.first)
        b = next(p for p in source["profiles"] if p["id"] == self.second)
        self.assertEqual(a["mapping"][0], 48)
        self.assertEqual(b["mapping"][9], 72)
        self.assertEqual(a["settings"]["trim"], [1.1] * 10)
        self.assertEqual(b["settings"]["trim"], [0.9] * 10)
        self.assertEqual(a["settings"]["current_ma"], 250)
        self.assertEqual(b["settings"]["current_ma"], 450)
        self.assertEqual(b["settings"]["fallback"], {"40": 1, "41": 9})
        self.assertEqual(source["active"], self.second)
        self.assertEqual(self.store.snapshot()["settings"]["fallback"], {"40": 19})

    def test_changed_selection_rejects_stale_writes(self):
        stale = self.store.snapshot()["active"]
        self.select_both()
        for action in [lambda: self.store.save_mapping([72] * 20, expected_id=stale),
                       lambda: self.store.change("update_settings", profile_id=stale, settings={"current_ma": 3000}),
                       lambda: self.store.change("select_ring", ring="jameson", profile_id=self.first, expected_id=stale)]:
            with self.assertRaisesRegex(ValueError, "changed"):
                action()
        self.assertEqual(self.source.path.read_bytes(), self.original)

    def test_invalid_selections_and_settings_leave_files_intact(self):
        selected = self.select_both()
        saved = self.store.path.read_bytes()
        for ring, profile in [("unknown", self.first), ("retuned", self.first), ("jameson", "missing")]:
            with self.assertRaises(ValueError):
                self.select(ring, profile)
        for settings in [{"trim": [1]}, {"fallback": {"40": 20}}, {"current_ma": float("nan")}]:
            with self.assertRaises(ValueError):
                self.store.change("update_settings", profile_id=selected["active"], settings=settings)
        self.assertEqual(self.store.path.read_bytes(), saved)
        self.assertEqual(self.source.path.read_bytes(), self.original)

    def test_unassigned_ring_stays_unmapped(self):
        result = self.select("retuned", self.second)
        self.assertEqual(result["mapping"][:10], [None] * 10)
        self.assertEqual(result["mapping"][10:], list(range(60, 70)))
        self.assertEqual(result["settings"]["current_ma"], 450)

    def test_saved_extra_slots_survive_edits_to_present_slots(self):
        self.source.save_mapping(list(range(60, 74)), expected_id=self.second)
        selected = self.select_both()
        self.store.save_mapping([72] * 20, expected_id=selected["active"])
        self.assertEqual(self.source.mapping(), [72] * 10 + [70, 71, 72, 73])

    def test_legacy_fleet_mapping_requires_matching_layout_and_is_not_modified(self):
        legacy = {"rings": self.layout, "mapping": [60] * 20}
        self.store.legacy_path.write_text(json.dumps(legacy))
        original = self.store.legacy_path.read_bytes()
        self.assertEqual(self.store.mapping(), [60] * 20)
        self.store.save_mapping([61] * 20)
        self.assertEqual(self.store.legacy_path.read_bytes(), original)
        changed = copy.deepcopy(self.layout)
        changed[0]["port"] = "COM9"
        other = FleetInstrumentProfiles(self.root / "other.json", self.store.legacy_path, changed, self.source)
        self.assertEqual(other.mapping(), [None] * 20)


if __name__ == "__main__":
    unittest.main()
