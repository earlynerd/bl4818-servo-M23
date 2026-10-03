import json
import sys
import tempfile
import threading
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from instrument_profiles import InstrumentProfiles  # noqa: E402
from ring_midi_server import Bridge, Handler  # noqa: E402


class InstrumentProfileTests(unittest.TestCase):
    def test_http_create_select_and_mapping_save(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            store = InstrumentProfiles(root / "instruments.json", root / "mapping.json")

            class TestHandler(Handler):
                bridge = SimpleNamespace(
                    instruments=store, maintenance=threading.Event(),
                    load_mapping=store.mapping, save_mapping=store.save_mapping,
                )

                def log_message(self, *_args):
                    pass

            http = ThreadingHTTPServer(("127.0.0.1", 0), TestHandler)
            thread = threading.Thread(target=http.serve_forever, daemon=True)
            thread.start()
            base = f"http://127.0.0.1:{http.server_port}"

            def request(path, payload=None):
                data = None if payload is None else json.dumps(payload).encode("utf-8")
                req = urllib.request.Request(base + path, data=data,
                                             headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(req, timeout=2) as response:
                    return json.load(response)

            try:
                self.assertIsNone(request("/api/mapping")["mapping"])
                created = request("/api/instruments", {
                    "action": "create", "name": "Handpan", "mapping": [60, 64],
                    "settings": {"trim": [0.8, 1.1], "current_ma": 900},
                })
                self.assertEqual(created["mapping"], [60, 64])
                self.assertEqual(created["settings"]["trim"], [0.8, 1.1])
                profile_id = created["active"]
                request("/api/mapping", {"mapping": [61, 65], "profile_id": profile_id})
                self.assertEqual(request("/api/mapping")["mapping"], [61, 65])
                request("/api/instruments", {"action": "select", "id": "default"})
                self.assertEqual(request("/api/mapping")["mapping"], [])
                request("/api/instruments", {"action": "select", "id": profile_id})
                self.assertEqual(request("/api/mapping")["mapping"], [61, 65])
            finally:
                http.shutdown()
                http.server_close()
                thread.join(timeout=2)

    def test_discovery_omits_saved_slots_beyond_present_count(self):
        bridge = object.__new__(Bridge)
        bridge.count = 2
        bridge.lock = threading.Lock()
        bridge.load_mapping = lambda: [60, 64, 67]
        bridge._query_strike_timed = lambda addr: SimpleNamespace(homed=True)
        self.assertEqual([item["slot"] for item in bridge.pitches()], [0, 1])

    def test_import_switch_and_independent_edits(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            legacy = root / "mapping.json"
            legacy.write_text(json.dumps({"mapping": [60, 64, 67, 72]}), encoding="utf-8")
            store = InstrumentProfiles(root / "instruments.json", legacy)
            self.assertEqual(store.mapping(), [60, 64, 67, 72])

            second = store.change("create", name="Tongue drum", mapping=[52, 55],
                                  settings={"trim": [0.8, 1.2], "fallback": {"70": 1},
                                            "vel_floor": 0.4, "comp_enabled": False,
                                            "comp_default_ms": 65, "current_ma": 950})
            second_id = second["active"]
            self.assertEqual(store.mapping(), [52, 55])
            self.assertEqual(second["settings"]["trim"], [0.8, 1.2])
            store.save_mapping([52, 57])
            store.change("update_settings", profile_id=second_id,
                         settings={"vel_floor": 0.5})
            store.change("select", profile_id="default")
            self.assertEqual(store.mapping(), [60, 64, 67, 72])
            store.change("select", profile_id=second_id)
            self.assertEqual(store.mapping(), [52, 57])
            self.assertEqual(json.loads(legacy.read_text())["mapping"], [60, 64, 67, 72])

            restarted = InstrumentProfiles(root / "instruments.json", legacy)
            self.assertEqual(restarted.snapshot()["active"], second_id)
            self.assertEqual(restarted.mapping(), [52, 57])
            self.assertEqual(restarted.snapshot()["settings"]["trim"], [0.8, 1.2])
            self.assertEqual(restarted.snapshot()["settings"]["vel_floor"], 0.5)

    def test_invalid_changes_leave_profiles_intact(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            store = InstrumentProfiles(root / "instruments.json", root / "mapping.json")
            self.assertIsNone(store.mapping())
            store.save_mapping([60, None])
            with self.assertRaises(ValueError):
                store.save_mapping([True])
            with self.assertRaises(ValueError):
                store.change("delete", profile_id="default")
            with self.assertRaises(ValueError):
                store.change("create", name="Default")
            with self.assertRaises(ValueError):
                store.save_mapping([72], expected_id="another-instrument")
            self.assertEqual(store.mapping(), [60, None])


if __name__ == "__main__":
    unittest.main()
