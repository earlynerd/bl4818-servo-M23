"""Exercise multiple real Bridges with simulated serial clients; no hardware I/O."""
import json
import sys
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from ring_bus import CommandAck, RingTimeout, REPLY_MODE_NONE
from ring_fleet import RingFleet, parse_ring_specs
from ring_midi_server import Bridge, Player, Handler, FirmwareUpdateManager


class SerialRing:
    def __init__(self, count):
        self.count = count
        self.calls = []
        self.opened = False
        self.on_strike = None
        self.on_home = None
        self.on_enumerate = None
        self.cancel_ok = True

    def open(self):
        self.opened = True

    def close(self):
        self.opened = False

    def enumerate(self):
        if self.on_enumerate:
            self.on_enumerate()
        return self.count

    def query_strike(self, address):
        self.calls.append(("status", address))
        return SimpleNamespace(homed=True, home_shift_warning=False,
                               home_offset=123, coast_distance=20, homing_duty=100)

    def strike(self, address, current_ma, reply_mode):
        self.calls.append(("strike", address, current_ma))
        if self.on_strike:
            self.on_strike()
        return CommandAck(address, 0, 0, 0)

    def strike_burst(self, pairs, reply_mode):
        self.calls.append(("chord", list(pairs), reply_mode))

    def strike_home(self, address, reply_mode):
        self.calls.append(("home", address))
        if self.on_home:
            self.on_home()
        return CommandAck(address, 0, 0, 0)

    def strike_cancel(self, address, reply_mode):
        self.calls.append(("cancel", address))
        if not self.cancel_ok:
            raise RingTimeout("unplugged")
        return CommandAck(address, 0, 0, 0)

    def stop(self, address, reply_mode):
        self.calls.append(("stop", address))
        return CommandAck(address, 0, 0, 0)

    def save_settings(self, address, reply_mode):
        self.calls.append(("save", address))
        return CommandAck(address, 0, 0, 0)

    def set_strike_param(self, address, param_id, value, reply_mode):
        self.calls.append(("param", address, param_id, value))
        return CommandAck(address, 0, 0, 0)


class FleetTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.clients = [SerialRing(14), SerialRing(10)]
        self.specs = parse_ring_specs(["pan=COM7,14", "drum=COM8,10"])
        with patch("ring_midi_server.RingClientV2", side_effect=self.clients):
            self.fleet = RingFleet(self.specs, Bridge, baud=250000,
                                   mapping_file=Path(self.tmp.name) / "mapping.json")
        self.fleet.player = Player(self.fleet)
        self.addCleanup(self.fleet.close)
        self.addCleanup(self.fleet.player.cancel)
        self.fleet.enumerate()

    def event(self, address, t_ms=0):
        return {"address": address, "t_ms": t_ms, "nominal_current_ma": 800}

    def wait_playback(self):
        self.fleet.player._thread.join(2)
        self.assertFalse(self.fleet.player.is_playing())

    def test_parse_rejects_ambiguous_or_oversized_layout(self):
        for values in [["x=COM1,17"], ["x=COM1,0"], ["x=COM1"],
                       ["x=COM1,2", "y=com1,2"], ["x=COM1,2", "X=COM2,2"]]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                parse_ring_specs(values)

    def test_same_local_address_routes_to_distinct_adapters(self):
        self.assertEqual(self.fleet.strike(0, 700)["address"], 0)
        response = self.fleet.strike(14, 900)
        self.assertEqual(response["address"], 14)
        self.assertEqual(response["local_address"], 0)
        self.assertEqual(response["ring"], "drum")
        self.assertIn(("strike", 0, 700), self.clients[0].calls)
        self.assertIn(("strike", 0, 900), self.clients[1].calls)
        self.assertEqual(set(self.fleet.health.snapshot()["addresses"]), {"0", "14"})
        for address in [-1, 24]:
            with self.assertRaises(ValueError):
                self.fleet.strike(address, 800)

    def test_status_and_pitch_mapping_span_more_than_sixteen_slots(self):
        mapping = [None] * 24
        mapping[0], mapping[14], mapping[23] = 48, 60, 72
        self.fleet.save_mapping(mapping)
        status = self.fleet.status()
        self.assertEqual(status["count"], 24)
        self.assertTrue(all(status["homed"]))
        self.assertEqual(status["slots"][23]["local_address"], 9)
        self.assertEqual(status["slots"][23]["ring"], "drum")
        self.assertEqual([p["slot"] for p in self.fleet.pitches()], [0, 14, 23])

    def test_changed_layout_does_not_reuse_mapping(self):
        self.fleet.save_mapping([60] * 24)
        obj = json.loads(self.fleet.mapping_file.read_text())
        obj["rings"][0]["port"] = "COM99"
        self.fleet.mapping_file.write_text(json.dumps(obj))
        self.assertEqual(self.fleet.load_mapping(), [None] * 24)
        with self.assertRaises(ValueError):
            self.fleet.save_mapping([60] * 14)

    def test_count_change_fails_without_shifting_other_ring(self):
        self.clients[0].count = 13
        with self.assertRaisesRegex(RuntimeError, "expected 14"):
            self.fleet.enumerate()
        self.assertEqual(self.fleet.count, 24)
        self.assertFalse(self.fleet.status()["rings"][0]["ready"])
        self.assertEqual(self.fleet.strike(14, 700)["local_address"], 0)
        with self.assertRaises(RuntimeError):
            self.fleet.strike(0, 700)
        with self.assertRaises(RuntimeError):
            self.fleet.player.play([self.event(14)])
        self.clients[0].count = 14
        self.assertEqual(self.fleet.enumerate(), 24)

    def test_home_retry_rejects_changed_count_before_second_motion(self):
        self.clients[0].count = 13
        self.clients[0].on_home = lambda: (_ for _ in ()).throw(RingTimeout("lost reply"))
        result = self.fleet.home([0, 1])
        self.assertFalse(result[0]["accepted"])
        self.assertFalse(result[1]["accepted"])
        self.assertEqual(self.clients[0].calls, [("home", 0)])
        self.assertEqual(self.fleet.rings[0].count, 0)

    def test_blocked_ring_does_not_stall_other_ring(self):
        blocked, release, delivered = threading.Event(), threading.Event(), threading.Event()
        def stall():
            blocked.set()
            release.wait(1.5)
        self.clients[0].on_strike = stall
        self.clients[1].on_strike = delivered.set
        try:
            self.fleet.player.play([self.event(0), self.event(14, 100)])
            self.assertTrue(blocked.wait(1))
            self.assertTrue(delivered.wait(0.8), "second adapter waited for first adapter's ACK")
            self.assertTrue(self.fleet.player.is_playing())
        finally:
            release.set()
        self.wait_playback()
        self.assertEqual(self.fleet.player.status()["dispatch_stats"]["accepted"], 2)

    def test_global_chord_uses_no_reply_even_with_one_note_per_ring(self):
        self.fleet.player.play([self.event(0), self.event(14)])
        self.wait_playback()
        for client in self.clients:
            self.assertEqual(client.calls, [("chord", [(0, 800)], REPLY_MODE_NONE)])
        stats = self.fleet.player.status()["dispatch_stats"]
        self.assertEqual(stats["attempted"], 2)
        self.assertEqual(stats["unacknowledged"], 2)

    def test_latency_is_local_to_each_ring(self):
        self.fleet.rings[0].latency.update(0, 800, 12)
        self.fleet.rings[1].latency.update(0, 800, 43)
        self.assertEqual(self.fleet.latency.compensation_ms(0, 800), 12)
        self.assertEqual(self.fleet.latency.compensation_ms(14, 800), 43)

    def test_cancel_stops_all_workers_and_reports_each_ring_failure(self):
        self.fleet.player.play([self.event(0, 5000), self.event(14, 5000)])
        self.clients[0].cancel_ok = False
        result = self.fleet.cancel_all()
        self.assertFalse(result["ok"])
        self.assertEqual(result["failed"], list(range(14)))
        self.assertEqual(len(result["results"]), 24)
        self.assertEqual(self.clients[1].calls, [("cancel", a) for a in range(10)])
        self.assertFalse(self.fleet.player.is_playing())
        self.assertFalse(any(t.name.startswith("player-") for t in threading.enumerate()))

    def test_live_mute_scale_and_preemption_apply_across_rings(self):
        self.fleet.player.play([self.event(0, 400), self.event(14, 400)])
        self.fleet.player.update(master_scale=0.5, muted=[0])
        self.wait_playback()
        self.assertEqual(self.clients[0].calls, [])
        self.assertEqual(self.clients[1].calls, [("chord", [(0, 400)], REPLY_MODE_NONE)])
        self.fleet.player.play([self.event(1, 5000), self.event(15, 5000)])
        self.fleet.player.play([self.event(23)])
        self.wait_playback()
        self.assertIn(("strike", 9, 800), self.clients[1].calls)
        self.assertFalse(any(call[0] == "strike" for call in self.clients[0].calls))

    def test_stop_home_save_and_parameters_use_global_addresses(self):
        for method in [self.fleet.home, self.fleet.save_settings, self.fleet.stop]:
            result = method([0, 14, 23])
            self.assertEqual([r["address"] for r in result], [0, 14, 23])
            self.assertTrue(all(r["accepted"] for r in result))
        result = self.fleet.set_strike_param(23, 1, 40)
        self.assertEqual(result["address"], 23)
        self.assertIn(("param", 9, 1, 40), self.clients[1].calls)

    def test_firmware_update_requires_single_ring_mode(self):
        manager = FirmwareUpdateManager(self.fleet)
        self.assertFalse(manager.snapshot()["update_supported"])
        with self.assertRaisesRegex(RuntimeError, "single-ring"):
            manager.start_update(0)

    def test_partial_open_failure_closes_already_open_ports(self):
        calls = []
        def factory(**kwargs):
            calls.append(kwargs)
            if len(calls) == 2:
                raise OSError("COM8 unavailable")
            return self.fleet.rings[0]
        with self.assertRaises(OSError):
            RingFleet(self.specs, factory, baud=250000,
                      mapping_file=Path(self.tmp.name) / "unused.json")
        self.assertFalse(self.clients[0].opened)

    def test_http_pitch_playback_mapping_and_cancel(self):
        class TestHandler(Handler):
            bridge = self.fleet
            firmware = FirmwareUpdateManager(self.fleet)
            def log_message(self, *args):
                pass
        server = ThreadingHTTPServer(("127.0.0.1", 0), TestHandler)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        def request(path, data=None):
            req = Request(f"http://127.0.0.1:{server.server_port}{path}",
                          data=None if data is None else json.dumps(data).encode(),
                          headers={"Content-Type": "application/json"})
            with urlopen(req, timeout=2) as response:
                return json.load(response)
        try:
            mapping = [None] * 24
            mapping[0], mapping[14] = 48, 60
            request("/api/mapping", {"mapping": mapping})
            self.assertEqual(request("/api/mapping")["context"], self.fleet.mapping_context)
            self.assertEqual(request("/api/status")["count"], 24)
            response = request("/api/play", {"events": [
                {"t_ms": 0, "pitch": 48, "velocity": 80},
                {"t_ms": 0, "pitch": 60, "velocity": 80},
            ]})
            self.assertEqual(response["scheduled"], 2)
            self.wait_playback()
            self.assertEqual(request("/api/play")["dispatch_stats"]["unacknowledged"], 2)
            self.assertTrue(request("/api/cancel", {})["ok"])
        finally:
            server.shutdown()
            server.server_close()
            worker.join(2)


if __name__ == "__main__":
    unittest.main()
