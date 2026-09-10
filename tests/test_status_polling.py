"""Exercise status/playback contention without opening a serial port."""
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from ring_bus import CommandAck, RingTimeout, ACK_RESULT_OK, SUBCMD_STRIKE
from ring_midi_server import Bridge, BusLock, Player


class StatusClient:
    def __init__(self, count=14):
        self.count = count
        self.calls = []
        self.homed = True
        self.on_query = None

    def open(self):
        self.opened = True

    def enumerate(self):
        return self.count

    def query_strike(self, address):
        self.calls.append(("status", address))
        if self.on_query:
            self.on_query(address)
        return SimpleNamespace(homed=self.homed, home_shift_warning=False,
                               home_offset=1024, coast_distance=300, homing_duty=100)

    def query_status(self, address):
        self.calls.append(("motor", address))
        return SimpleNamespace(fault=2, fault_name="ENCODER_TIMEOUT",
                               state=2, state_name="FAULT")

    def strike(self, address, current_ma, reply_mode):
        self.calls.append(("strike", address))
        return CommandAck(address, SUBCMD_STRIKE, ACK_RESULT_OK, 0)


class StatusPollingTests(unittest.TestCase):
    def setUp(self):
        self.client = StatusClient()
        with patch("ring_midi_server.RingClientV2", return_value=self.client):
            self.bridge = Bridge("test", 250000)
        self.bridge.enumerate()
        self.playing = threading.Event()
        self.bridge.player = SimpleNamespace(is_playing=self.playing.is_set)

    def test_idle_sweep_preserves_query_count_and_fault_details(self):
        status = self.bridge.status()
        self.assertEqual(self.client.calls, [("status", i) for i in range(14)])
        self.assertEqual(status["homed"], [True] * 14)
        self.assertFalse(status["status_deferred"])
        self.client.homed = False
        self.bridge.count = 1
        status = self.bridge.status()
        self.assertEqual(self.client.calls[-2:], [("status", 0), ("motor", 0)])
        self.assertEqual(status["slots"][0]["fault"], 2)

    def test_playback_uses_aged_cache_without_status_or_probe_traffic(self):
        with patch("ring_midi_server.time.monotonic", return_value=100.0):
            self.bridge.status()
        self.client.calls.clear()
        self.playing.set()
        with patch("ring_midi_server.time.monotonic", return_value=103.5):
            status = self.bridge.status()
            self.bridge.probe_bus()
        self.assertEqual(self.client.calls, [])
        self.assertTrue(status["status_deferred"])
        self.assertEqual(status["slots"][0]["status_age_ms"], 3500)
        self.assertTrue(all(status["homed"]))
        self.playing.clear()
        self.assertFalse(self.bridge.status()["status_deferred"])
        self.assertEqual(len(self.client.calls), 14)

    def test_no_cache_is_explicitly_unknown_and_maintenance_also_defers(self):
        self.bridge.maintenance.set()
        status = self.bridge.status()
        self.bridge.probe_bus()
        self.assertEqual(self.client.calls, [])
        self.assertTrue(status["status_deferred"])
        self.assertIsNone(status["slots"][0]["status_age_ms"])
        self.assertIn("not yet available", status["slots"][0]["status_error"])

    def test_playback_start_interrupts_sweep_before_motor_followup(self):
        self.client.homed = False
        self.client.on_query = lambda address: self.playing.set()
        status = self.bridge.status()
        self.assertEqual(self.client.calls, [("status", 0)])
        self.assertTrue(status["status_deferred"])
        self.assertIsNone(status["slots"][0]["fault"])

    def test_probe_results_seed_status_cache(self):
        self.bridge.probe_bus()
        self.playing.set()
        status = self.bridge.status()
        self.assertEqual(len(self.client.calls), 14)
        self.assertTrue(all(status["homed"]))
        self.assertIsNotNone(status["slots"][0]["status_age_ms"])

    def test_failed_query_keeps_last_reading_and_its_original_age(self):
        self.bridge.count = 1
        with patch("ring_midi_server.time.monotonic", return_value=100.0):
            self.bridge.status()
        def fail(address):
            raise RingTimeout("missing reply")
        self.client.on_query = fail
        with patch("ring_midi_server.time.monotonic", return_value=110.0):
            status = self.bridge.status()
        slot = status["slots"][0]
        self.assertTrue(slot["homed"])
        self.assertTrue(slot["status_cached"])
        self.assertEqual(slot["status_age_ms"], 10000)
        self.assertEqual(slot["status_error"], "missing reply")

    def test_enumeration_discards_cached_address_identities(self):
        self.bridge.status()
        self.client.count = 2
        self.bridge.enumerate()
        self.playing.set()
        status = self.bridge.status()
        self.assertEqual(status["count"], 2)
        self.assertEqual(status["homed"], [False, False])
        self.assertTrue(all(s["status_age_ms"] is None for s in status["slots"]))

    def test_overlapping_requests_return_cache_without_another_sweep(self):
        entered = threading.Event()
        release = threading.Event()
        def slow_query(address):
            if address == 0:
                entered.set()
                if not release.wait(2):
                    raise RingTimeout("test did not release query")
        self.client.on_query = slow_query
        worker = threading.Thread(target=self.bridge.status)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            status = self.bridge.status()
            self.bridge.probe_bus()
            self.assertTrue(status["status_deferred"])
            self.assertEqual(self.client.calls, [("status", 0)])
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(self.client.calls), 14)

    def test_waiting_strike_precedes_next_status_transaction(self):
        entered = threading.Event()
        release = threading.Event()
        waiting = threading.Event()
        def slow_query(address):
            if address == 0:
                entered.set()
                if not release.wait(2):
                    raise RingTimeout("test did not release query")
        self.client.on_query = slow_query
        original_wait = self.bridge.lock._condition.wait
        def note_wait(timeout=None):
            waiting.set()
            return original_wait(timeout)
        poller = threading.Thread(target=self.bridge.status)
        striker = threading.Thread(target=lambda: self.bridge.strike(13, 1000))
        with patch.object(self.bridge.lock._condition, "wait", side_effect=note_wait):
            poller.start()
            try:
                self.assertTrue(entered.wait(1))
                striker.start()
                self.assertTrue(waiting.wait(1))
            finally:
                release.set()
                poller.join(2)
                if striker.ident is not None:
                    striker.join(2)
        self.assertFalse(poller.is_alive())
        self.assertFalse(striker.is_alive())
        self.assertEqual(self.client.calls[:2], [("status", 0), ("strike", 13)])

    def test_exception_releases_poll_and_command_locks(self):
        lock = BusLock()
        with self.assertRaises(ValueError):
            with lock.poll() as acquired:
                self.assertTrue(acquired)
                raise ValueError("test")
        with self.assertRaises(ValueError):
            with lock:
                raise ValueError("test")
        with lock.poll() as acquired:
            self.assertTrue(acquired)

    def test_real_player_gate_covers_scheduled_wait_and_cancellation(self):
        player = Player(self.bridge)
        self.bridge.player = player
        player.play([{"t_ms": 60000, "address": 0, "nominal_current_ma": 1000}])
        try:
            self.assertTrue(player.is_playing())
            self.bridge.status()
            self.bridge.probe_bus()
            self.assertEqual(self.client.calls, [])
        finally:
            player.cancel()
        self.assertFalse(player.is_playing())


if __name__ == "__main__":
    unittest.main()
