"""Wire-level asynchronous playback regressions; no hardware access."""
import queue
import struct
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import ring_bus as bus
import ring_midi_server as server


def ack_frame(client, address, command_id, *, sequence=1, current=-1200,
              impact=27, result=0, completed=0, flags=bus.STRIKE_TIMING_IMPACT_VALID,
              impact_01ms=None, precise=True):
    """Timed ACK as current firmware sends it; precise=False omits the 0.1 ms
    impact field (the intermediate 19/23-byte form)."""
    metrics = struct.pack(">HHHHHHh", completed, flags, 12, impact, 42, 1000, current)
    if precise:
        metrics += struct.pack(">H", impact * 10 if impact_01ms is None else impact_01ms)
    payload = bytes([bus.CMD_ACK_BASE | address, bus.SUBCMD_STRIKE, result])
    payload += struct.pack(">H", sequence) + metrics
    if command_id is not None:
        payload += struct.pack(">I", command_id)
    return client._build_frame(payload)


class Wire:
    def __init__(self):
        self.rx = bytearray()
        self.cv = threading.Condition()
        self.writes = []
        self.is_open = True
        self.on_write = None

    @property
    def in_waiting(self):
        with self.cv:
            return len(self.rx)

    def inject(self, data):
        with self.cv:
            self.rx.extend(data)
            self.cv.notify_all()

    def read(self, n=1):
        with self.cv:
            if not self.rx:
                self.cv.wait(.005)
            data = bytes(self.rx[:n])
            del self.rx[:n]
            return data

    def write(self, data):
        self.writes.append(bytes(data))
        if self.on_write:
            self.on_write(data)
        return len(data)

    def flush(self):
        pass

    def close(self):
        self.is_open = False


class AsyncStrikeTests(unittest.TestCase):
    def client(self, timeout=1000):
        c = bus.RingClientV2("fake", timeout_ms=timeout)
        c.ser = Wire()
        c.device_count = 3
        self.addCleanup(c.close)
        return c

    def bridge(self, c):
        with patch.object(bus.RingClientV2, "open"), patch.object(server, "RingClientV2", return_value=c):
            b = server.Bridge("fake", 250000)
        b.count = 3
        b._async_supported = {0: True, 1: True, 2: True}
        return b

    def test_new_and_legacy_timing_formats(self):
        c = self.client()
        frame = ack_frame(c, 2, 0x12345678, impact_01ms=273)
        payload = frame[3:-2]
        self.assertEqual(len(payload), 25)
        ack = c._parse_ack_payload(2, bus.SUBCMD_STRIKE, payload)
        self.assertEqual(ack.command_id, 0x12345678)
        self.assertEqual(ack.metrics.strike_current_ma, -1200)
        self.assertEqual(ack.metrics.trigger_to_impact_ms, 27)
        self.assertEqual(ack.metrics.trigger_to_impact_ms_precise, 27.3)
        self.assertEqual(ack.metrics.sequence, 0)  # valid after sequence wrap
        untagged = c._parse_ack_payload(2, bus.SUBCMD_STRIKE, payload[:21])
        self.assertIsNone(untagged.command_id)
        self.assertEqual(untagged.metrics.trigger_to_impact_ms_precise, 27.3)
        intermediate = c._parse_ack_payload(
            2, bus.SUBCMD_STRIKE, ack_frame(c, 2, 7, precise=False)[3:-2])
        self.assertEqual((intermediate.command_id, intermediate.metrics.strike_current_ma), (7, -1200))
        self.assertIsNone(intermediate.metrics.trigger_to_impact_ms_precise)
        legacy = c._parse_ack_payload(2, bus.SUBCMD_STRIKE, payload[:17])
        self.assertIsNone(legacy.command_id)
        self.assertIsNone(legacy.metrics.strike_current_ma)
        self.assertIsNone(legacy.metrics.trigger_to_impact_ms_precise)

    def test_contact_flags_and_invalid_impact_hide_precise_time(self):
        c = self.client()
        flags = bus.STRIKE_TIMING_STALLED | bus.STRIKE_TIMING_NO_CONTACT
        ack = c._parse_ack_payload(1, bus.SUBCMD_STRIKE, ack_frame(c, 1, 3, flags=flags)[3:-2])
        self.assertTrue(ack.metrics.stalled and ack.metrics.no_contact)
        self.assertIsNone(ack.metrics.trigger_to_impact_ms)
        self.assertIsNone(ack.metrics.trigger_to_impact_ms_precise)

    def test_two_notes_send_before_any_reply_and_match_out_of_order(self):
        c = self.client()
        done = queue.Queue()
        first = c.strike_async(0, 700, lambda *args: done.put(("first", args)))
        second = c.strike_async(0, 1700, lambda *args: done.put(("second", args)))
        self.assertEqual(len(c.ser.writes), 2)
        self.assertTrue(done.empty())
        c.ser.inject(ack_frame(c, 0, second, result=bus.ACK_RESULT_REJECT_NOT_READY))
        c.ser.inject(ack_frame(c, 0, first))
        name, (ack, rtt, error) = done.get(timeout=1)
        self.assertEqual(name, "second")
        self.assertFalse(ack.accepted)
        self.assertIsNone(error)
        self.assertGreaterEqual(rtt, 0)
        self.assertEqual(done.get(timeout=1)[0], "first")
        self.assertEqual(c._pending, {})

    def test_missing_ack_expires_without_blocking_next_note(self):
        c = self.client(timeout=40)
        done = queue.Queue()
        first = c.strike_async(0, 700, lambda *args: done.put(("first", args)))
        second = c.strike_async(1, 900, lambda *args: done.put(("second", args)))
        c.ser.inject(ack_frame(c, 1, second))
        self.assertEqual(done.get(timeout=1)[0], "second")
        name, (_, _, error) = done.get(timeout=1)
        self.assertEqual(name, "first")
        self.assertIsInstance(error, bus.RingTimeout)
        c.ser.inject(ack_frame(c, 0, first))
        with self.assertRaises(queue.Empty):
            done.get(timeout=.06)

    def test_fragmented_corrupt_echoes_and_sync_replies_share_one_reader(self):
        c = self.client()
        done = queue.Queue()
        token = c.strike_async(1, 1300, lambda *args: done.put(args))
        valid = ack_frame(c, 1, token)
        corrupt = bytearray(valid)
        corrupt[-1] ^= 1
        c.ser.inject(b"noise" + corrupt + c.ser.writes[0] + valid[:6])
        # A frame split across read cycles must survive with its parser state intact.
        c.ser.inject(valid[6:])
        self.assertIsNone(done.get(timeout=1)[2])
        sync = bytes([bus.CMD_ACK_BASE | 1, bus.SUBCMD_STRIKE_CANCEL, 0, 0, 2])
        c.ser.inject(c._build_frame(sync))
        self.assertEqual(c._recv_frame(), sync)
        self.assertTrue(c._rx_queue.empty())

    def test_current_pair_is_independent_of_sent_command_and_deduplicated(self):
        c = self.client()
        b = self.bridge(c)
        done = queue.Queue()
        b.strike_playback(0, 2600, done.put)
        token = struct.unpack(">I", c.ser.writes[-1][7:11])[0]
        c.ser.inject(ack_frame(c, 0, token, current=-700, impact=21, impact_01ms=214,
                               completed=7))
        result = done.get(timeout=1)
        self.assertTrue(result["metrics_fresh"])
        # Learned at the report's own current, from the 0.1 ms impact time.
        self.assertAlmostEqual(b.latency.compensation_ms(0, 700), 21.4)
        b.strike_playback(0, 1800, done.put)
        token = struct.unpack(">I", c.ser.writes[-1][7:11])[0]
        c.ser.inject(ack_frame(c, 0, token, current=-700, impact=39, completed=7))
        self.assertFalse(done.get(timeout=1)["metrics_fresh"])
        self.assertAlmostEqual(b.latency.compensation_ms(0, 700), 21.4)
        for flags in (bus.STRIKE_TIMING_DEAD, bus.STRIKE_TIMING_RETRIGGERED):
            self.assertFalse(b._fold_metrics_into_ema(0, 2900, {
                "sequence": 8 + flags, "flags": flags | bus.STRIKE_TIMING_IMPACT_VALID,
                "strike_current_ma": 700, "trigger_to_impact_ms": 99}))
        self.assertAlmostEqual(b.latency.compensation_ms(0, 700), 21.4)

    def test_scheduler_finishes_rapid_sequence_with_all_acks_still_pending(self):
        c = self.client()
        b = self.bridge(c)
        b.player = server.Player(b)
        b.player.play([{"address": 0, "t_ms": t, "nominal_current_ma": 800}
                       for t in (0, 1, 2)])
        b.player._thread.join(.3)
        self.assertFalse(b.player.is_playing())
        stats = b.player.status()["dispatch_stats"]
        self.assertEqual((stats["attempted"], stats["pending"]), (3, 3))
        self.assertEqual(len(c.ser.writes), 3)
        # Cancel can proceed through the same receiver while strike ACKs are outstanding.
        c.ser.on_write = lambda frame: c.ser.inject(c._build_frame(
            bytes([bus.CMD_ACK_BASE, bus.SUBCMD_STRIKE_CANCEL, 0, 0, 3])))
        cancel = c.strike_cancel(0, reply_mode=bus.REPLY_MODE_ACK)
        self.assertTrue(cancel.accepted)
        b.player.cancel()
        c._expire_async(bus.RingError("cancelled test"))
        self.assertEqual(b.player.status()["dispatch_stats"]["accepted"], 0)

    def test_old_play_feedback_does_not_change_new_play_counters(self):
        c = self.client()
        b = self.bridge(c)
        player = server.Player(b)
        event = [{"address": 0, "t_ms": 0, "nominal_current_ma": 800}]
        player.play(event)
        player._thread.join(1)
        old = struct.unpack(">I", c.ser.writes[-1][7:11])[0]
        player.play(event)
        player._thread.join(1)
        c._route_received(ack_frame(c, 0, old)[3:-2])
        stats = player.status()["dispatch_stats"]
        self.assertEqual((stats["attempted"], stats["pending"], stats["accepted"]), (1, 1, 0))

    def test_legacy_capability_keeps_synchronous_path(self):
        c = self.client()
        b = self.bridge(c)
        b._async_supported.clear()
        legacy = bus.StrikeTiming(0, 0, 0, None, None, None, None)
        with patch.object(c, "query_strike_timing", return_value=legacy):
            b.prepare_playback([0])
        self.assertIsNone(c._rx_thread)
        done = []
        with patch.object(b, "strike", return_value={"accepted": True}) as sync:
            b.strike_playback(0, 800, done.append)
        sync.assert_called_once_with(0, 800)
        self.assertEqual(done, [{"accepted": True}])

    def test_write_failure_leaves_no_pending_entry(self):
        c = self.client()
        c.ser.on_write = lambda _: (_ for _ in ()).throw(OSError("disconnected"))
        with self.assertRaises(OSError):
            c.strike_async(0, 800, lambda *args: self.fail("unexpected callback"))
        self.assertFalse(c._pending)

    def test_chord_batch_matches_original_wire_bytes_with_one_write(self):
        c = self.client()
        expected = b"".join(c._build_frame(c._build_addressed_payload(a, bus.SUBCMD_STRIKE,
            struct.pack(">h", ma), bus.REPLY_MODE_NONE)) for a, ma in [(2, 1700), (0, 700)])
        c.strike_burst([(0, 700), (2, 1700)], reply_mode=bus.REPLY_MODE_NONE)
        self.assertEqual(c.ser.writes, [expected])
        self.assertIsNone(c._rx_thread)
        self.assertEqual(c._pending, {})

    def test_new_timing_poll_and_loader_reply_work_after_rx_starts(self):
        c = self.client()
        c.start_receiver()
        timing = bytes([bus.CMD_STATUS_BASE]) + struct.pack(">HHHHHHh", 3,
            bus.STRIKE_TIMING_IMPACT_VALID, 1, 29, 40, 1000, -1700)
        c.ser.on_write = lambda _: c.ser.inject(c._build_frame(timing))
        report = c.query_strike_timing(0)
        self.assertEqual((report.strike_current_ma, report.trigger_to_impact_ms), (-1700, 29))
        c.ser.on_write = lambda _: c.ser.inject(c._build_frame(timing + struct.pack(">H", 287)))
        report = c.query_strike_timing(0)
        self.assertEqual((report.trigger_to_impact_ms, report.trigger_to_impact_ms_precise), (29, 28.7))
        from ring_bootload import BootloaderClient, BOOT_SUB_VERIFY_IMAGE
        loader_reply = bytes([0x80, BOOT_SUB_VERIFY_IMAGE, 0, 1, 2, 3, 4])
        c.ser.on_write = lambda _: c.ser.inject(c._build_frame(loader_reply))
        self.assertEqual(BootloaderClient(c)._command(0, BOOT_SUB_VERIFY_IMAGE), b"\x01\x02\x03\x04")

    def test_new_play_feedback_settles_and_tracks_exact_residual_sequence(self):
        c = self.client()
        b = self.bridge(c)
        player = server.Player(b)
        player.play([{"address": 0, "t_ms": t, "nominal_current_ma": 800} for t in (0, 1)])
        player._thread.join(1)
        first, second = [struct.unpack(">I", frame[7:11])[0] for frame in c.ser.writes]
        c._route_received(ack_frame(c, 0, first, sequence=1, completed=0)[3:-2])
        c._route_received(ack_frame(c, 0, second, sequence=2, completed=1, impact=26)[3:-2])
        stats = player.status()["dispatch_stats"]
        self.assertEqual((stats["pending"], stats["accepted"]), (0, 2))
        self.assertEqual(player.residual_stats[0]["last"], 26 - 50)
        self.assertEqual(player.timing_stats["serial_rtt_ms"]["n"], 2)

    def test_manual_strike_residuals_pair_by_device_sequence(self):
        player = server.Player(self.bridge(self.client()))
        report = {"sequence": 5, "flags": bus.STRIKE_TIMING_IMPACT_VALID,
                  "trigger_to_impact_ms": 33, "trigger_to_impact_ms_precise": 32.6}
        player.record_manual_strike(0, 30.0, {"accepted": True, "detail": 5,
                                              "metrics": {**report, "sequence": 4}})
        player.record_manual_strike(0, 31.0, {"accepted": True, "detail": 6,
                                              "metrics": report, "metrics_fresh": True})
        self.assertAlmostEqual(player.residual_stats[0]["last"], 2.6)
        self.assertEqual(player.residual_stats[0]["n"], 1)
        # Stale or muted reports and rejected strikes never pair.
        player.record_manual_strike(0, None, {"accepted": True, "detail": 7,
                                              "metrics": {**report, "sequence": 6}})
        player.record_manual_strike(0, 30.0, {"accepted": False, "detail": 8,
                                              "metrics": {**report, "sequence": 7}, "metrics_fresh": True})
        self.assertEqual(player.residual_stats[0]["n"], 1)

    def test_reenumeration_discards_old_identity_telemetry(self):
        c = self.client()
        b = self.bridge(c)
        done = queue.Queue()
        b.strike_playback(0, 800, done.put)
        token = struct.unpack(">I", c.ser.writes[-1][7:11])[0]
        b.reset_status_cache(3)
        c._route_received(ack_frame(c, 0, token)[3:-2])
        self.assertIsNone(done.get(timeout=1))
        self.assertEqual(b.latency._per_slot, {})
        self.assertEqual(b.strike_timing, {})

    def test_close_settles_pending_and_stops_reader(self):
        c = self.client()
        done = queue.Queue()
        c.strike_async(0, 800, lambda *args: done.put(args))
        reader = c._rx_thread
        c.close()
        self.assertFalse(reader.is_alive())
        self.assertIsInstance(done.get(timeout=1)[2], bus.RingError)


if __name__ == "__main__":
    unittest.main()
