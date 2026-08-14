import struct
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from ring_bus import (  # noqa: E402
    ACK_RESULT_OK,
    ACK_RESULT_REJECT_NOT_READY,
    CMD_ACK_BASE,
    CMD_ADDR_BASE,
    CMD_ENTER_CT,
    CMD_ENTER_SF,
    CMD_SET_ADDRESS,
    FAULT_CODES,
    PREAMBLE,
    REPLY_MODE_ACK,
    REPLY_MODE_NONE,
    SUBCMD_REPLY_ACK,
    SUBCMD_REPLY_NONE,
    SUBCMD_QUERY_CONFIG,
    SUBCMD_STRIKE,
    SUBCMD_STRIKE_CANCEL,
    STRIKE_WARNING_HOME_SHIFT,
    CommandAck,
    RingClientV2,
    RingTimeout,
    StrikeStatus,
    crc16_ccitt,
)
from firmware_image import image_metadata, prepare_image  # noqa: E402
from ring_midi_server import (  # noqa: E402
    Bridge,
    FirmwareUpdateManager,
    LatencyTracker,
    Player,
)


class BufferSerial:
    def __init__(self, incoming=b""):
        self.incoming = bytearray(incoming)
        self.written = bytearray()
        self.flush_count = 0

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, count=1):
        if not self.incoming:
            return b""
        chunk = bytes(self.incoming[:count])
        del self.incoming[:count]
        return chunk

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def flush(self):
        self.flush_count += 1


class QueuedReplyClient(RingClientV2):
    def __init__(self, replies):
        super().__init__(port="test", timeout_ms=25)
        self.ser = BufferSerial()
        self.device_count = 4
        self.replies = list(replies)
        self.flush_rx_count = 0
        self.discard_rx_count = 0

    def _flush_rx(self):
        self.flush_rx_count += 1

    def _discard_rx_available(self):
        self.discard_rx_count += 1
        super()._discard_rx_available()

    def _recv_frame(self, timeout_ms=None):
        if not self.replies:
            raise RingTimeout("test reply queue exhausted")
        return self.replies.pop(0)


class FakeLatency:
    def __init__(self, compensation_by_address=None):
        self.compensation_by_address = compensation_by_address or {}

    def compensation_ms(self, address, current_ma):
        return float(self.compensation_by_address.get(address, 0.0))


class FakeBridge:
    def __init__(self, count=4):
        self.count = count
        self.maintenance = threading.Event()
        self.latency = FakeLatency()
        self.single_calls = []
        self.chord_calls = []
        self.single_result = {
            "accepted": True,
            "result_name": "OK",
            "server_timing_us": {},
        }

    def strike(self, address, current_ma):
        self.single_calls.append((address, current_ma))
        return dict(self.single_result)

    def strike_chord(self, items):
        self.chord_calls.append([dict(item) for item in items])
        return [
            {
                "accepted": None,
                "sent": True,
                "result_name": "SENT_NO_ACK",
                "server_timing_us": {},
            }
            for _ in items
        ]


class NeverStopsThread:
    def __init__(self):
        self.join_timeout = None

    def is_alive(self):
        return True

    def join(self, timeout=None):
        self.join_timeout = timeout


class FakeHomeClient:
    def __init__(self, reply):
        self.reply = reply
        self.calls = []

    def strike_home(self, address, reply_mode):
        self.calls.append((address, reply_mode))
        return self.reply


class FakeStrikeClient:
    def __init__(self, replies):
        self.replies = list(replies)

    def strike(self, address, current_ma, reply_mode):
        return self.replies.pop(0)


class FakeChordClient:
    def __init__(self):
        self.calls = []

    def strike_burst(self, pairs, reply_mode):
        self.calls.append((list(pairs), reply_mode))
        return [None] * len(pairs)


class FakeCancelClient:
    def __init__(self, replies):
        self.replies = list(replies)
        self.calls = []

    def strike_cancel(self, address, reply_mode):
        self.calls.append((address, reply_mode))
        reply = self.replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return reply


class FakeFleetConfigClient:
    def __init__(self):
        self.calls = []

    def set_pid(self, address, kp, ki, kd, reply_mode):
        self.calls.append(("velocity_pid", address, kp, ki, kd, reply_mode))
        return CommandAck(address, 0x07, ACK_RESULT_OK, 0)

    def clear_fault(self, address, reply_mode):
        self.calls.append(("clear", address, reply_mode))
        return CommandAck(address, 0x04, ACK_RESULT_OK, 0)

    def strike_home(self, address, reply_mode):
        self.calls.append(("home", address, reply_mode))
        return CommandAck(address, 0x0D, ACK_RESULT_OK, 0)


class FakeHealth:
    def __init__(self):
        self.failures = []

    def record_ok(self, address, elapsed_us):
        pass

    def record_fail(self, address, kind, message):
        self.failures.append((address, kind, message))


class RingParserTests(unittest.TestCase):
    def test_crc_failure_rescans_for_embedded_valid_frame(self):
        client = RingClientV2(port="test", timeout_ms=25)
        wanted_payload = bytes([0x40, 0x01])
        inner_frame = client._build_frame(wanted_payload)

        # The complete valid frame sits inside the payload of a corrupt outer
        # frame. Recovery must slip one byte and find it rather than discard it.
        outer_body = bytes([8]) + inner_frame + b"\x00"
        corrupt_crc = crc16_ccitt(outer_body) ^ 0x0001
        client.ser = BufferSerial(
            PREAMBLE + outer_body + struct.pack(">H", corrupt_crc)
        )

        self.assertEqual(client._recv_frame(), wanted_payload)

    def test_new_fault_names_and_home_shift_warning_are_exposed(self):
        self.assertEqual(FAULT_CODES[5], "HOMING_LIMIT")
        self.assertEqual(FAULT_CODES[6], "STRIKE_LIMIT")
        status = StrikeStatus(
            address=0,
            state=0,
            homed=1,
            flags=STRIKE_WARNING_HOME_SHIFT,
        )
        self.assertTrue(status.home_shift_warning)

    def test_query_config_decodes_complete_tuning_snapshot(self):
        values = (
            1536, 10, 20, 220,
            1200, 10, 20,
            96, 5, 3200,
            1024, 300, -100,
            30, 250, -25,
            0x000F, 1,
        )
        payload = bytes([0x42]) + struct.pack(">hhhhhhhhhHhhhhhhHB", *values)
        client = QueuedReplyClient([payload])

        config = client.query_config(2)

        self.assertEqual(config.address, 2)
        self.assertEqual(config.velocity_kp, 1536)
        self.assertEqual(config.torque_limit_ma, 3200)
        self.assertEqual(config.mute_engage_offset, -25)
        self.assertTrue(config.persisted)
        self.assertTrue(config.zero_valid)
        self.assertTrue(config.csn_polarity_valid)
        self.assertTrue(config.strike_calibration_valid)
        self.assertEqual(config.csn_assert_level, 1)
        self.assertIn(bytes([SUBCMD_QUERY_CONFIG]), client.ser.written)


class RingEnumerationTests(unittest.TestCase):
    def test_enumeration_skips_delayed_control_and_seed_echoes(self):
        client = QueuedReplyClient([
            bytes([CMD_ENTER_SF]),
            bytes([CMD_ENTER_SF]),
            bytes([CMD_SET_ADDRESS, 0]),
            bytes([CMD_SET_ADDRESS, 1]),
            bytes([CMD_ENTER_CT]),
        ])

        self.assertEqual(client.enumerate(), 1)
        self.assertEqual(client.device_count, 1)


class RingBurstTests(unittest.TestCase):
    @staticmethod
    def ack(address, result=ACK_RESULT_OK, detail=0):
        return bytes([
            CMD_ACK_BASE | address,
            SUBCMD_STRIKE,
            result,
            (detail >> 8) & 0xFF,
            detail & 0xFF,
        ])

    def test_burst_writes_far_to_near_and_matches_out_of_order_acks(self):
        client = QueuedReplyClient([
            self.ack(1, detail=11),
            self.ack(3, detail=33),
            self.ack(2, detail=22),
        ])

        results = client.strike_burst(
            [(1, 1100), (3, 1300), (2, 1200)], reply_mode=REPLY_MODE_ACK
        )

        frame_size = 9  # preamble + len + four-byte payload + CRC
        wire = bytes(client.ser.written)
        commands = [
            wire[offset + 3]
            for offset in range(0, len(wire), frame_size)
        ]
        subcommands = [
            wire[offset + 4]
            for offset in range(0, len(wire), frame_size)
        ]
        self.assertEqual(commands, [CMD_ADDR_BASE | 3, CMD_ADDR_BASE | 2, CMD_ADDR_BASE | 1])
        self.assertTrue(all(value == (SUBCMD_STRIKE | SUBCMD_REPLY_ACK) for value in subcommands))
        self.assertEqual([reply.address for reply in results], [1, 3, 2])
        self.assertEqual([reply.detail for reply in results], [11, 33, 22])
        self.assertEqual(client.ser.flush_count, 1)
        self.assertEqual(client.flush_rx_count, 1)
        self.assertEqual(client.discard_rx_count, 0)

    def test_no_reply_burst_sets_reply_none_on_every_command(self):
        client = QueuedReplyClient([])

        results = client.strike_burst(
            [(1, 1100), (3, 1300)], reply_mode=REPLY_MODE_NONE
        )

        frame_size = 9
        wire = bytes(client.ser.written)
        subcommands = [
            wire[offset + 4]
            for offset in range(0, len(wire), frame_size)
        ]
        self.assertTrue(
            all(value == (SUBCMD_STRIKE | SUBCMD_REPLY_NONE) for value in subcommands)
        )
        self.assertEqual(results, [None, None])
        self.assertEqual(client.flush_rx_count, 0)
        self.assertEqual(client.discard_rx_count, 1)


class BridgeTests(unittest.TestCase):
    def test_home_returns_truthful_rejection_ack(self):
        bridge = object.__new__(Bridge)
        bridge.lock = threading.Lock()
        bridge.client = FakeHomeClient(CommandAck(
            address=2,
            subcmd=0x0D,
            result=ACK_RESULT_REJECT_NOT_READY,
            detail=7,
        ))

        results = bridge.home([2])

        self.assertEqual(bridge.client.calls, [(2, REPLY_MODE_ACK)])
        self.assertFalse(results[0]["accepted"])
        self.assertEqual(results[0]["result_name"], "REJECT_NOT_READY")

    def test_rejected_strike_does_not_replace_latency_attribution(self):
        bridge = object.__new__(Bridge)
        bridge.lock = threading.Lock()
        bridge.latency = LatencyTracker()
        bridge.health = FakeHealth()
        bridge.strike_timing = {}
        bridge.client = FakeStrikeClient([
            CommandAck(0, SUBCMD_STRIKE, ACK_RESULT_REJECT_NOT_READY, 0),
            CommandAck(0, SUBCMD_STRIKE, ACK_RESULT_OK, 0),
        ])

        rejected = bridge.strike(0, 900)
        self.assertFalse(rejected["accepted"])
        self.assertIsNone(bridge.latency.last_accepted_current(0))

        accepted = bridge.strike(0, 1200)
        self.assertTrue(accepted["accepted"])
        self.assertEqual(bridge.latency.last_accepted_current(0), 1200)

    def test_chord_sends_without_ack_and_clears_latency_attribution(self):
        bridge = object.__new__(Bridge)
        bridge.lock = threading.Lock()
        bridge.latency = LatencyTracker()
        bridge.health = FakeHealth()
        bridge.client = FakeChordClient()
        bridge.latency.record_accepted(1, 900)
        bridge.latency.record_accepted(3, 1000)

        results = bridge.strike_chord([
            {"address": 1, "current_ma": 1100},
            {"address": 3, "current_ma": 1300},
        ])

        self.assertEqual(
            bridge.client.calls,
            [([(1, 1100), (3, 1300)], REPLY_MODE_NONE)],
        )
        self.assertTrue(all(result["sent"] for result in results))
        self.assertTrue(all(result["accepted"] is None for result in results))
        self.assertIsNone(bridge.latency.last_accepted_current(1))
        self.assertIsNone(bridge.latency.last_accepted_current(3))

    def test_cancel_all_reports_every_failure_and_continues(self):
        bridge = object.__new__(Bridge)
        bridge.lock = threading.Lock()
        bridge.count = 3
        bridge.player = type("FakePlayer", (), {
            "cancel": lambda self: setattr(self, "cancelled", True),
        })()
        bridge.player.cancelled = False
        bridge.health = FakeHealth()
        bridge.client = FakeCancelClient([
            CommandAck(0, SUBCMD_STRIKE_CANCEL, ACK_RESULT_OK, 10),
            RingTimeout("lost cancel reply"),
            CommandAck(2, SUBCMD_STRIKE_CANCEL, ACK_RESULT_REJECT_NOT_READY, 12),
        ])

        result = bridge.cancel_all()

        self.assertTrue(bridge.player.cancelled)
        self.assertEqual(
            bridge.client.calls,
            [(0, REPLY_MODE_ACK), (1, REPLY_MODE_ACK), (2, REPLY_MODE_ACK)],
        )
        self.assertFalse(result["ok"])
        self.assertEqual(result["failed"], [1, 2])
        self.assertEqual(len(result["results"]), 3)
        self.assertEqual(bridge.health.failures[0][0:2], (1, "timeouts"))

    def test_fleet_config_can_apply_same_group_to_selected_addresses(self):
        bridge = object.__new__(Bridge)
        bridge.lock = threading.Lock()
        bridge.client = FakeFleetConfigClient()

        results = bridge.apply_config(
            [0, 2, 3], "velocity_pid", {"kp": 100, "ki": 20, "kd": 3}
        )

        self.assertTrue(all(result["accepted"] for result in results))
        self.assertEqual([call[1] for call in bridge.client.calls], [0, 2, 3])
        self.assertTrue(all(call[-1] == REPLY_MODE_ACK for call in bridge.client.calls))

    def test_recovery_clears_fault_before_rehoming_each_address(self):
        bridge = object.__new__(Bridge)
        bridge.lock = threading.Lock()
        bridge.client = FakeFleetConfigClient()

        results = bridge.recover([1, 3])

        self.assertTrue(all(result["accepted"] for result in results))
        self.assertEqual(
            bridge.client.calls,
            [
                ("clear", 1, REPLY_MODE_ACK),
                ("home", 1, REPLY_MODE_ACK),
                ("clear", 3, REPLY_MODE_ACK),
                ("home", 3, REPLY_MODE_ACK),
            ],
        )


class PlayerTests(unittest.TestCase):
    @staticmethod
    def wait_for_player(player):
        thread = player._thread
        if isinstance(thread, threading.Thread):
            thread.join(timeout=1.0)
            if thread.is_alive():
                raise AssertionError("player worker did not finish")

    def test_same_midi_time_notes_use_one_unacknowledged_burst(self):
        bridge = FakeBridge()
        player = Player(bridge)
        player.play([
            {"t_ms": 0, "address": 1, "nominal_current_ma": 1100},
            {"t_ms": 0, "address": 3, "nominal_current_ma": 1300},
        ])
        self.wait_for_player(player)

        self.assertEqual(len(bridge.chord_calls), 1)
        self.assertEqual(
            [item["address"] for item in bridge.chord_calls[0]],
            [3, 1],
        )
        stats = player.status()["dispatch_stats"]
        self.assertEqual(stats["attempted"], 2)
        self.assertEqual(stats["accepted"], 0)
        self.assertEqual(stats["unacknowledged"], 2)
        self.assertEqual(stats["bursts"], 1)

    def test_matching_trigger_times_do_not_merge_distinct_midi_times(self):
        bridge = FakeBridge()
        bridge.latency = FakeLatency({1: 50.0, 2: 100.0})
        player = Player(bridge)
        player.play([
            {"t_ms": 50, "address": 1, "nominal_current_ma": 1100},
            {"t_ms": 100, "address": 2, "nominal_current_ma": 1200},
        ])
        self.wait_for_player(player)

        self.assertEqual(bridge.chord_calls, [])
        self.assertEqual(bridge.single_calls, [(1, 1100), (2, 1200)])
        stats = player.status()["dispatch_stats"]
        self.assertEqual(stats["accepted"], 2)
        self.assertEqual(stats["bursts"], 0)

    def test_same_midi_time_remains_chord_when_compensation_splits_deadlines(self):
        bridge = FakeBridge()
        bridge.latency = FakeLatency({1: 0.0, 2: 5.0})
        player = Player(bridge)
        player.play([
            {"t_ms": 5, "address": 1, "nominal_current_ma": 1100},
            {"t_ms": 5, "address": 2, "nominal_current_ma": 1200},
        ])
        self.wait_for_player(player)

        self.assertEqual(
            [[item["address"] for item in call] for call in bridge.chord_calls],
            [[2], [1]],
        )
        self.assertEqual(bridge.single_calls, [])
        stats = player.status()["dispatch_stats"]
        self.assertEqual(stats["unacknowledged"], 2)
        self.assertEqual(stats["bursts"], 0)

    def test_rejected_strike_is_reported(self):
        bridge = FakeBridge()
        bridge.single_result = {
            "accepted": False,
            "result_name": "REJECT_NOT_READY",
            "server_timing_us": {},
        }
        player = Player(bridge)
        player.play([
            {"t_ms": 0, "address": 0, "nominal_current_ma": 1000},
        ])
        self.wait_for_player(player)

        stats = player.status()["dispatch_stats"]
        self.assertEqual(stats["attempted"], 1)
        self.assertEqual(stats["accepted"], 0)
        self.assertEqual(stats["rejected"], 1)
        self.assertEqual(stats["reasons"], {"REJECT_NOT_READY": 1})

    def test_event_address_must_be_enumerated(self):
        player = Player(FakeBridge(count=2))
        with self.assertRaisesRegex(ValueError, "out of range"):
            player.play([
                {"t_ms": 0, "address": 2, "nominal_current_ma": 1000},
            ])

    def test_firmware_maintenance_rejects_new_playback(self):
        bridge = FakeBridge()
        bridge.maintenance.set()
        with self.assertRaisesRegex(RuntimeError, "firmware update"):
            Player(bridge).play([
                {"t_ms": 0, "address": 0, "nominal_current_ma": 1000},
            ])

    def test_live_worker_identity_is_not_discarded(self):
        player = Player(FakeBridge())
        live = NeverStopsThread()
        player._thread = live
        player._id = "still-running"

        with self.assertRaisesRegex(RuntimeError, "did not stop"):
            player._stop_and_join()

        self.assertIs(player._thread, live)
        self.assertEqual(player._id, "still-running")
        self.assertEqual(live.join_timeout, 2.0)

    def test_browser_does_not_wall_clock_cancel_natural_completion(self):
        html = (REPO_ROOT / "player" / "midi_player.html").read_text(encoding="utf-8")
        self.assertNotIn('setTimeout(() => stopPlayback("ended")', html)
        self.assertIn('if (reason !== "ended")', html)


class FirmwareUpdateManagerTests(unittest.TestCase):
    def test_build_freezes_validated_binary_and_source_identity(self):
        bridge = type("BuildBridge", (), {})()
        manager = FirmwareUpdateManager(bridge)
        raw = struct.pack("<II", 0x20000FF0, 0x00000009) + b"server build"

        with tempfile.TemporaryDirectory() as tmp:
            image_path = Path(tmp) / "build" / "m2003-motor.bin"
            image_path.parent.mkdir()
            image_path.write_bytes(raw)

            def fake_run(command, **kwargs):
                if command == ["make"]:
                    return type("Result", (), {
                        "returncode": 0,
                        "stdout": "build ok\n",
                        "stderr": "",
                    })()
                if command[:2] == ["git", "rev-parse"]:
                    return type("Result", (), {"stdout": "abc123def456\n"})()
                if command[:2] == ["git", "status"]:
                    return type("Result", (), {"stdout": ""})()
                raise AssertionError(f"unexpected command {command}")

            with patch("ring_midi_server.ROOT", Path(tmp)), \
                 patch.object(FirmwareUpdateManager, "IMAGE_PATH", image_path), \
                 patch("ring_midi_server.subprocess.run", side_effect=fake_run):
                manager.start_build(20260802)
                deadline = time.monotonic() + 1.0
                while manager.snapshot()["busy"] and time.monotonic() < deadline:
                    time.sleep(0.005)

        state = manager.snapshot()
        expected = prepare_image(raw)
        self.assertEqual(state["phase"], "ready")
        self.assertEqual(state["artifact"]["commit"], "abc123def456")
        self.assertFalse(state["artifact"]["dirty"])
        self.assertEqual(state["artifact"]["image_version"], 20260802)
        self.assertEqual(manager._image, expected)

    def test_update_reuses_bridge_connection_and_clears_maintenance(self):
        class FakePlayer:
            def __init__(self):
                self.cancelled = 0

            def cancel(self):
                self.cancelled += 1

        class FakeFirmwareBridge:
            def __init__(self):
                self.lock = threading.Lock()
                self.maintenance = threading.Event()
                self.client = object()
                self.player = FakePlayer()
                self.count = 3

        bridge = FakeFirmwareBridge()
        manager = FirmwareUpdateManager(bridge)
        image = prepare_image(
            struct.pack("<II", 0x20000FF0, 0x00000009) + b"firmware"
        )
        meta = image_metadata(image, image_version=11)
        manager._image = image
        manager._meta = meta
        manager._state["artifact"] = {"image_crc32": meta.image_crc32}
        calls = []
        release_update = threading.Event()

        def fake_update(client, frozen_image, frozen_meta, **kwargs):
            self.assertIs(client, bridge.client)
            self.assertIs(frozen_image, image)
            self.assertIs(frozen_meta, meta)
            self.assertTrue(bridge.maintenance.is_set())
            self.assertTrue(kwargs["broadcast_all"])
            kwargs["progress"]({
                "phase": "writing",
                "message": "pages 54/54",
                "done": 54,
                "total": 54,
            })
            calls.append(client)
            release_update.wait(timeout=1.0)
            return {
                "count": 3,
                "addresses": [0, 1, 2],
                "repaired": [],
                "image_size": meta.image_size,
                "image_crc32": meta.image_crc32,
                "image_version": meta.image_version,
                "loader_versions": [3, 3, 3],
            }

        with patch("ring_midi_server.update_ring", side_effect=fake_update):
            started = manager.start_update(meta.image_crc32)
            self.assertTrue(started["busy"])
            self.assertTrue(bridge.maintenance.is_set())
            release_update.set()
            deadline = time.monotonic() + 1.0
            while manager.snapshot()["busy"] and time.monotonic() < deadline:
                time.sleep(0.005)

        state = manager.snapshot()
        self.assertEqual(calls, [bridge.client])
        self.assertEqual(bridge.player.cancelled, 1)
        self.assertFalse(bridge.maintenance.is_set())
        self.assertEqual(state["phase"], "complete")
        self.assertEqual(state["result"]["addresses"], [0, 1, 2])


if __name__ == "__main__":
    unittest.main()
