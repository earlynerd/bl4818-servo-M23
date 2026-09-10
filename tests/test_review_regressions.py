import contextlib
import io
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import ring_bus
import tune_tool


class ReplyDeadlineTests(unittest.TestCase):
    def make_client(self):
        client = ring_bus.RingClientV2(port="test", timeout_ms=10)
        client.device_count = 1
        client._flush_rx = Mock()
        client._send_frame = Mock()
        return client

    def test_unrelated_frames_do_not_extend_any_reply_deadline(self):
        methods = (
            ("_recv_status_reply", (0,)),
            ("_recv_ack_reply", (0, ring_bus.SUBCMD_STRIKE)),
            ("query_config", (0,)),
            ("query_strike", (0,)),
            ("query_strike_timing", (0,)),
            ("query_timing", (0,)),
            ("enumerate", ()),
        )
        for name, args in methods:
            with self.subTest(method=name):
                client = self.make_client()
                client._recv_frame = Mock(return_value=b"\xff")
                with patch.object(ring_bus.time, "monotonic",
                                  side_effect=[0, .001, .002, .020]):
                    with self.assertRaises(ring_bus.RingTimeout):
                        getattr(client, name)(*args)
                self.assertEqual(client._recv_frame.call_count, 1)

    def test_matching_ack_before_deadline_is_accepted(self):
        client = self.make_client()
        client._recv_frame = Mock(return_value=bytes([
            ring_bus.CMD_ACK_BASE, ring_bus.SUBCMD_STRIKE,
            ring_bus.ACK_RESULT_OK, 0, 7,
        ]))
        with patch.object(ring_bus.time, "monotonic", side_effect=[0, .001, .009]):
            self.assertEqual(client._recv_ack_reply(0, ring_bus.SUBCMD_STRIKE).detail, 7)

    def test_matching_ack_after_deadline_is_rejected(self):
        client = self.make_client()
        client._recv_frame = Mock(return_value=bytes([
            ring_bus.CMD_ACK_BASE, ring_bus.SUBCMD_STRIKE,
            ring_bus.ACK_RESULT_OK, 0, 7,
        ]))
        with patch.object(ring_bus.time, "monotonic", side_effect=[0, .001, .020]):
            with self.assertRaises(ring_bus.RingTimeout):
                client._recv_ack_reply(0, ring_bus.SUBCMD_STRIKE)


class TuningCleanupTests(unittest.TestCase):
    def run_main(self, client, arguments):
        output = io.StringIO()
        with patch.object(tune_tool, "RingClientV2", return_value=client), \
             patch.object(tune_tool.time, "sleep"), \
             patch.object(sys, "argv", ["tune_tool.py", "-p", "TEST", "0", *arguments]), \
             contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            result = tune_tool.main()
        return result, output.getvalue()

    def test_lost_motion_reply_stops_before_closing(self):
        for flag, method in (("--velocity", "set_velocity"),
                             ("--duty", "set_duty"),
                             ("--position", "set_position")):
            with self.subTest(command=method):
                client = Mock()
                client.enumerate.return_value = 1
                getattr(client, method).side_effect = ring_bus.RingTimeout("lost reply")
                result, _ = self.run_main(client, [flag, "500", "--pre", "0", "--no-plot"])
                self.assertEqual(result, 1)
                calls = [call[0] for call in client.mock_calls]
                self.assertEqual(calls[-3:], [method, "stop", "close"])
                client.stop.assert_called_with(0, reply_mode=ring_bus.REPLY_MODE_ACK)

    def test_failed_stop_is_reported_and_port_is_closed(self):
        client = Mock()
        client.enumerate.return_value = 1
        client.set_velocity.side_effect = ring_bus.RingTimeout("lost reply")
        client.stop.side_effect = [None, ring_bus.RingTimeout("stop reply lost")]
        result, output = self.run_main(client, ["--velocity", "500", "--pre", "0", "--no-plot"])
        self.assertEqual(result, 1)
        self.assertIn("motor stop could not be confirmed", output)
        client.close.assert_called_once()

    def test_feedforward_failure_stops(self):
        client = Mock()
        client.enumerate.return_value = 1
        client.set_duty.side_effect = ring_bus.RingTimeout("lost reply")
        result, _ = self.run_main(client, ["--measure-ff"])
        self.assertEqual(result, 1)
        self.assertEqual([call[0] for call in client.mock_calls][-3:],
                         ["set_duty", "stop", "close"])

    def test_interrupt_during_motion_stops(self):
        client = Mock()
        client.enumerate.return_value = 1
        client.set_velocity.side_effect = KeyboardInterrupt()
        result, _ = self.run_main(client, ["--velocity", "500", "--pre", "0", "--no-plot"])
        self.assertEqual(result, 130)
        self.assertEqual([call[0] for call in client.mock_calls][-3:],
                         ["set_velocity", "stop", "close"])

    def test_successful_strike_retains_normal_hold_behavior(self):
        client = Mock()
        client.enumerate.return_value = 1
        with patch.object(tune_tool, "run_strike", return_value=([], 0, Mock())):
            result, _ = self.run_main(client, ["--strike", "500", "--no-plot"])
        self.assertEqual(result, 0)
        client.stop.assert_not_called()
        client.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
