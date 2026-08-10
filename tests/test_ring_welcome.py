import argparse
import contextlib
import io
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import ring_welcome as welcome  # noqa: E402


class FakeClient:
    def __init__(self, base_url):
        self.base_url = base_url

    def pitches(self):
        return [{"pitch": 60}]


class WelcomeDelayTests(unittest.TestCase):
    def _run_until_greeting(self, config_delay=60, cli_delay=None):
        config = {
            "ip_range": "192.0.2.0/30",
            "interval_s": 25,
            "arrival_delay_s": config_delay,
            "people": [{
                "name": "Ada",
                "match": {"hostname": ["ada"]},
                "song": {"builtin": "chirp"},
            }],
        }
        device = welcome.Device("192.0.2.1")
        device.add_name("ada")
        clock = [100.0]
        plays = []

        def fake_sleep(seconds):
            clock[0] += seconds
            if plays:
                raise KeyboardInterrupt

        def fake_play(client, person, pitches, log=print):
            plays.append((person["name"], clock[0]))
            return True

        args = argparse.Namespace(
            config="unused.json",
            state="unused-state.json",
            range=None,
            interval=None,
            delay=cli_delay,
            ping_timeout=1,
            no_netbios=True,
        )
        with (
            patch.object(welcome, "load_config", return_value=config),
            patch.object(welcome, "load_state", return_value={}),
            patch.object(welcome, "save_state"),
            patch.object(welcome, "scan_network", return_value=[device]),
            patch.object(welcome, "ChimeClient", FakeClient),
            patch.object(welcome, "play_person_song", side_effect=fake_play),
            patch.object(welcome.time, "monotonic", side_effect=lambda: clock[0]),
            patch.object(welcome.time, "sleep", side_effect=fake_sleep),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            result = welcome.cmd_watch(args)
        return result, plays

    def test_configured_delay_is_measured_from_first_detection(self):
        result, plays = self._run_until_greeting(config_delay=60)

        self.assertEqual(result, 0)
        self.assertEqual(plays, [("Ada", 160.0)])

    def test_cli_delay_overrides_config(self):
        result, plays = self._run_until_greeting(config_delay=60, cli_delay=5)

        self.assertEqual(result, 0)
        self.assertEqual(plays, [("Ada", 105.0)])

    def test_non_finite_delay_is_rejected(self):
        args = argparse.Namespace(
            config="unused.json",
            state="unused-state.json",
            range=None,
            interval=None,
            delay=float("nan"),
            ping_timeout=1,
            no_netbios=True,
        )
        config = {"ip_range": "192.0.2.0/30", "people": []}

        with (
            patch.object(welcome, "load_config", return_value=config),
            patch.object(welcome, "load_state", return_value={}),
            contextlib.redirect_stderr(io.StringIO()),
        ):
            self.assertEqual(welcome.cmd_watch(args), 1)


if __name__ == "__main__":
    unittest.main()
