"""Trigger-to-impact latency model (server copy); no hardware access.

tests/test_latency_model.cjs asserts the same parity scenario against
player/latency_model.js, so the browser and server models stay in step.
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from ring_midi_server import LatencyTracker

# (slot, current mA, impact ms) learned in order, then (slot, current) -> estimate.
PARITY_SAMPLES = [(0, 150, 48.0), (0, 220, 41.5), (0, 310, 36.2), (0, 480, 30.9),
                  (0, 220, 40.7), (1, 260, 40.0)]
PARITY_ESTIMATES = [(0, 100, 58.787754), (0, 260, 38.35218), (0, 700, 25.587631),
                    (1, 150, 48.219028), (1, 600, 27.6378), (2, 333, 34.951512)]


class LatencyTrackerTests(unittest.TestCase):
    def test_default_without_data(self):
        self.assertEqual(LatencyTracker().compensation_ms(0, 400), LatencyTracker.DEFAULT_MS)

    def test_one_sample_is_reproduced_at_its_own_current(self):
        for current in (30, 100, 137, 333, 800, 2900):
            tracker = LatencyTracker()
            tracker.update(0, current, 40.0)
            self.assertAlmostEqual(tracker.compensation_ms(0, current), 40.0)
            self.assertAlmostEqual(tracker.compensation_ms(5, current), 40.0)  # shared curve

    def test_common_range_has_ten_distinct_bins(self):
        currents = (100, 120, 140, 170, 200, 240, 280, 340, 400, 480)
        bins = {int(LatencyTracker._position(ma) + 0.5) for ma in currents}
        self.assertEqual(len(bins), len(currents))

    def test_interpolation_is_exact_for_constant_torque_scaling(self):
        tracker = LatencyTracker()
        tracker.update(0, 100, 60.0)
        tracker.update(0, 400, 30.0)
        self.assertAlmostEqual(tracker.compensation_ms(0, 200), 60.0 / math.sqrt(2))
        self.assertAlmostEqual(tracker.compensation_ms(0, 150), 60.0 / math.sqrt(1.5))

    def test_sparse_slot_follows_the_shared_shape(self):
        tracker = LatencyTracker()
        tracker.update(1, 100, 60.0)
        tracker.update(1, 400, 33.0)
        tracker.update(0, 100, 66.0)  # this mallet is slower and has one sample
        shared_100 = 60.0 + LatencyTracker.EMA_ALPHA * (66.0 - 60.0)
        self.assertAlmostEqual(tracker.compensation_ms(0, 400), 66.0 * 33.0 / shared_100)

    def test_parity_with_browser_model(self):
        tracker = LatencyTracker()
        for slot, current, impact in PARITY_SAMPLES:
            tracker.update(slot, current, impact)
        for slot, current, expected in PARITY_ESTIMATES:
            self.assertAlmostEqual(tracker.compensation_ms(slot, current), expected, places=5)

    def test_unusable_samples_are_ignored(self):
        tracker = LatencyTracker()
        for current, impact in ((0, 30.0), (None, 30.0), (400, None), (400, 0.0)):
            tracker.update(0, current, impact)
        self.assertEqual(tracker._per_slot.get(0, {}), {})


if __name__ == "__main__":
    unittest.main()
