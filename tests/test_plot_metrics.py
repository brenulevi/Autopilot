"""Behavior checks for reported tracking metrics; run with the plotting venv."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.plot_logs import tracking_metrics


def log(roll_deg, target_deg=5.0):
    count = len(roll_deg)
    return {
        "time_s": list(range(count)),
        "roll_command_rad": [0.0] + [math.radians(target_deg)] * (count - 1),
        "roll_rad": [math.radians(value) for value in roll_deg],
        "aileron_cmd_norm": [0.1] * count,
        "aileron_saturated": [0] * count,
        "bank_command_limited": [0] * count,
        "roll_kp": [4.0] * count,
        "roll_kd": [0.5] * count,
    }


class TrackingMetricsTests(unittest.TestCase):
    def test_settling_uses_last_entry_after_recrossing(self):
        result = tracking_metrics(log([0, 0, 4.9, 5.5, 5.1, 5.0]), 0.25)["segments"][0]
        self.assertAlmostEqual(result["overshoot_deg"], 0.5)
        self.assertAlmostEqual(result["overshoot_percent"], 10.0)
        self.assertEqual(result["settling_time_s"], 3)
        self.assertAlmostEqual(result["end_error_deg"], 0)

    def test_negative_step_uses_direction_for_overshoot(self):
        result = tracking_metrics(log([0, 0, -5.4, -5.1], -5), 0.25)["segments"][0]
        self.assertAlmostEqual(result["overshoot_deg"], 0.4)
        self.assertAlmostEqual(result["end_error_deg"], 0.1)
        self.assertEqual(result["settling_time_s"], 2)

    def test_unsettled_and_no_step_are_not_reported_as_success(self):
        result = tracking_metrics(log([0, 0, 3, 4]), 0.25)["segments"][0]
        self.assertIsNone(result["settling_time_s"])
        self.assertEqual(result["overshoot_deg"], 0)
        self.assertEqual(tracking_metrics(log([0, 0, 0], 0), 0.25)["segments"], [])


if __name__ == "__main__":
    unittest.main()
