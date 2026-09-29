"""Metric checks independent of the aircraft response."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.sweep_attitude import heading_change_degrees, segment_metrics


def fixture(axis, sign=1):
    # A request of 30 degrees is limited to 5; response recrosses the band.
    rate, actuator, limited = (("p_rad_s", "aileron", "bank_command_limited") if axis == "roll"
                              else ("q_rad_s", "elevator", "pitch_command_limited"))
    offset = 3 if axis == "pitch" else 0
    radians = lambda values: [math.radians(offset + sign * v) for v in values]
    return {
        "time_s": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
        f"{axis}_request_rad": radians([0, 0] + [30] * 8 + [0] * 3),
        f"{axis}_command_rad": radians([0, 0] + [5] * 8 + [0] * 3),
        f"{axis}_rad": radians([0, 0, 0, 1, 4, 4.9, 5.5, 5.1, 5, 5, 5, 2, 1]),
        rate: [math.radians(2)] * 13,
        f"{actuator}_cmd_norm": [0.2] * 13,
        f"{actuator}_saturated": [0, 0, 1] + [0] * 10,
        limited: [0, 0] + [1] * 8 + [0] * 3,
        "airspeed_m_s": [50 + i for i in range(13)],
        "altitude_m": [900 - i for i in range(13)],
    }


class SweepMetricsTests(unittest.TestCase):
    def test_both_axes_and_directions_use_effective_step(self):
        for axis in ("roll", "pitch"):
            for sign in (1, -1):
                with self.subTest(axis=axis, sign=sign):
                    step, returning = segment_metrics(fixture(axis, sign), axis, 0.25)
                    self.assertAlmostEqual(step["effective_step_deg"], sign * 5)
                    self.assertAlmostEqual(step["overshoot_deg"], 0.5)
                    self.assertAlmostEqual(step["overshoot_percent"], 10)
                    self.assertEqual(step["rise_10_90_s"], 2)
                    self.assertEqual(step["settling_s"], 5)
                    self.assertEqual(step["time_10_s"], 3)
                    self.assertEqual(step["time_90_s"], 5)
                    self.assertEqual(step["peak_time_s"], 6)
                    self.assertEqual(step["settling_at_s"], 7)
                    self.assertAlmostEqual(step["last_second_rms_error_deg"], 0)
                    self.assertEqual(step["limited_percent"], 100)
                    self.assertEqual(step["saturation_percent"], 12.5)
                    self.assertEqual(returning["phase"], "return")
                    self.assertIsNone(returning["settling_s"])
                    self.assertAlmostEqual(returning["effective_step_deg"], -sign * 5)

    def test_no_effective_step_has_no_rise_or_overshoot(self):
        data = fixture("roll")
        data["roll_command_rad"] = [0] * 13
        data["roll_rad"] = [0] * 13
        step = segment_metrics(data, "roll", 0.25)[0]
        self.assertIsNone(step["rise_10_90_s"])
        self.assertIsNone(step["overshoot_percent"])
        self.assertEqual(step["settling_s"], 0)
        self.assertIsNone(step["peak_time_s"])

    def test_heading_crosses_north_without_a_full_turn_jump(self):
        angles = [359, 1, 5, 358, 355]
        result = heading_change_degrees([math.radians(v) for v in angles])
        for actual, expected in zip(result, [0, 2, 6, -1, -4]):
            self.assertAlmostEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
