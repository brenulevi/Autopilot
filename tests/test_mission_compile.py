"""Checks for the host-side APM2 compiler; no plotting dependencies needed."""

import struct
import sys
import unittest
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from compile_mission import compile_mission  # noqa: E402


class MissionCompilerTest(unittest.TestCase):
    def setUp(self):
        self.source = {
            "version": 2,
            "waypoints": [
                {"lat_deg": 30.0, "lon_deg": 0.0, "altitude_m": 914.4, "airspeed_m_s": 56.2},
                {"lat_deg": 30.0089932, "lon_deg": 0.0020769, "altitude_m": 920, "airspeed_m_s": 55},
            ],
        }

    def test_layout_and_crc(self):
        data = compile_mission(self.source)
        self.assertEqual(len(data), 48)
        self.assertEqual(struct.unpack_from("<4sHHI", data),
                         (b"APM2", 2, 2, 0))
        self.assertEqual(struct.unpack_from("<iiiHH", data, 28),
                         (300089932, 20769, 92000, 5500, 0))
        self.assertEqual(struct.unpack_from("<I", data, 44)[0], zlib.crc32(data[:-4]))

    def test_version_3_waypoint_types(self):
        self.source["version"] = 3
        self.source["waypoints"][1]["type"] = "fly_over"
        data = compile_mission(self.source)
        self.assertEqual(struct.unpack_from("<4sHHI", data), (b"APM3", 3, 2, 0))
        self.assertEqual(struct.unpack_from("<H", data, 26)[0], 0)
        self.assertEqual(struct.unpack_from("<H", data, 42)[0], 1)
        self.assertEqual(struct.unpack_from("<I", data, 44)[0], zlib.crc32(data[:-4]))
        for invalid in ("flyover", "fly-by", 0, None, [], True):
            self.source["waypoints"][1]["type"] = invalid
            with self.assertRaises(ValueError):
                compile_mission(self.source)
        self.source["version"] = 2
        self.source["waypoints"][1]["type"] = "fly_over"
        with self.assertRaises(ValueError):
            compile_mission(self.source)

    def test_rejects_duplicate_and_nonfinite_waypoints(self):
        self.source["waypoints"][1]["lat_deg"] = 30.0
        self.source["waypoints"][1]["lon_deg"] = 0.0
        with self.assertRaises(ValueError):
            compile_mission(self.source)
        self.source["waypoints"][1]["lat_deg"] = float("nan")
        with self.assertRaises(ValueError):
            compile_mission(self.source)

    def test_rejects_invalid_version_and_unrepresentable_distance(self):
        self.source["version"] = True
        with self.assertRaises(ValueError):
            compile_mission(self.source)
        self.source["version"] = 2
        self.source["waypoints"][1]["lat_deg"] = 30.0000089476
        self.source["waypoints"][1]["lon_deg"] = 0.000001049
        with self.assertRaises(ValueError):
            compile_mission(self.source)
        self.source["waypoints"][1]["lat_deg"] = 10 ** 1000
        with self.assertRaises(ValueError):
            compile_mission(self.source)

    def test_accepts_short_leg_across_dateline(self):
        self.source["waypoints"][0]["lat_deg"] = 0
        self.source["waypoints"][0]["lon_deg"] = 179.99999
        self.source["waypoints"][1]["lat_deg"] = 0
        self.source["waypoints"][1]["lon_deg"] = -179.99999
        data = compile_mission(self.source)
        self.assertEqual(struct.unpack_from("<ii", data, 12), (0, 1799999900))
        self.assertEqual(struct.unpack_from("<ii", data, 28), (0, -1799999900))


if __name__ == "__main__":
    unittest.main()
