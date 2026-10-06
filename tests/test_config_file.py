"""Exercise the real binary config tool and simulator (Python standard library)."""

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
import zlib

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--tool", required=True, type=Path)
parser.add_argument("--sim", required=True, type=Path)
args, remaining = parser.parse_known_args()
TOOL = args.tool.resolve()
SIM = args.sim.resolve()
ROOT = Path(__file__).resolve().parents[1]


class ConfigFileTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.base = self.work / "base.apcf"
        self.run_command(TOOL, "create", self.base)

    def run_command(self, exe, *arguments, success=True):
        result = subprocess.run([str(exe), *map(str, arguments)], cwd=self.work,
                                capture_output=True, text=True, timeout=30)
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout)
        return result

    def simulate(self, name, *arguments, success=True):
        output = self.work / (name + ".csv")
        result = self.run_command(SIM, "--mode", "roll-hold", "--duration", "2.1",
                                  "--output", output, *arguments, success=success)
        return output, result

    def test_attitude_limit_overrides(self):
        for axis, flag, limits in (("roll", "bank", (20, 40)), ("pitch", "pitch", (10, 45))):
            for limit in limits:
                with self.subTest(axis=axis, limit=limit):
                    path = self.work / f"{axis}_{limit}.csv"
                    self.run_command(SIM, "--mode", f"{axis}-hold", "--duration", "2.1",
                                     f"--{flag}-deg", "30", f"--{flag}-limit-deg", str(limit),
                                     "--output", path)
                    with path.open() as stream:
                        row = list(csv.DictReader(stream))[-1]
                    request = float(row[f"{axis}_request_rad"])
                    effective = float(row[f"{axis}_command_rad"])
                    self.assertAlmostEqual(effective, min(request, math.radians(limit)), places=6)
                    self.assertEqual(int(row[f"{'bank' if axis == 'roll' else 'pitch'}_command_limited"]),
                                     int(request > math.radians(limit)))
            for invalid in ("0", "90", "nan"):
                self.run_command(SIM, "--mode", f"{axis}-hold", f"--{flag}-limit-deg", invalid, success=False)

    def test_wire_format_and_edit(self):
        # Independent schema oracle; not a C encode/decode roundtrip.
        fields = [2, 2, math.radians(20), 1, 1.5, 4, math.radians(10), .5,
                  .08, .005, 0, 1, .015, .05, math.radians(3), 4, 1, 1, 4, .5]
        expected = struct.pack("<4sHHI20f", b"APCF", 2, 80, 0, *fields)
        expected += struct.pack("<I", zlib.crc32(expected))
        self.assertEqual(self.base.read_bytes(), expected)
        shown = self.run_command(TOOL, "show", self.base).stdout
        self.assertIn("sequence=0", shown)
        self.assertIn("roll_attitude_gain=2", shown)
        edited = self.work / "edited.apcf"
        self.run_command(TOOL, "set", self.base, edited, "roll_attitude_gain=1", "l1_period_s=8")
        values = struct.unpack("<4sHHI20fI", edited.read_bytes())
        self.assertEqual(values[3], 1)
        self.assertEqual(values[4], 1)
        self.assertEqual(values[19], 8)
        self.assertEqual(self.base.read_bytes(), expected)
        self.run_command(SIM, "--config", self.base, "--output", self.base, success=False)
        self.assertEqual(self.base.read_bytes(), expected)
        self.run_command(TOOL, "set", self.base, self.base, "roll_attitude_gain=2", success=False)
        self.run_command(TOOL, "create", self.base, success=False)
        self.assertEqual(self.base.read_bytes(), expected)

    def test_bad_records_and_parameters(self):
        baseline = self.base.read_bytes()
        corruptions = [baseline[:-1], baseline + b"x", bytes(96)]
        for offset, replacement in [(4, b"\x03"), (6, b"\x00"),
                                    (12, struct.pack("<f", float("nan"))),
                                    (52, struct.pack("<f", 1.0))]:
            data = bytearray(baseline)
            data[offset:offset + len(replacement)] = replacement
            data[-4:] = struct.pack("<I", zlib.crc32(data[:-4]))
            corruptions.append(bytes(data))
        bad_crc = bytearray(baseline)
        bad_crc[-1] ^= 1
        corruptions.append(bytes(bad_crc))
        for index, data in enumerate(corruptions):
            path = self.work / f"bad{index}.apcf"
            path.write_bytes(data)
            self.run_command(TOOL, "show", path, success=False)
            output, _ = self.simulate(f"bad{index}", "--config", path, success=False)
            self.assertFalse(output.exists())
        for value in ["roll_attitude_gain=0", "pitch_rate_kp=-1", "max_bank_rad=2",
                      "airspeed_ki=nan", "l1_period_s=31", "unknown=1", "roll_attitude_gain=1x"]:
            output = self.work / "invalid.apcf"
            self.run_command(TOOL, "set", self.base, output, value, success=False)
            self.assertFalse(output.exists())
        missing = self.work / "missing.apcf"
        self.simulate("missing", "--config", missing, success=False)
        self.simulate("duplicate", "--config", self.base, "--config", self.base, success=False)
        exhausted = bytearray(baseline)
        exhausted[8:12] = struct.pack("<I", 0xFFFFFFFF)
        exhausted[-4:] = struct.pack("<I", zlib.crc32(exhausted[:-4]))
        self.base.write_bytes(exhausted)
        self.run_command(TOOL, "set", self.base, self.work / "wrap.apcf", "l1_period_s=4", success=False)

    def test_simulator_configuration_and_override_precedence(self):
        default, _ = self.simulate("default")
        loaded, _ = self.simulate("loaded", "--config", self.base)
        self.assertEqual(default.read_bytes(), loaded.read_bytes())
        edited = self.work / "limited.apcf"
        self.run_command(TOOL, "set", self.base, edited, "roll_attitude_gain=1", "max_bank_rad=0.04")
        changed, _ = self.simulate("changed", "--config", edited)
        with changed.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(float(rows[-1]["roll_kp"]), 2)
        self.assertAlmostEqual(float(rows[-1]["roll_command_rad"]), .04, places=6)
        self.assertEqual(rows[-1]["bank_command_limited"], "1")
        self.assertNotEqual(default.read_bytes(), changed.read_bytes())
        before, _ = self.simulate("before", "--roll-kp", "3", "--config", edited)
        after, _ = self.simulate("after", "--config", edited, "--roll-kp", "3")
        self.assertEqual(before.read_bytes(), after.read_bytes())
        with after.open(newline="") as stream:
            self.assertEqual(float(next(csv.DictReader(stream))["roll_kp"]), 3)

    def test_mission_uses_loaded_l1(self):
        spec = importlib.util.spec_from_file_location("compile_mission", ROOT / "tools/compile_mission.py")
        compiler = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(compiler)
        mission = self.work / "mission.apm"
        mission.write_bytes(compiler.compile_mission(json.loads((ROOT / "missions/c172x_line.json").read_text())))
        edited = self.work / "l1.apcf"
        self.run_command(TOOL, "set", self.base, edited, "l1_period_s=8")
        def run(name, *options):
            path = self.work / (name + ".csv")
            result = self.run_command(SIM, "--mode", "mission", "--mission", mission,
                "--duration", "3", "--wind-east-m-s", "5", "--output", path, *options, success=False)
            self.assertEqual(result.returncode, 2, result.stderr)  # Intentionally incomplete.
            return path.read_bytes()
        base = run("base_l1", "--config", self.base)
        changed = run("changed_l1", "--config", edited)
        self.assertNotEqual(base, changed)
        self.assertEqual(base, run("override_l1", "--l1-period-s", "4", "--config", edited))

    def test_legacy_v1_migration(self):
        fields = [4, .5, math.radians(20), .5, 10, 3, math.radians(10), .5,
                  .08, .005, 0, 1, .015, .05, math.radians(3), 4]
        data = struct.pack("<4sHHI16f", b"APCF", 1, 64, 0, *fields)
        data += struct.pack("<I", zlib.crc32(data))
        legacy = self.work / "legacy.apcf"
        legacy.write_bytes(data)
        shown = self.run_command(TOOL, "show", legacy).stdout
        self.assertIn("roll_attitude_gain=8", shown)
        self.assertIn("roll_rate_ki=0", shown)
        self.assertIn("max_roll_rate_rad_s=1", shown)
        self.simulate("legacy_loaded", "--config", legacy)
        migrated = self.work / "migrated.apcf"
        self.run_command(TOOL, "set", legacy, migrated, "roll_rate_ki=0.1")
        self.assertEqual(len(migrated.read_bytes()), 96)
        self.assertEqual(struct.unpack_from("<H", migrated.read_bytes(), 4)[0], 2)
        self.assertEqual(legacy.read_bytes(), data)
        invalid = bytearray(data)
        struct.pack_into("<f", invalid, 16, 0.0)
        invalid[-4:] = struct.pack("<I", zlib.crc32(invalid[:-4]))
        legacy.write_bytes(invalid)
        self.run_command(TOOL, "show", legacy, success=False)
        # Old normalized-angle units must never be accepted as new attitude gains.
        self.run_command(TOOL, "set", self.base, self.work / "old_names.apcf",
                         "roll_angle_gain=4", success=False)

    def test_explicit_rate_limits_and_telemetry(self):
        for axis in ("roll", "pitch"):
            for direction in (1, -1):
                output = self.work / f"{axis}_rate_limit_{direction}.csv"
                flag = "bank" if axis == "roll" else "pitch"
                self.run_command(SIM, "--mode", f"{axis}-hold", "--duration", "2.1",
                                 f"--{flag}-deg", str(5 * direction),
                                 f"--{axis}-rate-limit-deg-s", str(math.degrees(.01)),
                                 "--output", output)
                with output.open() as stream:
                    row = list(csv.DictReader(stream))[-1]
                self.assertEqual(row[f"{axis}_rate_limited"], "1")
                rate_target = float(row[f"{axis}_rate_command_rad_s"])
                self.assertAlmostEqual(rate_target, .01 * direction, places=6)
                body_rate = float(row["p_rad_s" if axis == "roll" else "q_rad_s"])
                self.assertAlmostEqual(float(row[f"{axis}_rate_error_rad_s"]), rate_target - body_rate, places=6)
                self.assertAlmostEqual(float(row[f"{axis}_rate_limit_rad_s"]), .01, places=6)
            for name, invalid in (("attitude-gain", "0"), ("rate-kp", "0"),
                                  ("rate-ki", "-1"), ("rate-limit-deg-s", "0")):
                self.run_command(SIM, "--mode", f"{axis}-hold", f"--{axis}-{name}", invalid, success=False)


if __name__ == "__main__":
    unittest.main(argv=[__file__, *remaining])
