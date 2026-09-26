"""Independent wire-format, recovery, and C-to-Python flight-log tests."""
import argparse
import csv
import importlib.util
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "decode_flight_log.py"
spec = importlib.util.spec_from_file_location("flight_log_decoder", TOOL)
decoder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(decoder)
FIXTURE = None


def record(kind=1, payload=None, *, source=1, session=7, sequence=0, timestamp=10000):
    if payload is None:
        payload = struct.pack("<III", 123, 456, 789)
    header = struct.pack("<4sBBHHHIIQI", b"FLG1", 1, source, kind, len(payload), 0,
                         sequence, session, timestamp, 0)
    data = header + payload
    return data + struct.pack("<I", zlib.crc32(data))


class FlightLogTest(unittest.TestCase):
    def test_independent_payloads(self):
        payloads = [
            record(),
            record(2, struct.pack("<4fBBH", .25, -.5, 0, .75, 3, 1, 0), source=2),
            record(3, struct.pack("<9fI", .125, 0, 0, 0, 0, -2, 21, 120, -1, 0x1ff)),
            record(4, struct.pack("<4B3I8H", 1, 1, 2, 1, 5, 0xffffffff, 2, 1000, 1750, *([0] * 6)), source=2),
            record(5, struct.pack("<HBBII", 42, 1, 0, 100, 200)),
            record(6, struct.pack("<B3xIQI", 2, 99, 0x100000123, 500)),
        ]
        stats = {}
        rows = list(decoder.records(b"".join(payloads), stats))
        self.assertEqual(len(rows), 6)
        self.assertEqual(stats, dict(records=6, skipped_bytes=0, invalid_candidates=0))
        self.assertEqual(rows[0]["reset_reason"], 789)
        self.assertEqual(rows[1]["elevator"], -.5)
        self.assertEqual(rows[1]["source"], "f405")
        self.assertEqual(rows[2]["r_rad_s"], -2)
        self.assertEqual(rows[3]["pulse_1_us"], 1750)
        self.assertNotIn("pulse_2_us", rows[3])
        self.assertEqual(rows[4]["argument1"], 200)
        self.assertEqual(rows[5]["peer_timestamp_us"], 0x100000123)

    def test_corruption_torn_records_and_erased_flash(self):
        valid = record()
        damaged = bytearray(valid)
        damaged[35] ^= 1
        stats = {}
        dump = b"\xff" * 256 + bytes(damaged) + valid + valid[:19] + b"\xff" * 17
        rows = list(decoder.records(dump, stats))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["offset"], 256 + len(valid))
        self.assertEqual(stats["skipped_bytes"], len(dump) - len(valid))
        self.assertEqual(stats["invalid_candidates"], 2)
        for end in range(len(valid)):
            self.assertEqual(list(decoder.records(valid[:end], {})), [])

    def test_reject_semantic_errors_even_with_valid_crc(self):
        invalid = [
            record(source=0), record(session=0),
            record(2, struct.pack("<4fBBH", 0, 0, 0, 0, 1, 2, 0)),
            record(2, struct.pack("<4fBBH", 0, 0, 0, 0, 1, 1, 3)),
            record(3, struct.pack("<9fI", *([0] * 9), 0x200)),
            record(4, struct.pack("<4B3I8H", 1, 1, 1, 1, 0, 0, 2, *([0] * 8))),
            record(5, struct.pack("<HBBII", 42, 3, 0, 0, 0)),
            record(6, struct.pack("<B3xIQI", 0, 99, 100, 0)),
            record(1, b"\0" * 64), record(99, b"\0" * 65),
        ]
        for data in invalid:
            self.assertEqual(list(decoder.records(data, {})), [])

    def test_unknown_types_and_independent_clock_domains(self):
        data = record(99, b"new") + record(source=2, session=99, timestamp=2)
        rows = list(decoder.records(data, {}))
        self.assertEqual(rows[0]["unknown_payload_hex"], "6e6577")
        self.assertEqual([r["timestamp_us"] for r in rows], [10000, 2])

    def test_c_fixture_csv_and_overwrite_protection(self):
        if FIXTURE is None:
            self.skipTest("Pass --fixture with the built log_core_test executable")
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            binary = folder / "flight.bin"
            csv_path = folder / "flight.csv"
            subprocess.run([FIXTURE, str(binary)], check=True, capture_output=True)
            stats = {}
            rows = list(decoder.records(binary.read_bytes(), stats))
            self.assertEqual(stats["records"], 4)
            self.assertEqual([r["source"] for r in rows], ["h723", "h723", "f405", "h723"])
            self.assertEqual(rows[0]["sequence"], 1)
            self.assertEqual(rows[0]["dropped_total"], 1)
            self.assertEqual(rows[1]["aileron"], .25)
            self.assertEqual(rows[2]["aileron"], -.25)
            self.assertEqual(rows[2]["stage"], "selected")
            self.assertEqual(rows[3]["peer_session_id"], 99)
            command = [sys.executable, str(TOOL), str(binary), "--output", str(csv_path)]
            first = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(first.returncode, 0, first.stderr)
            with csv_path.open(newline="", encoding="utf-8") as stream:
                exported = list(csv.DictReader(stream))
            self.assertEqual(len(exported), 4)
            before = csv_path.read_bytes()
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 1)
            self.assertEqual(csv_path.read_bytes(), before)
            empty = folder / "empty.bin"
            empty.write_bytes(b"")
            result = subprocess.run([sys.executable, str(TOOL), str(empty), "--output", str(folder / "empty.csv")],
                                    capture_output=True)
            self.assertEqual(result.returncode, 2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--fixture")
    args, remaining = parser.parse_known_args()
    FIXTURE = args.fixture
    unittest.main(argv=[sys.argv[0], *remaining])
