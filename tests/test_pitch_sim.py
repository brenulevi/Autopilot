"""Black-box C172X pitch-hold check through the public autopilot API."""

import csv
import subprocess
import sys
import tempfile
from pathlib import Path


def check_direction(sim: Path, offset_deg: int, output: Path) -> None:
    subprocess.run(
        [str(sim), "--mode", "pitch", "--pitch-offset-deg", str(offset_deg),
         "--duration", "60", "--output", str(output)],
        check=True,
        capture_output=True,
        text=True,
    )
    with output.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 6000, f"unexpected sample count: {len(rows)}"

    late = [row for row in rows if float(row["time_s"]) >= 15.0]
    largest_late_error = max(
        abs(float(row["pitch_deg"]) - float(row["target_pitch_deg"]))
        for row in late
    )
    initial_bank = float(rows[0]["bank_deg"])
    largest_bank_deviation = max(abs(float(row["bank_deg"]) - initial_bank) for row in rows)
    largest_elevator = max(abs(float(row["elevator_command"])) for row in rows)
    assert largest_late_error < 0.6, f"{offset_deg:+} deg late error: {largest_late_error}"
    final_error = abs(float(rows[-1]["pitch_deg"]) - float(rows[-1]["target_pitch_deg"]))
    assert final_error < 0.5, f"{offset_deg:+} deg final pitch error: {final_error}"
    assert largest_bank_deviation < 1.0, f"bank deviation: {largest_bank_deviation}"
    assert largest_elevator <= 0.500001, f"elevator exceeded limit: {largest_elevator}"


def main() -> None:
    sim = Path(sys.argv[1]).resolve()
    with tempfile.TemporaryDirectory() as directory:
        for offset_deg in (2, -2):
            check_direction(sim, offset_deg, Path(directory) / f"pitch_{offset_deg}.csv")


if __name__ == "__main__":
    main()
