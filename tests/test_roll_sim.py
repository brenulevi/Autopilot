"""Black-box C172X roll-hold check through the public autopilot API."""

import csv
import subprocess
import sys
import tempfile
from pathlib import Path


def check_direction(sim: Path, target_deg: int, output: Path) -> None:
    subprocess.run(
        [str(sim), "--mode", "roll", "--bank-deg", str(target_deg), "--output", str(output)],
        check=True,
        capture_output=True,
        text=True,
    )
    with output.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 2000, f"unexpected sample count: {len(rows)}"

    late = [row for row in rows if float(row["time_s"]) >= 15.0]
    largest_late_error = max(abs(float(row["bank_deg"]) - target_deg) for row in late)
    largest_aileron = max(abs(float(row["aileron_command"])) for row in rows)
    overshoot = max(
        (float(row["bank_deg"]) - target_deg) * (1 if target_deg > 0 else -1)
        for row in rows if float(row["time_s"]) >= 2.0
    )
    assert largest_late_error < 0.5, f"{target_deg:+} deg late error: {largest_late_error}"
    assert overshoot < 1.0, f"{target_deg:+} deg overshoot: {overshoot}"
    assert largest_aileron <= 0.500001, f"aileron exceeded limit: {largest_aileron}"


def main() -> None:
    sim = Path(sys.argv[1]).resolve()
    with tempfile.TemporaryDirectory() as directory:
        for target_deg in (5, -5):
            check_direction(sim, target_deg, Path(directory) / f"roll_{target_deg}.csv")


if __name__ == "__main__":
    main()
