"""Black-box C172X airspeed-hold check through the public autopilot API."""

import csv
import subprocess
import sys
import tempfile
from pathlib import Path


def check_direction(sim: Path, offset_kts: int, output: Path) -> None:
    subprocess.run(
        [str(sim), "--mode", "airspeed", "--airspeed-offset-kts", str(offset_kts),
         "--duration", "60", "--output", str(output)],
        check=True,
        capture_output=True,
        text=True,
    )
    with output.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 6000, f"unexpected sample count: {len(rows)}"

    target = float(rows[-1]["target_airspeed_kts"])
    final = float(rows[-1]["airspeed_kts"])
    first = float(rows[0]["airspeed_kts"])
    trim = float(rows[0]["throttle_trim"])
    early_command = float(rows[300]["throttle_command"])
    assert abs(final - target) < 0.1, f"{offset_kts:+} kt final error: {final - target}"
    assert (early_command - trim) * offset_kts > 0, "throttle moved in the wrong direction"
    assert (final - first) * offset_kts > 0, "airspeed moved in the wrong direction"
    assert all(0.0 <= float(row["throttle_command"]) <= 1.0 for row in rows)
    actuator_gap = max(
        abs(float(row["elevator_requested_deg"]) - float(row["elevator_actual_deg"]))
        for row in rows
    )
    assert actuator_gap > 0.05, "actuator request and position were not logged separately"
    final_pitch_error = abs(float(rows[-1]["pitch_deg"]) - float(rows[-1]["target_pitch_deg"]))
    assert final_pitch_error < 0.35, f"pitch error after airspeed step: {final_pitch_error}"


def main() -> None:
    sim = Path(sys.argv[1]).resolve()
    with tempfile.TemporaryDirectory() as directory:
        for offset_kts in (5, -5):
            check_direction(sim, offset_kts, Path(directory) / f"airspeed_{offset_kts}.csv")


if __name__ == "__main__":
    main()
