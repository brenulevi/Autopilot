"""Summarize comparable mission CSV runs without plotting dependencies."""

import argparse
import csv
import json
import math
from pathlib import Path


FIELDS = (
    "time_s", "control_mode", "cross_track_m", "roll_request_rad",
    "altitude_error_m", "airspeed_error_m_s", "ground_north_m_s",
    "ground_east_m_s", "wind_north_m_s", "wind_east_m_s",
    "aileron_saturated", "elevator_saturated", "throttle_saturated",
    "mission_leg",
)


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def summarize(path, bank_limit_deg=20.0):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = set(FIELDS) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"{path}: missing columns {', '.join(sorted(missing))}")
        rows = list(reader)
    if not rows or any(row["control_mode"] != "mission" for row in rows):
        raise ValueError(f"{path}: expected mission rows")

    def values(field, selected=rows):
        result = [float(row[field]) for row in selected]
        if not all(map(math.isfinite, result)):
            raise ValueError(f"{path}: nonfinite {field}")
        return result

    times = values("time_s")
    if any(later <= earlier for earlier, later in zip(times, times[1:])):
        raise ValueError(f"{path}: time must increase")
    late_rows = [row for row in rows if float(row["time_s"]) >= times[-1] - 10.0]
    cross = values("cross_track_m")
    late_cross = values("cross_track_m", late_rows)
    bank = values("roll_request_rad")
    altitude = values("altitude_error_m")
    airspeed = values("airspeed_error_m_s")
    north = values("ground_north_m_s")
    east = values("ground_east_m_s")
    limit_rad = math.radians(bank_limit_deg)

    return {
        "csv": str(path),
        "samples": len(rows),
        "last_sample_time_s": times[-1],
        "wind_north_m_s": values("wind_north_m_s")[0],
        "wind_east_m_s": values("wind_east_m_s")[0],
        "legs_seen": sorted({int(row["mission_leg"]) for row in rows}),
        "cross_track_rms_m": rms(cross),
        "cross_track_max_abs_m": max(map(abs, cross)),
        "last_10_s_cross_track_rms_m": rms(late_cross),
        "last_10_s_cross_track_max_abs_m": max(map(abs, late_cross)),
        "bank_at_limit_samples": sum(abs(value) >= limit_rad - 1e-4 for value in bank),
        "aileron_saturated_samples": sum(int(row["aileron_saturated"]) for row in rows),
        "elevator_saturated_samples": sum(int(row["elevator_saturated"]) for row in rows),
        "throttle_saturated_samples": sum(int(row["throttle_saturated"]) for row in rows),
        "altitude_error_max_abs_m": max(map(abs, altitude)),
        "airspeed_error_max_abs_m_s": max(map(abs, airspeed)),
        "groundspeed_min_m_s": min(map(math.hypot, north, east)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, nargs="+", help="Mission CSV files to compare")
    parser.add_argument("--output", type=Path, help="Also write the JSON summary here")
    parser.add_argument("--bank-limit-deg", type=float, default=20.0)
    args = parser.parse_args()
    if not math.isfinite(args.bank_limit_deg) or args.bank_limit_deg <= 0:
        parser.error("--bank-limit-deg must be finite and positive")
    try:
        report = [summarize(path, args.bank_limit_deg) for path in args.csv]
        content = json.dumps(report, indent=2) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(content, encoding="utf-8")
    except (OSError, ValueError) as error:
        parser.exit(1, f"mission_metrics: {error}\n")
    print(content, end="")


if __name__ == "__main__":
    main()
