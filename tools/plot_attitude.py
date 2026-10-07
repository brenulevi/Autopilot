#!/usr/bin/env python3
"""Plot one C172X controller experiment from its simulator CSV."""

import argparse
import csv
from pathlib import Path


SERIES = {
    "roll": {
        "angle": ("target_bank_deg", "bank_deg", "Bank angle (deg)"),
        "other": ("target_pitch_deg", "pitch_deg", "Pitch angle (deg)"),
        "rate": ("roll_rate_deg_s", "Roll rate (deg/s)"),
        "surface": ("aileron_command", "aileron_trim", "Aileron command"),
    },
    "pitch": {
        "angle": ("target_pitch_deg", "pitch_deg", "Pitch angle (deg)"),
        "other": ("target_airspeed_kts", "airspeed_kts", "Airspeed (kt)"),
        "rate": ("pitch_rate_deg_s", "Pitch rate (deg/s)"),
        "surface": ("elevator_command", "elevator_trim", "Elevator command"),
    },
    "airspeed": {
        "angle": ("target_airspeed_kts", "airspeed_kts", "Airspeed (kt)"),
        "other": ("target_pitch_deg", "pitch_deg", "Pitch angle (deg)"),
        "rate": ("pitch_rate_deg_s", "Pitch rate (deg/s)"),
        "surface": ("throttle_command", "throttle_trim", "Throttle command"),
    },
}


def read_csv(path: Path, columns: set[str]) -> dict[str, list[float]]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = columns.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing CSV columns: {', '.join(sorted(missing))}")
        data = {name: [] for name in columns}
        for line_number, row in enumerate(reader, start=2):
            try:
                for name in columns:
                    data[name].append(float(row[name]))
            except (TypeError, ValueError) as error:
                raise ValueError(f"Invalid value on CSV line {line_number}") from error
    if not data["time_s"]:
        raise ValueError("CSV has no samples")
    return data


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=SERIES, required=True)
    parser.add_argument("--input", type=Path, help="CSV to plot (defaults to the simulator output)")
    parser.add_argument("--output", type=Path, help="PNG destination (defaults beside the CSV)")
    parser.add_argument("--show", action="store_true", help="Open an interactive plot window")
    args = parser.parse_args()

    csv_path = args.input or Path(f"logs/c172x_{args.mode}_hold.csv")
    png_path = args.output or csv_path.with_suffix(".png")
    config = SERIES[args.mode]
    columns = {"time_s"}
    columns.update(config["angle"][:2])
    columns.update(config["other"][:2])
    columns.add(config["rate"][0])
    columns.update(config["surface"][:2])
    columns.update(("elevator_requested_deg", "elevator_actual_deg"))
    if args.mode == "airspeed":
        columns.update(("target_bank_deg", "bank_deg", "elevator_command", "elevator_trim"))
    else:
        columns.update(("target_airspeed_kts", "airspeed_kts", "throttle_command", "throttle_trim"))

    data = read_csv(csv_path, columns)
    if not args.show:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    time = data["time_s"]
    figure, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    figure.suptitle(f"C172X {args.mode} hold")

    target, measured, title = config["angle"]
    axes[0, 0].plot(time, data[target], "--", label="Target")
    axes[0, 0].plot(time, data[measured], label="Measured")
    axes[0, 0].set_title(title)

    target, measured, title = config["other"]
    axes[0, 1].plot(time, data[target], "--", label="Target")
    axes[0, 1].plot(time, data[measured], label="Measured")
    axes[0, 1].set_title(title)

    rate, title = config["rate"]
    axes[1, 0].plot(time, data[rate], label="Measured")
    axes[1, 0].set_title(title)

    command, trim, title = config["surface"]
    axes[1, 1].plot(time, data[command], label="Command")
    axes[1, 1].plot(time, data[trim], "--", label="Trim")
    axes[1, 1].set_title(title)

    if args.mode == "roll":
        axes[2, 0].plot(time, data["target_airspeed_kts"], "--", label="Target")
        axes[2, 0].plot(time, data["airspeed_kts"], label="Measured")
        axes[2, 0].set_title("Airspeed (kt)")
    else:
        axes[2, 0].plot(time, data["elevator_requested_deg"], "--", label="Requested")
        axes[2, 0].plot(time, data["elevator_actual_deg"], label="Actual")
        axes[2, 0].set_title("Elevator actuator position (deg)")

    if args.mode == "airspeed":
        extra_command, extra_trim, extra_command_title = (
            "elevator_command", "elevator_trim", "Elevator command"
        )
    else:
        extra_command, extra_trim, extra_command_title = (
            "throttle_command", "throttle_trim", "Throttle command"
        )
    axes[2, 1].plot(time, data[extra_command], label="Command")
    axes[2, 1].plot(time, data[extra_trim], "--", label="Trim")
    axes[2, 1].set_title(extra_command_title)

    for axis in axes.flat:
        axis.grid(True)
        axis.legend()
    for axis in axes[2]:
        axis.set_xlabel("Time (s)")
    figure.tight_layout()
    png_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(png_path, dpi=150)
    print(f"Plot: {png_path.resolve()}")
    if args.show:
        plt.show()
    plt.close(figure)


if __name__ == "__main__":
    main()
