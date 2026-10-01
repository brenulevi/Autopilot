"""Compare two autopilot_sim CSV logs and export a report-ready figure."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # Works without a desktop; does not affect the C/C++ build.
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator, ScalarFormatter


REQUIRED = (
    "time_s", "aileron_cmd_norm", "roll_rad", "p_rad_s", "pitch_rad",
    "airspeed_m_s", "altitude_m",
)
ROLL_FIELDS = (
    "roll_request_rad", "roll_command_rad", "roll_error_rad", "aileron_saturated",
    "bank_command_limited", "roll_kp", "roll_kd", "aileron_limit_norm",
)
PITCH_FIELDS = (
    "pitch_request_rad", "pitch_command_rad", "pitch_error_rad", "q_rad_s",
    "elevator_cmd_norm", "elevator_pos_rad", "elevator_saturated",
    "pitch_command_limited", "pitch_kp", "pitch_kd", "elevator_limit_norm",
)
SPEED_FIELDS = (
    "airspeed_request_m_s", "airspeed_command_m_s", "airspeed_error_m_s",
    "throttle_cmd_norm", "throttle_saturated", "speed_kp", "speed_ki",
    "speed_integral_norm", "throttle_min_norm", "throttle_max_norm",
)
ALTITUDE_FIELDS = (
    "altitude_command_m", "altitude_error_m", "climb_rate_m_s",
    "altitude_pitch_limited", "altitude_kh", "altitude_kv",
    "pitch_command_rad", "roll_command_rad", "throttle_cmd_norm",
)
THROTTLE_FIELDS = ("throttle_cmd_norm",)


def read_log(path, required=REQUIRED):
    data = {column: [] for column in required}
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = set(required) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line, row in enumerate(reader, start=2):
            for column in required:
                try:
                    value = float(row[column])
                except (TypeError, ValueError) as error:
                    raise ValueError(f"{path}:{line}: invalid {column}") from error
                if not math.isfinite(value):
                    raise ValueError(f"{path}:{line}: non-finite {column}")
                data[column].append(value)
    if len(data["time_s"]) < 2:
        raise ValueError(f"{path}: need at least two samples")
    if any(b <= a for a, b in zip(data["time_s"], data["time_s"][1:])):
        raise ValueError(f"{path}: timestamps must be strictly increasing")
    return data


def input_intervals(baseline, experiment):
    """Highlight differing inputs only when timestamps match (no interpolation)."""
    time = experiment["time_s"]
    if len(time) != len(baseline["time_s"]) or any(
        abs(a - b) > 1e-7 for a, b in zip(time, baseline["time_s"])
    ):
        return []
    active = [abs(a - b) > 1e-6 for a, b in zip(
        baseline["aileron_cmd_norm"], experiment["aileron_cmd_norm"]
    )]
    intervals = []
    start = None
    for index, differs in enumerate(active):
        if differs and start is None:
            start = time[index]
        elif not differs and start is not None:
            intervals.append((start, time[index]))
            start = None
    if start is not None:
        intervals.append((start, time[-1]))
    return intervals


def set_plot_style():
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 10,
        "axes.titlesize": 12, "axes.labelsize": 10,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.edgecolor": "#9ba7b4", "text.color": "#1c2938",
        "axes.labelcolor": "#1c2938", "xtick.color": "#465467",
        "ytick.color": "#465467", "svg.fonttype": "none",
    })
def save_figure(figure, output):
    output.parent.mkdir(parents=True, exist_ok=True)
    for extension in ("png", "svg"):
        destination = output.with_suffix(f".{extension}")
        figure.savefig(destination, dpi=180, facecolor="white")
        print(f"Saved {destination.resolve()}")
    plt.close(figure)


def draw(baseline, experiment, baseline_label, experiment_label, title, output):
    set_plot_style()
    figure, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    figure.subplots_adjust(left=0.09, right=0.97, top=0.83, bottom=0.18,
                           hspace=0.40, wspace=0.29)
    figure.suptitle(title, x=0.09, y=0.96, ha="left", fontsize=19, fontweight="bold")
    figure.text(0.09, 0.915, "C172X open-loop simulation · commands pass through the C library",
                fontsize=11, color="#465467")

    # Left column follows cause -> angular velocity -> angle.
    panels = [
        (axes[0, 0], "aileron_cmd_norm", "Aileron input", "Command (normalized)", 1, False),
        (axes[1, 0], "p_rad_s", "Body roll rate", "p (deg/s)", 180 / math.pi, False),
        (axes[2, 0], "roll_rad", "Bank angle", "Roll (deg)", 180 / math.pi, False),
        (axes[0, 1], "pitch_rad", "Pitch angle", "Pitch (deg)", 180 / math.pi, False),
        (axes[1, 1], "airspeed_m_s", "True airspeed change", "Change from initial (m/s)", 1, True),
        (axes[2, 1], "altitude_m", "Altitude change", "Change from initial (m)", 1, True),
    ]
    intervals = input_intervals(baseline, experiment)
    legend_lines = []
    for axis, key, heading, units, scale, relative in panels:
        for start, end in intervals:
            axis.axvspan(start, end, color="#dce4ed", alpha=0.7, linewidth=0)
        # Draw the baseline last so it remains visible where the traces overlap.
        for data, label, color, linestyle, width in (
            (experiment, experiment_label, "#1767b3", "-", 2.0),
            (baseline, baseline_label, "#a65516", "--", 1.6),
        ):
            offset = data[key][0] if relative else 0
            values = [(value - offset) * scale for value in data[key]]
            line, = axis.plot(data["time_s"], values, label=label,
                              color=color, linestyle=linestyle, linewidth=width,
                              drawstyle="steps-post" if key == "aileron_cmd_norm" else "default")
            if axis is axes[0, 0]:
                legend_lines.append(line)
        axis.set_title(heading, loc="left", pad=9)
        axis.set_ylabel(units)
        axis.grid(axis="both", color="#e1e6eb", linewidth=0.7)
        axis.set_axisbelow(True)
        axis.yaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
        formatter = ScalarFormatter(useOffset=False)
        formatter.set_scientific(False)
        axis.yaxis.set_major_formatter(formatter)
        axis.margins(x=0, y=0.15)
    for axis in axes[-1, :]:
        axis.set_xlabel("Simulation time (s)")
    handles = list(reversed(legend_lines))
    if intervals:
        handles.append(Patch(facecolor="#dce4ed", label="Input differs from baseline"))
    figure.legend(handles=handles, loc="upper left", bbox_to_anchor=(0.083, 0.895),
                  ncol=len(handles), frameon=False)
    figure.text(0.09, 0.035,
                f"Initial true airspeed: {experiment['airspeed_m_s'][0]:.2f} m/s. "
                f"Initial altitude: {experiment['altitude_m'][0]:.2f} m MSL.\n"
                "Changes are relative to each run's initial value. Normalized input is not a surface angle.",
                fontsize=9, color="#465467", linespacing=1.7)
    save_figure(figure, output)


def tracking_metrics(data, band_deg):
    """Measure each effective-command step over its observed dwell interval.

    Settling is the last entry into +/-band that persists through the segment's
    last sample. Null means it was not reached in the observed interval.
    """
    time = data["time_s"]
    command = data["roll_command_rad"]
    starts = [i for i in range(1, len(time)) if abs(command[i] - command[i - 1]) > 1e-6]
    segments = []
    for start, end in zip(starts, starts[1:] + [len(time)]):
        target = command[start]
        delta = target - command[start - 1]
        direction = 1 if delta > 0 else -1
        error = [math.degrees(target - value) for value in data["roll_rad"][start:end]]
        last_outside = max((i for i, value in enumerate(error) if abs(value) > band_deg), default=-1)
        settling_index = last_outside + 1
        settling = (time[start + settling_index] - time[start]
                    if settling_index < len(error) else None)
        overshoot = max(0.0, max(-direction * value for value in error))
        segments.append({
            "step_time_s": time[start], "last_observed_time_s": time[end - 1],
            "target_deg": math.degrees(target), "step_size_deg": math.degrees(delta),
            "overshoot_deg": overshoot, "overshoot_percent": 100 * overshoot / abs(math.degrees(delta)),
            "settling_time_s": settling, "end_error_deg": error[-1],
            "end_error_definition": "effective target minus actual bank at the last segment sample",
        })
    return {
        "settling_band_deg": band_deg,
        "settling_definition": "Last entry into the band that persists through the observed segment; null if not reached",
        "max_abs_aileron_norm": max(abs(value) for value in data["aileron_cmd_norm"]),
        "aileron_saturated_samples": sum(value != 0 for value in data["aileron_saturated"]),
        "bank_command_limited_samples": sum(value != 0 for value in data["bank_command_limited"]),
        "sample_count": len(time), "roll_kp": data["roll_kp"][0], "roll_kd": data["roll_kd"][0],
        "segments": segments,
    }


def draw_roll_hold(data, title, output, band_deg):
    set_plot_style()
    time = data["time_s"]
    command = [math.degrees(value) for value in data["roll_command_rad"]]
    roll = [math.degrees(value) for value in data["roll_rad"]]
    error = [target - value for target, value in zip(command, roll)]
    metrics = tracking_metrics(data, band_deg)
    figure, axes = plt.subplots(3, 2, figsize=(12, 11), sharex=True)
    figure.subplots_adjust(left=0.09, right=0.97, top=0.80, bottom=0.19,
                           hspace=0.42, wspace=0.28)
    figure.suptitle(title, x=0.09, y=0.96, ha="left", fontsize=19, fontweight="bold")
    figure.text(0.09, 0.91,
                f"C172X · Initial TAS {data['airspeed_m_s'][0] * 3600 / 1852:.1f} kt · "
                f"Kp = {metrics['roll_kp']:.2f}, Kd = {metrics['roll_kd']:.2f} (radian inputs)",
                fontsize=11, color="#465467")
    actual_line, = axes[0, 0].plot(time, roll, color="#1767b3", linewidth=2, label="Measured bank")
    target_line, = axes[0, 0].step(time, command, where="post", color="#a65516",
                                  linestyle="--", linewidth=1.7, label="Effective bank command")
    request_line, = axes[0, 0].step(time, [math.degrees(v) for v in data["roll_request_rad"]],
                                   where="post", color="#777777", linestyle=":",
                                   linewidth=1.2, label="Requested bank")
    axes[0, 0].set_title("Bank-angle tracking", loc="left", pad=9)
    axes[0, 0].set_ylabel("Bank (deg)")
    axes[0, 1].axhspan(-band_deg, band_deg, color="#dce4ed", linewidth=0)
    axes[0, 1].plot(time, error, color="#1767b3", linewidth=1.8)
    axes[0, 1].set_title("Tracking error (command − measured)", loc="left", pad=9)
    axes[0, 1].set_ylabel("Error (deg)")
    axes[1, 0].plot(time, [math.degrees(v) for v in data["p_rad_s"]], color="#1767b3", linewidth=1.8)
    axes[1, 0].set_title("Body roll rate", loc="left", pad=9)
    axes[1, 0].set_ylabel("p (deg/s)")
    axes[1, 1].step(time, data["aileron_cmd_norm"], where="post", color="#1767b3", linewidth=1.8)
    limit = data["aileron_limit_norm"][0]
    for value in (-limit, limit):
        axes[1, 1].axhline(value, color="#778596", linestyle=":", linewidth=1)
    axes[1, 1].set_title(f"Aileron demand (limits ±{limit:g})", loc="left", pad=9)
    axes[1, 1].set_ylabel("Command (normalized)")
    axes[2, 0].plot(time, [(v - data["airspeed_m_s"][0]) * 3600 / 1852
                          for v in data["airspeed_m_s"]], color="#1767b3")
    axes[2, 0].set_title("True airspeed change from initial", loc="left", pad=9)
    axes[2, 0].set_ylabel("Δ TAS (kt)")
    axes[2, 1].plot(time, [v - data["altitude_m"][0] for v in data["altitude_m"]], color="#1767b3")
    axes[2, 1].set_title("Altitude change from initial", loc="left", pad=9)
    axes[2, 1].set_ylabel("Δ altitude (m)")
    for axis in axes.flat:
        axis.grid(color="#e1e6eb", linewidth=0.7)
        axis.set_axisbelow(True)
        axis.margins(x=0, y=0.12)
        axis.yaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
        for segment in metrics["segments"]:
            axis.axvline(segment["step_time_s"], color="#c2cbd5", linewidth=0.8, zorder=0)
    for axis in axes[-1, :]:
        axis.set_xlabel("Simulation time (s)")
    figure.legend(handles=[target_line, request_line, actual_line, Patch(facecolor="#dce4ed", label=f"Error band ±{band_deg:g}°")],
                  loc="upper left", bbox_to_anchor=(0.083, 0.885), ncol=2, frameon=False)
    figure.text(0.09, 0.055,
                f"Aileron saturation: {metrics['aileron_saturated_samples']} / {metrics['sample_count']} samples. "
                f"Limited bank commands: {metrics['bank_command_limited_samples']} samples.\n"
                "Elevator, rudder and throttle remain at trim. Exact simulated state feedback; no sensor errors.",
                fontsize=9, color="#465467", linespacing=1.7)
    save_figure(figure, output)
    destination = output.with_suffix(".metrics.json")
    destination.write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Saved {destination.resolve()}")
    for segment in metrics["segments"]:
        settling = segment["settling_time_s"]
        settling_text = f"{settling:.2f} s" if settling is not None else "not reached in this interval"
        print(f"t={segment['step_time_s']:.2f} s -> {segment['target_deg']:.2f} deg: "
              f"overshoot {segment['overshoot_deg']:.3f} deg ({segment['overshoot_percent']:.1f}%), "
              f"settling (+/-{band_deg:g} deg) {settling_text}, "
              f"end error {segment['end_error_deg']:.3f} deg")


def draw_pitch_hold(data, title, output):
    set_plot_style()
    time = data["time_s"]
    figure, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    figure.subplots_adjust(left=0.09, right=0.96, top=0.88, bottom=0.09,
                           hspace=0.4, wspace=0.28)
    figure.suptitle(title, x=0.09, y=0.97, ha="left", fontsize=19, fontweight="bold")
    figure.text(0.09, 0.925,
                f"C172X · Initial TAS {data['airspeed_m_s'][0] * 3600 / 1852:.1f} kt · "
                f"Kp = {data['pitch_kp'][0]:.2f}, Kd = {data['pitch_kd'][0]:.2f} (radian inputs)",
                fontsize=11, color="#465467")
    target = [math.degrees(x) for x in data["pitch_command_rad"]]
    actual = [math.degrees(x) for x in data["pitch_rad"]]
    axes[0, 0].step(time, target, where="post", color="#a65516", linestyle="--",
                    linewidth=1.7, label="Effective command")
    axes[0, 0].step(time, [math.degrees(v) for v in data["pitch_request_rad"]],
                    where="post", color="#777777", linestyle=":", label="Request")
    axes[0, 0].plot(time, actual, color="#1767b3", linewidth=2, label="Measured")
    axes[0, 0].legend(frameon=False)
    panels = [
        (axes[0, 0], "Pitch-angle tracking", "Pitch (deg)"),
        (axes[0, 1], "Tracking error", "Error (deg)"),
        (axes[1, 0], "Body pitch rate", "q (deg/s)"),
        (axes[1, 1], "Elevator demand", "Command (normalized)"),
        (axes[2, 0], "True airspeed", "Airspeed (m/s)"),
        (axes[2, 1], "Altitude", "Altitude (m MSL)"),
    ]
    axes[0, 1].plot(time, [math.degrees(x) for x in data["pitch_error_rad"]], color="#1767b3")
    axes[1, 0].plot(time, [math.degrees(x) for x in data["q_rad_s"]], color="#1767b3")
    axes[1, 1].step(time, data["elevator_cmd_norm"], where="post", color="#1767b3")
    limit = data["elevator_limit_norm"][0]
    for value in (-limit, limit):
        axes[1, 1].axhline(value, color="#778596", linestyle=":", linewidth=1)
    axes[2, 0].plot(time, data["airspeed_m_s"], color="#1767b3")
    axes[2, 1].plot(time, data["altitude_m"], color="#1767b3")
    for axis, heading, ylabel in panels:
        axis.set_title(heading, loc="left", pad=8)
        axis.set_ylabel(ylabel)
        axis.grid(color="#e1e6eb", linewidth=0.7)
        axis.set_axisbelow(True)
        axis.margins(x=0, y=0.12)
        axis.yaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
        for step in (2, 10):
            axis.axvline(step, color="#c2cbd5", linewidth=0.8, zorder=0)
    for axis in axes[-1, :]:
        axis.set_xlabel("Simulation time (s)")
    save_figure(figure, output)


def draw_attitude_hold(data, title, output):
    set_plot_style()
    time = data["time_s"]
    figure, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    figure.subplots_adjust(left=0.09, right=0.96, top=0.89, bottom=0.09,
                           hspace=0.42, wspace=0.28)
    figure.suptitle(title, x=0.09, y=0.97, ha="left", fontsize=19, fontweight="bold")
    figure.text(0.09, 0.925, "C172X combined attitude control · exact simulated state feedback",
                fontsize=11, color="#465467")
    for axis, command_key, actual_key, heading, ylabel in (
        (axes[0, 0], "roll_command_rad", "roll_rad", "Bank-angle tracking", "Bank (deg)"),
        (axes[0, 1], "pitch_command_rad", "pitch_rad", "Pitch-angle tracking", "Pitch (deg)"),
    ):
        axis.step(time, [math.degrees(x) for x in data[command_key]], where="post",
                  color="#a65516", linestyle="--", linewidth=1.7, label="Command")
        axis.plot(time, [math.degrees(x) for x in data[actual_key]],
                  color="#1767b3", linewidth=2, label="Measured")
        axis.legend(frameon=False)
        axis.set_title(heading, loc="left", pad=8)
        axis.set_ylabel(ylabel)
    for axis, key, heading, ylabel in (
        (axes[1, 0], "aileron_cmd_norm", "Aileron demand", "Normalized command"),
        (axes[1, 1], "elevator_cmd_norm", "Elevator demand", "Normalized command"),
        (axes[2, 0], "airspeed_m_s", "True airspeed", "Airspeed (m/s)"),
        (axes[2, 1], "altitude_m", "Altitude", "Altitude (m MSL)"),
    ):
        axis.plot(time, data[key], color="#1767b3", linewidth=1.8)
        axis.set_title(heading, loc="left", pad=8)
        axis.set_ylabel(ylabel)
    for axis in axes.flat:
        axis.grid(color="#e1e6eb", linewidth=0.7)
        axis.set_axisbelow(True)
        axis.margins(x=0, y=0.12)
        axis.yaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
        for step in (2, 10):
            axis.axvline(step, color="#c2cbd5", linewidth=0.8, zorder=0)
    for axis in axes[-1, :]:
        axis.set_xlabel("Simulation time (s)")
    save_figure(figure, output)


def draw_airspeed_hold(data, title, output):
    set_plot_style()
    time = data["time_s"]
    figure, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    figure.subplots_adjust(left=0.09, right=0.96, top=0.89, bottom=0.09,
                           hspace=0.42, wspace=0.28)
    figure.suptitle(title, x=0.09, y=0.97, ha="left", fontsize=19, fontweight="bold")
    figure.text(0.09, 0.925,
                f"C172X true-airspeed PI · Kp = {data['speed_kp'][0]:.3f}, Ki = {data['speed_ki'][0]:.3f}",
                fontsize=11, color="#465467")
    axes[0, 0].step(time, data["airspeed_command_m_s"], where="post", color="#a65516",
                    linestyle="--", linewidth=1.7, label="Command")
    axes[0, 0].plot(time, data["airspeed_m_s"], color="#1767b3", linewidth=2, label="Measured")
    axes[0, 0].legend(frameon=False)
    panels = [
        (axes[0, 0], "True-airspeed tracking", "Airspeed (m/s)"),
        (axes[0, 1], "Tracking error", "Error (m/s)"),
        (axes[1, 0], "Throttle demand", "Normalized command"),
        (axes[1, 1], "PI integral", "Normalized throttle"),
        (axes[2, 0], "Pitch attitude", "Pitch (deg)"),
        (axes[2, 1], "Altitude", "Altitude (m MSL)"),
    ]
    axes[0, 1].plot(time, data["airspeed_error_m_s"], color="#1767b3")
    axes[1, 0].plot(time, data["throttle_cmd_norm"], color="#1767b3")
    for limit in (data["throttle_min_norm"][0], data["throttle_max_norm"][0]):
        axes[1, 0].axhline(limit, color="#778596", linestyle=":", linewidth=1)
    axes[1, 1].plot(time, data["speed_integral_norm"], color="#1767b3")
    axes[2, 0].plot(time, [math.degrees(x) for x in data["pitch_rad"]], color="#1767b3")
    axes[2, 1].plot(time, data["altitude_m"], color="#1767b3")
    for axis, heading, ylabel in panels:
        axis.set_title(heading, loc="left", pad=8)
        axis.set_ylabel(ylabel)
        axis.grid(color="#e1e6eb", linewidth=0.7)
        axis.set_axisbelow(True)
        axis.margins(x=0, y=0.12)
        axis.yaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
        for step in (2, 30):
            axis.axvline(step, color="#c2cbd5", linewidth=0.8, zorder=0)
    for axis in axes[-1, :]:
        axis.set_xlabel("Simulation time (s)")
    save_figure(figure, output)


def draw_altitude_hold(data, title, output):
    set_plot_style()
    time = data["time_s"]
    figure, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    figure.subplots_adjust(left=0.09, right=0.96, top=0.89, bottom=0.09,
                           hspace=0.42, wspace=0.28)
    figure.suptitle(title, x=0.09, y=0.97, ha="left", fontsize=19, fontweight="bold")
    figure.text(0.09, 0.925,
                f"C172X altitude outer loop · Kh = {data['altitude_kh'][0]:.3f} rad/m, "
                f"Kv = {data['altitude_kv'][0]:.3f} rad/(m/s)",
                fontsize=11, color="#465467")
    axes[0, 0].step(time, data["altitude_command_m"], where="post", color="#a65516",
                    linestyle="--", linewidth=1.7, label="Command")
    axes[0, 0].plot(time, data["altitude_m"], color="#1767b3", linewidth=2, label="Measured")
    axes[0, 0].legend(frameon=False)
    axes[0, 1].plot(time, data["altitude_error_m"], color="#1767b3")
    axes[1, 0].plot(time, data["climb_rate_m_s"], color="#1767b3")
    axes[1, 1].step(time, [math.degrees(x) for x in data["pitch_command_rad"]],
                    where="post", color="#a65516", linestyle="--", label="Command")
    axes[1, 1].plot(time, [math.degrees(x) for x in data["pitch_rad"]],
                    color="#1767b3", label="Measured")
    axes[1, 1].legend(frameon=False)
    axes[2, 0].step(time, [math.degrees(x) for x in data["roll_command_rad"]],
                    where="post", color="#a65516", linestyle="--", label="Command")
    axes[2, 0].plot(time, [math.degrees(x) for x in data["roll_rad"]],
                    color="#1767b3", label="Measured")
    axes[2, 0].legend(frameon=False)
    axes[2, 1].step(time, data["airspeed_command_m_s"], where="post",
                    color="#a65516", linestyle="--", label="Command")
    axes[2, 1].plot(time, data["airspeed_m_s"], color="#1767b3", label="Measured")
    axes[2, 1].legend(frameon=False)
    for axis, heading, ylabel in (
        (axes[0, 0], "Altitude tracking", "Altitude (m MSL)"),
        (axes[0, 1], "Altitude error", "Error (m)"),
        (axes[1, 0], "Vertical speed", "Climb rate (m/s)"),
        (axes[1, 1], "Pitch attitude", "Pitch (deg)"),
        (axes[2, 0], "Bank angle", "Bank (deg)"),
        (axes[2, 1], "True airspeed", "Airspeed (m/s)"),
    ):
        axis.set_title(heading, loc="left", pad=8)
        axis.set_ylabel(ylabel)
        axis.grid(color="#e1e6eb", linewidth=0.7)
        axis.set_axisbelow(True)
        axis.margins(x=0, y=0.12)
        axis.yaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
        for step in (2, 30, 40, 60):
            axis.axvline(step, color="#c2cbd5", linewidth=0.8, zorder=0)
    for axis in axes[-1, :]:
        axis.set_xlabel("Simulation time (s)")
    save_figure(figure, output)


def draw_throttle_step(baseline, positive, negative, output):
    set_plot_style()
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    figure.subplots_adjust(left=0.09, right=0.96, top=0.86, bottom=0.11,
                           hspace=0.42, wspace=0.28)
    figure.suptitle("C172X open-loop throttle response", x=0.09, y=0.97,
                    ha="left", fontsize=19, fontweight="bold")
    figure.text(0.09, 0.92, "Bank and pitch attitude held near trim · throttle pulse from 2 to 7 s",
                fontsize=11, color="#465467")
    cases = ((baseline, "Trim throttle", "#778596"),
             (positive, "+0.10 throttle", "#1767b3"),
             (negative, "−0.10 throttle", "#a65516"))
    for axis, key, heading, ylabel, scale in (
        (axes[0, 0], "throttle_cmd_norm", "Throttle command", "Normalized command", 1),
        (axes[0, 1], "airspeed_m_s", "True airspeed", "Airspeed (m/s)", 1),
        (axes[1, 0], "pitch_rad", "Pitch attitude", "Pitch (deg)", 180 / math.pi),
        (axes[1, 1], "altitude_m", "Altitude", "Altitude (m MSL)", 1),
    ):
        for data, label, color in cases:
            axis.plot(data["time_s"], [x * scale for x in data[key]],
                      color=color, label=label, linewidth=1.7)
        axis.set_title(heading, loc="left", pad=8)
        axis.set_ylabel(ylabel)
        axis.grid(color="#e1e6eb", linewidth=0.7)
        axis.set_axisbelow(True)
        axis.margins(x=0, y=0.12)
        axis.yaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
        for step in (2, 7):
            axis.axvline(step, color="#c2cbd5", linewidth=0.8, zorder=0)
    axes[0, 0].legend(frameon=False)
    for axis in axes[-1, :]:
        axis.set_xlabel("Simulation time (s)")
    save_figure(figure, output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=Path("logs/baseline.csv"))
    parser.add_argument("--experiment", type=Path)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--roll-hold", action="store_true", help="Plot command tracking from a roll-hold log")
    modes.add_argument("--pitch-hold", action="store_true", help="Plot command tracking from a pitch-hold log")
    modes.add_argument("--attitude-hold", action="store_true", help="Plot combined roll and pitch tracking")
    modes.add_argument("--airspeed-hold", action="store_true", help="Plot true-airspeed PI tracking")
    modes.add_argument("--altitude-hold", action="store_true", help="Plot altitude hold and optional banked segment")
    modes.add_argument("--throttle-step", action="store_true", help="Compare trimmed and +/- throttle pulses")
    parser.add_argument("--settling-band-deg", type=float, default=0.25)
    parser.add_argument("--baseline-label", default="Baseline: trim commands")
    parser.add_argument("--experiment-label", default="Aileron pulse")
    parser.add_argument("--title")
    parser.add_argument("--output", type=Path,
                        help="Output stem; writes .png and .svg (overwrites existing plots)")
    args = parser.parse_args()
    try:
        if not math.isfinite(args.settling_band_deg) or args.settling_band_deg <= 0:
            raise ValueError("Settling band must be finite and positive")
        if args.altitude_hold:
            experiment = read_log(args.experiment or Path("logs/c172x_altitude_hold.csv"),
                                  REQUIRED + SPEED_FIELDS + ALTITUDE_FIELDS)
            draw_altitude_hold(experiment, args.title or "C172X altitude hold",
                               args.output or Path("logs/plots/c172x_altitude_hold"))
        elif args.airspeed_hold:
            experiment = read_log(args.experiment or Path("logs/c172x_airspeed_hold.csv"),
                                  REQUIRED + SPEED_FIELDS)
            draw_airspeed_hold(experiment, args.title or "C172X true-airspeed hold",
                               args.output or Path("logs/plots/c172x_airspeed_hold"))
        elif args.throttle_step:
            required = REQUIRED + THROTTLE_FIELDS
            baseline = read_log(args.baseline if args.baseline != Path("logs/baseline.csv")
                                else Path("logs/throttle_attitude_0.csv"), required)
            positive = read_log(args.experiment or Path("logs/throttle_attitude_0.1.csv"), required)
            negative = read_log(Path("logs/throttle_attitude_-0.1.csv"), required)
            draw_throttle_step(baseline, positive, negative,
                               args.output or Path("logs/plots/c172x_throttle_step"))
        elif args.attitude_hold:
            experiment = read_log(args.experiment or Path("logs/c172x_attitude_hold.csv"),
                                  REQUIRED + ROLL_FIELDS + PITCH_FIELDS)
            draw_attitude_hold(experiment, args.title or "C172X combined attitude hold",
                               args.output or Path("logs/plots/c172x_attitude_hold"))
        elif args.pitch_hold:
            experiment = read_log(args.experiment or Path("logs/c172x_pitch_hold.csv"), REQUIRED + PITCH_FIELDS)
            draw_pitch_hold(experiment, args.title or "C172X pitch attitude hold",
                            args.output or Path("logs/plots/c172x_pitch_hold"))
        elif args.roll_hold:
            experiment = read_log(args.experiment or Path("logs/c172x_roll_hold.csv"), REQUIRED + ROLL_FIELDS)
            draw_roll_hold(experiment, args.title or "C172X roll hold and return to wings level",
                           args.output or Path("logs/plots/c172x_roll_hold"), args.settling_band_deg)
        else:
            baseline = read_log(args.baseline)
            experiment = read_log(args.experiment or Path("logs/c172x_pulse.csv"))
            draw(baseline, experiment, args.baseline_label, args.experiment_label,
                 args.title or "C172X response to an aileron pulse",
                 args.output or Path("logs/plots/c172x_comparison"))
    except (OSError, ValueError) as error:
        parser.exit(1, f"plot_logs: {error}\n")


if __name__ == "__main__":
    main()
