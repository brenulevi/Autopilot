"""Run a speed/angle matrix for roll or pitch and export comparison reports."""

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.plot_logs import REQUIRED, ROLL_FIELDS, PITCH_FIELDS, read_log, set_plot_style, save_figure
import matplotlib.pyplot as plt


KNOTS_PER_M_S = 3600 / 1852


def angle_token(angle):
    # save_figure uses Path.with_suffix; decimal dots must not become suffixes.
    return f"{angle:+}".removesuffix(".0").replace(".", "p")


def axis_fields(axis):
    return ("p_rad_s", "aileron", "bank_command_limited") if axis == "roll" else (
        "q_rad_s", "elevator", "pitch_command_limited")


def segment_metrics(data, axis, band_deg):
    """Sample-based metrics for the step [2,10) and return [10,end].

    Errors refer to the effective command. Settling requires remaining in the
    band through the last observed sample; None means not observed, not zero.
    """
    rate, actuator, limited = axis_fields(axis)
    time = data["time_s"]
    command = [math.degrees(v) for v in data[f"{axis}_command_rad"]]
    actual = [math.degrees(v) for v in data[f"{axis}_rad"]]
    result = []
    for phase, indices in (
        ("step", [i for i, t in enumerate(time) if 2 <= t < 10]),
        ("return", [i for i, t in enumerate(time) if t >= 10]),
    ):
        if not indices:
            continue
        start, end = indices[0], indices[-1]
        target = command[start]
        delta = target - command[start - 1] if start else 0.0
        error = [command[i] - actual[i] for i in indices]
        last_outside = max((k for k, e in enumerate(error) if abs(e) > band_deg), default=-1)
        settling = (time[indices[last_outside + 1]] - time[start]
                    if last_outside + 1 < len(indices) else None)
        last_second = [command[i] - actual[i] for i in indices if time[i] >= time[end] - 1]
        rise = None
        overshoot = None
        time10 = time90 = peak_time = peak_angle = None
        if abs(delta) > 1e-6:
            progress = [(actual[i] - command[start - 1]) / delta for i in indices]
            first10 = next((k for k, v in enumerate(progress) if v >= 0.1), None)
            first90 = next((k for k, v in enumerate(progress) if v >= 0.9), None)
            time10 = time[indices[first10]] if first10 is not None else None
            time90 = time[indices[first90]] if first90 is not None else None
            if first10 is not None and first90 is not None:
                rise = time[indices[first90]] - time[indices[first10]]
            direction = 1 if delta > 0 else -1
            overshoot = max(0.0, max(-direction * e for e in error))
            peak_index = max(indices, key=lambda i: direction * actual[i])
            peak_time, peak_angle = time[peak_index], actual[peak_index]
        result.append({
            "phase": phase, "start_s": time[start], "last_sample_s": time[end],
            "requested_target_deg": math.degrees(data[f"{axis}_request_rad"][start]),
            "effective_target_deg": target, "effective_step_deg": delta,
            "rise_10_90_s": rise, "settling_s": settling,
            "time_10_s": time10, "time_90_s": time90,
            "peak_time_s": peak_time, "peak_angle_deg": peak_angle,
            "settling_at_s": time[start] + settling if settling is not None else None,
            "overshoot_deg": overshoot,
            "overshoot_percent": 100 * overshoot / abs(delta) if overshoot is not None else None,
            "rms_error_deg": math.sqrt(sum(e * e for e in error) / len(error)),
            "last_second_mean_error_deg": sum(last_second) / len(last_second),
            "last_second_rms_error_deg": math.sqrt(sum(e * e for e in last_second) / len(last_second)),
            "max_abs_rate_deg_s": max(abs(math.degrees(data[rate][i])) for i in indices),
            "max_abs_actuator": max(abs(data[f"{actuator}_cmd_norm"][i]) for i in indices),
            "saturation_percent": 100 * sum(data[f"{actuator}_saturated"][i] != 0 for i in indices) / len(indices),
            "limited_percent": 100 * sum(data[limited][i] != 0 for i in indices) / len(indices),
            "max_abs_delta_tas_kt": max(abs(data["airspeed_m_s"][i] - data["airspeed_m_s"][0]) for i in indices) * KNOTS_PER_M_S,
            "max_abs_delta_altitude_m": max(abs(data["altitude_m"][i] - data["altitude_m"][0]) for i in indices),
        })
    return result


def read_case_log(path, axis):
    fields = REQUIRED + (ROLL_FIELDS if axis == "roll" else PITCH_FIELDS) + (
        "yaw_rad", "p_rad_s", "q_rad_s", "r_rad_s", "elevator_cmd_norm",
        "rudder_cmd_norm", "throttle_cmd_norm")
    return read_log(path, tuple(dict.fromkeys(fields)))


def heading_change_degrees(yaw):
    """Unwrap heading so crossing north does not create a 360-degree jump."""
    change = [0.0]
    for before, after in zip(yaw, yaw[1:]):
        change.append(change[-1] + math.degrees(math.atan2(math.sin(after - before), math.cos(after - before))))
    return change


def performance_plot(case, axis, output, band):
    """Time history of one experiment, with response events and coupled states."""
    set_plot_style()
    data, metrics = case["data"], case["metrics"]
    time = data["time_s"]
    command = [math.degrees(v) for v in data[f"{axis}_command_rad"]]
    actual = [math.degrees(v) for v in data[f"{axis}_rad"]]
    _, actuator, limited = axis_fields(axis)
    fig, axes = plt.subplots(4, 2, figsize=(14, 15), sharex=True)
    fig.subplots_adjust(top=0.89, bottom=0.21, left=0.08, right=0.97, hspace=0.46, wspace=0.25)
    angle_name = "pitch offset from trim" if axis == "pitch" else "bank request"
    fig.suptitle(f"C172X · {case['angle_deg']:+g}° {angle_name} · {case['cas_kt']:g} kt CAS",
                 x=0.08, y=0.98, ha="left", fontsize=20)
    fig.text(0.08, 0.95, f"Initial TAS {case['initial_tas_kt']:.1f} kt · "
             f"Kp {data[f'{axis}_kp'][0]:g} / Kd {data[f'{axis}_kd'][0]:g} · "
             "step at 2 s; return at 10 s", fontsize=11)
    fig.text(0.08, 0.925, "Shading: 10–90% rise interval. P: directional peak. S: settling observed through phase end.", fontsize=10)
    tracking = axes[0, 0]
    tracking.plot(time, actual, color="#1767b3", label="Measured")
    tracking.step(time, command, where="post", color="#a65516", linestyle="--", label="Effective")
    tracking.step(time, [math.degrees(v) for v in data[f"{axis}_request_rad"]],
                  where="post", color="#777777", linestyle=":", label="Requested")
    tracking.fill_between(time, [v - band for v in command], [v + band for v in command],
                          color="#a65516", alpha=0.10, step="post")
    for metric in metrics:
        if metric["rise_10_90_s"] is not None:
            tracking.axvspan(metric["time_10_s"], metric["time_90_s"], color="#c4dbe9", alpha=0.5)
        if metric["peak_time_s"] is not None:
            tracking.plot(metric["peak_time_s"], metric["peak_angle_deg"], "o", color="#8d3e8f", markersize=4)
            tracking.annotate("P", (metric["peak_time_s"], metric["peak_angle_deg"]),
                              xytext=(4, 6), textcoords="offset points", fontsize=8)
        if metric["settling_at_s"] is not None:
            tracking.plot(metric["settling_at_s"], metric["effective_target_deg"], "v", color="#25804d", markersize=5)
            tracking.annotate("S", (metric["settling_at_s"], metric["effective_target_deg"]),
                              xytext=(4, -12), textcoords="offset points", fontsize=8)
    tracking.legend(fontsize=8, ncol=3, frameon=False)
    axes[0, 1].plot(time, [c - v for c, v in zip(command, actual)], color="#1767b3")
    axes[0, 1].axhspan(-band, band, color="#dce4ed")
    for field, label, color in (("roll_rad", "Bank", "#1767b3"), ("pitch_rad", "Pitch", "#a65516")):
        axes[1, 0].plot(time, [math.degrees(v) for v in data[field]], label=label, color=color)
    axes[1, 0].legend(fontsize=8, frameon=False)
    axes[1, 1].plot(time, heading_change_degrees(data["yaw_rad"]), color="#704a99")
    for field, label in (("p_rad_s", "p (roll)"), ("q_rad_s", "q (pitch)"), ("r_rad_s", "r (yaw)")):
        axes[2, 0].plot(time, [math.degrees(v) for v in data[field]], label=label)
    axes[2, 0].legend(fontsize=8, ncol=3, frameon=False)
    for name in ("aileron", "elevator", "rudder", "throttle"):
        axes[2, 1].plot(time, data[f"{name}_cmd_norm"], label=name)
    limit = data[f"{actuator}_limit_norm"][0]
    for value in (-limit, limit):
        axes[2, 1].axhline(value, color="#777777", linestyle=":", linewidth=1)
    axes[2, 1].legend(fontsize=7, ncol=2, frameon=False)
    saturated = [bool(v) for v in data[f"{actuator}_saturated"]]
    axes[2, 1].fill_between(time, 0, 1, where=saturated, transform=axes[2, 1].get_xaxis_transform(),
                           color="#e39680", alpha=0.2, step="post")
    axes[3, 0].plot(time, [v * KNOTS_PER_M_S for v in data["airspeed_m_s"]], color="#1767b3")
    axes[3, 0].axhline(case["initial_tas_kt"], color="#777777", linestyle=":")
    axes[3, 1].plot(time, data["altitude_m"], color="#1767b3")
    axes[3, 1].axhline(data["altitude_m"][0], color="#777777", linestyle=":")
    titles = [(f"{axis.title()} response", "Angle (deg)"), (f"Error vs effective target · band ±{band:g}°", "Error (deg)"),
              ("Bank and pitch evolution", "Attitude (deg)"), ("Heading change from initial (unwrapped)", "Δ heading (deg)"),
              ("Body angular rates", "Rate (deg/s)"), (f"Control commands · shaded {actuator} saturation", "Normalized command"),
              ("True airspeed evolution", "TAS (kt)"), ("Altitude evolution", "Altitude MSL (m)")]
    for ax, (title, ylabel) in zip(axes.flat, titles):
        ax.set_title(title, loc="left", fontsize=10)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.grid(alpha=0.2)
        ax.axvline(2, color="#999999", linewidth=0.7)
        ax.axvline(10, color="#999999", linewidth=0.7)
        ax.margins(x=0, y=0.15)
    for ax in axes[-1]:
        ax.set_xlabel("Time (s)")
    fmt = lambda value: "—" if value is None else f"{value:.3g}"
    rows = [[m["phase"], fmt(m["rise_10_90_s"]), fmt(m["overshoot_deg"]),
             "NS" if m["settling_s"] is None else fmt(m["settling_s"]),
             fmt(m["last_second_rms_error_deg"]), fmt(m["saturation_percent"]), fmt(m["limited_percent"])] for m in metrics]
    table_ax = fig.add_axes([0.08, 0.075, 0.89, 0.085])
    table_ax.axis("off")
    table = table_ax.table(cellText=rows, colLabels=["Phase", "Rise 10–90% (s)", "Overshoot (deg)", "Settling (s)",
                                                  "Last 1 s RMS (deg)", "Saturation (%)", "Limited (%)"],
                          cellLoc="center", loc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.8)
    fig.text(0.08, 0.027, "NS: no settling observed; —: no measurable effective step/crossing. Timing is relative to each phase start.\n"
             "Only the selected axis is controlled. Other controls stay at trim; heading, speed and altitude can drift.", fontsize=9)
    save_figure(fig, output)


def comparison_plot(cases, axis, angle, speeds, output, band):
    selected = [case for case in cases if case["angle_deg"] == angle]
    set_plot_style()
    fig, axes = plt.subplots(3, 2, figsize=(13, 11), sharex=True)
    fig.subplots_adjust(top=0.81, bottom=0.10, left=0.09, right=0.97, hspace=0.36, wspace=0.25)
    suffix = "offset from trim" if axis == "pitch" else "bank request"
    fig.suptitle(f"C172X {axis} · {angle:+g}° {suffix}", x=0.09, y=0.97, ha="left", fontsize=19)
    fig.text(0.09, 0.93, "Solid: measured · dashed: effective command · dotted: request. Colors identify initial CAS.", fontsize=10)
    colors = {speed: plt.get_cmap("tab10")(i % 10) for i, speed in enumerate(speeds)}
    rate, actuator, _ = axis_fields(axis)
    handles, labels = [], []
    for case in selected:
        if case["status"] == "failed":
            continue
        data = case["data"]
        time = data["time_s"]
        color = colors[case["cas_kt"]]
        line, = axes[0, 0].plot(time, [math.degrees(v) for v in data[f"{axis}_rad"]], color=color)
        axes[0, 0].step(time, [math.degrees(v) for v in data[f"{axis}_command_rad"]], where="post", color=color, linestyle="--", alpha=0.7)
        axes[0, 0].step(time, [math.degrees(v) for v in data[f"{axis}_request_rad"]], where="post", color=color, linestyle=":", alpha=0.6)
        axes[0, 1].plot(time, [math.degrees(c - v) for c, v in zip(data[f"{axis}_command_rad"], data[f"{axis}_rad"])], color=color)
        axes[1, 0].plot(time, [math.degrees(v) for v in data[rate]], color=color)
        axes[1, 1].plot(time, data[f"{actuator}_cmd_norm"], color=color)
        axes[2, 0].plot(time, [(v - data["airspeed_m_s"][0]) * KNOTS_PER_M_S for v in data["airspeed_m_s"]], color=color)
        axes[2, 1].plot(time, [v - data["altitude_m"][0] for v in data["altitude_m"]], color=color)
        handles.append(line)
        label = f"{case['cas_kt']:g} CAS / {case['initial_tas_kt']:.1f} TAS kt"
        if any(m["limited_percent"] for m in case["metrics"]):
            label += " [limited]"
        labels.append(label)
    axes[0, 1].axhspan(-band, band, color="#dce4ed", alpha=0.5)
    for case in selected:
        if case["status"] != "failed":
            limit = case["data"][f"{actuator}_limit_norm"][0]
            for value in (-limit, limit):
                axes[1, 1].axhline(value, color="#777777", linestyle=":", linewidth=1)
            break
    panels = [(f"{axis.title()} tracking", "Angle (deg)"), (f"Error vs effective command (band ±{band:g}°)", "Error (deg)"),
              ("Body angular rate", "Rate (deg/s)"), (f"{actuator.title()} demand", "Normalized command"),
              ("True airspeed change", "Δ TAS (kt)"), ("Altitude change", "Δ altitude (m)")]
    for ax, (title, ylabel) in zip(axes.flat, panels):
        ax.set_title(title, loc="left", fontsize=11)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.2)
        for t in (2, 10):
            ax.axvline(t, color="#999999", linewidth=0.7)
    for ax in axes[-1]:
        ax.set_xlabel("Time (s)")
    if handles:
        fig.legend(handles, labels, loc="upper left", bbox_to_anchor=(0.08, 0.91), ncol=2, frameon=False, fontsize=9)
    failed = [f"{c['cas_kt']:g}" for c in selected if c["status"] == "failed"]
    note = "Step: 2–10 s; return: 10 s onward. Other controls remain at trim; speed and altitude are not held."
    if failed:
        note += "\nFailed CAS (kt): " + ", ".join(failed) + ". See summary.json and simulator logs."
    fig.text(0.09, 0.025, note, fontsize=9)
    save_figure(fig, output)


def matrix_plot(cases, phase, speeds, angles, axis, output, band):
    metrics = [("settling_s", "Settling time (s)"), ("overshoot_deg", "Overshoot (deg)"),
               ("last_second_rms_error_deg", "Last-second RMS error (deg)"),
               ("saturation_percent", "Actuator saturation (%)"),
               ("max_abs_delta_tas_kt", "Max |Δ TAS| (kt)"),
               ("max_abs_delta_altitude_m", "Max |Δ altitude| (m)")]
    fig, axes = plt.subplots(3, 2, figsize=(13, max(10, 0.9 * len(angles) + 4)))
    fig.subplots_adjust(top=0.89, bottom=0.10, hspace=0.48, wspace=0.35)
    fig.suptitle(f"C172X {axis} · {phase} metrics", fontsize=19, y=0.97)
    fig.text(0.08, 0.93, f"Settling band ±{band:g}° through end of observed phase. Lower values are darker.", fontsize=10)
    lookup = {(c["cas_kt"], c["angle_deg"]): c for c in cases}
    cmap = plt.get_cmap("YlOrRd").reversed().copy()
    cmap.set_bad("#e1e5e9")
    for ax, (key, title) in zip(axes.flat, metrics):
        values, texts = [], []
        for angle in angles:
            row, labels = [], []
            for speed in speeds:
                case = lookup[(speed, angle)]
                metric = next((m for m in case.get("metrics", []) if m["phase"] == phase), None)
                value = metric[key] if metric else None
                row.append(value if value is not None else float("nan"))
                label = f"{value:.2g}" if value is not None else (
                    ("NS" if key == "settling_s" else "N/A") if metric else "FAIL")
                if metric and metric["limited_percent"]:
                    label += "*"
                labels.append(label)
            values.append(row)
            texts.append(labels)
        im = ax.imshow(values, aspect="auto", cmap=cmap, vmin=0)
        ax.set_xticks(range(len(speeds)), [f"{v:g}" for v in speeds])
        ax.set_yticks(range(len(angles)), [f"{v:+g}°" for v in angles])
        ax.set_title(title, loc="left", fontsize=11)
        ax.set_xlabel("Initial CAS (kt)")
        ax.set_ylabel("Pitch offset" if axis == "pitch" else "Bank request")
        for i, row in enumerate(texts):
            for j, label in enumerate(row):
                ax.text(j, i, label, ha="center", va="center", fontsize=9,
                        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none", "pad": 1})
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03)
    fig.text(0.08, 0.025, "NS: settling not observed. FAIL: simulation failed. *: command limited; errors use effective target.\n"
             "Step: [2,10) s; return: [10,end]. Speed/altitude changes are relative to the initial state.", fontsize=9)
    save_figure(fig, output)


def export_report(cases, args, output):
    for case in cases:
        if case["status"] != "failed":
            stem = output / f"performance_{args.axis}_{angle_token(case['angle_deg'])}deg_{str(case['cas_kt']).replace('.', 'p')}kt"
            performance_plot(case, args.axis, stem, args.settling_band_deg)
            case["performance_png"] = str(stem.with_suffix(".png"))
            case["performance_svg"] = str(stem.with_suffix(".svg"))
    public_cases = [{k: v for k, v in c.items() if k != "data"} for c in cases]
    report = {"settings": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
              "definitions": {"angles": "Roll: absolute bank. Pitch: offset from trim; limit is absolute attitude.",
                              "errors": "Effective command minus measured attitude; degrees.",
                              "settling": "Last entry into ±band persisting through last observed sample; null means not observed.",
                              "rise": "First 10% to first 90% crossing of the effective command step, sampled without interpolation.",
                              "last_second": "Last second of each observed phase; not assumed steady state.",
                              "saturation": "Percentage of phase samples with actuator saturation flag.",
                              "changes": "TAS and altitude changes are relative to the run's initial state.",
                              "phases": "Step [2,10) s; return [10,duration) s."}, "cases": public_cases}
    (output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    rows = []
    for case in public_cases:
        base = {k: case.get(k, "") for k in ("axis", "cas_kt", "angle_deg", "initial_tas_kt", "status", "error", "csv", "log", "performance_png", "performance_svg")}
        rows.extend([{**base, **m} for m in case.get("metrics", [{}])])
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with (output / "summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    for angle in args.angles:
        comparison_plot(cases, args.axis, angle, args.speeds, output / f"compare_{angle_token(angle)}deg", args.settling_band_deg)
    for phase in ("step", "return"):
        matrix_plot(cases, phase, args.speeds, args.angles, args.axis, output / f"matrix_{phase}", args.settling_band_deg)
    lines = [f"# C172X {args.axis}: performance over time", "",
             "Each link opens an individual case: attitude evolution, response events, body rates, controls, TAS and altitude.", "",
             "| CAS (kt) | Request (deg) | Time history | SVG |",
             "| --- | --- | --- | --- |"]
    for case in cases:
        if case["status"] == "failed":
            lines.append(f"| {case['cas_kt']:g} | {case['angle_deg']:+g} | FAILED: see simulator log | — |")
        else:
            lines.append(f"| {case['cas_kt']:g} | {case['angle_deg']:+g} | [PNG]({case['performance_png']}) | [SVG]({case['performance_svg']}) |")
    (output / "index.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    total = len(cases)
    failures = sum(c["status"] == "failed" for c in cases)
    print(f"Report: {output}\n{total - failures}/{total} completed; {failures} failed. See summary.csv / summary.json.")
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replot", type=Path, help="Regenerate a saved sweep folder from its CSVs, without rerunning simulations; uses saved settings")
    parser.add_argument("--axis", choices=("roll", "pitch"), default="roll")
    parser.add_argument("--sim", type=Path, default=Path("build/host/bin/Debug/autopilot_sim"))
    parser.add_argument("--speeds", nargs="+", type=float, default=[60, 75, 90, 100, 110], help="Initial calibrated airspeeds in knots")
    parser.add_argument("--angles", nargs="+", type=float, default=[5, -5, 10, -10, 15, -15, 30, -30], help="Roll bank angles or pitch offsets from trim in degrees")
    parser.add_argument("--angle-limit-deg", type=float, help="Absolute attitude limit (defaults: roll 30, pitch 45)")
    parser.add_argument("--duration", type=float, default=20)
    parser.add_argument("--kp", type=float, help="Defaults: roll 4, pitch 10")
    parser.add_argument("--kd", type=float, help="Defaults: roll 0.5, pitch 3")
    parser.add_argument("--settling-band-deg", type=float, default=0.25)
    parser.add_argument("--timeout", type=float, default=60, help="Wall-clock timeout per simulation")
    parser.add_argument("--output-dir", type=Path, default=Path("logs/sweeps"))
    args = parser.parse_args()
    if args.replot:
        output = args.replot.resolve()
        try:
            report = json.loads((output / "summary.json").read_text())
            saved = argparse.Namespace(**report["settings"])
            cases = report["cases"]
            for case in cases:
                if case["status"] != "failed":
                    case["data"] = read_case_log(Path(case["csv"]), saved.axis)
                    case["metrics"] = segment_metrics(case["data"], saved.axis, saved.settling_band_deg)
            return export_report(cases, saved, output)
        except (OSError, ValueError, KeyError) as error:
            parser.error(f"Cannot replot saved report: {error}")
    args.angle_limit_deg = args.angle_limit_deg if args.angle_limit_deg is not None else (30 if args.axis == "roll" else 45)
    args.kp = args.kp if args.kp is not None else (4 if args.axis == "roll" else 10)
    args.kd = args.kd if args.kd is not None else (0.5 if args.axis == "roll" else 3)
    numbers = [*args.speeds, *args.angles, args.angle_limit_deg, args.duration, args.kp, args.kd, args.settling_band_deg, args.timeout]
    if not all(map(math.isfinite, numbers)):
        parser.error("All numeric arguments must be finite")
    if any(v <= 0 for v in args.speeds) or any(v == 0 or abs(v) > 90 for v in args.angles):
        parser.error("Speeds must be positive; angles must be nonzero and within ±90 degrees")
    if not 0 < args.angle_limit_deg < 90 or not 11 <= args.duration <= 3600:
        parser.error("Angle limit must be in (0,90); duration must be in [11,3600] to observe both phases")
    if args.kp <= 0 or args.kd < 0 or args.settling_band_deg <= 0 or args.timeout <= 0:
        parser.error("Kp, settling band and timeout must be positive; Kd must be nonnegative")
    args.speeds = list(dict.fromkeys(args.speeds))
    args.angles = list(dict.fromkeys(args.angles))
    sim = args.sim.resolve()
    if not sim.is_file():
        parser.error(f"Simulator not found: {sim}. Build the host preset first.")
    output = args.output_dir.resolve() / f"{args.axis}_{datetime.now():%Y%m%d_%H%M%S_%f}"
    output.mkdir(parents=True)
    cases = []
    total = len(args.speeds) * len(args.angles)
    print(f"Running {total} cases. Report: {output}", flush=True)
    for angle in args.angles:
        for speed in args.speeds:
            stem = f"{args.axis}_{angle_token(angle)}deg_{speed}kt"
            csv_path = output / f"{stem}.csv"
            log_path = output / f"{stem}.log"
            command = [str(sim), "--mode", f"{args.axis}-hold", "--airspeed-kts", str(speed),
                       "--bank-deg" if args.axis == "roll" else "--pitch-deg", str(angle),
                       "--bank-limit-deg" if args.axis == "roll" else "--pitch-limit-deg", str(args.angle_limit_deg),
                       f"--{args.axis}-kp", str(args.kp), f"--{args.axis}-kd", str(args.kd),
                       "--duration", str(args.duration), "--output", str(csv_path)]
            case = {"axis": args.axis, "cas_kt": speed, "angle_deg": angle, "command": command,
                    "csv": str(csv_path), "log": str(log_path), "status": "failed"}
            try:
                with log_path.open("w") as log:
                    process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
                if process.returncode:
                    raise ValueError(f"Simulator exit {process.returncode}: {log_path.read_text()[-1200:].strip()}")
                data = read_case_log(csv_path, args.axis)
                if data["time_s"][-1] < args.duration - 0.02:
                    raise ValueError("Simulation log is incomplete")
                case.update(status="ok", data=data, initial_tas_kt=data["airspeed_m_s"][0] * KNOTS_PER_M_S,
                            metrics=segment_metrics(data, args.axis, args.settling_band_deg))
            except (OSError, ValueError, subprocess.TimeoutExpired) as error:
                case["error"] = str(error)
            cases.append(case)
            print(f"[{len(cases)}/{total}] {angle:+g} deg, {speed:g} kt: {case['status']}", flush=True)
    return export_report(cases, args, output)



if __name__ == "__main__":
    sys.exit(main())
