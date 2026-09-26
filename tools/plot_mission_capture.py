"""Plot planned and flown paths from the JSBSim mission_capture_test CSVs."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="Output base path for PNG and SVG")
    args = parser.parse_args()
    files = sorted(p for p in args.directory.glob("*.csv") if not p.stem.endswith("_plan"))
    if not files:
        parser.error("No trajectory CSVs found")
    fig, axes = plt.subplots((len(files)+1)//2, 2, figsize=(12, 3.5*((len(files)+1)//2)), squeeze=False)
    for axis, path in zip(axes.flat, files):
        rows = read(path)
        plan = read(path.with_stem(path.stem+"_plan"))
        axis.plot([0,0,.4],[0,2,4], "o--", color="#596579", lw=1.3, label="Mission route")
        axis.plot([float(r["east_m"])/1000 for r in plan], [float(r["north_m"])/1000 for r in plan],
                  color="#d97706", lw=2.5, alpha=.65, label="Dubins entry plan")
        axis.plot([float(r["east_m"])/1000 for r in rows], [float(r["north_m"])/1000 for r in rows],
                  color="#1565c0", lw=1, label="Flown path")
        axis.scatter(float(rows[0]["east_m"])/1000, float(rows[0]["north_m"])/1000,
                     marker="^", color="#1565c0", s=45, zorder=5, label="Engagement")
        axis.set_title(f"{path.stem.replace('_',' ')} | entry leg {int(rows[0]['mission_leg'])+1}", fontsize=11)
        axis.set_xlabel("East (km)"); axis.set_ylabel("North (km)")
        axis.set_aspect("equal", adjustable="datalim"); axis.grid(alpha=.2)
    for axis in list(axes.flat)[len(files):]:
        axis.set_visible(False)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, bbox_to_anchor=(.5,.965), frameon=False)
    fig.suptitle("C172X mission entry — nearest leg, then follow in order", fontsize=15, y=.993)
    fig.tight_layout(rect=(0,0,1,.94))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for suffix in (".png", ".svg"):
        fig.savefig(args.output.with_suffix(suffix), dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
