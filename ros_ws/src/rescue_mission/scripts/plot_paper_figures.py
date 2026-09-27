#!/usr/bin/env python3
"""Paper figures with error bars from a summary CSV (paper-hardening).

Generates one bar chart per key metric (mean +/- std across seeds), one strategy
per bar. Degrades gracefully if matplotlib is unavailable (writes a manifest so
the experiment pipeline never fails just because plotting is missing).

Usage:
    rosrun rescue_mission plot_paper_figures.py summary_pr5_scene_b.csv \
        --output-dir results/figures_pr5_scene_b
"""
import argparse
import csv
import os

METRICS = [
    ("success_rate", "Success rate", False),
    ("stuck_event_count", "Stuck events (mean +/- std)", True),
    ("total_energy_j", "Total energy J (mean +/- std)", True),
    ("irreversible_failure", "Irreversible failure (mean +/- std)", True),
    ("shield_override_count", "Shield overrides (mean +/- std)", True),
]


def load_summary(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def fval(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("summary_csv")
    ap.add_argument("--output-dir", default="")
    ap.add_argument("--title-prefix", default="")
    args = ap.parse_args()

    rows = load_summary(args.summary_csv)
    out_dir = args.output_dir or os.path.join(
        os.path.dirname(os.path.abspath(args.summary_csv)),
        "figures_" + os.path.splitext(os.path.basename(args.summary_csv))[0],
    )
    os.makedirs(out_dir, exist_ok=True)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        manifest = os.path.join(out_dir, "plot_manifest.md")
        with open(manifest, "w") as f:
            f.write("# Figures skipped\n\nmatplotlib unavailable: %s\n" % exc)
        print("matplotlib unavailable (%s); wrote %s" % (exc, manifest))
        return

    labels = [r.get("strategy", "?") for r in rows]
    written = []
    for key, title, has_std in METRICS:
        mean_key = key if key == "success_rate" else key + "_mean"
        std_key = None if key == "success_rate" else key + "_std"
        if not any(mean_key in r for r in rows):
            continue
        means = [fval(r, mean_key) for r in rows]
        errs = [fval(r, std_key) for r in rows] if std_key else None

        fig, ax = plt.subplots(figsize=(6, 4))
        x = range(len(labels))
        ax.bar(x, means, yerr=errs, capsize=5, color="#4C72B0", edgecolor="black")
        ax.set_xticks(list(x))
        ax.set_xticklabels(labels, rotation=20, ha="right")
        ax.set_title((args.title_prefix + " " if args.title_prefix else "") + title)
        ax.grid(axis="y", linestyle=":", alpha=0.5)
        fig.tight_layout()
        out_path = os.path.join(out_dir, key + ".png")
        fig.savefig(out_path, dpi=130)
        plt.close(fig)
        written.append(out_path)

    manifest = os.path.join(out_dir, "plot_manifest.md")
    with open(manifest, "w") as f:
        f.write("# Paper figures\n\n")
        for p in written:
            f.write("- %s\n" % os.path.basename(p))
    print("Wrote %d figure(s) to %s" % (len(written), out_dir))


if __name__ == "__main__":
    main()
