#!/usr/bin/env python3
"""
Generate paper-ready figures from rescue_mission summary CSV files.

Examples:
    python3 plot_results.py
    python3 plot_results.py results/summary_scene_a_latest.csv
    python3 plot_results.py results/summary.csv --output-dir results/figures_rule_check
"""
import argparse
import csv
import os
import sys

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ModuleNotFoundError:
    matplotlib = None
    plt = None

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_SCRIPT_DIR)

STRATEGY_ORDER = [
    "ground_only",
    "air_preferred",
    "fixed_switch",
    "rule",
    "energy_rule",
    "risk_aware",
    "rule_energy",
    "rule_energy_shield",
    "safe_rule",
    "rl",
    "safe_rl",
]

STRATEGY_LABELS = {
    "ground_only": "Ground only",
    "air_preferred": "Air preferred",
    "fixed_switch": "Fixed switch",
    "rule": "Rule",
    "energy_rule": "Energy rule",
    "risk_aware": "Risk-aware",
    "rule_energy": "Rule energy",
    "rule_energy_shield": "Rule + shield",
    "safe_rule": "Rule + shield",
    "rl": "RL",
    "safe_rl": "Safe RL",
}

COLORS = {
    "ground_only": "#4C78A8",
    "air_preferred": "#F58518",
    "fixed_switch": "#54A24B",
    "rule": "#B279A2",
    "energy_rule": "#4C78A8",
    "risk_aware": "#E45756",
    "rule_energy": "#B279A2",
    "rule_energy_shield": "#72B7B2",
    "safe_rule": "#72B7B2",
    "rl": "#E45756",
    "safe_rl": "#E45756",
}

_CURRENT_ROWS = []


def _float(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def row_sort_key(row):
    strategy = row.get("strategy", "")
    try:
        strategy_index = STRATEGY_ORDER.index(strategy)
    except ValueError:
        strategy_index = len(STRATEGY_ORDER)
    return (row.get("scene", ""), strategy_index, strategy)


def load_summary(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"No rows found in {path}")
    return sorted(rows, key=row_sort_key)


def row_label(row):
    scene = row.get("scene", "")
    strategy = row.get("strategy", "")
    label = STRATEGY_LABELS.get(strategy, strategy)
    if len({r.get("scene", "") for r in _CURRENT_ROWS}) > 1:
        return f"{scene}\n{label}"
    return label


def row_color(row):
    return COLORS.get(row.get("strategy", ""), "#777777")


def style_axes(ax, ylabel):
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", alpha=0.25, linewidth=0.8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(axis="x", rotation=20)


def save_bar(rows, value_key, std_key, ylabel, title, output_path):
    labels = [row_label(row) for row in rows]
    values = [_float(row, value_key) for row in rows]
    errors = [_float(row, std_key) for row in rows]
    colors = [row_color(row) for row in rows]

    fig_width = max(7.0, 1.3 * len(rows))
    fig, ax = plt.subplots(figsize=(fig_width, 4.5), constrained_layout=True)
    ax.bar(labels, values, yerr=errors, capsize=4, color=colors, edgecolor="#333333", linewidth=0.8)
    ax.set_title(title)
    style_axes(ax, ylabel)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def save_energy_split(rows, output_path):
    labels = [row_label(row) for row in rows]
    ground = [_float(row, "ground_energy_j_mean") for row in rows]
    takeoff = [_float(row, "takeoff_energy_j_mean") for row in rows]
    air = [_float(row, "air_energy_j_mean") for row in rows]
    land = [_float(row, "land_energy_j_mean") for row in rows]
    switch = [_float(row, "switch_energy_j_mean") for row in rows]
    totals = [_float(row, "total_energy_j_mean") for row in rows]

    fig_width = max(7.0, 1.3 * len(rows))
    fig, ax = plt.subplots(figsize=(fig_width, 4.5), constrained_layout=True)
    ax.bar(labels, ground, color="#4C78A8", edgecolor="#333333", linewidth=0.8, label="Ground energy")
    bottom = ground[:]
    ax.bar(labels, takeoff, bottom=bottom, color="#9D755D", edgecolor="#333333", linewidth=0.8, label="Takeoff")
    bottom = [a + b for a, b in zip(bottom, takeoff)]
    ax.bar(labels, air, bottom=bottom, color="#F58518", edgecolor="#333333", linewidth=0.8, label="Air cruise")
    bottom = [a + b for a, b in zip(bottom, air)]
    ax.bar(labels, land, bottom=bottom, color="#FF9DA6", edgecolor="#333333", linewidth=0.8, label="Land")
    bottom = [a + b for a, b in zip(bottom, land)]
    ax.bar(labels, switch, bottom=bottom, color="#BAB0AC", edgecolor="#333333", linewidth=0.8, label="Switch penalty")
    y_offset = max(totals) * 0.025 if totals else 0.1
    for i, total in enumerate(totals):
        ax.text(i, total + y_offset, f"{total:.1f}", ha="center", va="bottom", fontsize=9)
    ax.set_title("Energy split by mode")
    style_axes(ax, "Energy (J)")
    ax.legend(frameon=False, ncol=3)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def save_time_success(rows, output_path):
    labels = [row_label(row) for row in rows]
    times = [_float(row, "total_time_s_mean") for row in rows]
    time_std = [_float(row, "total_time_s_std") for row in rows]
    success = [_float(row, "success_rate") for row in rows]
    colors = [row_color(row) for row in rows]

    fig_width = max(8.0, 1.4 * len(rows))
    fig, axes = plt.subplots(1, 2, figsize=(fig_width, 4.3), constrained_layout=True)
    axes[0].bar(labels, times, yerr=time_std, capsize=4, color=colors, edgecolor="#333333", linewidth=0.8)
    axes[0].set_title("Completion time")
    style_axes(axes[0], "Time (s)")

    axes[1].bar(labels, success, color=colors, edgecolor="#333333", linewidth=0.8)
    axes[1].set_ylim(0.0, 1.08)
    axes[1].set_title("Success rate")
    style_axes(axes[1], "Success rate")
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def save_diagnostics(rows, output_path):
    labels = [row_label(row) for row in rows]
    metrics = [
        ("path_length_m_mean", "path_length_m_std", "Path length", "m"),
        ("switch_count_mean", "switch_count_std", "Switch count", "count"),
        ("collision_count_mean", "collision_count_std", "Collision/stuck count", "count"),
        ("final_distance_m_mean", "final_distance_m_std", "Final distance", "m"),
    ]
    colors = [row_color(row) for row in rows]

    fig_width = max(9.0, 1.35 * len(rows))
    fig, axes = plt.subplots(2, 2, figsize=(fig_width, 7.0), constrained_layout=True)
    for ax, (value_key, std_key, title, ylabel) in zip(axes.flatten(), metrics):
        values = [_float(row, value_key) for row in rows]
        errors = [_float(row, std_key) for row in rows]
        ax.bar(labels, values, yerr=errors, capsize=4, color=colors, edgecolor="#333333", linewidth=0.8)
        ax.set_title(title)
        style_axes(ax, ylabel)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def save_stuck_slip(rows, output_path):
    labels = [row_label(row) for row in rows]
    stuck_key = "stuck_event_count_mean"
    if stuck_key not in rows[0]:
        stuck_key = "stuck_count_mean"
    stuck_std_key = "stuck_event_count_std" if "stuck_event_count_std" in rows[0] else "stuck_count_std"
    stuck = [_float(row, stuck_key) for row in rows]
    stuck_std = [_float(row, stuck_std_key) for row in rows]
    slip = [_float(row, "slip_ratio_mean") for row in rows]
    slip_std = [_float(row, "slip_ratio_std") for row in rows]
    colors = [row_color(row) for row in rows]

    fig_width = max(8.0, 1.4 * len(rows))
    fig, axes = plt.subplots(1, 2, figsize=(fig_width, 4.3), constrained_layout=True)
    axes[0].bar(labels, stuck, yerr=stuck_std, capsize=4, color=colors, edgecolor="#333333", linewidth=0.8)
    axes[0].set_title("Stuck events")
    style_axes(axes[0], "count")

    axes[1].bar(labels, slip, yerr=slip_std, capsize=4, color=colors, edgecolor="#333333", linewidth=0.8)
    axes[1].set_ylim(0.0, max(1.0, max(slip) * 1.15 if slip else 1.0))
    axes[1].set_title("Ground slip ratio")
    style_axes(axes[1], "ratio")
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def write_manifest(rows, summary_path, output_dir, figures):
    manifest = os.path.join(output_dir, "plot_manifest.md")
    scenes = sorted({row.get("scene", "") for row in rows})
    strategies = [row.get("strategy", "") for row in rows]
    with open(manifest, "w") as f:
        f.write("# Rescue Mission Result Figures\n\n")
        f.write(f"- Source summary: `{os.path.abspath(summary_path)}`\n")
        f.write(f"- Scenes: {', '.join(scenes)}\n")
        f.write(f"- Strategies: {', '.join(strategies)}\n\n")
        f.write("## Generated Files\n\n")
        for path, description in figures:
            f.write(f"- `{os.path.basename(path)}`: {description}\n")
        f.write("\nNote: these figures reflect the input summary CSV only. ")
        f.write("If a strategy has `n_runs=1`, its standard deviation is shown as 0.\n")
    return manifest


def plot(summary_path, output_dir):
    rows = load_summary(summary_path)
    os.makedirs(output_dir, exist_ok=True)

    if plt is None:
        manifest = os.path.join(output_dir, "plot_manifest.md")
        with open(manifest, "w") as f:
            f.write("# Rescue Mission Result Figures\n\n")
            f.write(f"- Source summary: `{os.path.abspath(summary_path)}`\n")
            f.write("- Status: skipped because Python package `matplotlib` is not installed in this environment.\n")
            f.write("\nInstall it in the container or run this script on the host Python environment.\n")
        print("Warning: matplotlib is not installed; skipped figure generation.")
        print(f"Wrote {manifest}")
        return

    global _CURRENT_ROWS
    _CURRENT_ROWS = rows

    figures = [
        (os.path.join(output_dir, "energy_total.png"), "Total energy mean with standard deviation"),
        (os.path.join(output_dir, "energy_split.png"), "Ground/air energy split"),
        (os.path.join(output_dir, "time_success.png"), "Completion time and success rate"),
        (
            os.path.join(output_dir, "path_switch_collision_distance.png"),
            "Path length, switch count, collision count, and final distance",
        ),
        (os.path.join(output_dir, "stuck_slip.png"), "Stuck events and ground slip ratio"),
    ]

    save_bar(
        rows,
        "total_energy_j_mean",
        "total_energy_j_std",
        "Energy (J)",
        "Total energy",
        figures[0][0],
    )
    save_energy_split(rows, figures[1][0])
    save_time_success(rows, figures[2][0])
    save_diagnostics(rows, figures[3][0])
    save_stuck_slip(rows, figures[4][0])
    manifest = write_manifest(rows, summary_path, output_dir, figures)

    print(f"Loaded {len(rows)} summary row(s) from {summary_path}")
    for path, _description in figures:
        print(f"Wrote {path}")
    print(f"Wrote {manifest}")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "summary_path",
        nargs="?",
        default=os.path.join(_PKG_DIR, "results", "summary.csv"),
        help="Input summary CSV generated by summarize_results.py",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Output figure directory. Defaults to results/figures_<summary_name>.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    if plt is None:
        # Keep --plot from making experiment runner fail after a valid ROS run.
        # The plot() function still writes a manifest explaining the skip.
        pass
    args = parse_args()
    summary_path = os.path.abspath(args.summary_path)
    if args.output_dir:
        output_dir = os.path.abspath(args.output_dir)
    else:
        stem = os.path.splitext(os.path.basename(summary_path))[0]
        output_dir = os.path.join(os.path.dirname(summary_path), f"figures_{stem}")
    plot(summary_path, output_dir)
