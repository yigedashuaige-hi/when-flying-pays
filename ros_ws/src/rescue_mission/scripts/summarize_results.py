#!/usr/bin/env python3
"""
Summarize experiment runs from results/runs/*.csv.

Usage:
    python3 summarize_results.py                    # uses default paths
    python3 summarize_results.py <runs_dir>
    python3 summarize_results.py <runs_dir> <output_summary.csv>

Filename convention: {strategy}__{YYYYMMDD_HHMMSS}.csv
Outputs:
    results/summary.csv   — per-scene/per-strategy mean ± std
    results/all_runs.csv  — flat table of every run
"""
import csv
import glob
import os
import statistics
import argparse

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_SCRIPT_DIR)

NUMERIC_KEYS = [
    "total_time_s",
    "total_energy_j",
    "ground_energy_j",
    "air_energy_j",
    "takeoff_energy_j",
    "land_energy_j",
    "switch_energy_j",
    "path_length_m",
    "ground_distance_m",
    "air_distance_m",
    "switch_count",
    "collision_count",
    "stuck_count",
    "stuck_event_count",
    "slip_ratio",
    "final_distance_m",
    "J_ground",
    "J_air",
    "risk_cvar",
    "energy_term",
    "shield_override_count",
    "unsafe_action_count",
    "irreversible_failure",
    "entered_unrecoverable_set",
    "recoverability_margin_min",
    "safe_landing_margin_j",
    # Analysis-only ground-truth terrain characterization. NEVER a strategy
    # decision input (strategies only ever see the noisy p_trav); these columns
    # let us verify that, under a given seed, both strategies faced comparable
    # terrain fields.
    "theta_true_mean",
    "theta_true_min",
]


def row_float(row, key, default=0.0):
    value = row.get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def row_int(row, key, default=0):
    value = row.get(key, default)
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def strategy_from_filename(basename):
    """Extract strategy tag from  '{strategy}__{timestamp}.csv'."""
    name = basename.replace(".csv", "")
    if "__" in name:
        return name.split("__")[0]
    # Fallback: everything before last two underscore-separated tokens (date + time)
    parts = name.rsplit("_", 2)
    return parts[0] if len(parts) == 3 else name


def timestamp_from_filename(basename):
    name = basename.replace(".csv", "")
    if "__" not in name:
        return ""
    return name.split("__", 1)[1]


def parse_run(filepath):
    """Return a summary dict for a single run CSV (uses last row for cumulative stats)."""
    rows = []
    try:
        with open(filepath) as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)
    except Exception as exc:
        print(f"  [warn] could not read {filepath}: {exc}")
        return None

    if not rows:
        print(f"  [warn] empty file: {filepath}")
        return None

    if not any(row.get("mode", "").strip() for row in rows):
        print(f"  [warn] no mode samples in {os.path.basename(filepath)} — skipping infrastructure-failed run")
        return None

    last = rows[-1]
    terminal = next((row for row in rows if row.get("success") == "1"), last)
    basename = os.path.basename(filepath)
    strategy = strategy_from_filename(basename)

    # Ground-truth terrain characterization over the whole run (analysis only).
    theta_vals = [row_float(r, "traversability_true", float("nan")) for r in rows]
    theta_vals = [v for v in theta_vals if v == v]
    theta_true_mean = statistics.mean(theta_vals) if theta_vals else 0.0
    theta_true_min = min(theta_vals) if theta_vals else 0.0

    try:
        return {
            "scene":           last.get("scene", "scene_a"),
            "strategy":        strategy,
            "seed":            row_int(last, "seed", -1),
            "timestamp":       timestamp_from_filename(basename),
            "file":            basename,
            "total_time_s":    row_float(terminal, "time"),
            "total_energy_j":  row_float(terminal, "energy_j"),
            "ground_energy_j": row_float(terminal, "ground_energy_j"),
            "air_energy_j":    row_float(terminal, "air_energy_j"),
            "takeoff_energy_j": row_float(terminal, "takeoff_energy_j"),
            "land_energy_j":   row_float(terminal, "land_energy_j"),
            "switch_energy_j": row_float(terminal, "switch_energy_j"),
            "path_length_m":   row_float(terminal, "path_length"),
            "ground_distance_m": row_float(terminal, "ground_distance_m"),
            "air_distance_m":  row_float(terminal, "air_distance_m"),
            "switch_count":    row_float(terminal, "switch_count"),
            "collision_count": row_float(terminal, "collision_count"),
            "stuck_count":     row_float(terminal, "stuck_count", row_float(terminal, "collision_count")),
            "stuck_event_count": row_float(
                terminal,
                "stuck_event_count",
                row_float(terminal, "stuck_count", row_float(terminal, "collision_count")),
            ),
            "slip_ratio":       row_float(terminal, "slip_ratio"),
            "final_distance_m": row_float(terminal, "final_distance"),
            "J_ground":        row_float(terminal, "J_ground"),
            "J_air":           row_float(terminal, "J_air"),
            "risk_cvar":       row_float(terminal, "risk_cvar"),
            "energy_term":     row_float(terminal, "energy_term"),
            # Supervisor metrics are cumulative/latched -> read from the final row.
            "shield_override_count": row_float(last, "shield_override_count"),
            "unsafe_action_count":   row_float(last, "unsafe_action_count"),
            "irreversible_failure":  row_float(last, "irreversible_failure"),
            "entered_unrecoverable_set": row_float(last, "entered_unrecoverable_set"),
            "recoverability_margin_min": row_float(last, "recoverability_margin_min"),
            "safe_landing_margin_j": row_float(last, "safe_landing_margin_j"),
            "theta_true_mean": theta_true_mean,
            "theta_true_min":  theta_true_min,
            "success":         row_int(terminal, "success"),
            "timeout":         row_int(terminal, "timeout"),
        }
    except KeyError as exc:
        print(f"  [warn] missing column {exc} in {basename} — skipping")
        return None


def mean_std(values):
    m = statistics.mean(values)
    s = statistics.stdev(values) if len(values) > 1 else 0.0
    return m, s


def summarize(runs_dir, summary_path, min_duration=0.0, scene="", since="", until=""):
    files = sorted(glob.glob(os.path.join(runs_dir, "*.csv")))
    if not files:
        print(f"No CSV files found in {runs_dir}")
        return

    print(f"Found {len(files)} run file(s) in {runs_dir}")
    parsed_runs = [r for r in (parse_run(f) for f in files) if r is not None]
    runs = [r for r in parsed_runs if r["total_time_s"] >= min_duration]
    if scene:
        runs = [r for r in runs if r["scene"] == scene]
    if since:
        runs = [r for r in runs if r["timestamp"] >= since]
    if until:
        runs = [r for r in runs if r["timestamp"] <= until]
    skipped = len(parsed_runs) - len(runs)
    if skipped:
        print(f"Filtered out {skipped} run(s) by duration/scene/timestamp criteria")
    if not runs:
        print("No valid runs to summarize.")
        return

    # ── per-run flat table ─────────────────────────────────────────────────────
    summary_basename = os.path.basename(summary_path)
    if summary_basename == "summary.csv":
        all_runs_name = "all_runs.csv"
    else:
        all_runs_name = summary_basename.replace("summary", "all_runs", 1)
    all_runs_path = os.path.join(os.path.dirname(summary_path), all_runs_name)
    with open(all_runs_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(runs[0].keys()))
        writer.writeheader()
        writer.writerows(runs)
    print(f"Per-run table  → {all_runs_path}")

    # ── group by scene + strategy ──────────────────────────────────────────────
    groups: dict = {}
    for run in runs:
        key = (run["scene"], run["strategy"])
        groups.setdefault(key, []).append(run)

    summary_rows = []
    for scene, strategy in sorted(groups):
        group = groups[(scene, strategy)]
        n = len(group)
        success_rate = sum(r["success"] for r in group) / n

        row = {
            "scene": scene,
            "strategy": strategy,
            "n_runs": n,
            "success_rate": f"{success_rate:.2f}",
        }
        for key in NUMERIC_KEYS:
            m, s = mean_std([r[key] for r in group])
            row[f"{key}_mean"] = f"{m:.3f}"
            row[f"{key}_std"]  = f"{s:.3f}"
        summary_rows.append(row)

    with open(summary_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)
    print(f"Summary table  → {summary_path}")

    # ── console report ─────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  EXPERIMENT SUMMARY")
    print("=" * 60)
    header = f"{'Scene':<10} {'Strategy':<16} {'N':>3}  {'Success':>7}  {'Time(s)':>10}  {'Energy(J)':>10}  {'Gnd(J)':>8}  {'Air(J)':>8}  {'Switches':>8}  {'Dist(m)':>8}"
    print(header)
    print("-" * len(header))
    for row in summary_rows:
        print(
            f"{row['scene']:<10} {row['strategy']:<16} {row['n_runs']:>3}  "
            f"{row['success_rate']:>7}  "
            f"{row['total_time_s_mean']:>6}±{row['total_time_s_std']:<4}  "
            f"{row['total_energy_j_mean']:>6}±{row['total_energy_j_std']:<4}  "
            f"{row['ground_energy_j_mean']:>5}±{row['ground_energy_j_std']:<3}  "
            f"{row['air_energy_j_mean']:>5}±{row['air_energy_j_std']:<3}  "
            f"{row['switch_count_mean']:>5}±{row['switch_count_std']:<3}  "
            f"{row['final_distance_m_mean']:>5}±{row['final_distance_m_std']:<3}"
        )
    print("=" * 60)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs_dir", nargs="?", default=os.path.join(_PKG_DIR, "results", "runs"))
    parser.add_argument("summary_path", nargs="?", default=os.path.join(_PKG_DIR, "results", "summary.csv"))
    parser.add_argument("--min-duration", type=float, default=0.0)
    parser.add_argument("--scene", default="")
    parser.add_argument("--since", default="")
    parser.add_argument("--until", default="")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    runs_dir = args.runs_dir
    summary_path = args.summary_path

    os.makedirs(os.path.dirname(summary_path), exist_ok=True)
    summarize(runs_dir, summary_path, args.min_duration, args.scene, args.since, args.until)
