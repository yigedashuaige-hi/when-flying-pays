#!/usr/bin/env python3
"""Check whether strategies saw comparable shared perception inputs."""
import argparse
import csv
import glob
import math
import os
import statistics
import sys

SHARED_INPUT_COLUMNS = [
    "traversability_mean",
    "traversability_uncertainty",
    "obstacle_density",
    "terrain_cost",
    "visibility_confidence",
    "stuck_risk_prior",
]


def strategy_from_filename(path):
    basename = os.path.basename(path).replace(".csv", "")
    if "__" in basename:
        return basename.split("__", 1)[0]
    parts = basename.rsplit("_", 2)
    return parts[0] if len(parts) == 3 else basename


def as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def parse_list(values):
    parsed = []
    for value in values or []:
        for item in value.split(","):
            item = item.strip()
            if item:
                parsed.append(item)
    return parsed


def timestamp_from_filename(path):
    basename = os.path.basename(path).replace(".csv", "")
    if "__" not in basename:
        return ""
    return basename.split("__", 1)[1]


def read_run(path):
    values = {column: [] for column in SHARED_INPUT_COLUMNS}
    times = []
    scene = ""
    with open(path) as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            return {
                "path": path,
                "values": values,
                "times": times,
                "scene": scene,
                "missing": [],
            }
        missing = [column for column in SHARED_INPUT_COLUMNS if column not in reader.fieldnames]
        if missing:
            return {
                "path": path,
                "values": values,
                "times": times,
                "scene": scene,
                "missing": missing,
            }
        for row in reader:
            if not scene:
                scene = row.get("scene", "")
            times.append(as_float(row.get("time", 0.0)))
            for column in SHARED_INPUT_COLUMNS:
                value = as_float(row.get(column))
                values[column].append(value)
    return {
        "path": path,
        "values": values,
        "times": times,
        "scene": scene,
        "missing": [],
    }


def values_in_window(run, min_duration, max_duration):
    values = {column: [] for column in SHARED_INPUT_COLUMNS}
    for index, time_value in enumerate(run["times"]):
        if math.isnan(time_value) or time_value < min_duration:
            continue
        if max_duration is not None and time_value > max_duration:
            continue
        for column in SHARED_INPUT_COLUMNS:
            column_values = run["values"][column]
            if index < len(column_values) and not math.isnan(column_values[index]):
                values[column].append(column_values[index])
    return values


def mean_std(values):
    if not values:
        return math.nan, math.nan
    mean = statistics.mean(values)
    std = statistics.stdev(values) if len(values) > 1 else 0.0
    return mean, std


def check_shared_inputs(
    runs_dir,
    tolerance,
    min_duration,
    max_duration,
    common_window,
    scene_filter,
    strategy_filter,
    since_filter,
):
    files = sorted(glob.glob(os.path.join(runs_dir, "*.csv")))
    if not files:
        print(f"No CSV files found in {runs_dir}")
        return 2

    strategies = set(strategy_filter)
    grouped = {}
    skipped_missing = []
    skipped_filter = 0
    runs = []
    for path in files:
        strategy = strategy_from_filename(path)
        timestamp = timestamp_from_filename(path)
        if strategies and strategy not in strategies:
            skipped_filter += 1
            continue
        if since_filter and timestamp and timestamp < since_filter:
            skipped_filter += 1
            continue

        run = read_run(path)
        if run["missing"]:
            skipped_missing.append((path, run["missing"]))
            continue
        if scene_filter and run["scene"] and run["scene"] != scene_filter:
            skipped_filter += 1
            continue
        run["strategy"] = strategy
        runs.append(run)

    if common_window and runs:
        terminal_times = [
            max(run["times"])
            for run in runs
            if run["times"] and not math.isnan(max(run["times"]))
        ]
        common_end = min(terminal_times) if terminal_times else None
        if max_duration is not None and common_end is not None:
            common_end = min(common_end, max_duration)
    else:
        common_end = max_duration

    for run in runs:
        values = values_in_window(run, min_duration, common_end)
        strategy = run["strategy"]
        bucket = grouped.setdefault(strategy, {column: [] for column in SHARED_INPUT_COLUMNS})
        for column in SHARED_INPUT_COLUMNS:
            bucket[column].extend(values[column])

    if skipped_filter:
        print(f"[info] skipped {skipped_filter} file(s) by scene/strategy/timestamp filters")
    if skipped_missing:
        print(f"[warn] skipped {len(skipped_missing)} file(s) missing shared input columns")
        for path, missing in skipped_missing[:5]:
            print(f"       {os.path.basename(path)}: {', '.join(missing)}")
        if len(skipped_missing) > 5:
            print("       ...")

    if not grouped:
        print("No runs with all shared input columns were found.")
        return 2

    if common_window and common_end is not None:
        print(
            f"[info] comparing common time window: "
            f"{min_duration:.3f}s <= time <= {common_end:.3f}s"
        )
    else:
        print(f"[info] comparing time >= {min_duration:.3f}s")

    print("\nShared input distribution by strategy")
    print("=" * 86)
    header = f"{'strategy':<22} {'column':<32} {'n':>7} {'mean':>10} {'std':>10}"
    print(header)
    print("-" * len(header))
    summary = {}
    for strategy in sorted(grouped):
        summary[strategy] = {}
        for column in SHARED_INPUT_COLUMNS:
            values = grouped[strategy][column]
            mean, std = mean_std(values)
            summary[strategy][column] = mean
            print(f"{strategy:<22} {column:<32} {len(values):>7} {mean:>10.4f} {std:>10.4f}")

    if len(summary) < 2:
        print("\nOnly one strategy has shared-input data; fairness comparison needs at least two strategies.")
        return 0

    failed = False
    print("\nCross-strategy mean spread")
    print("=" * 86)
    for column in SHARED_INPUT_COLUMNS:
        means = [summary[strategy][column] for strategy in summary if not math.isnan(summary[strategy][column])]
        spread = max(means) - min(means) if means else math.nan
        status = "OK" if spread <= tolerance else "WARN"
        if spread > tolerance:
            failed = True
        print(f"{column:<32} spread={spread:.4f} tolerance={tolerance:.4f} {status}")

    return 1 if failed else 0


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs_dir", nargs="?", default=os.path.join(os.path.dirname(os.path.dirname(__file__)), "results", "runs"))
    parser.add_argument("--tolerance", type=float, default=0.05)
    parser.add_argument("--min-duration", type=float, default=0.0)
    parser.add_argument("--max-duration", type=float, default=None)
    parser.add_argument(
        "--no-common-window",
        action="store_true",
        help="Compare each run over its full selected duration instead of the shared shortest time window.",
    )
    parser.add_argument("--scene", default="")
    parser.add_argument("--strategies", nargs="*", default=[])
    parser.add_argument("--since", default="", help="Only include run files with timestamp >= YYYYMMDD_HHMMSS.")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    sys.exit(
        check_shared_inputs(
            os.path.abspath(args.runs_dir),
            args.tolerance,
            args.min_duration,
            args.max_duration,
            not args.no_common_window,
            args.scene,
            parse_list(args.strategies),
            args.since,
        )
    )
