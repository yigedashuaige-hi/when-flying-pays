#!/usr/bin/env python3
"""Summarize Phase E1a per-physics-step contact diagnostics.

The input remains the canonical evidence.  This script emits a compact CSV and
Markdown table without resampling or silently dropping non-finite values.
"""

import argparse
import csv
import math
import statistics
from pathlib import Path


NUMERIC = {
    "sim_time_s", "pre_z_m", "pre_roll_rad", "pre_pitch_rad",
    "pre_vx_m_s", "pre_vy_m_s", "pre_vz_m_s", "pre_kinetic_j",
    "post_vx_m_s", "post_vy_m_s", "post_vz_m_s", "post_kinetic_j",
    "end_z_m", "end_roll_rad", "end_pitch_rad", "end_vx_m_s",
    "end_vy_m_s", "end_vz_m_s", "end_kinetic_j",
    "plugin_delta_mechanical_j", "physics_delta_mechanical_j",
    "cumulative_plugin_work_proxy_j",
    "cumulative_positive_plugin_work_proxy_j", "max_contact_depth_m",
    "max_contact_force_n", "planar_target_world_x_m_s",
    "planar_target_world_y_m_s",
}


def finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    return number if math.isfinite(number) else math.nan


def maximum(rows, key, absolute=False):
    values = [finite(row[key]) for row in rows]
    values = [abs(value) if absolute else value for value in values
              if math.isfinite(value)]
    return max(values) if values else math.nan


def summarize(path):
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    nonfinite = 0
    for row in rows:
        for key in NUMERIC:
            if key in row and not math.isfinite(finite(row[key])):
                nonfinite += 1

    barrier = [row for row in rows if "low_barrier" in row["contact_pairs"]]
    first = barrier[0] if barrier else None
    contact_pairs = sorted({
        pair for row in barrier for pair in row["contact_pairs"].split("|")
        if "low_barrier" in pair
    })
    target_errors = []
    commanded_speeds = []
    actual_speeds = []
    for row in rows:
        tx = finite(row["planar_target_world_x_m_s"])
        ty = finite(row["planar_target_world_y_m_s"])
        px = finite(row["post_vx_m_s"])
        py = finite(row["post_vy_m_s"])
        if all(math.isfinite(value) for value in (tx, ty, px, py)):
            target_errors.append(math.hypot(px - tx, py - ty))
            cmd_speed = math.hypot(finite(row["cmd_body_x_m_s"]),
                                   finite(row["cmd_body_y_m_s"]))
            if cmd_speed > 0.1 and (not first or
                                    finite(row["sim_time_s"]) <
                                    finite(first["sim_time_s"])):
                commanded_speeds.append(cmd_speed)
                actual_speeds.append(math.hypot(finite(row["end_vx_m_s"]),
                                                finite(row["end_vy_m_s"])))

    drive_path = Path(str(path) + ".drive.csv")
    drive_rows = []
    if drive_path.exists():
        with drive_path.open(newline="") as handle:
            drive_rows = list(csv.DictReader(handle))
    active_drive = [row for row in drive_rows if row["active"] == "1"]

    def drive_norm(row, keys):
        return math.sqrt(sum(finite(row[key]) ** 2 for key in keys))

    result = {
        "run": path.stem,
        "steps": len(rows),
        "nonfinite_numeric_cells": nonfinite,
        "barrier_contact_steps": len(barrier),
        "first_barrier_time_s": finite(first["sim_time_s"]) if first else math.nan,
        "first_barrier_pairs": first["contact_pairs"] if first else "",
        "barrier_collision_parts": ";".join(contact_pairs),
        "max_abs_roll_rad": maximum(rows, "end_roll_rad", True),
        "max_abs_pitch_rad": maximum(rows, "end_pitch_rad", True),
        "min_z_m": min(finite(row["end_z_m"]) for row in rows),
        "max_z_m": maximum(rows, "end_z_m"),
        "max_speed_m_s": max(math.sqrt(
            finite(row["end_vx_m_s"]) ** 2 +
            finite(row["end_vy_m_s"]) ** 2 +
            finite(row["end_vz_m_s"]) ** 2) for row in rows),
        "max_kinetic_j": maximum(rows, "end_kinetic_j"),
        "max_penetration_m": maximum(barrier, "max_contact_depth_m") if barrier else 0.0,
        # The diagnostic reports the maximum across all simultaneous contacts;
        # ground and barrier contacts can coexist, so do not mislabel this as a
        # barrier-only force.
        "max_contact_force_during_barrier_n": (
            maximum(barrier, "max_contact_force_n") if barrier else 0.0),
        "max_plugin_step_injection_j": maximum(rows, "plugin_delta_mechanical_j"),
        "final_plugin_work_proxy_j": finite(rows[-1]["cumulative_plugin_work_proxy_j"]),
        "final_positive_plugin_work_proxy_j": finite(
            rows[-1]["cumulative_positive_plugin_work_proxy_j"]),
        "max_post_planar_target_error_m_s": max(target_errors),
        "pre_contact_median_command_speed_m_s": (
            statistics.median(commanded_speeds) if commanded_speeds else math.nan),
        "pre_contact_median_actual_speed_m_s": (
            statistics.median(actual_speeds) if actual_speeds else math.nan),
        "max_applied_planar_force_n": (
            max(drive_norm(row, ("force_x_n", "force_y_n"))
                for row in active_drive) if active_drive else math.nan),
        "max_applied_vertical_force_n": (
            max(abs(finite(row["force_z_n"])) for row in active_drive)
            if active_drive else math.nan),
        "max_applied_torque_nm": (
            max(drive_norm(row, ("torque_x_nm", "torque_y_nm", "torque_z_nm"))
                for row in active_drive) if active_drive else math.nan),
        "drive_work_proxy_j": (
            finite(drive_rows[-1]["cumulative_work_proxy_j"])
            if drive_rows else math.nan),
        "drive_positive_work_proxy_j": (
            finite(drive_rows[-1]["cumulative_positive_work_proxy_j"])
            if drive_rows else math.nan),
    }
    if first:
        for prefix in ("pre", "post", "end"):
            for key in ("vx_m_s", "vy_m_s", "vz_m_s", "kinetic_j"):
                result[f"first_{prefix}_{key}"] = finite(first[f"{prefix}_{key}"])
    return result


def fmt(value):
    if isinstance(value, float):
        return "NA" if not math.isfinite(value) else f"{value:.9g}"
    return str(value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    args = parser.parse_args()
    summaries = [summarize(path) for path in args.inputs]
    fields = []
    for summary in summaries:
        for field in summary:
            if field not in fields:
                fields.append(field)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)

    selected = [
        "run", "barrier_contact_steps", "first_barrier_time_s",
        "first_barrier_pairs", "max_abs_pitch_rad", "max_z_m",
        "max_speed_m_s", "max_penetration_m",
        "max_contact_force_during_barrier_n", "max_plugin_step_injection_j",
        "drive_positive_work_proxy_j", "max_applied_planar_force_n",
        "max_applied_vertical_force_n", "max_applied_torque_nm",
        "pre_contact_median_actual_speed_m_s", "nonfinite_numeric_cells",
    ]
    lines = ["| " + " | ".join(selected) + " |",
             "| " + " | ".join("---" for _ in selected) + " |"]
    for item in summaries:
        lines.append("| " + " | ".join(fmt(item[key]).replace("|", "/")
                                            for key in selected) + " |")
    args.markdown.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
