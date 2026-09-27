#!/usr/bin/env python3
"""Validate whether a rescue_mission CSV run is usable for analysis."""
import argparse
import csv
import os
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_SCRIPT_DIR)

REQUIRED_COLUMNS = [
    "scene",
    "time",
    "mode",
    "mission_state",
    "energy_j",
    "ground_energy_j",
    "air_energy_j",
    "path_length",
    "switch_count",
    "collision_count",
    "final_distance",
    "success",
    "timeout",
]

TOP_TIER_COLUMNS = [
    "takeoff_energy_j",
    "land_energy_j",
    "switch_energy_j",
    "ground_distance_m",
    "air_distance_m",
    "stuck_count",
    "stuck_event_count",
    "slip_ratio",
]


def as_float(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def load_rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def validate(path):
    rows = load_rows(path)
    if not rows:
        print(f"INVALID: {path} is empty")
        return 2

    columns = set(rows[0].keys())
    missing = [key for key in REQUIRED_COLUMNS if key not in columns]
    if missing:
        print(f"INVALID: missing required columns: {', '.join(missing)}")
        return 2

    missing_top_tier = [key for key in TOP_TIER_COLUMNS if key not in columns]
    mode_samples = sum(1 for row in rows if row.get("mode", "").strip())
    if mode_samples == 0:
        print("INVALID: no /rescue/mode samples were recorded. mode_switcher likely crashed.")
        return 2

    terminal = next((row for row in rows if row.get("success") == "1"), rows[-1])
    success = int(as_float(terminal, "success"))
    timeout = int(as_float(terminal, "timeout"))
    elapsed = as_float(terminal, "time")
    energy = as_float(terminal, "energy_j")
    ground_energy = as_float(terminal, "ground_energy_j")
    air_energy = as_float(terminal, "air_energy_j")
    final_distance = as_float(terminal, "final_distance")
    switches = as_float(terminal, "switch_count")
    collisions = as_float(terminal, "collision_count")
    stuck_events = as_float(
        terminal,
        "stuck_event_count",
        as_float(terminal, "stuck_count", collisions),
    )
    slip_ratio = as_float(terminal, "slip_ratio")

    print(f"CSV: {path}")
    print(f"Rows: {len(rows)}")
    print(f"Mode samples: {mode_samples}")
    print(f"Success: {success}")
    print(f"Timeout: {timeout}")
    print(f"Time: {elapsed:.3f} s")
    print(f"Energy: {energy:.3f} J (ground={ground_energy:.3f}, air={air_energy:.3f})")
    print(f"Switches: {switches:.0f}")
    print(f"Collisions/stuck: {collisions:.0f} (stuck_events={stuck_events:.0f}, slip={slip_ratio:.3f})")
    print(f"Final distance: {final_distance:.3f} m")

    if missing_top_tier:
        print(f"WARNING: missing top-tier columns: {', '.join(missing_top_tier)}")
    else:
        print(
            "Transition energy: "
            f"takeoff={as_float(terminal, 'takeoff_energy_j'):.3f}, "
            f"land={as_float(terminal, 'land_energy_j'):.3f}, "
            f"switch={as_float(terminal, 'switch_energy_j'):.3f}"
        )
        print(
            "Mode distance: "
            f"ground={as_float(terminal, 'ground_distance_m'):.3f}, "
            f"air={as_float(terminal, 'air_distance_m'):.3f}"
        )

    if success:
        print("VALID: completed successfully.")
        return 0
    if timeout:
        print("VALID FAILURE: infrastructure was running, but the mission timed out.")
        return 1
    print("VALID FAILURE: infrastructure was running, but the mission did not complete.")
    return 1


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "csv_path",
        nargs="?",
        default=os.path.join(_PKG_DIR, "results", "rescue_metrics.csv"),
    )
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(validate(os.path.abspath(parse_args().csv_path)))
