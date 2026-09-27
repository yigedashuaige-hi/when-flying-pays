#!/usr/bin/env python3
"""Inspect mode transitions from a run CSV and highlight rule trigger signals."""
import argparse
import csv


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--slope-air-deg", type=float, default=30.0)
    parser.add_argument("--obstacle-air-density", type=float, default=0.65)
    parser.add_argument("--slope-fly-min-distance", type=float, default=3.0)
    parser.add_argument("--return-ground-finish-distance", type=float, default=3.5)
    return parser.parse_args()


def trigger_label(row, args):
    distance = float(row["final_distance"])
    slope = float(row["slope_deg"])
    obstacle = float(row["obstacle_density"])
    state = row["mission_state"]

    labels = []
    if obstacle > args.obstacle_air_density:
        labels.append("obstacle")
    if slope > args.slope_air_deg:
        slope_active = (
            distance > args.slope_fly_min_distance
            and not (state == "RETURN" and distance < args.return_ground_finish_distance)
        )
        labels.append("slope_active" if slope_active else "slope_ignored")
    return ",".join(labels) if labels else "-"


def main():
    args = parse_args()
    last_mode = None
    transitions = 0
    with open(args.csv_path) as f:
        for row in csv.DictReader(f):
            mode = row["mode"]
            if mode == last_mode:
                continue
            transitions += 1 if last_mode is not None else 0
            print(
                f"t={float(row['time']):6.1f} "
                f"mode={mode:7s} state={row['mission_state']:8s} "
                f"x={float(row['x']):5.2f} y={float(row['y']):5.2f} "
                f"dist={float(row['final_distance']):5.2f} "
                f"obs={float(row['obstacle_density']):4.2f} "
                f"slope={float(row['slope_deg']):4.1f} "
                f"trigger={trigger_label(row, args)}"
            )
            last_mode = mode
            if row.get("success") == "1":
                break
    print(f"transitions={transitions}")


if __name__ == "__main__":
    main()
