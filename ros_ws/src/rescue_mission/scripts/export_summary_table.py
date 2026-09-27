#!/usr/bin/env python3
"""Export a summary CSV as a compact Markdown table for reports/papers."""
import argparse
import csv
import os


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("summary_csv")
    parser.add_argument("--output", default="")
    parser.add_argument("--title", default="")
    return parser.parse_args()


def fmt_pair(row, prefix, unit=""):
    mean = row.get(f"{prefix}_mean", "")
    std = row.get(f"{prefix}_std", "")
    suffix = f" {unit}" if unit else ""
    return f"{mean}±{std}{suffix}"


def export_markdown(summary_csv):
    with open(summary_csv) as f:
        rows = list(csv.DictReader(f))

    lines = [
        "| Scene | Strategy | N | Success | Time | Energy | Ground E | Air E | Takeoff/Land/Switch E | Switches | Stuck | Slip | Override | Irrev | Final dist |",
        "|-------|----------|---|---------|------|--------|----------|-------|-----------------------|----------|-------|------|----------|-------|------------|",
    ]
    for row in rows:
        lines.append(
            "| {scene} | {strategy} | {n_runs} | {success_rate} | {time} | {energy} | "
            "{ground} | {air} | {transitions} | {switches} | {stuck} | {slip} | {override} | {irrev} | {dist} |".format(
                scene=row["scene"],
                strategy=row["strategy"],
                n_runs=row["n_runs"],
                success_rate=row["success_rate"],
                time=fmt_pair(row, "total_time_s", "s"),
                energy=fmt_pair(row, "total_energy_j", "J"),
                ground=fmt_pair(row, "ground_energy_j", "J"),
                air=fmt_pair(row, "air_energy_j", "J"),
                transitions=(
                    f"{row.get('takeoff_energy_j_mean', '0.000')}/"
                    f"{row.get('land_energy_j_mean', '0.000')}/"
                    f"{row.get('switch_energy_j_mean', '0.000')} J"
                ),
                switches=fmt_pair(row, "switch_count"),
                stuck=fmt_pair(row, "stuck_event_count"),
                slip=fmt_pair(row, "slip_ratio"),
                override=fmt_pair(row, "shield_override_count"),
                irrev=fmt_pair(row, "irreversible_failure"),
                dist=fmt_pair(row, "final_distance_m", "m"),
            )
        )
    return "\n".join(lines) + "\n"


def main():
    args = parse_args()
    table = export_markdown(args.summary_csv)
    if args.title:
        table = f"## {args.title}\n\n{table}"
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, "w") as f:
            f.write(table)
    print(table)


if __name__ == "__main__":
    main()
