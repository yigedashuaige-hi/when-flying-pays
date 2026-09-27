#!/usr/bin/env python3
"""Fit a small descriptor model and evaluate held-out RotorS maneuvers."""

import argparse
import csv
import json
import math
import os

import numpy as np


NUMERIC_FIELDS = [
    "rotors_mechanical_power_w",
    "rotors_trial_mechanical_energy_j",
    "rotors_trial_flight_duration_s",
    "rotors_trial_distance_m",
    "rotors_takeoff_mechanical_energy_j",
    "rotors_hover_mechanical_energy_j",
    "rotors_translation_mechanical_energy_j",
    "rotors_return_mechanical_energy_j",
    "rotors_landing_mechanical_energy_j",
    "rotors_takeoff_duration_s",
    "rotors_hover_duration_s",
    "rotors_translation_duration_s",
    "rotors_return_duration_s",
    "rotors_landing_duration_s",
    "rotors_altitude_m",
    "rotors_velocity_x_m_s",
    "rotors_velocity_y_m_s",
    "rotors_velocity_z_m_s",
]


def load_rows(path):
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("calibration CSV is empty")
    for row_number, row in enumerate(rows, start=2):
        if row["air_backend"] != "rotors":
            raise ValueError("row %d has backend %r" %
                             (row_number, row["air_backend"]))
        if row["energy_model"] != "rotors_aero_shaft_mechanical_proxy_v1":
            raise ValueError("row %d has wrong energy model" % row_number)
        for field in NUMERIC_FIELDS:
            if not math.isfinite(float(row[field])):
                raise ValueError("row %d field %s is non-finite" %
                                 (row_number, field))
    return rows


def summarize_trials(rows, plan):
    grouped = {}
    for row in rows:
        trial_id = row["calibration_trial_id"]
        if trial_id:
            grouped.setdefault(trial_id, []).append(row)
    output = []
    for spec in plan:
        trial_rows = grouped.get(spec["id"], [])
        if not trial_rows:
            raise ValueError("no CSV rows for %s" % spec["id"])
        final = max(trial_rows,
                    key=lambda row: float(row["rotors_trial_mechanical_energy_j"]))
        legacy_values = [float(row["energy_j"]) for row in trial_rows]
        item = dict(spec)
        item.update({
            "mechanical_energy_j": float(
                final["rotors_trial_mechanical_energy_j"]),
            "flight_duration_s": float(final["rotors_trial_flight_duration_s"]),
            "measured_distance_m": float(final["rotors_trial_distance_m"]),
            "takeoff_energy_j": float(
                final["rotors_takeoff_mechanical_energy_j"]),
            "hover_energy_j": float(final["rotors_hover_mechanical_energy_j"]),
            "translation_energy_j": float(
                final["rotors_translation_mechanical_energy_j"]),
            "return_energy_j": float(final["rotors_return_mechanical_energy_j"]),
            "landing_energy_j": float(
                final["rotors_landing_mechanical_energy_j"]),
            "hover_duration_measured_s": float(final["rotors_hover_duration_s"]),
            "translation_return_duration_s": (
                float(final["rotors_translation_duration_s"])
                + float(final["rotors_return_duration_s"])
            ),
            "legacy_proxy_delta_j": max(legacy_values) - min(legacy_values),
            "all_energy_rows_valid": all(
                int(row["rotors_energy_valid"]) == 1 for row in trial_rows
                if row["mode"] in ("TAKEOFF", "AIR", "LAND")
            ),
            "row_count": len(trial_rows),
        })
        output.append(item)
    return output


def features(item):
    # Compact supervisor-facing descriptor model. The intercept absorbs the
    # repeatable takeoff/landing cost; remaining terms represent hover time,
    # translation/return time, and measured 3-D distance.
    return [1.0, item["hover_duration_measured_s"],
            item["translation_return_duration_s"],
            item["measured_distance_m"]]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--plan", required=True,
                        help="Calibration runner JSON containing the fixed plan")
    parser.add_argument("--output", required=True)
    parser.add_argument("--summary-csv", default="")
    args = parser.parse_args()
    with open(args.plan) as stream:
        plan_doc = json.load(stream)
    rows = load_rows(args.csv)
    samples = summarize_trials(rows, plan_doc["plan"])
    calibration = [item for item in samples if item["split"] == "calibration"]
    heldout = [item for item in samples if item["split"] == "heldout"]
    x_train = np.asarray([features(item) for item in calibration], dtype=float)
    y_train = np.asarray([item["mechanical_energy_j"] for item in calibration])
    coefficients, _, rank, singular_values = np.linalg.lstsq(
        x_train, y_train, rcond=None)

    errors = []
    for item in samples:
        prediction = float(np.dot(features(item), coefficients))
        item["predicted_mechanical_energy_j"] = prediction
        item["absolute_error_j"] = abs(prediction - item["mechanical_energy_j"])
        item["absolute_percentage_error_pct"] = (
            100.0 * item["absolute_error_j"] / item["mechanical_energy_j"]
        )
        if item["split"] == "heldout":
            errors.append(prediction - item["mechanical_energy_j"])
    actual_heldout = [item["mechanical_energy_j"] for item in heldout]
    legacy_values = [item["legacy_proxy_delta_j"] for item in samples]
    mechanical_values = [item["mechanical_energy_j"] for item in samples]
    ratios = [mechanical / legacy for mechanical, legacy in
              zip(mechanical_values, legacy_values)]
    result = {
        "date": "2026-07-18",
        "backend": "rotors",
        "observed_energy_model": "rotors_aero_shaft_mechanical_proxy_v1",
        "predictor": {
            "formula": "E_hat = beta0 + beta_hover*t_hover + beta_translation*t_translation_return + beta_distance*d_measured",
            "units": {"E_hat": "J mechanical proxy", "beta0": "J",
                      "beta_hover": "W", "beta_translation": "W",
                      "beta_distance": "J/m"},
            "coefficients": {
                "beta0_j": coefficients[0],
                "beta_hover_w": coefficients[1],
                "beta_translation_w": coefficients[2],
                "beta_distance_j_per_m": coefficients[3],
            },
            "fit_rank": int(rank),
            "singular_values": singular_values.tolist(),
        },
        "sample_counts": {"calibration": len(calibration),
                          "heldout": len(heldout)},
        "heldout_metrics": {
            "mae_j": float(np.mean(np.abs(errors))),
            "rmse_j": float(math.sqrt(np.mean(np.square(errors)))),
            "mape_pct": float(np.mean([
                100.0 * abs(error) / actual
                for error, actual in zip(errors, actual_heldout)
            ])),
            "max_absolute_error_j": float(np.max(np.abs(errors))),
        },
        "samples": samples,
        "csv_rows": len(rows),
        "all_rows_finite": True,
        "all_flight_rows_energy_valid": all(
            item["all_energy_rows_valid"] for item in samples),
        "legacy_vs_rotors_separate_comparison": {
            "legacy_model": "legacy_task_distance_hover_proxy_v1",
            "legacy_value_label": "task energy proxy J",
            "rotors_model": "rotors_aero_shaft_mechanical_proxy_v1",
            "rotors_value_label": "modeled shaft mechanical energy J",
            "legacy_proxy_range_j": [min(legacy_values), max(legacy_values)],
            "mechanical_proxy_range_j": [min(mechanical_values),
                                         max(mechanical_values)],
            "mechanical_to_legacy_ratio_range": [min(ratios), max(ratios)],
            "mechanical_to_legacy_ratio_median": float(np.median(ratios)),
            "pearson_correlation": float(np.corrcoef(
                legacy_values, mechanical_values)[0, 1]),
        },
        "comparison_warning": (
            "legacy_proxy_delta_j and mechanical_energy_j are different models; "
            "they are reported side-by-side and must not be pooled."
        ),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    if args.summary_csv:
        os.makedirs(os.path.dirname(os.path.abspath(args.summary_csv)),
                    exist_ok=True)
        fields = [
            "id", "split", "hover_s", "distance_m", "target_speed_m_s",
            "flight_duration_s", "measured_distance_m",
            "takeoff_energy_j", "hover_energy_j", "translation_energy_j",
            "return_energy_j", "landing_energy_j", "mechanical_energy_j",
            "predicted_mechanical_energy_j", "absolute_error_j",
            "absolute_percentage_error_pct", "legacy_proxy_delta_j",
            "all_energy_rows_valid",
        ]
        with open(args.summary_csv, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for item in samples:
                writer.writerow({field: item[field] for field in fields})
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
