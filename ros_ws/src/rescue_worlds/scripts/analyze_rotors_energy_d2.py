#!/usr/bin/env python3
"""Phase-D2 staged RotorS energy model and conservative one-sided budget."""

import argparse
import json
import math
import os

import numpy as np

from analyze_rotors_energy_calibration import load_rows, summarize_trials, features


MODEL = "rotors_aero_shaft_mechanical_proxy_v1"


def metric_block(samples, prediction_key):
    residuals = np.asarray([
        item["mechanical_energy_j"] - item[prediction_key]
        for item in samples
    ], dtype=float)
    actual = np.asarray([item["mechanical_energy_j"] for item in samples])
    return {
        "mae_j": float(np.mean(np.abs(residuals))),
        "rmse_j": float(math.sqrt(np.mean(residuals ** 2))),
        "mape_pct": float(np.mean(100.0 * np.abs(residuals) / actual)),
        "max_underprediction_j": float(max(0.0, np.max(residuals))),
        "max_overprediction_j": float(max(0.0, np.max(-residuals))),
    }


def percentile_summary(values, unit):
    values = np.asarray(values, dtype=float)
    mean = float(np.mean(values))
    return {
        "unit": unit,
        "bootstrap_p05": float(np.percentile(values, 5.0)),
        "bootstrap_median": float(np.percentile(values, 50.0)),
        "bootstrap_p95": float(np.percentile(values, 95.0)),
        "coefficient_of_variation": (
            float(np.std(values, ddof=1) / abs(mean)) if abs(mean) > 1e-12 else None
        ),
    }


def ceil_to(value, quantum=100.0):
    return float(math.ceil(max(0.0, value) / quantum) * quantum)


def fit_stage(calibration):
    heights = np.asarray([item["height_m"] for item in calibration])
    takeoff = np.asarray([item["takeoff_energy_j"] for item in calibration])
    x_takeoff = np.column_stack([np.ones(len(heights)), heights])
    takeoff_beta = np.linalg.lstsq(x_takeoff, takeoff, rcond=None)[0]

    hover_duration = np.asarray([
        item["hover_duration_measured_s"] for item in calibration
    ])
    hover_energy = np.asarray([item["hover_energy_j"] for item in calibration])
    hover_power = float(np.sum(hover_energy) / np.sum(hover_duration))

    moving = [item for item in calibration if item["distance_m"] > 0.0]
    trans_duration = np.asarray([
        item["translation_return_duration_s"] for item in moving
    ])
    trans_energy = np.asarray([
        item["translation_energy_j"] + item["return_energy_j"]
        for item in moving
    ])
    translation_power = float(np.sum(trans_energy) / np.sum(trans_duration))
    planned_duration = np.asarray([
        2.0 * item["distance_m"] / item["target_speed_m_s"] for item in moving
    ])
    settle_overhead = float(np.median(trans_duration - planned_duration))
    landing_base = float(np.median([
        item["landing_energy_j"] for item in calibration
    ]))
    return {
        "takeoff_intercept_j": float(takeoff_beta[0]),
        "takeoff_height_j_per_m": float(takeoff_beta[1]),
        "hover_power_w": hover_power,
        "translation_power_w": translation_power,
        "translation_settle_overhead_s": settle_overhead,
        "landing_base_j": landing_base,
    }


def stage_prediction(item, params):
    takeoff = (params["takeoff_intercept_j"]
               + params["takeoff_height_j_per_m"] * item["height_m"])
    hover = params["hover_power_w"] * item["hover_s"]
    translation = 0.0
    if item["distance_m"] > 0.0:
        planned = 2.0 * item["distance_m"] / item["target_speed_m_s"]
        translation = params["translation_power_w"] * (
            planned + params["translation_settle_overhead_s"]
        )
    landing = params["landing_base_j"]
    return takeoff, hover, translation, landing


def bootstrap_stability(calibration, count=2000):
    rng = np.random.default_rng(20260719)
    draws = {key: [] for key in fit_stage(calibration)}
    for _ in range(count):
        sample = [calibration[index] for index in
                  rng.integers(0, len(calibration), len(calibration))]
        # Very rarely a resample contains no moving trial or only one altitude.
        if (not any(item["distance_m"] > 0.0 for item in sample)
                or len({item["height_m"] for item in sample}) < 2):
            continue
        fitted = fit_stage(sample)
        for key, value in fitted.items():
            draws[key].append(value)
    units = {
        "takeoff_intercept_j": "J mechanical proxy",
        "takeoff_height_j_per_m": "J mechanical proxy/m",
        "hover_power_w": "W mechanical proxy",
        "translation_power_w": "W mechanical proxy",
        "translation_settle_overhead_s": "s",
        "landing_base_j": "J mechanical proxy",
    }
    return {
        "valid_bootstrap_draws": len(next(iter(draws.values()))),
        "parameters": {
            key: percentile_summary(values, units[key])
            for key, values in draws.items()
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--plan", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    with open(args.plan) as stream:
        plan_doc = json.load(stream)
    rows = load_rows(args.csv)
    samples = summarize_trials(rows, plan_doc["plan"])
    calibration = [item for item in samples if item["split"] == "calibration"]
    heldout = [item for item in samples if item["split"] == "heldout"]

    # D1/current joint model, retained strictly as a comparison.
    x_train = np.asarray([features(item) for item in calibration])
    y_train = np.asarray([item["mechanical_energy_j"] for item in calibration])
    joint_beta = np.linalg.lstsq(x_train, y_train, rcond=None)[0]
    for item in samples:
        item["joint_prediction_j"] = float(np.dot(features(item), joint_beta))

    stage = fit_stage(calibration)
    for item in samples:
        takeoff, hover, translation, landing = stage_prediction(item, stage)
        item["stage_takeoff_prediction_j"] = takeoff
        item["stage_hover_prediction_j"] = hover
        item["stage_translation_prediction_j"] = translation
        item["stage_landing_prediction_j"] = landing
        item["stage_prediction_j"] = takeoff + hover + translation + landing
        actual_nonlanding = (
            item["takeoff_energy_j"] + item["hover_energy_j"]
            + item["translation_energy_j"] + item["return_energy_j"]
        )
        item["stage_nonlanding_underprediction_j"] = actual_nonlanding - (
            takeoff + hover + translation
        )

    moving_cal = [item for item in calibration if item["distance_m"] > 0.0]
    planned_times = np.asarray([
        2.0 * item["distance_m"] / item["target_speed_m_s"]
        for item in moving_cal
    ])
    distances = np.asarray([2.0 * item["distance_m"] for item in moving_cal])
    actual_times = np.asarray([
        item["translation_return_duration_s"] for item in moving_cal
    ])
    planned_corr = float(np.corrcoef(planned_times, distances)[0, 1])
    actual_corr = float(np.corrcoef(actual_times, distances)[0, 1])
    vif = None if abs(planned_corr) >= 0.999999 else float(
        1.0 / (1.0 - planned_corr ** 2)
    )

    heldout_total_under = max(
        0.0, max(item["mechanical_energy_j"] - item["stage_prediction_j"]
                 for item in heldout)
    )
    heldout_nonlanding_under = max(
        0.0, max(item["stage_nonlanding_underprediction_j"] for item in heldout)
    )
    all_landing = [item["landing_energy_j"] for item in samples]
    all_takeoff = [item["takeoff_energy_j"] for item in samples]
    # Validated operating envelope: <=1.5 m altitude, <=5 s hover, <=1.2 m
    # one-way translation, and speed no lower than the tested 0.25 m/s.
    envelope_air = (
        stage["hover_power_w"] * 5.0
        + stage["translation_power_w"]
        * (2.0 * 1.2 / 0.25 + stage["translation_settle_overhead_s"])
    )
    budget = {
        "unit": "J shaft mechanical energy proxy (not battery electrical energy)",
        "E_takeoff_j": ceil_to(max(all_takeoff)),
        "E_air_reserve_j": ceil_to(envelope_air),
        "E_land_j": ceil_to(max(all_landing)),
        # Deliberately use maximum held-out total underprediction, never mean
        # error. Landing also receives its own observed-tail upper reserve.
        "E_margin_j": ceil_to(heldout_total_under),
        "heldout_max_total_underprediction_j": heldout_total_under,
        "heldout_max_nonlanding_underprediction_j": heldout_nonlanding_under,
        "landing_observed_max_j": max(all_landing),
        "takeoff_observed_max_j": max(all_takeoff),
        "rounding_quantum_j": 100.0,
        "validated_envelope": {
            "maximum_height_m": 1.5,
            "maximum_hover_s": 5.0,
            "maximum_one_way_distance_m": 1.2,
            "minimum_translation_speed_m_s": 0.25,
        },
    }
    budget["takeoff_approval_floor_j"] = sum(
        budget[key] for key in
        ("E_takeoff_j", "E_air_reserve_j", "E_land_j", "E_margin_j")
    )

    legacy = np.asarray([item["legacy_proxy_delta_j"] for item in samples])
    mechanical = np.asarray([item["mechanical_energy_j"] for item in samples])
    result = {
        "date": "2026-07-19",
        "phase": "D2",
        "backend": "rotors",
        "energy_model": MODEL,
        "sample_counts": {"calibration": len(calibration), "heldout": len(heldout)},
        "all_rows_finite": True,
        "all_flight_rows_energy_valid": all(
            item["all_energy_rows_valid"] for item in samples
        ),
        "collinearity_audit": {
            "translation_calibration_trials": len(moving_cal),
            "planned_translation_time_vs_roundtrip_distance_pearson": planned_corr,
            "actual_translation_time_vs_roundtrip_distance_pearson": actual_corr,
            "planned_time_distance_vif": vif,
            "vif_interpretation": (
                "infinite/perfect collinearity" if vif is None else "finite"
            ),
            "decision": "do not use simultaneous translation-time and distance coefficients",
        },
        "current_joint_model_comparison": {
            "formula": "D1 joint model: intercept + hover time + measured translation time + measured 3-D distance",
            "coefficients": joint_beta.tolist(),
            "heldout": metric_block(heldout, "joint_prediction_j"),
            "limitation": "uses post-flight measured time/distance and has collinear translation descriptors",
        },
        "selected_stage_model": {
            "formula": (
                "E_hat = (a0+a_h*h) + P_hover*t_hover + "
                "P_translation*(2*d/v+t_settle) + E_land_base"
            ),
            "parameters": stage,
            "heldout": metric_block(heldout, "stage_prediction_j"),
            "parameter_stability": bootstrap_stability(calibration),
        },
        "conservative_supervisor_budget": budget,
        "legacy_vs_mechanical_proxy": {
            "legacy_model": "legacy_task_distance_hover_proxy_v1",
            "mechanical_model": MODEL,
            "legacy_range_j": [float(np.min(legacy)), float(np.max(legacy))],
            "mechanical_range_j": [float(np.min(mechanical)), float(np.max(mechanical))],
            "median_numeric_ratio_not_conversion_factor": float(
                np.median(mechanical / legacy)
            ),
            "warning": "different proxy semantics; never pool or convert using this ratio",
        },
        "samples": samples,
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({
        "sample_counts": result["sample_counts"],
        "collinearity_audit": result["collinearity_audit"],
        "joint_heldout": result["current_joint_model_comparison"]["heldout"],
        "stage_heldout": result["selected_stage_model"]["heldout"],
        "budget": budget,
    }, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
