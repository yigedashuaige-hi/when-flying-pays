#!/usr/bin/env python3
"""Offline sampled-state audit of the Stage-0 CVaR decision boundary."""

import csv
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1] / "results"
OUT = (Path(__file__).resolve().parents[2] / "rescue_worlds" / "results" /
       "phase_e1a_contact_20260731" / "readonly_audit_cvar")

ALPHA = 0.8
TAIL = 1.0 - ALPHA
K_UNC = 1.0
C_STUCK = 60.0
GROUND_SPEED = 0.45
AIR_SPEED = 0.70
GROUND_ENERGY_PER_M = 0.35
AIR_ENERGY_PER_M = 1.2
TAKEOFF_ENERGY = 11.0
LAND_ENERGY = 12.0
SWITCH_PENALTY = 2.0
WCVAR = 0.75
WD = 0.10
LANDING_RISK_WEIGHT = 10.0


def cvar(fail_probability, c_nom, c_stuck=C_STUCK):
    fail_probability = min(1.0, max(0.0, fail_probability))
    if fail_probability >= TAIL:
        return c_stuck
    return (fail_probability * c_stuck +
            (TAIL - fail_probability) * c_nom) / TAIL


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    output = []
    for scene, selected_tau in (("scene_b", 0.2), ("scene_c", 0.3)):
        directory = ROOT / "runs_claim_a_airtight" / f"{scene}_recoverable_risk_aware"
        files = sorted(directory.glob("*.csv"))
        if len(files) != 20:
            raise RuntimeError(f"expected 20 risk-aware logs in {directory}")
        for path in files:
            with path.open(newline="") as handle:
                rows = list(csv.DictReader(handle))
            for index, row in enumerate(rows):
                reason = row["switch_reason"]
                actual = None
                if reason.endswith("air_cost_lower"):
                    actual = True
                elif reason.endswith("ground_cost_lower"):
                    actual = False
                if actual is None:
                    continue
                distance = math.hypot(float(row["goal_x"]) - float(row["x"]),
                                      float(row["goal_y"]) - float(row["y"]))
                p_fail_raw = 1.0 - float(row["traversability_mean"])
                uncertainty = float(row["traversability_uncertainty"])
                p_fail_eff = min(1.0, max(0.0, p_fail_raw + K_UNC * uncertainty))
                c_nom = distance / GROUND_SPEED
                risk = cvar(p_fail_eff, c_nom)
                ground_base = (GROUND_ENERGY_PER_M * distance +
                               WD * float(row["terrain_cost"]) * distance +
                               SWITCH_PENALTY)
                air = (TAKEOFF_ENERGY + AIR_ENERGY_PER_M * distance / AIR_SPEED +
                       LAND_ENERGY + SWITCH_PENALTY +
                       (1.0 - float(row["visibility_confidence"])) *
                       LANDING_RISK_WEIGHT)
                ground = ground_base + WCVAR * risk
                theoretical = air < ground

                required_cvar = (air - ground_base) / WCVAR
                if required_cvar <= c_nom:
                    fail_critical = 0.0
                    boundary_case = "always_air_at_nonnegative_failure"
                elif required_cvar >= C_STUCK or c_nom >= C_STUCK:
                    fail_critical = math.inf
                    boundary_case = "never_air_from_cvar_alone"
                else:
                    fail_critical = (TAIL * (required_cvar - c_nom) /
                                     (C_STUCK - c_nom))
                    boundary_case = "interior"
                raw_fail_critical = fail_critical - K_UNC * uncertainty
                fixed_threshold = p_fail_raw > selected_tau
                logged_cost_intent = float(row["J_air"]) < float(row["J_ground"])
                output.append({
                    "scene": scene, "seed": int(row["seed"]),
                    "source_csv": str(path.relative_to(ROOT)), "row_index": index,
                    "time_s": float(row["time"]), "mode": row["mode"],
                    "mission_state": row["mission_state"], "reason": reason,
                    "distance_m": distance, "p_fail_raw": p_fail_raw,
                    "uncertainty": uncertainty, "p_fail_eff": p_fail_eff,
                    "c_nom_s": c_nom, "cvar_s": risk,
                    "local_fail_eff_critical": fail_critical,
                    "local_raw_fail_critical": raw_fail_critical,
                    "boundary_case": boundary_case,
                    "actual_cost_intent_air": int(actual),
                    "recomputed_cvar_intent_air": int(theoretical),
                    "logged_j_intent_air": int(logged_cost_intent),
                    "selected_fixed_tau": selected_tau,
                    "selected_fixed_tau_intent_air": int(fixed_threshold),
                    "recomputed_matches_reason": int(theoretical == actual),
                    "logged_j_matches_reason": int(logged_cost_intent == actual),
                    "fixed_tau_matches_reason": int(fixed_threshold == actual),
                })
    write_csv(OUT / "sampled_decision_boundary_comparison.csv", output)
    summary = []
    for scene in ("scene_b", "scene_c"):
        items = [x for x in output if x["scene"] == scene]
        finite_thresholds = [x["local_raw_fail_critical"] for x in items
                             if math.isfinite(x["local_raw_fail_critical"])]
        summary.append({
            "scene": scene, "sampled_cost_decision_rows": len(items),
            "recomputed_matches_reason": sum(x["recomputed_matches_reason"] for x in items),
            "recomputed_agreement": sum(x["recomputed_matches_reason"] for x in items) / len(items),
            "logged_j_matches_reason": sum(x["logged_j_matches_reason"] for x in items),
            "logged_j_agreement": sum(x["logged_j_matches_reason"] for x in items) / len(items),
            "selected_fixed_tau_matches_reason": sum(x["fixed_tau_matches_reason"] for x in items),
            "selected_fixed_tau_agreement": sum(x["fixed_tau_matches_reason"] for x in items) / len(items),
            "local_raw_threshold_min": min(finite_thresholds),
            "local_raw_threshold_median": sorted(finite_thresholds)[len(finite_thresholds)//2],
            "local_raw_threshold_max": max(finite_thresholds),
            "interior_rows": sum(x["boundary_case"] == "interior" for x in items),
            "always_air_rows": sum(x["boundary_case"].startswith("always") for x in items),
            "never_air_rows": sum(x["boundary_case"].startswith("never") for x in items),
        })
    write_csv(OUT / "sampled_decision_boundary_summary.csv", summary)


if __name__ == "__main__":
    main()
