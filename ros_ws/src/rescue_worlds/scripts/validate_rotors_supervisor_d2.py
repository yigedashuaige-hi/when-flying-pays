#!/usr/bin/env python3
"""Replay explicit guarded/unguarded Phase-D2 supervisor budget cases."""

import argparse
import csv
import json
import os
import sys

import yaml


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MISSION_SCRIPTS = os.path.abspath(os.path.join(
    SCRIPT_DIR, "..", "..", "rescue_mission", "scripts"
))
if MISSION_SCRIPTS not in sys.path:
    sys.path.insert(0, MISSION_SCRIPTS)

from safety_supervisor import SafetySupervisor  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--analysis", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-csv", required=True)
    args = parser.parse_args()
    with open(args.config) as stream:
        config = yaml.safe_load(stream)
    with open(args.analysis) as stream:
        analysis = json.load(stream)
    params = config["mode_switcher"]["rotors_safety_supervisor"]
    budget = analysis["conservative_supervisor_budget"]
    floor = budget["takeoff_approval_floor_j"]
    landing = budget["E_land_j"]
    margin = budget["E_margin_j"]
    heldout = [item for item in analysis["samples"]
               if item["split"] == "heldout"]
    actual_high = max(item["mechanical_energy_j"] for item in heldout)
    actual_mid = sorted(item["mechanical_energy_j"] for item in heldout)[-2]
    landing_actual = budget["landing_observed_max_j"]

    cases = [
        {"case": "sufficient_takeoff", "current_mode": "GROUND",
         "proposed_mode": "TAKEOFF", "available_j": floor + 5100.0,
         "observed_actual_consumption_j": actual_high},
        {"case": "near_boundary_safe", "current_mode": "GROUND",
         "proposed_mode": "TAKEOFF", "available_j": floor + 1.0,
         "observed_actual_consumption_j": actual_mid},
        {"case": "insufficient_full_envelope_and_landing_reserve",
         "current_mode": "GROUND", "proposed_mode": "TAKEOFF",
         "available_j": floor - 100.0,
         "observed_actual_consumption_j": actual_high},
        {"case": "airborne_low_margin", "current_mode": "AIR",
         "proposed_mode": "AIR", "available_j": landing + margin - 100.0,
         "observed_actual_consumption_j": landing_actual},
    ]
    rows = []
    for case in cases:
        for guarded in (True, False):
            supervisor = SafetySupervisor(
                params, enforce=guarded, active=False, energy_backend="rotors"
            )
            approved = supervisor.filter(case["proposed_mode"], {
                "current_mode": case["current_mode"],
                "energy_remaining_j": case["available_j"],
                "landing_area_safety": 1.0,
                "stuck_risk_prior": 0.0,
            })
            if approved == "TAKEOFF":
                approved_requirement = floor
            elif approved == "AIR":
                approved_requirement = landing + margin
            elif approved == "LAND":
                approved_requirement = landing_actual
            else:
                approved_requirement = 0.0
            rows.append({
                **case,
                "guarded": int(guarded),
                "energy_model": supervisor.energy_model,
                "predicted_consumption_j": supervisor.predicted_consumption_j,
                "approved_mode": approved,
                "reserve_j": supervisor.energy_reserve_j,
                "block_reason": supervisor.block_reason,
                "approved_requirement_j": approved_requirement,
                "reserve_violation": int(
                    approved_requirement > case["available_j"]
                ),
            })
    guarded_rows = [row for row in rows if row["guarded"] == 1]
    result = {
        "date": "2026-07-19",
        "backend": "rotors",
        "energy_model": "rotors_aero_shaft_mechanical_proxy_v1",
        "test_kind": "decision-layer replay using held-out observed energy",
        "rows": rows,
        "acceptance": {
            "guarded_reserve_violations": sum(
                row["reserve_violation"] for row in guarded_rows
            ),
            "unguarded_reserve_violations": sum(
                row["reserve_violation"] for row in rows
                if row["guarded"] == 0
            ),
            "sufficient_guarded_allowed": any(
                row["case"] == "sufficient_takeoff"
                and row["guarded"] == 1 and row["approved_mode"] == "TAKEOFF"
                for row in rows
            ),
            "near_boundary_guarded_allowed": any(
                row["case"] == "near_boundary_safe"
                and row["guarded"] == 1 and row["approved_mode"] == "TAKEOFF"
                for row in rows
            ),
            "insufficient_guarded_blocked": any(
                row["case"].startswith("insufficient")
                and row["guarded"] == 1 and row["approved_mode"] == "GROUND"
                for row in rows
            ),
            "airborne_low_margin_forced_land": any(
                row["case"] == "airborne_low_margin"
                and row["guarded"] == 1 and row["approved_mode"] == "LAND"
                for row in rows
            ),
        },
        "limitation": (
            "Logic/replay validation, not a paper batch or battery-level safety proof."
        ),
    }
    required_flags = (
        "sufficient_guarded_allowed",
        "near_boundary_guarded_allowed",
        "insufficient_guarded_blocked",
        "airborne_low_margin_forced_land",
    )
    if (result["acceptance"]["guarded_reserve_violations"] != 0
            or not all(result["acceptance"][key] for key in required_flags)):
        raise RuntimeError("guarded D2 acceptance failed")
    os.makedirs(os.path.dirname(os.path.abspath(args.output_json)), exist_ok=True)
    with open(args.output_json, "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    fields = list(rows[0])
    with open(args.output_csv, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(result["acceptance"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
