#!/usr/bin/env python3
"""Compare Phase-D2 supervisor logic with its backed-up ideal implementation."""

import argparse
import importlib.util
import json
import os
import sys


def load_class(path, module_name):
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.SafetySupervisor


def snapshot(instance, approved):
    return {
        "approved": approved,
        "shield_override_count": instance.shield_override_count,
        "unsafe_action_count": instance.unsafe_action_count,
        "irreversible_failure": instance.irreversible_failure,
        "entered_unrecoverable_set": instance.entered_unrecoverable_set,
        "override_reason": instance.override_reason,
        "safe_landing_margin_j": instance.safe_landing_margin_j,
        "recoverability_margin": instance.recoverability_margin,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--before", required=True)
    parser.add_argument("--current", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    before_class = load_class(args.before, "safety_supervisor_before_d2")
    current_class = load_class(args.current, "safety_supervisor_current_d2")
    params = {
        "E_takeoff_j": 11.0, "E_air_reserve_j": 12.0,
        "E_land_j": 12.0, "E_margin_j": 8.0, "eta_land": 0.30,
    }
    cases = []
    for enforce in (False, True):
        for current_mode in ("GROUND", "TAKEOFF", "AIR", "LAND"):
            for proposed in ("GROUND", "TAKEOFF", "AIR", "LAND"):
                for battery_j in (0.0, 11.0, 19.9, 20.0, 42.9, 43.0, 60.0):
                    for landing_safety in (0.2, 0.8):
                        state = {
                            "current_mode": current_mode,
                            "battery_j": battery_j,
                            "landing_area_safety": landing_safety,
                            "stuck_risk_prior": 0.0,
                        }
                        before = before_class(params, enforce=enforce, active=False)
                        current = current_class(
                            params, enforce=enforce, active=False,
                            energy_backend="ideal",
                        )
                        a = snapshot(before, before.filter(proposed, state))
                        b = snapshot(current, current.filter(proposed, state))
                        cases.append({"same": a == b, "before": a, "current": b})
    mismatches = [item for item in cases if not item["same"]]
    result = {
        "date": "2026-07-19",
        "case_count": len(cases),
        "mismatch_count": len(mismatches),
        "passed": not mismatches,
        "compared_fields": list(cases[0]["before"]),
        "mismatches": mismatches[:10],
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    if mismatches:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
