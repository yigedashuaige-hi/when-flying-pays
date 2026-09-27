#!/usr/bin/env python3
"""Run the small repeated Phase-D2 RotorS calibration and held-out set."""

import argparse
import json
import os

import rospy

from uav_v4_rotors_energy_calibration import CalibrationRunner


# Calibration and held-out trials are declared before execution.  Repetitions
# expose run-to-run variation; held-out settings interpolate between calibration
# settings and are never used to fit the model.
TRIALS = [
    # Repeated hover-duration/height calibration (8 trials).
    {"id": "cal_h10_t2_r1", "split": "calibration", "height_m": 1.0,
     "hover_s": 2.0, "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "cal_h10_t2_r2", "split": "calibration", "height_m": 1.0,
     "hover_s": 2.0, "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "cal_h10_t5_r1", "split": "calibration", "height_m": 1.0,
     "hover_s": 5.0, "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "cal_h10_t5_r2", "split": "calibration", "height_m": 1.0,
     "hover_s": 5.0, "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "cal_h15_t2_r1", "split": "calibration", "height_m": 1.5,
     "hover_s": 2.0, "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "cal_h15_t2_r2", "split": "calibration", "height_m": 1.5,
     "hover_s": 2.0, "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "cal_h15_t5_r1", "split": "calibration", "height_m": 1.5,
     "hover_s": 5.0, "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "cal_h15_t5_r2", "split": "calibration", "height_m": 1.5,
     "hover_s": 5.0, "distance_m": 0.0, "target_speed_m_s": 0.0},
    # Repeated distance/speed calibration (4 trials).
    {"id": "cal_d06_v025_h10_r1", "split": "calibration", "height_m": 1.0,
     "hover_s": 3.0, "distance_m": 0.6, "target_speed_m_s": 0.25},
    {"id": "cal_d06_v025_h10_r2", "split": "calibration", "height_m": 1.0,
     "hover_s": 3.0, "distance_m": 0.6, "target_speed_m_s": 0.25},
    {"id": "cal_d12_v045_h15_r1", "split": "calibration", "height_m": 1.5,
     "hover_s": 3.0, "distance_m": 1.2, "target_speed_m_s": 0.45},
    {"id": "cal_d12_v045_h15_r2", "split": "calibration", "height_m": 1.5,
     "hover_s": 3.0, "distance_m": 1.2, "target_speed_m_s": 0.45},
    # Held-out duration/distance/speed combinations (6 trials).
    {"id": "held_h10_t3_r1", "split": "heldout", "height_m": 1.0,
     "hover_s": 3.0, "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "held_h15_t3_r1", "split": "heldout", "height_m": 1.5,
     "hover_s": 3.0, "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "held_d09_v030_h10_r1", "split": "heldout", "height_m": 1.0,
     "hover_s": 3.0, "distance_m": 0.9, "target_speed_m_s": 0.30},
    {"id": "held_d09_v030_h10_r2", "split": "heldout", "height_m": 1.0,
     "hover_s": 3.0, "distance_m": 0.9, "target_speed_m_s": 0.30},
    {"id": "held_d09_v030_h15_r1", "split": "heldout", "height_m": 1.5,
     "hover_s": 3.0, "distance_m": 0.9, "target_speed_m_s": 0.30},
    {"id": "held_d09_v030_h15_r2", "split": "heldout", "height_m": 1.5,
     "hover_s": 3.0, "distance_m": 0.9, "target_speed_m_s": 0.30},
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--max-trials", type=int, default=len(TRIALS))
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("uav_v4_rotors_energy_d2_calibration", anonymous=True)
    runner = CalibrationRunner()
    plan = TRIALS[:args.max_trials]
    result = {
        "date": "2026-07-19",
        "phase": "D2",
        "backend": "rotors",
        "energy_model": "rotors_aero_shaft_mechanical_proxy_v1",
        "plan": plan,
        "trials": [],
    }
    if not runner.wait_ready():
        result.update(passed=False, failure="topics_not_ready")
    else:
        result["passed"] = True
        for spec in plan:
            passed, stages = runner.run_trial(spec)
            result["trials"].append({
                "id": spec["id"], "split": spec["split"],
                "height_m": spec["height_m"], "passed": passed,
                "stages": stages,
            })
            if not passed:
                result["passed"] = False
                break
    result["non_finite_samples"] = runner.non_finite
    result["all_values_finite"] = not runner.non_finite
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    if not result.get("passed"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
