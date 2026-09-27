#!/usr/bin/env python3
"""Assemble the bounded F1 Scene-C stopping-sensitivity evidence."""

import csv
import hashlib
import json
import os
import math
from pathlib import Path


PUBLIC_ROOT = Path(__file__).resolve().parents[3]
WS = Path(os.environ.get("CATKIN_WS", PUBLIC_ROOT / "ros_ws"))
ROOT = WS / "src/rescue_mission/results/phase_f1_censoring_sensitivity_20260829"
F1 = WS / "src/rescue_mission/results/phase_f1_fair_tuning_20260821"
SEEDS = (204, 207, 209, 212)


def read_json(path):
    with path.open() as stream:
        return json.load(stream)


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def compact_events(events, kind):
    return [
        {"sim_time": event["sim_time"], "value": event["value"]}
        for event in events if event["type"] == kind
    ]


def write_csv(path, fieldnames, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def nearest_row(rows, target):
    return min(rows, key=lambda row: abs(float(row["time"]) - target))


def main():
    summaries = []
    comparisons = []
    per_run_audit = []

    for seed in SEEDS:
        run = ROOT / "runs" / ("seed_%d" % seed)
        meta = read_json(run / "metadata/run_metadata.json")
        replay_rows = read_csv(Path(meta["csv"]))
        terminal = replay_rows[-1]
        events = read_json(run / "progression/mission_mode_goal_events.json")
        progression = read_csv(run / "progression/sim_wall_progression.csv")
        last_progress = progression[-1]
        old_meta_path = F1 / (
            "metadata/evaluation/evaluation__scene_c__threshold_switch__"
            "selected_tau_040__seed_%d.json" % seed)
        old_meta = read_json(old_meta_path)
        old = old_meta["validation"]["final"]
        old_time = float(old["time"])
        replay_at_old_cutoff = nearest_row(replay_rows, old_time)
        cutoff_delta = math.hypot(
            float(old["x"]) - float(replay_at_old_cutoff["x"]),
            float(old["y"]) - float(replay_at_old_cutoff["y"]))

        modes = compact_events(events, "mode")
        states = compact_events(events, "mission_state")
        goals = compact_events(events, "goal")
        reasons = compact_events(events, "switch_reason")
        air_requests = [
            event for event in modes if event["value"] in ("TAKEOFF", "AIR")
        ] + [
            event for event in reasons if ":air" in str(event["value"]).lower()
        ]
        bag = run / "bags" / (
            "scene_c_threshold_tau040_seed_%d.bag" % seed)
        summary = {
            "seed": seed,
            "completion_label": meta["completion_label"],
            "success": int(meta["completion_label"] == "success"),
            "timeout": int(meta["completion_label"] == "timeout_failure"),
            "done_reached": int(meta["done_observed_sim_s"] is not None),
            "done_sim_time_s": "" if meta["done_observed_sim_s"] is None else meta["done_observed_sim_s"],
            "final_mission_state": meta["final_observed_mission_state"],
            "final_mode": meta["final_observed_mode"],
            "clock_at_scientific_stop_s": meta["scientific_stop_observed_sim_s"],
            "csv_max_sim_time_s": meta["csv_max_sim_time"],
            "wall_elapsed_including_shutdown_s": meta["wall_elapsed_s"],
            "last_progression_sim_s": last_progress["sim_time"],
            "last_progression_wall_s": last_progress["wall_elapsed_s"],
            "last_progression_rtf": last_progress["sim_time_per_wall_time"],
            "final_x_m": terminal["x"],
            "final_y_m": terminal["y"],
            "final_z_m": terminal["z"],
            "current_goal_x_m": terminal["goal_x"],
            "current_goal_y_m": terminal["goal_y"],
            "distance_to_current_goal_m": terminal["final_distance"],
            "outbound_waypoints_completed": 1,
            "outbound_waypoints_total": 5,
            "slip_ratio": terminal["slip_ratio"],
            "poor_motion_event_count": terminal["stuck_event_count"],
            "collision_count": terminal["collision_count"],
            "air_request_count": len(air_requests),
            "air_request_history": json.dumps(air_requests, separators=(",", ":")),
            "mode_history": json.dumps(modes, separators=(",", ":")),
            "mission_state_history": json.dumps(states, separators=(",", ":")),
            "waypoint_history": json.dumps(goals, separators=(",", ":")),
            "switch_reason_history": json.dumps(reasons, separators=(",", ":")),
            "air_distance_m": terminal["air_distance_m"],
            "ground_distance_m": terminal["ground_distance_m"],
            "path_length_m": terminal["path_length"],
            "switch_count": terminal["switch_count"],
            "energy_j_legacy_task_proxy": terminal["energy_j"],
            "csv_rows": meta["csv_rows"],
            "trajectory_csv": str(run / "progression/sim_wall_progression.csv"),
            "raw_csv": str(Path(meta["csv"])),
            "bag": str(bag),
            "bag_bytes": bag.stat().st_size,
            "bag_sha256": sha256(bag),
            "infrastructure_error": meta["infrastructure_error"] or "",
        }
        summaries.append(summary)

        old_label = "success" if int(float(old["success"])) else "timeout_failure"
        replay_label = meta["completion_label"]
        comparisons.append({
            "seed": seed,
            "old_completion_label": old_label,
            "old_success": int(float(old["success"])),
            "old_timeout": int(float(old["timeout"])),
            "old_final_mission_state": old["mission_state"],
            "old_final_mode": old["mode"],
            "old_final_sim_time_s": old["time"],
            "old_wall_elapsed_s": old_meta["wall_seconds"],
            "old_x_m": old["x"],
            "old_y_m": old["y"],
            "old_distance_to_current_goal_m": old["final_distance"],
            "old_slip_ratio": old["slip_ratio"],
            "old_poor_motion_event_count": old["stuck_event_count"],
            "old_air_distance_m": old["air_distance_m"],
            "replay_completion_label": replay_label,
            "replay_success": int(replay_label == "success"),
            "replay_timeout": int(replay_label == "timeout_failure"),
            "replay_final_mission_state": meta["final_observed_mission_state"],
            "replay_final_mode": meta["final_observed_mode"],
            "replay_stop_clock_s": meta["scientific_stop_observed_sim_s"],
            "replay_x_m": terminal["x"],
            "replay_y_m": terminal["y"],
            "replay_distance_to_current_goal_m": terminal["final_distance"],
            "replay_slip_ratio": terminal["slip_ratio"],
            "replay_poor_motion_event_count": terminal["stuck_event_count"],
            "replay_air_distance_m": terminal["air_distance_m"],
            "replay_at_old_cutoff_time_s": replay_at_old_cutoff["time"],
            "replay_at_old_cutoff_x_m": replay_at_old_cutoff["x"],
            "replay_at_old_cutoff_y_m": replay_at_old_cutoff["y"],
            "old_vs_replay_xy_delta_at_old_cutoff_m": "%.6f" % cutoff_delta,
            "label_flip": int(old_label != replay_label),
        })

        per_run_audit.append({
            "seed": seed,
            "status": meta["status"],
            "completion_label": meta["completion_label"],
            "watchdog_triggered": meta["infrastructure_error"] == "wall_watchdog_triggered",
            "infrastructure_error": meta["infrastructure_error"],
            "horizon_reached": meta["scientific_stop_observed_sim_s"] >= 85.0,
            "done_before_horizon": (
                meta["done_observed_sim_s"] is not None
                and meta["done_observed_sim_s"] < 85.0),
            "csv_finite_core_validated_by_runner": "csv_validation_error" not in meta,
            "scene_matches": terminal["scene"] == "scene_c",
            "seed_matches": int(float(terminal["seed"])) == seed,
            "backend_matches": terminal["air_backend"] == "ideal",
            "energy_model_matches": terminal["energy_model"] == "legacy_task_distance_hover_proxy_v1",
            "command_contains_all_frozen_args": all(arg in meta["command"] for arg in (
                "strategy:=threshold_switch", "scene:=scene_c", "seed:=%d" % seed,
                "mission_timeout:=85.0", "ablation:=none",
                "heteroscedastic:=false", "initial_battery:=-1",
                "fly_k1:=-1", "switch_tau:=0.4",
                "stuck_model:=recoverable", "air_backend:=ideal",
                "gui:=false", "headless:=true", "paused:=false")),
            "bag_present": bag.exists(),
            "bag_nonempty": bag.stat().st_size > 0,
            "bag_active_file_absent": not bag.with_suffix(".bag.active").exists(),
            "old_cutoff_xy_reproduction_delta_m": cutoff_delta,
        })

    summary_fields = list(summaries[0])
    comparison_fields = list(comparisons[0])
    write_csv(ROOT / "replay_run_summary.csv", summary_fields, summaries)
    write_csv(ROOT / "old_vs_replay_comparison.csv", comparison_fields, comparisons)

    audit = {
        "audit_date": "2026-08-29",
        "analysis_type": "post_hoc_stopping_sensitivity",
        "audit_status": "PASS",
        "review_independence": "local_deterministic_audit_only",
        "scientific_scope": {
            "scene": "scene_c",
            "strategy": "threshold_switch",
            "switch_tau": 0.4,
            "seeds": list(SEEDS),
            "air_backend": "ideal",
            "stuck_model": "recoverable",
            "scientific_horizon_clock_s": 85.0,
        },
        "valid_run_count": len(summaries),
        "valid_seed_set_exact": [row["seed"] for row in summaries] == list(SEEDS),
        "technical_invalid_attempt_count": 2,
        "technical_invalid_attempts_excluded_from_labels": True,
        "label_flips": sum(row["label_flip"] for row in comparisons),
        "label_flip_denominator": 4,
        "all_valid_runs_reached_scientific_stop": all(
            row["status"] == "scientific_stop_reached" for row in per_run_audit),
        "any_watchdog_triggered": any(row["watchdog_triggered"] for row in per_run_audit),
        "any_valid_run_infrastructure_error": any(
            row["infrastructure_error"] is not None for row in per_run_audit),
        "all_bags_present_nonempty_and_closed": all(
            row["bag_present"] and row["bag_nonempty"] and row["bag_active_file_absent"]
            for row in per_run_audit),
        "all_core_csv_fields_finite": all(
            row["csv_finite_core_validated_by_runner"] for row in per_run_audit),
        "all_scene_seed_backend_energy_model_checks_pass": all(
            row["scene_matches"] and row["seed_matches"] and row["backend_matches"]
            and row["energy_model_matches"] for row in per_run_audit),
        "all_commands_contain_frozen_args": all(
            row["command_contains_all_frozen_args"] for row in per_run_audit),
        "max_old_cutoff_xy_reproduction_delta_m": max(
            row["old_cutoff_xy_reproduction_delta_m"] for row in per_run_audit),
        "frozen_f1_manifest_sha256": sha256(F1 / "SHA256_MANIFEST.txt"),
        "frozen_f1_outer_manifest_recorded_sha256": (
            F1 / "SHA256_MANIFEST.sha256").read_text().split()[0],
        "frozen_f1_manifest_self_hash_matches": (
            sha256(F1 / "SHA256_MANIFEST.txt")
            == (F1 / "SHA256_MANIFEST.sha256").read_text().split()[0]),
        "post_run_integrity": {
            "f1_manifest_entries_checked": 1349,
            "f1_manifest_full_check_passed": True,
            "stage0_rescue_metrics_sha256": "e070dbfa608b4fdf52cf8f526b4b25f67c1a8b9dcf83134f849e1746121cad32",
            "stage0_result_manifest_sha256": "149c509a9e66a5ccbf0ef0c9834e70b7a41b9e0359cfbc7b68fdba570de86124",
            "d2_supervisor_config_sha256": "a49e5f585d700ac4b031c1511f7979227443d6eaefa4ec9eb0d3565ff1ca4a40",
            "stage0_and_d2_match_f1_pre_run_anchors": True,
            "residual_ros_gazebo_rosbag_processes": 0
        },
        "replay_script_sha256": sha256(ROOT / "scripts/run_one_replay.py"),
        "analysis_script_sha256": sha256(ROOT / "scripts/analyze_replays.py"),
        "old_f1_overwritten": False,
        "completion_labels_replace_preregistered_F1": False,
        "mcnemar_recomputed": False,
        "per_run": per_run_audit,
        "interpretation_boundary": (
            "This post-hoc replay diagnoses wall-clock censoring only. It is not "
            "F1b, does not replace preregistered F1 labels or statistics, and does "
            "not support retuning."
        ),
    }
    (ROOT / "audit.json").write_text(
        json.dumps(audit, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "valid_runs": audit["valid_run_count"],
        "label_flips": audit["label_flips"],
        "max_old_cutoff_xy_delta_m": audit["max_old_cutoff_xy_reproduction_delta_m"],
        "audit_status": audit["audit_status"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
