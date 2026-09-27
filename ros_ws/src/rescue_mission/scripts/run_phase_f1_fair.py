#!/usr/bin/env python3
"""Run the preregistered Phase F1 tuning or held-out evaluation batch.

This runner does not alter policy semantics. It isolates every launch in its
own output directory, records exact launch metadata, validates every CSV, and
supports safe resume without silently replacing an observed run.
"""

import argparse
import concurrent.futures
import csv
import datetime as dt
import hashlib
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import time


PKG = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_ROOT = PKG / "results" / "phase_f1_fair_tuning_20260821"
TUNING_SEEDS = tuple(range(100, 110))
EVALUATION_SEEDS = tuple(range(200, 220))
SCENES = ("scene_b", "scene_c")
THRESHOLDS = (0.2, 0.3, 0.4, 0.5, 0.6)
RISK = (
    (0.25, "f1_risk_wcvar_025"),
    (0.50, "f1_risk_wcvar_050"),
    (0.75, "f1_risk_wcvar_075"),
    (1.00, "f1_risk_wcvar_100"),
    (1.25, "f1_risk_wcvar_125"),
)


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stop_group(proc):
    if proc.poll() is not None:
        return proc.returncode
    for sig, wait_s in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return proc.poll()
        try:
            return proc.wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            pass
    return proc.poll()


CORE_FINITE_FIELDS = {
    "seed", "time", "x", "y", "z", "roll", "pitch", "yaw",
    "goal_x", "goal_y", "cmd_vx", "cmd_vy", "cmd_wz", "slope_deg",
    "obstacle_density", "traversability_mean", "traversability_true",
    "traversability_uncertainty", "terrain_cost", "visibility_confidence",
    "stuck_risk_prior", "J_ground", "J_air", "risk_cvar", "energy_term",
    "battery", "power_w", "energy_j", "ground_energy_j", "air_energy_j",
    "takeoff_energy_j", "land_energy_j", "switch_energy_j", "path_length",
    "ground_distance_m", "air_distance_m", "switch_count", "collision_count",
    "stuck_count", "stuck_event_count", "slip_ratio", "final_distance",
    "success", "timeout",
}


def finite_csv(path, expected_scene, expected_seed, minimum_duration):
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("CSV has no data rows")
    last = rows[-1]
    if last.get("scene") != expected_scene:
        raise ValueError("scene mismatch: %r" % last.get("scene"))
    if int(float(last.get("seed", "nan"))) != expected_seed:
        raise ValueError("seed mismatch: %r" % last.get("seed"))
    if float(last.get("time", "nan")) < minimum_duration:
        raise ValueError("run shorter than minimum duration")
    if last.get("air_backend") != "ideal":
        raise ValueError("backend mismatch: %r" % last.get("air_backend"))
    if last.get("energy_model") != "legacy_task_distance_hover_proxy_v1":
        raise ValueError("energy-model mismatch: %r" % last.get("energy_model"))
    for row_number, row in enumerate(rows, 2):
        for key in CORE_FINITE_FIELDS:
            value = row.get(key)
            if value in (None, ""):
                raise ValueError("missing core field %s at row %d" % (key, row_number))
            try:
                number = float(value)
            except ValueError:
                raise ValueError("non-numeric core field %s at row %d" % (key, row_number))
            if not math.isfinite(number):
                raise ValueError("non-finite %s at row %d" % (key, row_number))
    return {"rows": len(rows), "final": last}


def tuning_tasks():
    tasks = []
    for scene in SCENES:
        for tau in THRESHOLDS:
            label = "tau_%03d" % round(tau * 100)
            for seed in TUNING_SEEDS:
                tasks.append(dict(stage="tuning", scene=scene, family="threshold_switch",
                                  candidate=label, value=tau, seed=seed,
                                  strategy="threshold_switch", switch_tau=tau, ablation="none"))
        for value, ablation in RISK:
            label = "wcvar_%03d" % round(value * 100)
            for seed in TUNING_SEEDS:
                tasks.append(dict(stage="tuning", scene=scene, family="risk_aware",
                                  candidate=label, value=value, seed=seed,
                                  strategy="risk_aware", switch_tau=-1.0, ablation=ablation))
    return tasks


def evaluation_tasks(selected_path):
    with open(selected_path) as stream:
        selected = json.load(stream)
    if selected.get("status") != "frozen_before_evaluation":
        raise ValueError("selected config is not marked frozen_before_evaluation")
    tasks = []
    for scene in SCENES:
        scene_selected = selected["selected"][scene]
        tau = float(scene_selected["threshold_switch"]["value"])
        risk_value = float(scene_selected["risk_aware"]["value"])
        risk_ablation = dict(RISK)[risk_value]
        methods = (
            ("threshold_switch", "selected_tau_%03d" % round(tau * 100), tau,
             "threshold_switch", tau, "none"),
            ("risk_aware", "selected_wcvar_%03d" % round(risk_value * 100), risk_value,
             "risk_aware", -1.0, risk_ablation),
            ("fixed_reference", "fixed_switch", None, "fixed_switch", -1.0, "none"),
        )
        for family, candidate, value, strategy, switch_tau, ablation in methods:
            for seed in EVALUATION_SEEDS:
                tasks.append(dict(stage="evaluation", scene=scene, family=family,
                                  candidate=candidate, value=value, seed=seed,
                                  strategy=strategy, switch_tau=switch_tau, ablation=ablation))
    return tasks


def task_id(task):
    return "__".join((task["stage"], task["scene"], task["family"],
                      task["candidate"], "seed_%d" % task["seed"]))


def run_task(task, args, worker_index):
    ident = task_id(task)
    run_dir = pathlib.Path(args.root) / "runs" / task["stage"] / task["scene"] / task["family"] / task["candidate"] / ("seed_%d" % task["seed"])
    log_dir = pathlib.Path(args.root) / "logs" / task["stage"]
    metadata_dir = pathlib.Path(args.root) / "metadata" / task["stage"]
    for directory in (run_dir, log_dir, metadata_dir):
        directory.mkdir(parents=True, exist_ok=True)
    marker = metadata_dir / (ident + ".json")
    if marker.exists():
        with open(marker) as stream:
            old = json.load(stream)
        if old.get("status") == "valid":
            return ident, "skipped_valid"
        raise RuntimeError("Refusing to replace prior non-valid run: %s" % marker)

    latest = run_dir / "latest.csv"
    cmd = [
        "roslaunch", "rescue_mission", "rescue_phase1_demo.launch",
        "strategy:=%s" % task["strategy"],
        "scene:=%s" % task["scene"],
        "seed:=%d" % task["seed"],
        "mission_timeout:=%s" % args.mission_timeout,
        "runs_dir:=%s" % run_dir,
        "latest_csv_path:=%s" % latest,
        "ablation:=%s" % task["ablation"],
        "heteroscedastic:=false", "initial_battery:=-1", "fly_k1:=-1",
        "switch_tau:=%s" % task["switch_tau"],
        "stuck_model:=recoverable", "air_backend:=ideal",
        "gui:=false", "headless:=true", "paused:=false",
    ]
    env = os.environ.copy()
    if args.workers > 1:
        offset = worker_index * args.port_stride
        env["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % (args.ros_port_base + offset)
        env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:%d" % (args.gazebo_port_base + offset)
    started = dt.datetime.now(dt.timezone.utc)
    log_path = log_dir / (ident + ".log")
    record = dict(task)
    record.update({
        "task_id": ident,
        "status": "started",
        "backend": "ideal_set_model_state",
        "command": cmd,
        "started_utc": started.isoformat(),
        "worker_index": worker_index,
        "ros_master_uri": env.get("ROS_MASTER_URI", ""),
        "gazebo_master_uri": env.get("GAZEBO_MASTER_URI", ""),
        "log": str(log_path),
    })
    with open(marker, "w") as stream:
        json.dump(record, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with open(log_path, "wb") as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True, env=env)
        timed_out = False
        try:
            code = proc.wait(timeout=args.wall_timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            code = stop_group(proc)
        except BaseException:
            stop_group(proc)
            raise
    ended = dt.datetime.now(dt.timezone.utc)
    record.update({"ended_utc": ended.isoformat(), "wall_seconds": (ended - started).total_seconds(),
                   "launch_return_code": code, "wall_timeout_reached": timed_out})
    try:
        if not latest.exists():
            raise ValueError("latest CSV missing")
        validation = finite_csv(latest, task["scene"], task["seed"], args.min_duration)
        record.update({"status": "valid", "csv": str(latest), "csv_sha256": sha256(latest),
                       "validation": validation})
    except Exception as exc:
        record.update({"status": "invalid", "validation_error": str(exc)})
    with open(marker, "w") as stream:
        json.dump(record, stream, indent=2, sort_keys=True)
        stream.write("\n")
    if record["status"] != "valid":
        raise RuntimeError("%s invalid: %s" % (ident, record.get("validation_error")))
    return ident, "valid"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=("tuning", "evaluation"))
    parser.add_argument("--root", default=str(DEFAULT_ROOT))
    parser.add_argument("--selected-configs", default="")
    parser.add_argument("--mission-timeout", type=float, default=85.0)
    parser.add_argument("--wall-timeout", type=float, default=130.0)
    parser.add_argument("--min-duration", type=float, default=30.0)
    parser.add_argument("--cooldown", type=float, default=2.0)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--ros-port-base", type=int, default=11411)
    parser.add_argument("--gazebo-port-base", type=int, default=11445)
    parser.add_argument("--port-stride", type=int, default=10,
                        help="Spacing between task-specific master ports.")
    parser.add_argument("--task-start", type=int, default=0,
                        help="Inclusive index in the frozen ordered task list.")
    parser.add_argument("--task-stop", type=int, default=-1,
                        help="Exclusive index; -1 means the end of the frozen task list.")
    args = parser.parse_args()
    if args.workers < 1:
        parser.error("--workers must be positive")
    if args.port_stride < 1:
        parser.error("--port-stride must be positive")
    if args.stage == "evaluation" and not args.selected_configs:
        parser.error("evaluation requires --selected-configs")
    all_tasks = tuning_tasks() if args.stage == "tuning" else evaluation_tasks(args.selected_configs)
    stop = len(all_tasks) if args.task_stop < 0 else args.task_stop
    if not (0 <= args.task_start <= stop <= len(all_tasks)):
        parser.error("invalid --task-start/--task-stop range")
    tasks = all_tasks[args.task_start:stop]
    print("Phase F1 %s: %d tasks, %d worker(s)" % (args.stage, len(tasks), args.workers), flush=True)
    completed = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        pending = {}
        for index, task in enumerate(tasks):
            # A unique master-port pair per task prevents a newly scheduled
            # task from colliding with a slower task in another pool thread.
            worker_index = index
            future = pool.submit(run_task, task, args, worker_index)
            pending[future] = task_id(task)
        try:
            for future in concurrent.futures.as_completed(pending):
                ident, status = future.result()
                completed += 1
                print("[%d/%d] %s %s" % (completed, len(tasks), status, ident), flush=True)
                time.sleep(args.cooldown)
        except BaseException:
            for future in pending:
                future.cancel()
            raise
    return 0


if __name__ == "__main__":
    sys.exit(main())
