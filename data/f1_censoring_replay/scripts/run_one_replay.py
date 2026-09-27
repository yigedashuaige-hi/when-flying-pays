#!/usr/bin/env python3
"""Run one post-hoc F1 Scene-C stopping-sensitivity replay.

The scientific launch arguments are copied verbatim from the frozen F1
evaluation metadata.  This wrapper changes only the outer stopping rule:
mission completion is observed against /clock=85 s, while the wall clock is
used solely as a broad infrastructure watchdog.
"""

import argparse
import csv
import datetime as dt
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import threading
import time
import xmlrpc.client


TOPICS = [
    "/clock",
    "/odom",
    "/rescue/mission_state",
    "/rescue/mode",
    "/rescue/goal",
    "/rescue/switch_reason",
    "/rescue/ground_cmd_nominal",
    "/mecanum/cmd_vel",
    "/air_stub/cmd_vel",
    "/rescue/terrain_slope_deg",
    "/rescue/obstacle_density",
    "/rescue/traversability_mean",
    "/rescue/traversability_true",
    "/rescue/traversability_uncertainty",
    "/rescue/terrain_cost",
    "/rescue/stuck_risk_prior",
]


def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def stop_group(proc, name, grace=12.0):
    if proc is None or proc.poll() is not None:
        return None if proc is None else proc.returncode
    for sig, wait_s in ((signal.SIGINT, grace), (signal.SIGTERM, 5.0),
                        (signal.SIGKILL, 2.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return proc.poll()
        try:
            return proc.wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            pass
    return proc.poll()


def wait_for_master(uri, proc, timeout=40.0):
    deadline = time.monotonic() + timeout
    master = xmlrpc.client.ServerProxy(uri)
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError("roslaunch exited before ROS master became ready")
        try:
            code, _, _ = master.getPid("/f1_censoring_replay_probe")
            if code == 1:
                return
        except Exception:
            pass
        time.sleep(0.25)
    raise RuntimeError("ROS master did not become ready within %.1f wall s" % timeout)


def read_terminal_csv(path):
    if not path.exists():
        raise RuntimeError("latest CSV missing: %s" % path)
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("latest CSV has no rows")
    return rows, rows[-1]


def finite_core(rows):
    keys = (
        "time", "x", "y", "z", "goal_x", "goal_y", "slip_ratio",
        "stuck_event_count", "air_distance_m", "success", "timeout",
    )
    for row_number, row in enumerate(rows, 2):
        for key in keys:
            try:
                value = float(row[key])
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError(
                    "invalid %s at CSV row %d: %s" % (key, row_number, exc))
            if not math.isfinite(value):
                raise RuntimeError("non-finite %s at CSV row %d" % (key, row_number))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, required=True,
                        choices=(204, 207, 209, 212))
    parser.add_argument("--root", required=True)
    parser.add_argument("--horizon", type=float, default=85.0)
    parser.add_argument("--wall-watchdog", type=float, default=900.0)
    parser.add_argument("--ros-port", type=int, required=True)
    parser.add_argument("--gazebo-port", type=int, required=True)
    args = parser.parse_args()

    root = pathlib.Path(args.root).resolve()
    run_dir = root / "runs" / ("seed_%d" % args.seed)
    if run_dir.exists():
        raise RuntimeError("refusing to overwrite existing run directory: %s" % run_dir)
    for child in ("csv", "logs", "bags", "metadata", "progression"):
        (run_dir / child).mkdir(parents=True, exist_ok=False)

    latest = run_dir / "csv" / "latest.csv"
    roslaunch_log = run_dir / "logs" / "roslaunch.log"
    bag_log = run_dir / "logs" / "rosbag.log"
    bag_path = run_dir / "bags" / ("scene_c_threshold_tau040_seed_%d.bag" % args.seed)
    metadata_path = run_dir / "metadata" / "run_metadata.json"
    progression_path = run_dir / "progression" / "sim_wall_progression.csv"
    events_path = run_dir / "progression" / "mission_mode_goal_events.json"

    env = os.environ.copy()
    env["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % args.ros_port
    env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:%d" % args.gazebo_port
    # rospy reads the current process environment, not the child-process env
    # dictionary used below.  Keep the monitor on the same isolated master.
    os.environ["ROS_MASTER_URI"] = env["ROS_MASTER_URI"]
    os.environ["GAZEBO_MASTER_URI"] = env["GAZEBO_MASTER_URI"]

    command = [
        "roslaunch", "rescue_mission", "rescue_phase1_demo.launch",
        "strategy:=threshold_switch",
        "scene:=scene_c",
        "seed:=%d" % args.seed,
        "mission_timeout:=85.0",
        "runs_dir:=%s" % (run_dir / "csv"),
        "latest_csv_path:=%s" % latest,
        "ablation:=none",
        "heteroscedastic:=false",
        "initial_battery:=-1",
        "fly_k1:=-1",
        "switch_tau:=0.4",
        "stuck_model:=recoverable",
        "air_backend:=ideal",
        "gui:=false",
        "headless:=true",
        "paused:=false",
    ]
    bag_command = ["rosbag", "record", "--buffsize=256", "-O", str(bag_path)] + TOPICS

    record = {
        "analysis_type": "post_hoc_stopping_sensitivity",
        "completion_label": None,
        "seed": args.seed,
        "scene": "scene_c",
        "strategy": "threshold_switch",
        "switch_tau": 0.4,
        "stuck_model": "recoverable",
        "air_backend": "ideal",
        "backend_manifest_label": "ideal_set_model_state",
        "mission_horizon_sim_s": args.horizon,
        "infrastructure_wall_watchdog_s": args.wall_watchdog,
        "command": command,
        "bag_command": bag_command,
        "ros_master_uri": env["ROS_MASTER_URI"],
        "gazebo_master_uri": env["GAZEBO_MASTER_URI"],
        "started_utc": utc_now(),
        "status": "starting",
    }
    metadata_path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")

    started_wall = time.monotonic()
    launch_proc = None
    bag_proc = None
    launch_stream = None
    bag_stream = None
    lock = threading.Lock()
    observed = {
        "sim_time": None,
        "first_clock_wall_s": None,
        "mission_state": "",
        "mode": "",
        "goal": None,
        "odom": None,
        "switch_reason": "",
        "done_sim_time": None,
    }
    events = []
    progression = []
    last_progress_sim = -1e9
    last_console_bucket = -1
    infrastructure_error = None

    try:
        launch_stream = roslaunch_log.open("wb")
        launch_proc = subprocess.Popen(
            command, stdout=launch_stream, stderr=subprocess.STDOUT,
            start_new_session=True, env=env)
        wait_for_master(env["ROS_MASTER_URI"], launch_proc)

        bag_stream = bag_log.open("wb")
        bag_proc = subprocess.Popen(
            bag_command, stdout=bag_stream, stderr=subprocess.STDOUT,
            start_new_session=True, env=env)

        # Import rospy only after the isolated master URI is set.
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String

        rospy.init_node(
            "f1_censoring_replay_monitor_%d" % args.seed,
            anonymous=False, disable_signals=True)

        def wall_elapsed():
            return time.monotonic() - started_wall

        def clock_cb(msg):
            with lock:
                observed["sim_time"] = msg.clock.to_sec()
                if observed["first_clock_wall_s"] is None:
                    observed["first_clock_wall_s"] = wall_elapsed()

        def state_cb(msg):
            now_sim = rospy.Time.now().to_sec()
            with lock:
                if msg.data != observed["mission_state"]:
                    events.append({"type": "mission_state", "value": msg.data,
                                   "sim_time": now_sim, "wall_s": wall_elapsed()})
                observed["mission_state"] = msg.data
                if msg.data == "DONE" and observed["done_sim_time"] is None:
                    observed["done_sim_time"] = now_sim

        def mode_cb(msg):
            now_sim = rospy.Time.now().to_sec()
            with lock:
                if msg.data != observed["mode"]:
                    events.append({"type": "mode", "value": msg.data,
                                   "sim_time": now_sim, "wall_s": wall_elapsed()})
                observed["mode"] = msg.data

        def goal_cb(msg):
            now_sim = rospy.Time.now().to_sec()
            goal = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            with lock:
                if goal != observed["goal"]:
                    events.append({"type": "goal", "value": goal,
                                   "sim_time": now_sim, "wall_s": wall_elapsed()})
                observed["goal"] = goal

        def odom_cb(msg):
            with lock:
                observed["odom"] = [
                    msg.pose.pose.position.x, msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                    msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ]

        def reason_cb(msg):
            now_sim = rospy.Time.now().to_sec()
            with lock:
                if msg.data != observed["switch_reason"]:
                    events.append({"type": "switch_reason", "value": msg.data,
                                   "sim_time": now_sim, "wall_s": wall_elapsed()})
                observed["switch_reason"] = msg.data

        rospy.Subscriber("/clock", Clock, clock_cb, queue_size=100)
        rospy.Subscriber("/rescue/mission_state", String, state_cb, queue_size=20)
        rospy.Subscriber("/rescue/mode", String, mode_cb, queue_size=20)
        rospy.Subscriber("/rescue/goal", PoseStamped, goal_cb, queue_size=20)
        rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=20)
        rospy.Subscriber("/rescue/switch_reason", String, reason_cb, queue_size=20)

        record["status"] = "running"
        metadata_path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")

        # We observe slightly beyond 85 s (0.05 s) only to drain callbacks.
        # A DONE event is successful only when its own observed /clock is <85 s.
        while True:
            time.sleep(0.05)
            wall_s = wall_elapsed()
            if wall_s >= args.wall_watchdog:
                infrastructure_error = "wall_watchdog_triggered"
                break
            if launch_proc.poll() is not None:
                infrastructure_error = "roslaunch_exited_before_scientific_stop"
                break
            if bag_proc.poll() is not None:
                infrastructure_error = "rosbag_exited_before_scientific_stop"
                break

            with lock:
                snapshot = dict(observed)
            sim_t = snapshot["sim_time"]
            if sim_t is None:
                if wall_s > 60.0:
                    infrastructure_error = "clock_missing_for_60_wall_s"
                    break
                continue

            if sim_t - last_progress_sim >= 0.5:
                last_progress_sim = sim_t
                odom = snapshot["odom"] or [float("nan")] * 6
                goal = snapshot["goal"] or [float("nan")] * 3
                active_wall = max(
                    wall_s - (snapshot["first_clock_wall_s"] or 0.0), 1e-9)
                progression.append({
                    "sim_time": sim_t,
                    "wall_elapsed_s": wall_s,
                    "sim_time_per_wall_time": sim_t / active_wall,
                    "mission_state": snapshot["mission_state"],
                    "mode": snapshot["mode"],
                    "goal_x": goal[0], "goal_y": goal[1], "goal_z": goal[2],
                    "x": odom[0], "y": odom[1], "z": odom[2],
                    "vx": odom[3], "vy": odom[4], "vz": odom[5],
                    "switch_reason": snapshot["switch_reason"],
                })

            bucket = int(sim_t // 10)
            if bucket > last_console_bucket:
                last_console_bucket = bucket
                print(
                    "seed=%d sim=%.3f wall=%.1f state=%s mode=%s x=%s y=%s" % (
                        args.seed, sim_t, wall_s, snapshot["mission_state"],
                        snapshot["mode"],
                        "nan" if snapshot["odom"] is None else "%.3f" % snapshot["odom"][0],
                        "nan" if snapshot["odom"] is None else "%.3f" % snapshot["odom"][1]),
                    flush=True)

            done_t = snapshot["done_sim_time"]
            if done_t is not None and done_t < args.horizon:
                record["completion_label"] = "success"
                break
            if sim_t >= args.horizon + 0.05:
                record["completion_label"] = "timeout_failure"
                break

        with lock:
            final_observed = dict(observed)
        record.update({
            "scientific_stop_observed_sim_s": final_observed["sim_time"],
            "done_observed_sim_s": final_observed["done_sim_time"],
            "final_observed_mission_state": final_observed["mission_state"],
            "final_observed_mode": final_observed["mode"],
            "infrastructure_error": infrastructure_error,
        })
        if infrastructure_error is not None:
            record["completion_label"] = None
            record["status"] = "infrastructure_failed"
        else:
            record["status"] = "scientific_stop_reached"

    except BaseException as exc:
        infrastructure_error = "%s: %s" % (type(exc).__name__, exc)
        record.update({
            "status": "infrastructure_failed",
            "infrastructure_error": infrastructure_error,
            "completion_label": None,
        })
    finally:
        bag_code = stop_group(bag_proc, "rosbag", grace=8.0)
        launch_code = stop_group(launch_proc, "roslaunch", grace=15.0)
        if bag_stream is not None:
            bag_stream.close()
        if launch_stream is not None:
            launch_stream.close()
        record.update({
            "ended_utc": utc_now(),
            "wall_elapsed_s": time.monotonic() - started_wall,
            "rosbag_return_code": bag_code,
            "roslaunch_return_code": launch_code,
        })

    fieldnames = [
        "sim_time", "wall_elapsed_s", "sim_time_per_wall_time",
        "mission_state", "mode", "goal_x", "goal_y", "goal_z",
        "x", "y", "z", "vx", "vy", "vz", "switch_reason",
    ]
    with progression_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(progression)
    events_path.write_text(json.dumps(events, indent=2, sort_keys=True) + "\n")

    try:
        rows, terminal = read_terminal_csv(latest)
        finite_core(rows)
        record["csv"] = str(latest)
        record["csv_rows"] = len(rows)
        record["csv_terminal"] = terminal
        record["csv_max_sim_time"] = max(float(row["time"]) for row in rows)
        # The external observer is authoritative for the replay label.
        if record["completion_label"] == "success":
            if terminal.get("mission_state") != "DONE":
                raise RuntimeError("external DONE observation disagrees with terminal CSV")
        elif record["completion_label"] == "timeout_failure":
            if record["csv_max_sim_time"] < args.horizon - 0.5:
                raise RuntimeError("CSV did not approach the 85 s simulation horizon")
    except Exception as exc:
        record["csv_validation_error"] = "%s: %s" % (type(exc).__name__, exc)
        record["status"] = "infrastructure_failed"
        record["infrastructure_error"] = record["csv_validation_error"]
        record["completion_label"] = None

    record["progression_rows"] = len(progression)
    record["event_count"] = len(events)
    metadata_path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "seed": args.seed,
        "status": record["status"],
        "completion_label": record["completion_label"],
        "sim_time": record.get("scientific_stop_observed_sim_s"),
        "wall_elapsed_s": record["wall_elapsed_s"],
        "infrastructure_error": record.get("infrastructure_error"),
    }, sort_keys=True), flush=True)
    return 0 if record["status"] == "scientific_stop_reached" else 2


if __name__ == "__main__":
    sys.exit(main())
