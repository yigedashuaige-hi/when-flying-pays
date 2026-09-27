#!/usr/bin/env python3
"""Create the E1a-R2 acceptance audit from the single frozen Scene A bag."""

import argparse
import csv
import json
import math
from pathlib import Path

import rosbag


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def mean_abs(values):
    values = list(values)
    return sum(abs(value) for value in values) / len(values) if values else 0.0


def diagnostic_values(msg):
    if not msg.status:
        return {}
    return {item.key: item.value for item in msg.status[0].values}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    bag_path = Path(args.bag).resolve()
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)

    latest = {
        "requested_mode": "",
        "approved_mode": "",
        "mission_state": "",
        "handoff_state": "",
        "ground_owner": False,
        "ground_active": False,
        "motor_enable": False,
        "disarmed": True,
        "site_measurement_valid": False,
        "site_valid": False,
        "site_reason": "measurement_missing",
        "site_stamp": float("-inf"),
        "persistent_site_count": 0,
        "target_site_id": 0,
        "target_source": "none",
        "target_reachability": "none",
        "target_revalidated": False,
        "reposition_result": "none",
        "x": float("nan"), "y": float("nan"), "z": float("nan"),
        "vx": float("nan"), "vy": float("nan"), "vz": float("nan"),
        "motor_mean": 0.0,
    }
    event_rows = []
    trace_rows = []
    last_trace_time = float("-inf")
    arm_events = []
    disarm_events = []
    landing_events = []
    approved_takeoff_events = []
    nonfinite = []
    topic_counts = {}
    max_motor_during_reposition = 0.0
    dual_owner_samples = 0
    invalid_takeoff_motor_samples = 0
    model_count_min = None
    model_count_max = None
    done_time = None
    start_time = None
    end_time = None
    previous = {}

    def add_event(t, event):
        event_rows.append({
            "sim_time_s": "%.6f" % t,
            "event": event,
            **{key: latest[key] for key in (
                "requested_mode", "approved_mode", "mission_state",
                "handoff_state", "ground_owner", "ground_active",
                "motor_enable", "disarmed", "site_measurement_valid",
                "site_valid", "site_reason", "persistent_site_count",
                "target_site_id", "target_source", "target_reachability",
                "target_revalidated", "reposition_result",
            )},
            "site_measurement_age_s": (
                "%.6f" % max(0.0, t - latest["site_stamp"])
                if math.isfinite(latest["site_stamp"]) else "nan"),
            "x_m": "%.6f" % latest["x"],
            "y_m": "%.6f" % latest["y"],
            "z_m": "%.6f" % latest["z"],
            "vx_m_s": "%.6f" % latest["vx"],
            "vy_m_s": "%.6f" % latest["vy"],
            "vz_m_s": "%.6f" % latest["vz"],
            "actual_motor_mean_abs_rad_s": "%.6f" % latest["motor_mean"],
        })

    topics = [
        "/rescue/requested_mode", "/rescue/mode", "/rescue/mission_state",
        "/rescue/rotors/handoff_state", "/rescue/rotors/ground_ownership",
        "/rescue/rotors/ground_control_active", "/rescue/rotors/motor_enable",
        "/rescue/rotors/disarmed", "/rescue/rotors/takeoff_site_status",
        "/rescue/rotors/approved_takeoff_pose",
        "/rescue/rotors/persistent_safe_site_count",
        "/rescue/rotors/reposition_target_site_id",
        "/rescue/rotors/reposition_target_source",
        "/rescue/rotors/reposition_target_reachability",
        "/rescue/rotors/reposition_target_revalidated",
        "/rescue/rotors/reposition_result",
        "/rescue/rotors/reposition_active",
        "/uav_v4/odometry", "/uav_v4/imu", "/uav_v4/motor_speed",
        "/uav_v4/command/motor_speed", "/gazebo/model_states",
    ]

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            t = stamp.to_sec()
            start_time = t if start_time is None else start_time
            end_time = t
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
            event = None

            if topic == "/rescue/requested_mode":
                latest["requested_mode"] = msg.data
                if previous.get(topic) != msg.data:
                    event = "requested_mode:%s" % msg.data
                previous[topic] = msg.data
            elif topic == "/rescue/mode":
                latest["approved_mode"] = msg.data
                if previous.get(topic) != msg.data:
                    event = "approved_mode:%s" % msg.data
                previous[topic] = msg.data
            elif topic == "/rescue/mission_state":
                latest["mission_state"] = msg.data
                if previous.get(topic) != msg.data:
                    event = "mission_state:%s" % msg.data
                previous[topic] = msg.data
                if msg.data == "DONE" and done_time is None:
                    done_time = t
            elif topic == "/rescue/rotors/handoff_state":
                old_handoff = previous.get(topic)
                latest["handoff_state"] = msg.data
                if old_handoff != msg.data:
                    event = "handoff:%s" % msg.data
                    if (msg.data == "LAND_VERTICAL"
                            and old_handoff != "TOUCHDOWN_SETTLE"):
                        landing_events.append(t)
                previous[topic] = msg.data
            elif topic == "/rescue/rotors/ground_ownership":
                latest["ground_owner"] = bool(msg.data)
            elif topic == "/rescue/rotors/ground_control_active":
                latest["ground_active"] = bool(msg.data)
            elif topic == "/rescue/rotors/motor_enable":
                old = previous.get(topic, False)
                latest["motor_enable"] = bool(msg.data)
                if not old and msg.data:
                    site_age = t - latest["site_stamp"]
                    arm_events.append({
                        "time": t,
                        "site_measurement_valid": latest["site_measurement_valid"],
                        "site_valid": latest["site_valid"],
                        "site_age_s": site_age,
                        "handoff_state": latest["handoff_state"],
                    })
                    event = "motor_enable:true"
                elif old and not msg.data:
                    event = "motor_enable:false"
                previous[topic] = bool(msg.data)
            elif topic == "/rescue/rotors/disarmed":
                old = previous.get(topic, True)
                latest["disarmed"] = bool(msg.data)
                if not old and msg.data:
                    disarm_events.append(t)
                    event = "disarmed:true"
                previous[topic] = bool(msg.data)
            elif topic == "/rescue/rotors/takeoff_site_status":
                values = diagnostic_values(msg)
                latest["site_measurement_valid"] = (
                    values.get("measurement_valid") == "true")
                latest["site_valid"] = values.get("takeoff_site_valid") == "true"
                latest["site_reason"] = msg.status[0].message if msg.status else "missing"
                latest["site_stamp"] = msg.header.stamp.to_sec()
            elif topic == "/rescue/rotors/approved_takeoff_pose":
                approved_takeoff_events.append({
                    "time": t, "x": msg.pose.position.x,
                    "y": msg.pose.position.y, "z": msg.pose.position.z,
                    "site_measurement_valid": latest["site_measurement_valid"],
                    "site_valid": latest["site_valid"],
                    "site_age_s": t - latest["site_stamp"],
                })
                event = "approved_takeoff_pose"
            elif topic == "/rescue/rotors/persistent_safe_site_count":
                latest["persistent_site_count"] = int(msg.data)
            elif topic == "/rescue/rotors/reposition_target_site_id":
                latest["target_site_id"] = int(msg.data)
            elif topic == "/rescue/rotors/reposition_target_source":
                latest["target_source"] = msg.data
            elif topic == "/rescue/rotors/reposition_target_reachability":
                latest["target_reachability"] = msg.data
                if previous.get(topic) != msg.data:
                    event = "target_reachability:%s" % msg.data
                previous[topic] = msg.data
            elif topic == "/rescue/rotors/reposition_target_revalidated":
                latest["target_revalidated"] = bool(msg.data)
            elif topic == "/rescue/rotors/reposition_result":
                latest["reposition_result"] = msg.data
                if previous.get(topic) != msg.data:
                    event = "reposition_result:%s" % msg.data
                previous[topic] = msg.data
            elif topic == "/rescue/rotors/reposition_active":
                previous[topic] = bool(msg.data)
            elif topic == "/uav_v4/odometry":
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                v = msg.twist.twist.linear
                w = msg.twist.twist.angular
                values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                          v.x, v.y, v.z, w.x, w.y, w.z]
                if not finite(values):
                    nonfinite.append([t, topic])
                latest.update({"x": p.x, "y": p.y, "z": p.z,
                               "vx": v.x, "vy": v.y, "vz": v.z})
                # A deterministic 10 Hz paper trace derived from the frozen bag.
                # Event timing remains in R2_EVENT_TIMELINE.csv at native edges.
                if t - last_trace_time >= 0.099:
                    trace_rows.append({
                        "sim_time_s": "%.6f" % t,
                        "x_m": "%.6f" % p.x,
                        "y_m": "%.6f" % p.y,
                        "z_m": "%.6f" % p.z,
                        "vx_m_s": "%.6f" % v.x,
                        "vy_m_s": "%.6f" % v.y,
                        "vz_m_s": "%.6f" % v.z,
                        "requested_mode": latest["requested_mode"],
                        "approved_mode": latest["approved_mode"],
                        "mission_state": latest["mission_state"],
                        "handoff_state": latest["handoff_state"],
                        "ground_owner": latest["ground_owner"],
                        "ground_active": latest["ground_active"],
                        "motor_enable": latest["motor_enable"],
                        "disarmed": latest["disarmed"],
                        "site_measurement_valid": latest["site_measurement_valid"],
                        "site_valid": latest["site_valid"],
                        "persistent_site_count": latest["persistent_site_count"],
                        "target_site_id": latest["target_site_id"],
                        "target_source": latest["target_source"],
                        "target_reachability": latest["target_reachability"],
                        "target_revalidated": latest["target_revalidated"],
                        "reposition_result": latest["reposition_result"],
                        "actual_motor_mean_abs_rad_s": "%.6f" % latest["motor_mean"],
                    })
                    last_trace_time = t
            elif topic == "/uav_v4/imu":
                q = msg.orientation
                w = msg.angular_velocity
                a = msg.linear_acceleration
                if not finite([q.x, q.y, q.z, q.w, w.x, w.y, w.z,
                               a.x, a.y, a.z]):
                    nonfinite.append([t, topic])
            elif topic in ("/uav_v4/motor_speed", "/uav_v4/command/motor_speed"):
                values = list(msg.angular_velocities)
                if not finite(values):
                    nonfinite.append([t, topic])
                if topic == "/uav_v4/motor_speed":
                    latest["motor_mean"] = mean_abs(values)
            elif topic == "/gazebo/model_states":
                values = []
                for pose, twist in zip(msg.pose, msg.twist):
                    values.extend([pose.position.x, pose.position.y, pose.position.z,
                                   pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w,
                                   twist.linear.x, twist.linear.y, twist.linear.z,
                                   twist.angular.x, twist.angular.y, twist.angular.z])
                if not finite(values):
                    nonfinite.append([t, topic])
                count = msg.name.count("uav_v4")
                model_count_min = count if model_count_min is None else min(model_count_min, count)
                model_count_max = count if model_count_max is None else max(model_count_max, count)

            reposition_active = previous.get("/rescue/rotors/reposition_active", False)
            if reposition_active:
                max_motor_during_reposition = max(
                    max_motor_during_reposition, latest["motor_mean"])
            if latest["ground_active"] and latest["motor_mean"] > 5.0:
                dual_owner_samples += 1
            if (latest["handoff_state"] in (
                    "TAKEOFF_VERTICAL", "TAKEOFF_CLEARANCE_CONFIRM")
                    and latest["motor_enable"] and not latest["site_valid"]):
                invalid_takeoff_motor_samples += 1
            if event:
                add_event(t, event)

    landing_audit = []
    for landing_time in landing_events:
        later = [time for time in disarm_events if time >= landing_time]
        landing_audit.append({
            "landing_time": landing_time,
            "disarm_time": min(later) if later else None,
            "completed_disarm": bool(later),
        })

    success = int(done_time is not None)
    timeout = int(done_time is None or done_time - start_time >= 120.0)
    acceptance = {
        "mission_done": success == 1,
        "success_1": success == 1,
        "timeout_0": timeout == 0,
        "all_takeoffs_fresh_site_valid": bool(arm_events) and all(
            item["site_measurement_valid"] and item["site_valid"]
            and item["site_age_s"] <= 0.15 for item in arm_events),
        "all_landings_disarmed": bool(landing_audit) and all(
            item["completed_disarm"] for item in landing_audit),
        "final_motor_zero": latest["motor_mean"] <= 5.0,
        "final_disarmed": latest["disarmed"],
        "finite": not nonfinite,
        "one_uav_v4": model_count_min == 1 and model_count_max == 1,
        "no_dual_ownership": dual_owner_samples == 0,
        "no_invalid_site_takeoff_motor": invalid_takeoff_motor_samples == 0,
        "motor_zero_during_reposition": max_motor_during_reposition <= 5.0,
    }
    summary = {
        "bag": str(bag_path),
        "bag_start_sim_s": start_time,
        "bag_end_sim_s": end_time,
        "done_sim_s": done_time,
        "success": success,
        "timeout": timeout,
        "final_mission_state": latest["mission_state"],
        "final_handoff_state": latest["handoff_state"],
        "final_actual_motor_mean_abs_rad_s": latest["motor_mean"],
        "persistent_safe_site_count_final": latest["persistent_site_count"],
        "arm_events": arm_events,
        "approved_takeoff_events": approved_takeoff_events,
        "landing_audit": landing_audit,
        "model_count_min": model_count_min,
        "model_count_max": model_count_max,
        "max_motor_during_reposition_rad_s": max_motor_during_reposition,
        "dual_owner_samples": dual_owner_samples,
        "invalid_takeoff_motor_samples": invalid_takeoff_motor_samples,
        "nonfinite": nonfinite,
        "topic_counts": topic_counts,
        "acceptance": acceptance,
    }

    with (output / "R2_BAG_AUDIT.json").open("w") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with (output / "R2_MISSION_SUMMARY.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "success", "timeout", "done_sim_s", "final_mission_state",
            "final_handoff_state", "takeoff_count", "landing_count",
            "all_takeoffs_fresh_site_valid", "all_landings_disarmed",
            "final_motor_mean_abs_rad_s", "final_disarmed", "finite",
            "one_uav_v4", "dual_owner_samples",
            "invalid_takeoff_motor_samples", "max_motor_during_reposition_rad_s",
            "persistent_safe_site_count_final",
        ])
        writer.writeheader()
        writer.writerow({
            "success": success, "timeout": timeout, "done_sim_s": done_time,
            "final_mission_state": latest["mission_state"],
            "final_handoff_state": latest["handoff_state"],
            "takeoff_count": len(arm_events), "landing_count": len(landing_audit),
            "all_takeoffs_fresh_site_valid": int(
                acceptance["all_takeoffs_fresh_site_valid"]),
            "all_landings_disarmed": int(acceptance["all_landings_disarmed"]),
            "final_motor_mean_abs_rad_s": latest["motor_mean"],
            "final_disarmed": int(latest["disarmed"]),
            "finite": int(acceptance["finite"]),
            "one_uav_v4": int(acceptance["one_uav_v4"]),
            "dual_owner_samples": dual_owner_samples,
            "invalid_takeoff_motor_samples": invalid_takeoff_motor_samples,
            "max_motor_during_reposition_rad_s": max_motor_during_reposition,
            "persistent_safe_site_count_final": latest["persistent_site_count"],
        })
    with (output / "R2_EVENT_TIMELINE.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(event_rows[0]))
        writer.writeheader()
        writer.writerows(event_rows)
    with (output / "R2_PAPER_TRACE_10HZ.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(trace_rows[0]))
        writer.writeheader()
        writer.writerows(trace_rows)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
