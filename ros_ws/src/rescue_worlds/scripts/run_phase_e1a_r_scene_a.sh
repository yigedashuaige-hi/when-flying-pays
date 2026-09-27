#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
CATKIN_WS_ROOT="${CATKIN_WS:-$HOME/catkin_ws}"
source "$CATKIN_WS_ROOT/devel/setup.bash"
set -u

OUTPUT_ROOT=${1:?output directory required}
mkdir -p "${OUTPUT_ROOT}/runs"

cleanup_ros() {
  rosnode kill -a >/dev/null 2>&1 || true
  pkill -TERM -f 'rosbag record' >/dev/null 2>&1 || true
  pkill -TERM -f 'roslaunch' >/dev/null 2>&1 || true
  pkill -TERM -f 'rosmaster' >/dev/null 2>&1 || true
  pkill -TERM -f 'gzserver' >/dev/null 2>&1 || true
  pkill -TERM -f 'rosout' >/dev/null 2>&1 || true
  sleep 1
}

cleanup_ros
timeout --signal=INT --kill-after=10s 140s roslaunch rescue_mission rescue_phase1_demo.launch \
  gui:=false headless:=true strategy:=fixed_switch scene:=scene_a seed:=20260720 \
  mission_timeout:=120 runs_dir:="${OUTPUT_ROOT}/runs" \
  latest_csv_path:="${OUTPUT_ROOT}/noncanonical_latest.csv" \
  stuck_model:=recoverable air_backend:=rotors \
  >"${OUTPUT_ROOT}/roslaunch.log" 2>&1 &
launch_pid=$!
sleep 3
timeout --signal=INT --kill-after=5s 132s rosbag record \
  -O "${OUTPUT_ROOT}/scene_a_e1a_r.bag" \
  /clock /rescue/requested_mode /rescue/mode /rescue/mission_state \
  /rescue/rotors/handoff_state /rescue/rotors/ground_ownership \
  /rescue/rotors/motor_enable /rescue/rotors/disarmed \
  /rescue/rotors/takeoff_site_status /rescue/rotors/takeoff_clearance \
  /rescue/rotors/takeoff_site_valid /rescue/rotors/takeoff_decision_reason \
  /rescue/rotors/takeoff_blocked_duration_s \
  /rescue/rotors/takeoff_blocked_distance_m \
  /rescue/rotors/approved_takeoff_pose /rescue/rotors/obstacle_contact \
  /rescue/rotors/obstacle_contact_force_n \
  /rescue/rotors/reposition_active /rescue/rotors/reposition_goal \
  /rescue/rotors/reposition_duration_s /rescue/rotors/reposition_distance_m \
  /rescue/rotors/reposition_initial_distance_m \
  /rescue/rotors/reposition_target_history_age_s \
  /rescue/rotors/reposition_result \
  /rescue/rotors/reposition_site_restored_pose \
  /mecanum/cmd_vel /uav_v4/command/pose \
  /uav_v4/command/motor_speed /uav_v4/motor_speed /uav_v4/odometry \
  >"${OUTPUT_ROOT}/rosbag.log" 2>&1 &
bag_pid=$!
wait "${launch_pid}"
launch_status=$?
kill -INT "${bag_pid}" >/dev/null 2>&1 || true
wait "${bag_pid}" >/dev/null 2>&1 || true
cleanup_ros
echo "scene_a_launch_status=${launch_status}"
