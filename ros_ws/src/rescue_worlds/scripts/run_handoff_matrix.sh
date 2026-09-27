#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
CATKIN_WS_ROOT="${CATKIN_WS:-$HOME/catkin_ws}"
source "$CATKIN_WS_ROOT/devel/setup.bash"
set -u

OUTPUT_ROOT=${1:?output directory required}
REPEATS=${2:-5}
SCENARIOS=${3:-"static moving yaw near_barrier"}

mkdir -p "${OUTPUT_ROOT}"

cleanup_ros() {
  rosnode kill -a >/dev/null 2>&1 || true
  pkill -TERM -f 'roslaunch' >/dev/null 2>&1 || true
  pkill -TERM -f 'rosmaster' >/dev/null 2>&1 || true
  pkill -TERM -f 'gzserver' >/dev/null 2>&1 || true
  pkill -TERM -f 'rosout' >/dev/null 2>&1 || true
  sleep 1
}

cleanup_ros
for scenario in ${SCENARIOS}; do
  run=1
  while [ "${run}" -le "${REPEATS}" ]; do
    run_dir="${OUTPUT_ROOT}/${scenario}/run_${run}"
    mkdir -p "${run_dir}"
    spawn_x=0.0
    spawn_yaw=0.0
    if [ "${scenario}" = near_barrier ]; then
      spawn_x=6.50
    fi
    if [ "${scenario}" = yaw ]; then
      spawn_yaw=0.15
    fi
    timeout 65s roslaunch rescue_worlds uav_v4_rotors_handoff_test.launch \
      gui:=false headless:=true spawn_x:="${spawn_x}" spawn_y:=0.0 spawn_yaw:="${spawn_yaw}" \
      >"${run_dir}/roslaunch.log" 2>&1 &
    launch_pid=$!
    sleep 2
    timeout 55s rosrun rescue_worlds uav_v4_rotors_handoff_test.py \
      --scenario "${scenario}" \
      --timeline "${run_dir}/timeline.csv" \
      --output "${run_dir}/result.json" \
      >"${run_dir}/driver.log" 2>&1
    status=$?
    kill -INT "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
    cleanup_ros
    echo "${scenario} run ${run}: status=${status}"
    if [ "${status}" -ne 0 ]; then
      exit "${status}"
    fi
    run=$((run + 1))
  done
done
