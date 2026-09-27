#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
CATKIN_WS_ROOT="${CATKIN_WS:-$HOME/catkin_ws}"
source "$CATKIN_WS_ROOT/devel/setup.bash"
set -u

OUTPUT_ROOT=${1:?output directory required}
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
for case_name in head_on oblique corner; do
  x=-1.0; y=0.0; yaw=0.0
  if [ "${case_name}" = oblique ]; then
    y=-0.30; yaw=0.174532925
  elif [ "${case_name}" = corner ]; then
    y=0.65; yaw=0.34906585
  fi
  output="${OUTPUT_ROOT}/${case_name}_r1.csv"
  timeout 20s roslaunch rescue_worlds ground_contact_microtest.launch \
    gui:=false headless:=true output_csv:="${output}" \
    x:="${x}" y:="${y}" z:=0.05 yaw:="${yaw}" \
    duration:=4 settle:=0.5 vx:=0.45 \
    >"${OUTPUT_ROOT}/${case_name}_r1.log" 2>&1
  status=$?
  cleanup_ros
  echo "${case_name} status=${status}"
  if [ "${status}" -ne 0 ]; then
    exit "${status}"
  fi
done
