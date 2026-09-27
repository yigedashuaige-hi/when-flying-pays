#!/bin/bash
# Active-recoverability pilot (pre-registered). 3 conditions x 4 battery levels x
# 5 seeds on Scene E. Absolute paths. Frozen per the pre-registration; no tuning.
set +e
source /opt/ros/noetic/setup.bash
CATKIN_WS_ROOT="${CATKIN_WS:-$HOME/catkin_ws}"
source "$CATKIN_WS_ROOT/devel/setup.bash"
PKG="$CATKIN_WS_ROOT/src/rescue_mission"
cd "$PKG" || exit 1
PH="$PKG/results/runs_pilot_scene_e"
mkdir -p "$PH"
echo "Active-recoverability pilot -> $PH"
for BATT in 0.55 0.65 0.75 0.85; do
  for STRAT in air_preferred risk_aware_safety risk_aware_active; do
    TAG="b${BATT}_${STRAT}"
    echo ">>> [$(date +%H:%M:%S)] scene_e $STRAT initial_battery=$BATT"
    rosrun rescue_mission run_phase1_experiments.py \
      --scene scene_e --strategies "$STRAT" --num-seeds 5 --base-seed 0 --runs 1 \
      --duration 150 --cooldown 2 --mission-timeout 110 --initial-battery "$BATT" \
      --runs-dir "$PH/$TAG" \
      --summary-path "$PKG/results/summary_pilot_${TAG}.csv"
  done
done
echo "=== pilot DONE ==="
