#!/bin/bash
# Task A: collect the 3 missing baselines (ground_only / air_preferred /
# fixed_switch) under the SAME homoscedastic seeded environment + seeds 0-19 as
# the existing energy_rule / risk_aware 20-seed data, so the success-vs-energy
# Pareto frontier is properly paired. Absolute paths (logger CWD lesson).
set +e
source /opt/ros/noetic/setup.bash
CATKIN_WS_ROOT="${CATKIN_WS:-$HOME/catkin_ws}"
source "$CATKIN_WS_ROOT/devel/setup.bash"
PKG="$CATKIN_WS_ROOT/src/rescue_mission"
cd "$PKG" || exit 1
NSEEDS=20
PH="$PKG/results/runs_paper_hardening_baselines"
mkdir -p "$PH"
echo "Baseline frontier collection -> $PH"
for SC in scene_b scene_c; do
  for STRAT in ground_only air_preferred fixed_switch; do
    echo ">>> [$(date +%H:%M:%S)] $SC / $STRAT (20 seeds)"
    rosrun rescue_mission run_phase1_experiments.py \
      --scene "$SC" --strategies "$STRAT" --num-seeds "$NSEEDS" --base-seed 0 --runs 1 \
      --duration 130 --cooldown 2 --mission-timeout 85 \
      --runs-dir "$PH/${SC}_${STRAT}" \
      --summary-path "$PKG/results/summary_ph_${SC}_${STRAT}.csv"
  done
done
echo "=== baseline frontier collection DONE ==="
