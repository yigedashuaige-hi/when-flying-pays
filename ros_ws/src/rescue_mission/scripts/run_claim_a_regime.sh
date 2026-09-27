#!/bin/bash
# Claim A stress test (pre-registered 2026-06-11): risk_aware vs tuned
# threshold_switch + fixed_switch + energy_rule, under BOTH stuck models
# (permanent vs recoverable), on Scene B. Threshold tau is swept; nothing else is
# tuned to favour risk_aware. Absolute paths. Start small (SEEDS), expand later.
set +e
source /opt/ros/noetic/setup.bash
CATKIN_WS_ROOT="${CATKIN_WS:-$HOME/catkin_ws}"
source "$CATKIN_WS_ROOT/devel/setup.bash"
PKG="$CATKIN_WS_ROOT/src/rescue_mission"
cd "$PKG" || exit 1
SEEDS="${SEEDS:-5}"
SCENE="${SCENE:-scene_b}"
PH="$PKG/results/runs_claim_a_regime"
mkdir -p "$PH"
echo "Claim A regime sweep -> $PH  (seeds=$SEEDS scene=$SCENE)"

run() {  # stuck_model strategy tau tag
  local sm="$1" strat="$2" tau="$3" tag="$4"
  echo ">>> [$(date +%H:%M:%S)] $SCENE stuck=$sm $strat tau=$tau"
  rosrun rescue_mission run_phase1_experiments.py \
    --scene "$SCENE" --strategies "$strat" --num-seeds "$SEEDS" --base-seed 0 --runs 1 \
    --duration 130 --cooldown 2 --mission-timeout 85 \
    --stuck-model "$sm" --switch-tau "$tau" \
    --runs-dir "$PH/${SCENE}_${sm}_${tag}" \
    --summary-path "$PKG/results/summary_regime_${SCENE}_${sm}_${tag}.csv"
}

for SM in recoverable permanent; do
  run "$SM" energy_rule   -1   energy_rule
  run "$SM" fixed_switch  -1   fixed_switch
  run "$SM" risk_aware    -1   risk_aware
  for TAU in 0.2 0.3 0.4 0.5 0.6; do
    run "$SM" threshold_switch "$TAU" "threshold_t${TAU}"
  done
done
echo "=== Claim A regime sweep DONE ==="
