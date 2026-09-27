#!/bin/bash
# Make the DOWNGRADE verdict airtight (pre-registered 2026-06-11). Two batches,
# recoverable-slip stuck model only (the realistic regime). Nothing tuned to make
# risk_aware win except threshold_switch's tau. Absolute paths everywhere.
#
#  BATCH 1 (sparse corner): the ONE untested place Claim A could be NARROW —
#    expensive flight (--fly-k1 3.0) + sparse hazard (scene_sparse: a single
#    narrow risky band, no slope) + recoverable stuck. If risk-based selectivity
#    is going to beat a tuned threshold anywhere, it's here.
#  BATCH 2 (robustness): expand the recoverable-regime comparison from 5 seeds /
#    Scene B to 20 seeds on Scene B AND Scene C, so DOWNGRADE is statistically
#    and scene-wise solid.
set +e
source /opt/ros/noetic/setup.bash
CATKIN_WS_ROOT="${CATKIN_WS:-$HOME/catkin_ws}"
source "$CATKIN_WS_ROOT/devel/setup.bash"
PKG="$CATKIN_WS_ROOT/src/rescue_mission"
cd "$PKG" || exit 1
OUT="$PKG/results/runs_claim_a_airtight"
mkdir -p "$OUT"
echo "Claim A airtight batches -> $OUT  (start $(date))"

# run <scene> <stuck> <strategy> <tau> <fly_k1> <seeds> <tag>
run() {
  local scene="$1" sm="$2" strat="$3" tau="$4" flyk1="$5" seeds="$6" tag="$7"
  echo ">>> [$(date +%H:%M:%S)] $scene stuck=$sm $strat tau=$tau fly_k1=$flyk1 seeds=$seeds"
  rosrun rescue_mission run_phase1_experiments.py \
    --scene "$scene" --strategies "$strat" --num-seeds "$seeds" --base-seed 0 --runs 1 \
    --duration 130 --cooldown 2 --mission-timeout 85 \
    --stuck-model "$sm" --switch-tau "$tau" --fly-k1 "$flyk1" \
    --runs-dir "$OUT/${scene}_${sm}_${tag}" \
    --summary-path "$PKG/results/summary_airtight_${scene}_${sm}_${tag}.csv"
}

# ---- BATCH 1: sparse-hazard + expensive-flight corner (recoverable), 10 seeds ----
# Set SKIP_BATCH1=1 to resume only BATCH 2 (BATCH 1 already complete 2026-06-11).
SC_SEEDS="${SC_SEEDS:-10}"
FLYK1_EXP="${FLYK1_EXP:-3.0}"
if [ "${SKIP_BATCH1:-0}" = "1" ]; then
  echo "=== SKIP_BATCH1=1 -> BATCH 1 skipped (already complete) ==="
else
echo "=== BATCH 1: scene_sparse, recoverable, fly_k1=$FLYK1_EXP, $SC_SEEDS seeds ==="
run scene_sparse recoverable energy_rule      -1  "$FLYK1_EXP" "$SC_SEEDS" energy_rule
run scene_sparse recoverable fixed_switch      -1  "$FLYK1_EXP" "$SC_SEEDS" fixed_switch
run scene_sparse recoverable risk_aware        -1  "$FLYK1_EXP" "$SC_SEEDS" risk_aware
for TAU in 0.2 0.3 0.4 0.5 0.6; do
  run scene_sparse recoverable threshold_switch "$TAU" "$FLYK1_EXP" "$SC_SEEDS" "threshold_t${TAU}"
done
fi  # end SKIP_BATCH1 guard (BATCH 1 only)

# ---- BATCH 2: DOWNGRADE robustness, default flight, Scene B + C, 20 seeds ----
RB_SEEDS="${RB_SEEDS:-20}"
echo "=== BATCH 2: scene_b + scene_c, recoverable, default flight, $RB_SEEDS seeds ==="
for SCENE in scene_b scene_c; do
  run "$SCENE" recoverable fixed_switch     -1  -1 "$RB_SEEDS" fixed_switch
  run "$SCENE" recoverable risk_aware       -1  -1 "$RB_SEEDS" risk_aware
  for TAU in 0.2 0.3 0.4; do
    run "$SCENE" recoverable threshold_switch "$TAU" -1 "$RB_SEEDS" "threshold_t${TAU}"
  done
done

echo "=== Claim A airtight batches DONE ($(date)) ==="
