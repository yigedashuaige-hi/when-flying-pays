#!/bin/bash
# Overnight paper-hardening batch: 20-seed Scene B/C/D main comparisons + Scene B/C
# ablations, then significance tests + figures. Robust to per-run failure (set +e);
# every run writes its CSV immediately and summaries regenerate at the end of each
# group, so partial progress survives an interruption. Results go to a fresh
# timestamped dir under results/runs_paper_hardening_<ts>/ (originals untouched).
#
# Run inside the uav_noetic container:
#   docker exec uav_noetic bash ${CATKIN_WS:-$HOME/catkin_ws}/src/rescue_mission/scripts/run_overnight_batch.sh
set +e
source /opt/ros/noetic/setup.bash
CATKIN_WS_ROOT="${CATKIN_WS:-$HOME/catkin_ws}"
source "$CATKIN_WS_ROOT/devel/setup.bash"
cd "$CATKIN_WS_ROOT/src/rescue_mission" || exit 1

NSEEDS="${NSEEDS:-20}"
TS=$(date +%Y%m%d_%H%M%S)
# Absolute paths: the logger node resolves --runs-dir against its own CWD (~/.ros),
# not the package, so a relative path would scatter CSVs into /root/.ros. Always
# pass absolute paths through to the runner/launch.
PKG="$CATKIN_WS_ROOT/src/rescue_mission"
PH="$PKG/results/runs_paper_hardening_${TS}"
RES="$PKG/results"
mkdir -p "$PH"
echo "$PH" > /tmp/overnight_ph_dir.txt
echo "============================================================"
echo " Overnight batch ${TS}  seeds=${NSEEDS}  outdir=${PH}"
echo "============================================================"

run_group() {  # scene strategies duration mission_timeout ablation tag
  local scene="$1" strat="$2" dur="$3" mto="$4" abl="$5" tag="$6"
  echo ""
  echo ">>> [$(date +%H:%M:%S)] scene=$scene strat=[$strat] ablation=$abl tag=$tag"
  rosrun rescue_mission run_phase1_experiments.py \
    --scene "$scene" --strategies $strat --num-seeds "$NSEEDS" --base-seed 0 --runs 1 \
    --duration "$dur" --cooldown 2 --mission-timeout "$mto" --ablation "$abl" \
    --runs-dir "$PH/$tag" \
    --summary-path "$RES/summary_ph_${tag}.csv" --plot
}

# ── Main comparisons ────────────────────────────────────────────────────────
run_group scene_b "energy_rule risk_aware"        130 85 none scene_b
run_group scene_c "energy_rule risk_aware"        130 85 none scene_c
run_group scene_d "risk_aware risk_aware_safety"  110 80 none scene_d

# ── Ablations (risk_aware variants) on Scene B and C ────────────────────────
for sc in scene_b scene_c; do
  for abl in w_o_uncertainty w_o_cvar w_o_switch_penalty; do
    run_group "$sc" "risk_aware" 130 85 "$abl" "${sc}_${abl}"
  done
done

# ── Significance + figures ──────────────────────────────────────────────────
echo ""
echo "=== [$(date +%H:%M:%S)] significance + figures ==="
for sc in scene_b scene_c; do
  rosrun rescue_mission analyze_significance.py "$RES/all_runs_ph_${sc}.csv" \
    --scene "$sc" --treatment risk_aware --baseline energy_rule \
    --output "$RES/significance_ph_${sc}.txt"
  rosrun rescue_mission plot_paper_figures.py "$RES/summary_ph_${sc}.csv" \
    --output-dir "$RES/figures_ph_${sc}" --title-prefix "$sc (${NSEEDS} seeds)"
done
rosrun rescue_mission analyze_significance.py "$RES/all_runs_ph_scene_d.csv" \
  --scene scene_d --treatment risk_aware_safety --baseline risk_aware \
  --output "$RES/significance_ph_scene_d.txt"
rosrun rescue_mission plot_paper_figures.py "$RES/summary_ph_scene_d.csv" \
  --output-dir "$RES/figures_ph_scene_d" --title-prefix "scene_d (${NSEEDS} seeds)"

echo ""
echo "============================================================"
echo " Overnight batch DONE ${TS} -> ${PH}"
echo "============================================================"
