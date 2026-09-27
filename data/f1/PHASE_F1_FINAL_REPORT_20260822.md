# Phase F1 Final Report: Equal-Budget Tuning and Fresh Held-Out Evaluation

**Completed:** 2026-08-22  
**Backend:** `ideal_set_model_state` only  
**Scenes:** B/C, `stuck_model=recoverable`  
**Status:** complete; stop after Phase F1

## 1. Fair protocol

The preregistration was written and hash-frozen before a Phase F1 simulation.
Both policy families received the same rights in each scene: five
one-dimensional candidates, ten identical paired tuning seeds per candidate,
one run per cell, the same lexicographic selection rule, and scene-specific
selection. This is 50 runs per family/scene and 100 per family overall. The
held-out comparison used 20 new paired seeds per scene. `fixed_switch` was a
reference only and did not participate in tuning.

The inference is deliberately limited to the preregistered one-dimensional
families. It is not an exhaustive optimization of every CVaR or threshold
parameter.

### Candidate spaces

- threshold: `tau={0.2,0.3,0.4,0.5,0.6}`;
- risk-aware: `wcvar={0.25,0.50,0.75,1.00,1.25}`;
- frozen risk parameters: `alpha=0.8`, `k_unc=1.0`, `C_stuck=60`,
  `switch_penalty=2`, and all existing physical-cost/score weights.

The threshold range reproduces the historical grid. The risk-aware grid is a
uniform range centered on the existing 0.75 default; it excludes the
CVaR-removing zero ablation and the unrelated Scene-D stress value 2.5.

### Seeds and selection

- tuning: 100--109;
- untouched held-out evaluation: 200--219;
- historical scan: only `-1`, 0--19, and 20260720 were found in parseable
  seed-bearing result CSVs, so both new sets were unused.

Selection first maximized success count, then minimized successful-run median
energy, air distance, and switch count; remaining ties used distance from the
historical default and then the lower numeric value. Failures/timeouts were
never removed. The winner file was frozen before evaluation at SHA-256
`27c093b7a05fe6ea3b12c5ec5d25d14fe1d93cfb197987b62a84364c4c4ec46e`.

## 2. Tuning results and selected parameters

| Scene | Family | Selected | Tuning success | Successful median energy | Successful median air distance | Successful median switches |
|---|---|---:|---:|---:|---:|---:|
| B | threshold | tau=0.20 | 10/10 | 54.8025 J | 9.9045 m | 10 |
| B | risk-aware | wcvar=1.25 | 10/10 | 57.5395 J | 15.7679 m | 8 |
| C | threshold | tau=0.40 | 10/10 | 77.2770 J | 11.4567 m | 16 |
| C | risk-aware | wcvar=1.00 | 10/10 | 62.4218 J | 18.3792 m | 8 |

All 200 tuning runs are retained, including losing candidates. Actual budgets
were exactly equal: 100 valid tuning runs for threshold and 100 for risk-aware.

## 3. Untouched held-out results

Differences below are risk-aware minus threshold. Energy and other continuous
confirmatory comparisons use only paired seeds on which both methods
succeeded, as preregistered. This avoids calling an early-failing low-energy
run efficient. A sortie is counted as one transition into `TAKEOFF`, rather
than equating the raw mode-switch count with a flight.

### Scene B

- success: threshold 20/20, risk-aware 20/20; no discordant pairs, exact
  McNemar p=1.0;
- energy (20 joint successes): mean difference -2.339 J, median -3.338 J,
  bootstrap 95% CI for the mean [-9.112, 3.815] J, Wilcoxon p=0.9273,
  matched rank-biserial effect=0.0286;
- air distance: +5.482 m, CI [4.703, 6.210], p=1.907e-6,
  rank-biserial=1.0;
- sorties: -0.80, CI [-1.20, -0.45], p=0.0009766,
  rank-biserial=-1.0;
- slip ratio: +0.07387, CI [0.06015, 0.08780], p=1.907e-6,
  rank-biserial=1.0;
- stuck events: no difference (zero for both on all joint-success pairs).

Thus Scene B supports comparable success and no detected energy difference,
but different mechanisms: risk-aware flies farther and records more slip while
using fewer sorties.

### Scene C

- success: threshold 16/20, risk-aware 20/20; four risk-only successes and no
  threshold-only success, exact McNemar p=0.125;
- threshold failed seeds 204, 207, 209, and 212; none was removed;
- energy (16 joint successes): mean difference -11.267 J, median -14.618 J,
  bootstrap 95% CI [-14.002, -8.085] J, Wilcoxon p=0.0002136,
  matched rank-biserial=-0.9412;
- air distance: +7.161 m, CI [6.919, 7.540], p=3.052e-5,
  rank-biserial=1.0;
- sorties: -1.75, CI [-1.938, -1.50], p=3.052e-5,
  rank-biserial=-1.0;
- slip ratio: +0.19986, CI [0.17744, 0.22161], p=3.052e-5,
  rank-biserial=1.0;
- stuck events on the 16 joint successes: zero for both. Across all seeds,
  threshold averaged 14.15 stuck events because its four failed runs became
  stuck; risk-aware averaged zero.

The success point estimate favors risk-aware, but 20 pairs give a non-significant
exact McNemar p=0.125. On the prespecified joint-success subset, risk-aware used
less legacy task-proxy energy, while flying farther and recording more slip.

### Why the all-seed energy number is not the primary comparison

In Scene C, threshold's all-seed mean is 60.661 J versus risk-aware's 62.519 J,
but threshold's successful-run mean is 73.752 J and its four early failures
reduce the all-seed mean. The all-seed value is therefore descriptive only and
cannot be interpreted as threshold being more efficient.

## 4. Fixed-switch reference

| Scene | Success | Mean energy | Mean air distance | Mean sorties | Mean stuck events | Mean slip ratio |
|---|---:|---:|---:|---:|---:|---:|
| B | 20/20 | 48.248 J | 9.928 m | 2.0 | 0.00 | 0.1174 |
| C | 20/20 | 70.962 J | 13.029 m | 3.0 | 8.05 | 0.3620 |

This baseline remains strong, especially in Scene B. It was not tuned and its
comparisons are descriptive, not part of the confirmatory family selection.

## 5. Interpretation of the earlier CVaR negative result

The earlier broad negative story must be **materially rewritten**. It remains
accurate to say that the historical *fixed, untuned* two-point-CVaR
implementation did not outperform tuned/simple switching in the old ideal
experiments. It is no longer accurate to imply that a fairly tuned risk-aware
family is generally no better than a fairly tuned threshold family.

Under this equal-budget experiment, the two selected methods are close on
Scene B success and energy, while selected risk-aware has a favorable Scene C
success point estimate and significantly lower energy among joint successes.
The benefit comes with greater air distance and greater logged slip, and
fixed periodic switching remains competitive. These tradeoffs replace a
one-dimensional winner/loser claim.

The most accurate paper statement is:

> Under equal five-candidate, scene-specific one-dimensional tuning on fresh
> seeds, selected risk-aware and threshold switching both achieved 20/20 in
> Scene B with no detected joint-success energy difference. In Scene C,
> selected risk-aware achieved 20/20 versus 16/20 (exact McNemar p=0.125) and
> used less legacy task-proxy energy on the 16 joint-success pairs (mean paired
> difference -11.27 J; Wilcoxon p=0.000214), while flying farther and recording
> more slip. These ideal-backend results do not establish exhaustive CVaR
> optimality, fixed-threshold equivalence, RotorS performance, or hardware
> safety.

## 6. Integrity, deviations, and limitations

- Backend was `ideal` and energy model was
  `legacy_task_distance_hover_proxy_v1` on every valid row checked by the
  runner. No RotorS process or experiment was started.
- A transparent pre-outcome amendment excluded NaN/Inf sentinel fields that
  are structurally inapplicable to ideal runs; every common F1 field remained
  required and finite. Original preregistration and failed preflight evidence
  are retained.
- Three ROS-master port binds failed before simulation/CSV creation. Each
  original log/metadata was retained and the identical cell was rerun once on
  a new isolated port. No seed or launch input changed.
- One empty orphan tuning rosmaster was found after held-out completion. It had
  no Gazebo/task nodes, used a port disjoint from all held-out masters, and
  could not exchange topics across ROS masters; it was recorded and killed
  before analysis. No data contamination was found.
- The first analysis attempt retained null Wilcoxon p-values because SciPy
  1.5 names the compatibility argument `mode`, not `method`. The attempt is
  retained; the same tests were rerun with the compatible API.
- Scope is two scenes, recoverable stuck physics, 10 tuning and 20 evaluation
  seeds, scene-specific tuning, and two five-point one-dimensional grids.
  RotorS, battery energy, other regimes, and a multidimensional policy search
  are outside the claim.

Stage 0 canonical data and all frozen D2 anchors were hash-checked unchanged.

## 7. Evidence paths

All paths below are relative to `catkin_ws/src/rescue_mission/results/phase_f1_fair_tuning_20260821/`:

- protocol: `PHASE_F1_PREREGISTRATION_20260821.md`, amendment, seed audit, and
  `configs/`;
- all tuning/evaluation raw CSVs: `runs/`;
- exact commands, final rows, CSV hashes, and validation: `metadata/`;
- launch logs: `logs/`;
- complete tuning table: `tuning_candidate_summary.csv`;
- winner freeze: `SELECTED_CONFIGS_BEFORE_EVALUATION.json`;
- held-out per-run table: `heldout_run_summary.csv`;
- statistics: `heldout_paired_statistics.json`;
- figures: `plots/scene_{b,c}_heldout.png`;
- technical-invalid retained attempts: `invalid_preflight_schema_sentinel/`
  and `technical_invalid_port_collision*`;
- complete integrity listing: `SHA256_MANIFEST.txt`.

Experiment scripts are
`rescue_mission/scripts/{run_phase_f1_fair.py,select_phase_f1_configs.py,analyze_phase_f1.py}`.
