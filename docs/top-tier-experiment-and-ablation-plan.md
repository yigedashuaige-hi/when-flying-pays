# Top-Tier Experiment and Ablation Plan

This plan follows the top-tier oriented spec and avoids mixing tuning runs with
final paper runs.

## 1. Strategy Groups

| Strategy | Role | Current implementation |
|----------|------|------------------------|
| `ground_only` | Single-mode ground baseline | Implemented |
| `air_preferred` | Air-heavy baseline | Implemented |
| `fixed_switch` | Naive multimodal baseline | Implemented |
| `rule` | Energy-aware rule policy without explicit shield wrapper | Implemented |
| `energy_rule` | Fair baseline using shared traversability mean plus energy/time/risk mean terms | Implemented |
| `risk_aware` | Proposed policy using shared inputs plus uncertainty-aware two-point CVaR | Implemented |
| `risk_aware_safety` | `risk_aware` + decision-layer reachability safety supervisor (PR-4) | Implemented |
| `rule_energy_shield` | Rule policy with safety shield | Implemented |
| `rl` / `safe_rl` | Optional learning baselines | Future work |

## 1.1 Latest Valid Shield Check

Latest valid single run:

```text
scene = scene_a
strategy = rule_energy_shield
success = 1
timeout = 0
time = 55.600 s
total_energy = 46.084 J
ground_energy = 4.826 J
air_cruise_energy = 15.564 J
takeoff_energy = 11.429 J
land_energy = 12.265 J
switch_energy = 2.000 J
switch_count = 8
collision/stuck_count = 1
final_distance = 0.358 m
ground_distance = 10.543 m
air_distance = 7.952 m
```

Interpretation:
- The safety-shield stack now runs successfully.
- Transition-energy accounting is working.
- This is still a single run, so it is a validation check rather than a paper
  result.

## 2. Minimum Debug Experiment

Use this when the laptop is noisy or time is limited:

```bash
rosrun rescue_mission run_phase1_experiments.py \
  --scene scene_a \
  --strategies rule_energy_shield \
  --runs 1 \
  --duration 90 \
  --cooldown 0 \
  --plot
```

Purpose:
- Verify the new CSV fields are populated.
- Verify safety shield does not block normal completion.
- Verify figures are generated automatically.

After a run, validate the latest CSV:

```bash
rosrun rescue_mission validate_rescue_run.py
```

If it reports `INVALID: no /rescue/mode samples were recorded`, the mode
switching node crashed and the run must not be used for analysis.

## 3. Scene A Paper-Debug Experiment

Run when the machine is available:

```bash
rosrun rescue_mission run_phase1_experiments.py \
  --scene scene_a \
  --strategies ground_only air_preferred fixed_switch rule rule_energy_shield \
  --runs 5 \
  --duration 90 \
  --cooldown 1 \
  --summary-path ros_ws/src/rescue_mission/results/summary_scene_a_shield.csv \
  --plot
```

Purpose:
- Compare `rule` and `rule_energy_shield`.
- Check whether shield increases safety without excessive energy/time cost.
- Keep this separate from older `summary_scene_a_latest.csv` to avoid mixing
  pre-optimization and post-optimization runs.

## 4. Scene B/C Deferred Experiments

Run later:

```bash
rosrun rescue_mission run_phase1_experiments.py \
  --scene scene_b \
  --strategies ground_only air_preferred fixed_switch rule rule_energy_shield \
  --runs 5 \
  --duration 110 \
  --cooldown 1 \
  --summary-path ros_ws/src/rescue_mission/results/summary_scene_b.csv \
  --plot

rosrun rescue_mission run_phase1_experiments.py \
  --scene scene_c \
  --strategies ground_only air_preferred fixed_switch rule rule_energy_shield \
  --runs 5 \
  --duration 120 \
  --cooldown 1 \
  --summary-path ros_ws/src/rescue_mission/results/summary_scene_c.csv \
  --plot
```

## 5. Ablation Roadmap

First implement and run only the low-risk ablation groups:

| Ablation | Implementation idea | Status |
|----------|---------------------|--------|
| w/o shield | Compare `rule` vs `rule_energy_shield` | Ready |
| w/o switch penalty | Set `switch_energy_cost := 0.0` and compare | Config-ready, needs launch override |
| w/o energy | Disable battery/energy terms in rule | Future parameter flag |
| w/o terrain | Disable slope/obstacle hazard terms | Future parameter flag |
| full | `rule_energy_shield` with all terms | Ready |

Do not overbuild the ablation layer before Scene A shield data is checked.

## 7. PR-1 Shared Inputs Status

Implemented code changes:

- `scenario_sensor_sim.py` now publishes:
  - `/rescue/traversability_mean`
  - `/rescue/traversability_uncertainty`
  - `/rescue/terrain_cost`
  - `/rescue/visibility_confidence`
  - `/rescue/stuck_risk_prior`
  - existing `/rescue/obstacle_density`
- `rescue_metrics_logger.py` records those shared inputs as additive CSV columns.
- `check_shared_inputs.py` groups run CSVs by strategy and reports shared-input
  mean/std plus cross-strategy mean spread.

Pending PR-1 acceptance:

```bash
roslaunch rescue_mission rescue_phase1_demo.launch scene:=scene_a
rosrun rescue_mission validate_rescue_run.py
rosrun rescue_mission check_shared_inputs.py ros_ws/src/rescue_mission/results/runs --min-duration 30
```

Old runs generated before PR-1 do not contain these columns and will be skipped
by `check_shared_inputs.py`.

## 8. PR-2 Energy Rule / Risk-Aware Status

Implemented code changes:

- `mode_switcher.py` now supports:
  - `energy_rule`: uses `traversability_mean`, energy/time/stuck mean risk, and
    does not use `traversability_uncertainty`.
  - `risk_aware`: uses the same shared inputs plus
    `traversability_uncertainty` to compute two-point CVaR.
- `mode_switcher.py` publishes:
  - `/rescue/J_ground`
  - `/rescue/J_air`
  - `/rescue/risk_cvar`
  - `/rescue/energy_term`
  - `/rescue/switch_reason`
- `rescue_metrics_logger.py` records those values as additive CSV columns.
- `summarize_results.py` carries terminal `J_ground`, `J_air`, `risk_cvar`,
  and `energy_term` into per-run/summary outputs.

Pending PR-2 acceptance:

```bash
rosrun rescue_mission run_phase1_experiments.py \
  --scene scene_a \
  --strategies energy_rule risk_aware \
  --runs 1 \
  --duration 90 \
  --cooldown 0 \
  --summary-path ros_ws/src/rescue_mission/results/summary_pr2_scene_a.csv

rosrun rescue_mission check_shared_inputs.py \
  ros_ws/src/rescue_mission/results/runs \
  --scene scene_a \
  --strategies energy_rule risk_aware \
  --since <batch_start_timestamp> \
  --min-duration 0
```

Scene A is only a smoke test. The real PR-2 story still depends on Scene B:

```bash
rosrun rescue_mission run_phase1_experiments.py \
  --scene scene_b \
  --strategies energy_rule risk_aware \
  --runs 5 \
  --duration 110 \
  --cooldown 1 \
  --summary-path ros_ws/src/rescue_mission/results/summary_pr2_scene_b.csv
```

## 9. PR-3 Scene B Stuck/Slip Proxy Status

Implemented code changes:

- `ground_goal_controller.py` now publishes nominal ground intent on
  `/rescue/ground_cmd_nominal` and can apply terrain-risk traction degradation
  before publishing `/mecanum/cmd_vel`.
- Terrain degradation is disabled by default and enabled in Scene B/C configs.
- `rescue_metrics_logger.py` records additive columns:
  - `stuck_event_count`
  - `slip_ratio`
- `summarize_results.py`, `plot_results.py`, `validate_rescue_run.py`, and
  `export_summary_table.py` now carry stuck/slip metrics.
- Scene A now uses time-scripted perception only, so the PR-2 fairness smoke
  test is not contaminated by trajectory-dependent perception.

PR-3 smoke commands:

```bash
rosrun rescue_mission run_phase1_experiments.py \
  --scene scene_a \
  --strategies energy_rule risk_aware \
  --runs 1 \
  --duration 90 \
  --cooldown 0 \
  --summary-path ros_ws/src/rescue_mission/results/summary_pr2_scene_a.csv

rosrun rescue_mission check_shared_inputs.py \
  ros_ws/src/rescue_mission/results/runs \
  --scene scene_a \
  --strategies energy_rule risk_aware \
  --since <batch_start_timestamp> \
  --min-duration 0
```

Deferred Scene B acceptance:

```bash
rosrun rescue_mission run_phase1_experiments.py \
  --scene scene_b \
  --strategies energy_rule risk_aware \
  --runs 5 \
  --duration 110 \
  --cooldown 1 \
  --summary-path ros_ws/src/rescue_mission/results/summary_pr3_scene_b.csv \
  --plot
```

Acceptance target:
- `energy_rule` should show nonzero `stuck_event_count` and/or high
  `slip_ratio` in Scene B.
- `risk_aware` should reduce stuck/slip by choosing AIR before the risky
  terrestrial segment.
- If this difference is weak, tune only Scene B risk-zone parameters and
  `risk_aware` YAML weights before moving to RotorS.

### B7 Gate Result (Scene B, 2026-06-09) — PASSED

Full batch: `--scene scene_b --strategies energy_rule risk_aware --runs 5
--duration 130` (`summary_pr3_scene_b.csv`). Results are essentially
deterministic (std ~ 0):

| Strategy | success | stuck_event_count | slip_ratio | switch_count | air_distance_m | total_time_s | total_energy_j |
|----------|---------|-------------------|------------|--------------|----------------|--------------|----------------|
| `energy_rule` | 0/5 | 113.0 ± 0.0 | 0.856 | 0.0 | 0.0 | 96.6 | 15.1 |
| `risk_aware`  | 5/5 | 0.0 ± 0.0   | 0.333 | 8.0 | 14.4 | 44.8 | 56.6 |

Interpretation: in the barrier zone the ground controller's traction collapses
to zero for the ground-biased baseline (`energy_rule` J_ground < J_air, stays
GROUND, gets stuck ~113x and never reaches the goal). `risk_aware`'s two-point
CVaR saturates to `C_stuck`, making J_ground > J_air, so it flies over the
barrier, records **zero** stuck events, and completes the full round trip. The
higher energy (56.6 vs 15.1 J) is the honest cost of flying and is reported as
such — the baseline is "cheaper" only because it fails.

Bug fixed during this gate: the `energy_rule` / `risk_aware` branch in
`mode_switcher.py` had no terminal-goal landing. With distance ~ 0 at the goal
the CVaR tail keeps J_ground high, so `risk_aware` hovered at the delivery point
forever and never advanced DELIVER -> RETURN -> DONE (success stuck at 0). Fix:
when `mission_state in (DELIVER, RETURN)` and `distance < ground_distance`,
force `want_air = False` so the robot lands and the mission can complete. This
mirrors the existing `air_preferred` / `rule` terminal-goal handling.

Fairness note: Scene B/C use position-scripted perception, so `energy_rule` and
`risk_aware` traverse different paths and therefore sense different input
distributions — `check_shared_inputs.py` will WARN here, which is a false alarm.
Fairness in B/C is guaranteed at the code level (identical perception node and
params; only the decision logic differs, and `risk_aware` additionally consumes
`traversability_uncertainty`). The distribution check is only meaningful for the
time-scripted Scene A fairness smoke test.

Caveat for paper runs: Scene B runs are near-deterministic because perception
has no RNG; the 5 "seeds" only vary via Gazebo physics jitter (Scene C shows
more natural spread, see below). Add seedable perception noise before final
multi-seed paper tables. Also, `energy_rule` here ends at ~96.6 s sim time
(< the 120 s `mission_timeout_sec`) so `timeout` stays 0 even though it never
succeeds; bump `--duration` to ~155 s for canonical runs if an explicit timeout
flag is wanted.

### Scene C Result (2026-06-09)

Full batch: `--scene scene_c --strategies energy_rule risk_aware --runs 5
--duration 130` (`summary_pr3_scene_c.csv`). Scene C is harsher than B (38° slope,
0.95 barrier obstacle density, wider barrier `barrier_y_half_width=2.0`):

| Strategy | success | stuck_event_count | slip_ratio | switch_count | air_distance_m | total_time_s | total_energy_j |
|----------|---------|-------------------|------------|--------------|----------------|--------------|----------------|
| `energy_rule` | 0/5 | 140.4 ± 13.0 | 0.909 | 0.0 | 0.0  | 95.1 | 14.9 |
| `risk_aware`  | 5/5 | 17.8 ± 1.9   | 0.780 | 8.0 | 18.4 | 56.3 | 64.1 |

Interpretation: the contrast holds in the harder scene. `risk_aware` is no longer
perfectly stuck-free (the wider barrier forces more ground exposure during the
takeoff/landing transitions near the risky band), but it reduces stuck events by
~87% and goes from 0% to 100% mission success. Unlike Scene B, Scene C produces
genuine run-to-run variance (energy_rule stuck std ~13), which is useful natural
spread for the multi-seed story even before adding perception RNG.

Combined B7 conclusion: across Scene B and Scene C, with shared perception and
only the decision logic differing, `risk_aware` strongly dominates `energy_rule`
on both `success` and `stuck_event_count`. The gate is passed; the next
engineering steps (Stage 1 RotorS integration, PR-4 safety supervisor + Scene D)
are now justified.

## 10. PR-4 Safety Supervisor + Scene D (2026-06-09)

Implements the spec B3 decision-layer reachability supervisor (Claim B, the most
certifiable contribution): mechanically prevent the irreversible "took off with
too little energy to land safely" state.

Implemented code changes:

- **Battery is now a task-level joule budget** (Section 1). `rescue_metrics_logger.py`
  is the sole battery owner: it integrates `energy_j` across all modes and
  publishes `/rescue/battery` (fraction) and `/rescue/battery_j` (joules) from
  `battery_capacity_j` + `initial_battery`. `scenario_sensor_sim.py` no longer
  publishes battery (the old time-decay is gone). Default capacity 2000 J keeps
  legacy scenes ~full (backward compatible).
- **`safety_supervisor.py`** (new): pure, ROS-free reachability guard, distinct
  from the control-layer `safety_shield.py`. Rules (joules vs `/rescue/battery_j`):
  TAKEOFF allowed iff `battery_j >= E_takeoff + E_air_reserve + E_land + E_margin`;
  AIR continues iff `battery_j >= E_land + E_margin` (else force LAND); LAND gated
  by `landing_area_safety >= eta_land` (proxy = `visibility_confidence`);
  `safe_landing_margin_j = battery_j - (E_land + E_margin)`; `irreversible_failure`
  latches if airborne with `battery_j < E_land`. The forced-LAND-with-no-safe-area
  case records `critical_warning_unsafe_forced_land`.
- **Monitor-vs-enforce**: the supervisor is always instantiated and always
  monitors (counts unsafe proposals, tracks margin, latches irreversible), but
  only *enforces* overrides when `strategy == risk_aware_safety`. `mode_switcher.py`
  routes every proposed mode through `supervisor.filter(...)` in `spin()` and
  publishes 5 metrics. This lets plain `risk_aware` actually experience the
  irreversible failure while `risk_aware_safety` prevents it, on one code path.
- New strategy `risk_aware_safety` (= risk_aware decision + supervisor enforce).
- 5 additive CSV columns: `shield_override_count`, `override_reason`,
  `safe_landing_margin_j`, `unsafe_action_count`, `irreversible_failure`
  (carried into `summarize_results.py` and `export_summary_table.py`).
- New `config/scenes/scene_d.yaml` (Scene B terrain + low battery) and
  `disaster_scene_d.world` (copy of Scene B world); `scene_d` wired into both
  launch files.

Scene D calibration: start budget = `0.85 * 30 = 25.5 J`. The base takeoff floor
(`battery_takeoff_min = 0.40` fraction) is passed, so the *base* policy will take
off, but (a) the supervisor gate (`11+12+12+8 = 43 J`) blocks it when enforcing,
and (b) 25.5 J is genuinely insufficient for a flight, so monitor-only
`risk_aware` drains below `E_land` mid-air. Two calibration bugs were caught by
the smoke run before batching: the takeoff fired before the first `/rescue/battery_j`
message (fixed by initialising `mode_switcher.battery_j` from the logger's params
at startup), and an earlier 40 J budget was survivable (lowered to 25.5 J).

### PR-4 Gate Result (Scene D, 5 runs each) — PASSED

Full batch `--scene scene_d --strategies risk_aware risk_aware_safety --runs 5
--duration 100` (`summary_pr4_scene_d.csv`). Perfectly repeatable (std = 0):

| Strategy | success | irreversible_failure | shield_override_count | unsafe_action_count | air_distance_m |
|----------|---------|----------------------|-----------------------|---------------------|----------------|
| `risk_aware` (monitor) | 0/5 | **1.0 ± 0.0** | 0.0 | 2.0 | 4.0 |
| `risk_aware_safety` (enforce) | 0/5 | **0.0 ± 0.0** | **1.0 ± 0.0** | 1.0 | 0.0 |

In every run `risk_aware` takes off at 25.5 J (`takeoff_unsafe_low_battery` is
detected but not acted on), flies, and strands itself airborne below the landing
floor (`irreversible_failure=1`). `risk_aware_safety` blocks the same takeoff
(`shield_override_count=1`) and stays grounded (`irreversible_failure=0`). Neither
completes the mission, which is expected and correct: Claim B is about preventing
the irreversible crash, not mission success — the guarded robot fails *safely* on
the ground instead of crashing. This is the only hard evidence the spec requires
for the safety contribution (`shield_override_count > 0`, `irreversible_failure`
driven to 0).

Honesty caveats for this result: the energy model is a proxy (`set_model_state`
air, no RotorS), so the supervisor's joule thresholds are illustrative, not
calibrated flight energetics; this must be stated in the paper's limitations
alongside the existing `set_model_state` disclosure. (The earlier
near-deterministic concern is addressed by the seedable RNG in section 11.)

## 11. Paper-Hardening: seedable RNG, statistics, ablations (2026-06-09)

Goal: turn the previously deterministic (std = 0) results into a real statistical
experiment with seed-controlled randomness, without changing the algorithm logic
or touching RotorS / hardware. All new output goes to a separate directory
(`results/runs_paper_hardening_*`) so original CSVs are never overwritten.

Design (truth/perception separation + Common Random Numbers):

- `terrain_field.py` (new, pure): every random quantity is a deterministic hash
  of `(seed, cell, salt)` — no global RNG state, so the random terrain field is
  identical for all strategies under one seed and independent of visiting order.
- Ground truth vs perception: `scenario_sensor_sim.py` now publishes `theta_true`
  (ground-truth traversability, seed-keyed per cell) on `/rescue/traversability_true`
  and the strategies observe a *noisy* estimate `p_trav = theta_true + noise` on
  `/rescue/traversability_mean`. `theta_true` is consumed ONLY by the physics
  (`ground_goal_controller.py`) and the logger — `mode_switcher.py` never reads it
  (verified), so strategies have no god's-eye view.
- Stuck/slip physics: `ground_goal_controller.py` draws a per-cell Bernoulli trap
  `stuck ~ Bernoulli(1 - theta_true)` keyed by `(seed, cell)` plus a continuous
  slip from `theta_true`. So whichever strategy enters a given cell on the ground
  sees the identical stuck outcome (fair CRN); a strategy avoids bad cells only by
  flying over them.
- `seed = -1` (default) disables all of the above and restores the legacy
  deterministic behaviour — fully backward compatible.

Plumbing:

- `seed`, `mission_timeout`, `runs_dir`, `ablation` threaded through
  `rescue_phase1_demo.launch` -> `rescue_mission.launch` -> nodes.
- `run_phase1_experiments.py` gains `--seeds` / `--num-seeds` / `--base-seed`
  (paired multi-seed batches, seed is the loop just inside strategy so every
  strategy faces the same seed/terrain), `--mission-timeout`, `--ablation`.
- Logger records `seed` and `traversability_true` columns. `timeout` is now
  robust to Gazebo's variable real-time factor: a run shut down without reaching
  DONE writes a final `timeout = 1` row (no dependence on a fixed sim-time
  threshold). `summarize_results.py` adds analysis-only `theta_true_mean` /
  `theta_true_min` columns (never a strategy input; used to verify terrain
  comparability across strategies per seed).
- `analyze_significance.py` (new): pairs treatment vs baseline by seed and runs
  Wilcoxon signed-rank (continuous: stuck/energy/time/slip) with rank-biserial
  effect size + bootstrap 95% CI, and exact McNemar (binary: success / timeout /
  irreversible_failure).
- `plot_paper_figures.py` (new): mean +/- std bar charts (success rate,
  stuck_event_count, total_energy_j, irreversible_failure, shield_override_count).
  scipy 1.5.4 + matplotlib 3.5.3 installed in the `uav_noetic` container.
- Ablation configs in `config/ablations/` (`full`, `w_o_uncertainty` [k_unc=0],
  `w_o_cvar` [wcvar=0], `w_o_switch_penalty` [switch costs=0], `w_o_shield`
  [= run `risk_aware` instead of `risk_aware_safety`]), loaded after the scene so
  they override; selected with `--ablation`.

### Validation (Scene B, 5 seeds, runs=1) — real variance confirmed

`summary_ph_val_scene_b.csv` / `runs_paper_hardening_val`:

| Strategy | n | success | stuck_event_count | total_energy_j | total_time_s |
|----------|---|---------|-------------------|----------------|--------------|
| `energy_rule` | 5 | 0/5 | 85.0 ± 5.9 | 9.1 ± 0.1 | 58.1 |
| `risk_aware`  | 5 | 3/5 | 16.8 ± 23.0 | 94.3 ± 6.6 | 70.4 |

std is now > 0 (the std = 0 problem is solved). Per seed, `risk_aware` flies clean
on the easier seeds (0 stuck, success) but also gets stuck on the harder seeds —
a more honest distribution than the old deterministic result. Paired significance
(`analyze_significance.py`):

```
stuck_event_count  median_diff=-80.0  95%CI[-88,-37]  rank_biserial=-1.00  p=0.0625
total_energy_j     median_diff=+89.1  95%CI[+77,+91]  rank_biserial=+1.00  p=0.0625
total_time_s       median_diff=+7.2                   rank_biserial=+1.00  p=0.0625
success (McNemar)  treat=0.60 base=0.00 discordant(b=3,c=0)                p=0.25
```

p=0.0625 is the Wilcoxon floor at n=5 (2^-4); effect sizes are already maximal
(rank-biserial = +/-1). This is a power limitation of 5 seeds, not a weak effect —
20-30 seeds will drive p well below 0.001. Validate small first (this run), then
launch the overnight batch.

### Overnight batch commands (run when ready, ~hours)

Each run is real-time Gazebo (~15 s startup + duration). 20 seeds x 2 strategies
x 3 scenes ~ a few hours; run with `run_in_background` / `nohup`.

```bash
TS=$(date +%Y%m%d_%H%M%S)
PH=results/runs_paper_hardening_$TS
# Scenes B and C: risk_aware vs energy_rule (the B7 contrast, now multi-seed)
for SC in scene_b scene_c; do
  rosrun rescue_mission run_phase1_experiments.py \
    --scene $SC --strategies energy_rule risk_aware --num-seeds 20 --base-seed 0 --runs 1 \
    --duration 130 --cooldown 2 --mission-timeout 85 \
    --runs-dir $PH/$SC --summary-path results/summary_ph_$SC.csv --plot
done
# Scene D: risk_aware vs risk_aware_safety (Claim B, multi-seed)
rosrun rescue_mission run_phase1_experiments.py \
  --scene scene_d --strategies risk_aware risk_aware_safety --num-seeds 20 --base-seed 0 --runs 1 \
  --duration 110 --cooldown 2 --mission-timeout 80 \
  --runs-dir $PH/scene_d --summary-path results/summary_ph_scene_d.csv --plot

# Significance + figures per scene
for SC in scene_b scene_c; do
  rosrun rescue_mission analyze_significance.py results/all_runs_ph_$SC.csv \
    --scene $SC --treatment risk_aware --baseline energy_rule \
    --output results/significance_ph_$SC.txt
  rosrun rescue_mission plot_paper_figures.py results/summary_ph_$SC.csv \
    --output-dir results/figures_ph_$SC
done
rosrun rescue_mission analyze_significance.py results/all_runs_ph_scene_d.csv \
  --scene scene_d --treatment risk_aware_safety --baseline risk_aware \
  --output results/significance_ph_scene_d.txt
```

Ablations (build-now/run-later): re-run Scene B/C with `--ablation w_o_uncertainty`
/ `w_o_cvar` / `w_o_switch_penalty` (and `risk_aware` alone for `w_o_shield`),
writing to `results/runs_paper_hardening_$TS/<scene>_<ablation>/`, then compare
each against `full`.

> Bug fixed after the first overnight run: `run_overnight_batch.sh` passed a
> *relative* `--runs-dir`, which the logger node resolved against its own CWD
> (`~/.ros`) rather than the package, scattering CSVs into `/root/.ros/results`.
> The driver now uses absolute paths. (The first run's data was recovered intact.)

### 20-seed results (2026-06-09, `runs_paper_hardening_20260609_054657`)

Main comparison `risk_aware` vs `energy_rule`, 20 seeds, paired CRN:

| Scene | strategy | success | stuck_event_count | total_energy_j |
|-------|----------|---------|-------------------|----------------|
| B | energy_rule | 0/20 | 153.2 ± 14.8 | 15.7 ± 1.4 |
| B | risk_aware  | 9/20 | 53.5 ± 51.1   | 84.8 ± 16.0 |
| C | energy_rule | 0/20 | 144.4 ± 10.0 | 14.7 ± 0.9 |
| C | risk_aware  | 10/20 | 44.5 ± 45.3  | 92.3 ± 11.8 |

Paired significance (`analyze_significance.py`, Wilcoxon + McNemar):

- Scene B & C `stuck_event_count`: **p = 1.9e-6**, rank-biserial = -1.0,
  `risk_aware` lower on **20/20** seeds (perfectly consistent, not mean-driven).
- `success`: Scene B McNemar p = 0.0039 (9 vs 0), Scene C p = 0.0020 (10 vs 0).
- `timeout`: risk_aware times out less (B 0.55 vs 1.00 p=0.0039; C 0.50 vs 1.00 p=0.0020).
- Higher energy for `risk_aware` (p=1.9e-6) is the honest cost of flying.

This is the multi-seed, statistically powered version of the B7 result: with
shared perception and only the decision logic differing, `risk_aware` dominates
`energy_rule` on stuck/success across every seed. (Mean success ~0.45-0.50 because
on the hardest seeds even `risk_aware` gets stuck — an honest distribution.)

Ablations (20 seeds, vs `full`):

| Variant | B stuck / succ | C stuck / succ |
|---------|----------------|----------------|
| full | 53.5 / 0.45 | 44.5 / 0.50 |
| w_o_uncertainty | 57.4 / 0.50 | 28.9 / 0.75 |
| w_o_cvar | 146.3 / 0.00 | 145.2 / 0.00 |
| w_o_switch_penalty | 57.7 / 0.40 | 38.9 / 0.55 |

- **CVaR is the essential component**: `w_o_cvar` collapses to baseline behaviour
  (0% success, ~145 stuck) — strongest evidence for the tail-risk objective.
- `w_o_uncertainty` is comparable-or-better than full: the epistemic-inflation
  term (`k_unc`) is not contributing in the current calibration. Honest finding —
  report it, and either recalibrate the uncertainty model or frame it as a
  smaller/secondary contribution. Do not silently drop it.

### Scene D 20-seed result (recalibrated)

The first Scene D run was underpowered: with seeded perception the unguarded
policy only took the fatal low-battery flight on ~4/20 seeds (on the rest it
trapped on the ground instead), giving irreversible McNemar p = 0.125 even though
`shield_override_count` was always > 0. Terrain risk and ground-trap probability
are coupled through `theta_true` (by design — you cannot cheat perception), so
harshening the barrier would also trap the robot before it flew. The fix is a
*scene-local* aggressive tail-risk weight (`mode_switcher.risk_aware.wcvar: 2.5`
in `scene_d.yaml`) that makes the policy reliably choose to fly while still
mobile. This tunes the decision's eagerness to fly (the adversarial scenario the
supervisor must handle), not the supervisor itself.

Recalibrated Scene D (`risk_aware_safety` vs `risk_aware`, 20 seeds,
`summary_ph_scene_d_recal.csv`):

| strategy | irreversible_failure | shield_override_count | stuck | energy |
|----------|----------------------|-----------------------|-------|--------|
| risk_aware | **1.00 ± 0.00** | 0.0 | 112.6 | 31.3 |
| risk_aware_safety | **0.00 ± 0.00** | **1.0 ± 0.0** | 122.25 | 12.8 |

Now the unguarded policy flies into the fatal low-battery state and crashes on
**all 20 seeds**, and the supervisor blocks the takeoff on all 20. Paired
significance: `irreversible_failure` McNemar discordant (b=0, c=20),
**p = 1.9e-6** — the supervisor prevents the irreversible crash on every seed.
`shield_override_count` fires on every guarded seed and never on the baseline.
This is the statistically powered version of the PR-4 / Claim B result. The
guarded robot still fails the mission (stuck on the ground) — Claim B is about
removing the irreversible crash, not mission success.

## 12. Diagnostic PR: uncertainty (k_unc) + honest Scene D (2026-06-10)

Scope: diagnose *why* the `w_o_uncertainty` ablation was no worse than full, and
report Scene D honestly. No RotorS, no algorithm-logic change, no tuning for
significance.

### A. Why the uncertainty term (`k_unc`) does not help

`analyze_uncertainty.py` (new) reads the per-row run CSVs (which log both the
noisy estimate `p_trav` and the ground truth `theta_true`) and checks whether
`u_trav` actually predicts the perception error.

Homoscedastic (original) perception, Scene B, 20 seeds:

```
corr(u_trav, |p_trav - theta_true|) = +0.012   # ~zero: u_trav is UNINFORMATIVE
corr(u_trav, obstacle_density)      = +0.173   # u_trav tracks obstacles (by construction)
corr(|err|,  obstacle_density)      = -0.014   # but the actual error is constant everywhere
p_fail_eff saturated (>=0.99): 0.0% ;  mean inflation +0.165
```

Root cause: the perception noise scale was a constant (`percept_noise_scale`
= 0.10) everywhere, but `u_trav` was built to rise in obstacles. So `u_trav` was
high where perception was *just as accurate* — an uncalibrated, uninformative
signal. `k_unc` therefore added a roughly uniform 0.165 bias uncorrelated with
real risk, which is why dropping it (`w_o_uncertainty`) did no harm. This is a
*perception-model* problem, not an algorithm flaw.

Fix: a `heteroscedastic_noise` toggle (new launch arg `heteroscedastic:=true`)
that makes the actual `p_trav` noise scale equal `u_trav`, i.e. a *calibrated*
uncertainty. One run confirms it works:

```
corr(u_trav, |p_trav - theta_true|) = +0.883   # u_trav now strongly predicts the error
```

A/B test under calibrated uncertainty (Scene B, **20 seeds**, paired):

| variant | stuck (mean±std) | success | paired test |
|---------|------------------|---------|-------------|
| full (`k_unc=1`) | 54.4 ± 50.6 | 9/20 | stuck Wilcoxon p=0.451 (rbc=-0.23) |
| w_o_uncertainty (`k_unc=0`) | 64.7 ± 55.6 | 8/20 | success McNemar p=1.000 |

Per seed, `k_unc` lowers stuck on 8 seeds, raises it on 6, ties on 6 — at 20 seeds
`full` is *marginally* lower-stuck (54.4 vs 64.7) and one success higher (9 vs 8),
but nowhere near significant (p=0.45 / p=1.0; small effect size rbc=-0.23).
**Reported as "no significant benefit" (small effect size, statistically
underpowered) — NOT as a proven clean null.** We do not add seeds to chase
significance in either direction; the point is that `k_unc` is not a defensible
headline effect. Mechanism: the inflation is *symmetric* —
it correctly flies to avoid a hidden stuck when `p_trav` was optimistically wrong,
but equally flies unnecessarily (wasting energy, risking takeoff/landing stuck)
when `p_trav` was pessimistically wrong; the two roughly cancel.

Decision (category 2: informative-but-redundant): **downgrade the
uncertainty-inflation to a secondary/auxiliary component in the paper**, not a
headline. Do NOT tune `k_unc` to manufacture a benefit. The headline remains the
two-point CVaR (the `w_o_cvar` ablation collapses the method). The calibrated
heteroscedastic perception model is itself a worthwhile realism improvement and a
natural home for future *asymmetric/risk-directed* uncertainty use (future work) —
a constant symmetric inflation is the wrong instrument; a directional rule (only
inflate failure probability, never deflate, and only where the tail cost is high)
is the obvious next design, deferred.

### B. Scene D reported honestly (nominal + adversarial)

Both Scene D variants are reported; the recalibrated one is explicitly a stress
test, not the main result.

| Scene D | unguarded irreversible | guarded irreversible | override seeds | false-block rate | guarded success |
|---------|------------------------|----------------------|----------------|------------------|-----------------|
| nominal (original) | 0.20 (4/20) | 0.00 | 20/20 | **0.80** | 0.00 |
| adversarial (`wcvar=2.5`) | 1.00 (20/20) | 0.00 | 20/20 | 0.00 | 0.00 |

The decisive Claim B numbers (nominal Scene D, 20 seeds, `risk_aware` vs
`risk_aware_safety`):

| metric | unguarded | guarded |
|--------|-----------|---------|
| success_rate | **0.00** (0/20) | **0.00** (0/20) |
| irreversible_failure | 0.20 (4/20) | 0.00 |
| attempted takeoff | 20/20 | (blocked) |

Supervisor block decisions as a classifier (block vs would-crash ground truth):

```
would-crash takeoffs (unguarded irrev=1): 4/20
blocked takeoffs (guarded override>0):    20/20
TP=4  FP=16  FN=0  TN=0
RECALL    (would-crash takeoffs blocked) = 1.00
PRECISION (blocks that were necessary)   = 0.20
```

**Scene D nature: a low-battery DEAD-END.** The unguarded policy's success rate is
0.00 — at this battery the mission is unwinnable whether or not it flies (on the
16 non-crash seeds it survives the flight via reactive low-battery landing but
still never completes). Therefore:

- **Claim B framing (use this): the supervisor converts a catastrophic,
  irreversible failure into a safe abort at *zero mission-success cost*.** Recall
  is 1.00 (every would-crash takeoff is blocked) and `irreversible_failure` goes
  4/20 -> 0/20. The low precision (0.20) / 0.80 false-block rate is *not* a
  mission cost here: those 16 blocked flights would not have completed the mission
  anyway (unguarded success on them is 0). Report precision/recall honestly, but
  frame the false blocks as "no successes were lost", not "16 good flights wasted".
- **Do NOT tighten the supervisor** (the spec's conditional step 4): it only
  applies if the scene is winnable and the supervisor is killing real successes.
  Here unguarded success = 0, so there is nothing to recover; tightening would be
  tuning for appearance. Keep the supervisor as-is and report.
- Honest limitation to still state: the supervisor prevents the catastrophe but
  does not by itself complete the mission ("safely incomplete"). In a *winnable*
  low-margin scene this conservatism could cost real successes — not demonstrated
  here, flagged as a scenario-dependent caveat.
- Irreversible reduction is weakly significant in nominal (McNemar b=0,c=4,
  p=0.125 — rare event); the adversarial `wcvar=2.5` stress test (100% crash,
  p=1.9e-6, false-block 0.0) demonstrates the mechanism under certain danger and
  is labelled a stress test, not the headline number.

New tooling: `analyze_uncertainty.py`, the `heteroscedastic` launch arg /
`scenario_sensor_sim.heteroscedastic_noise` param, and a Scene-D Claim B analysis
(success comparison + supervisor precision/recall + false-block + guarded success).

## 6. Reporting Notes

- Old CSV files do not contain `takeoff_energy_j`, `land_energy_j`,
  `switch_energy_j`, `ground_distance_m`, or `air_distance_m`; the summarizer
  reports these as zero for backward compatibility.
- Final paper tables should only use runs generated after the top-tier metrics
  update, otherwise transition-energy comparisons are invalid.
- `summary_scene_a_latest.csv` currently mixes older rule runs with one
  optimized rule run. Use a fresh `summary_scene_a_shield.csv` for paper-facing
  analysis.
