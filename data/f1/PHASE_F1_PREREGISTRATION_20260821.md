# Phase F1 Preregistration: Equal-Budget Tuning and Fresh Paired Evaluation

Frozen on 2026-08-21, before any Phase F1 simulation was launched.

## Scope and research question

This is the only new post-freeze scientific experiment. It asks whether the
existing `risk_aware` policy and the existing `threshold_switch` policy differ
after equal tuning opportunity and evaluation on untouched paired seeds.
Only Scene B and Scene C with `stuck_model=recoverable` and
`air_backend=ideal` (`ideal_set_model_state`) are in scope. Stage 0 remains
immutable. RotorS, Scene A, Scene D, local planning, energy-model changes,
supervisor changes, terrain changes, perception changes, and policy-code
changes are excluded.

The confirmatory comparison is between two *preregistered one-dimensional
families*, not an exhaustive optimization of either algorithm.

## Candidate sets and equal budget

Each family receives exactly five candidates, ten common tuning seeds per
candidate, one run per candidate/seed, and independent scene-specific
selection in both scenes. Thus each family receives 50 tuning runs per scene
and 100 in total.

- `threshold_switch`: `switch_tau` in `{0.2, 0.3, 0.4, 0.5, 0.6}`.
- `risk_aware`: `mode_switcher.risk_aware.wcvar` in
  `{0.25, 0.50, 0.75, 1.00, 1.25}`.

The threshold grid is the already-used historical 0.2--0.6 grid. The
risk-aware grid is centered on the frozen default `wcvar=0.75`, uses uniform
0.25 spacing, and spans +/-0.50. The historical `wcvar=0` ablation is excluded
because it removes the CVaR contribution; the Scene-D stress value 2.5 is
excluded because it was introduced for a different adversarial scene. The
risk parameters `alpha=0.8`, `k_unc=1.0`, `C_stuck=60`, all physical cost
parameters, and all other weights remain frozen. No candidate may be added,
removed, or moved after tuning begins.

## Seeds and blinding boundary

- Tuning: `100..109` (10 paired seeds).
- Untouched held-out evaluation: `200..219` (20 paired seeds).

A pre-run scan of 993 seed-bearing historical CSV files found historical seeds
`-1`, `0..19`, and `20260720`; therefore neither set overlaps prior result
data. One retained, already-documented corrupted D2 first-pass CSV was
unparseable and excluded; it is not a paper-facing seed source. Both families
use identical paired seeds. Evaluation outcomes must not be
generated or inspected until both scene-specific winners are written to a
hash-frozen selected-config file.

## Selection rule

Selection is performed independently for Scene B and Scene C and identically
for both families. Candidates are ordered lexicographically by:

1. highest mission-success count across all ten tuning seeds;
2. lowest median `energy_j` among successful tuning runs only;
3. lowest median `air_distance_m` among successful tuning runs only;
4. lowest median `switch_count` among successful tuning runs only;
5. smallest absolute distance from the historical default (`tau=0.4` or
   `wcvar=0.75`);
6. lower numeric candidate value as a deterministic final tie-break.

Failed or timed-out runs are never deleted. A candidate with zero successful
runs has infinite values for success-conditioned tie-break metrics. No tuning
metric may be changed after results are seen.

## Held-out evaluation and fixed reference

After selection and winner hashing, each selected method is run once on each
of the 20 held-out seeds in each scene. `fixed_switch` is run on the same seeds
as a frozen descriptive reference; it neither participates in tuning nor
affects selection. There are 80 confirmatory selected-method evaluation runs
and 40 reference runs.

## Outcomes and statistics

Primary outcomes:

1. paired mission success, with the discordant-pair table and two-sided exact
   McNemar test;
2. total task energy on the subset of paired seeds where both methods succeed.

Secondary outcomes on the both-success paired subset are `air_distance_m`,
`switch_count` (sortie/switch effort), `stuck_event_count`, and `slip_ratio`.
For each continuous/count outcome report the paired risk-aware-minus-threshold
difference, mean and median difference, two-sided Wilcoxon signed-rank test
when nonzero differences make it applicable, matched rank-biserial effect
size, and a deterministic seed-20260821 paired bootstrap 95% percentile
confidence interval for the mean difference (10,000 resamples). All-seed
energy is reported descriptively with success status and is not interpreted
as efficiency because early failure can consume less energy.

For the fixed reference, report descriptive success and the same raw metrics;
its comparisons are explicitly non-confirmatory. A two-point-CVaR local
threshold calculation may be included only as mechanism analysis and must not
be described as strict equivalence to a fixed threshold.

## Stop and integrity rules

- Stop the batch on NaN/Inf in a completed CSV, malformed/missing CSV, seed or
  scene mismatch, a wall cutoff without a valid completed CSV, residual
  ROS/Gazebo process contamination, or any non-ideal backend observation. As
  in the historical Stage 0 runner, `roslaunch` itself is intentionally ended
  at the registered 130 s wall cutoff after the logger has reached its 85 s
  simulation mission timeout; that controlled shutdown is not a failed run
  when the isolated CSV passes all validity checks.
- Do not delete, replace, rerun selectively, or exchange a failed/timeout seed.
  A technically invalid run may be rerun only after its original log and CSV
  are retained and the reason is recorded; the same rule applies to both
  families.
- Do not adjust candidates, seeds, scenes, selection criteria, timeouts, or
  metrics based on observed outcomes.
- Every output goes to the independent Phase F1 directory. The Stage 0
  canonical mirror/manifest and frozen D2 artifacts are hash-checked before
  and after Phase F1.
- Results are accepted whether threshold wins, risk-aware wins, or they are
  indistinguishable. No full paper batch, RotorS run, or engineering extension
  follows this phase.

## Planned run count

- Tuning: 2 scenes x 2 families x 5 candidates x 10 seeds = 200.
- Held-out selected methods: 2 scenes x 2 methods x 20 seeds = 80.
- Held-out fixed reference: 2 scenes x 1 method x 20 seeds = 40.
- Total: 320 runs.
