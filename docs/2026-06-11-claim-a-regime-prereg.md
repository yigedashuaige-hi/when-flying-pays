# Claim A Stress Test — Pre-Registration (regime sweep + tuned threshold baseline)

**Conclusions are written here BEFORE the runs and judged mechanically after,
accepted whatever they say (as with k_unc). No tuning of scene geometry, flight
cost, hazard density, or stuck model to make risk_aware win — the only thing
swept for a baseline's benefit is `threshold_switch`'s threshold tau.**

Trigger: the raw success-vs-energy frontier showed `risk_aware` Pareto-dominated
by `fixed_switch`. Step-1 diagnosis showed that frontier is confounded.

## Step 1 findings (no model changed)

- `air_preferred` is **not** a clean fly-heavy baseline: its terminal-landing
  rule lands it inside the slope risk zone on the RETURN leg, where the
  permanent-trap stuck model traps it (stuck ~110, times out 1.7 m short,
  success 5/20). Mode-time airborne fraction is only ~0.35 (the earlier 0.95 was
  distance-based and misleading).
- **`risk_aware` shares this failure mode** (failing seeds end in RETURN at
  final_dist ~1.7, trapped near the goal). So the failures are driven by
  *forced terminal-landing into a risky zone* + the *permanent-trap* stuck model,
  not purely by risk logic. `fixed_switch` escapes only via scheduled periodic
  flight that breaks out of traps.
- Therefore the raw "fixed_switch dominates" result is confounded; it is not yet
  evidence that selective risk-aware switching has no value.

## Step 2 — the real opponent: tuned `threshold_switch`

New baseline `threshold_switch`: fly iff `(1 - p_trav) > tau` (else ground),
using only the shared perceived `p_trav` (no CVaR, no uncertainty). Sweep
`tau in {0.2, 0.3, 0.4, 0.5, 0.6}` and take, per regime, the tau on the
success-vs-energy Pareto frontier ("tuned threshold_switch"). **risk_aware/CVaR
must beat the *tuned* threshold_switch to justify Claim A — this is the decisive
test, more than beating fixed_switch.**

## Step 3 — regime as an explicit research axis (the principled Option 2)

Two environment axes, both reported, neither tuned to favour risk_aware:

- **flight cost**: cheap vs expensive air energy (`air_energy_per_m` /
  `fly_k1` low vs high).
- **hazard density**: dense vs sparse risky terrain (fraction of the route that
  is risky — wide/continuous barrier vs a narrow localized one).

Plus the stuck model as a **third explicit axis (reported side by side, NOT
silently swapped):**

- **permanent trap** (current): traction→0 forever once a risky cell fires.
- **recoverable slip** (added after diagnosing the permanent-trap artifact): low
  theta → slow/slip; only very low theta briefly halts; the robot can crawl out.
  More realistic; presented as a labelled alternative, not a replacement.

Pilot scope (keep tractable): the 2x2 regime corners (cheap/expensive x
dense/sparse) x 2 stuck models x {energy_rule, fixed_switch, threshold_switch(tuned),
risk_aware} x >=10 seeds. Expand later only if a corner is decisive.

## Step 4 — pre-registered conclusions (judged mechanically after)

Let "realistic regime" = recoverable-slip stuck model AND flight has real cost
AND hazard is sparse.

1. **GO / Claim A stronger**: if `risk_aware` is on the success-vs-energy Pareto
   frontier in the realistic regime AND beats BOTH `fixed_switch` and the tuned
   `threshold_switch` there (>= as much success at <= energy, strictly better on
   one) → Claim A holds and is *stronger* than before, because the regime sweep
   now maps where selective switching pays off.
2. **NARROW**: if `risk_aware` wins only in a narrow corner of the
   (flight-cost x hazard-density x stuck-model) space → honestly restrict Claim A
   to that regime and report the rest as where naive switching suffices.
3. **DOWNGRADE**: if `risk_aware` cannot beat even the tuned `threshold_switch`
   in any realistic regime → CVaR is over-engineered for this problem; downgrade
   Claim A, and pivot the paper's weight to Claim B (safety supervisor) + the
   regime characterization itself as the contribution.

Forbidden: changing scene geometry, flight cost, hazard density, or the stuck
model *to make risk_aware win*. Those are swept as honest independent variables;
only `threshold_switch`'s tau is optimized (in the baseline's favour).

## RESULT (2026-06-11) — verdict: DOWNGRADE (pending one untested corner)

Scene B, 5 seeds, both stuck models (`runs_claim_a_regime`):

| stuck | strategy | success | energy_j |
|-------|----------|---------|----------|
| recoverable | fixed_switch | 1.00 | 48.5 |
| recoverable | tuned threshold (tau=0.2) | 1.00 | 58.9 |
| recoverable | **risk_aware (CVaR)** | 1.00 | **94.5** |
| permanent | fixed_switch | 1.00 | 74.3 |
| permanent | tuned threshold (tau=0.4) | 0.20 | 24.6 |
| permanent | **risk_aware (CVaR)** | 0.40 | **101.3** |

- In the **realistic (recoverable) regime**, `risk_aware` matches the 100% success
  of both `fixed_switch` and the tuned `threshold_switch` but at **60-95% more
  energy** -> **Pareto-dominated by both**. It does NOT beat the tuned threshold in
  either stuck model.
- Pre-registered Step-4 criterion 3 fires: **DOWNGRADE.** The CVaR/uncertainty
  machinery is over-engineered for this problem — a *tuned scalar threshold* on
  `(1 - p_trav)` matches it at lower energy. Mechanistically expected: with a 1-D
  perceived-risk signal and a non-informative uncertainty term (k_unc null), CVaR
  reduces to a threshold on `(1 - p_trav)`; risk_aware's calibration just makes it
  over-fly (higher energy, no extra success).
- What SURVIVES: *risk-based* mode switching (both threshold_switch and risk_aware)
  massively beats single-mode / energy-greedy baselines (energy_rule, ground_only:
  0% success). The contribution is "use perceived risk to switch modes" + the
  regime characterization + Claim B — NOT the specific CVaR formulation.

Caveat (honest): only the recoverable + default-flight-cost + Scene-B-hazard
regime and the permanent counterpart were run. The **expensive-flight + sparse-
hazard** corner (where selective flying is most rewarded) was NOT run. But theory
+ the observed over-flying predict it will not rescue risk_aware: a tuned
threshold is equally selective and flies less, so expensive flight penalizes
risk_aware's extra flying *more*. Running that corner is the only remaining step
to make DOWNGRADE airtight; recommended before final write-up. n=5 / Scene B only
— confirm at more seeds/scenes for the paper.

## RESULT — untested corner run (2026-06-11): DOWNGRADE **LOCKED** (not NARROW)

The expensive-flight + sparse-hazard corner was run (`scene_sparse`, `--fly-k1 3.0`,
recoverable stuck, 10 seeds; `runs_claim_a_airtight`). This is the one place
Claim A could have been NARROW. It is not:

| strategy (sparse, fly_k1=3.0, recoverable) | success | total_energy_j | fly_frac |
|--------------------------------------------|---------|----------------|----------|
| energy_rule (never flies)                  | 1.00    | **7.3**        | 0.00 |
| tuned threshold (τ=0.6, flies least)       | 1.00    | 91.0           | 0.26 |
| fixed_switch                               | 1.00    | 135.5          | 0.57 |
| **risk_aware (CVaR)**                       | 1.00    | **258.4**      | 0.80 |

- Paired per-seed (n=10) risk_aware vs tuned threshold τ=0.6: success tied 1.00/1.00;
  risk_aware spends **+167.3 J/seed** more. risk_aware is Pareto-dominated again, and
  by a *wider* margin than at default flight cost — exactly the mechanistic
  prediction (expensive flight amplifies the penalty for risk_aware's over-flying).
- **Stronger finding:** in this recoverable + sparse regime, flying is *unnecessary
  altogether* — `energy_rule` (and therefore `ground_only`) walks through the narrow
  band and crawls out at 7.3 J, 100% success. So the corner that was supposed to most
  reward selective flying instead shows selective flying is not even needed; CVaR's
  over-flying (258 J = 35× the walk) is maximally wasteful here.
- Pre-registered Step-4: criterion 1 (GO) and criterion 2 (NARROW) both FAIL in the
  one remaining corner. **Criterion 3 (DOWNGRADE) is LOCKED.** Note for the paper:
  this also sharpens contribution (iii) — whether *any* flying is needed is itself
  regime-dependent (recoverable+sparse needs none; switching's necessity is not
  universal but regime-scoped).

Remaining: 20-seed Scene B + C recoverable robustness batch is running to confirm
DOWNGRADE statistically; verdict is already locked on direction.

## ARCHIVAL STATUS ADDENDUM (2026-07-18; original pre-registration retained)

The remaining 20-seed Scene B + C recoverable robustness batch subsequently
completed. All ten configurations have exactly one run for every seed 0--19.
This update records the result without changing the pre-registered decision rule
or deleting the historical `running` statement above.

| scene | strategy | success | mean energy J | mean stuck events |
|---|---|---:|---:|---:|
| B | fixed_switch | 1.00 | 48.306 | 0.0 |
| B | threshold tau=0.2 | 1.00 | 67.161 | 0.0 |
| B | threshold tau=0.3 | 0.95 | 49.687 | 5.3 |
| B | threshold tau=0.4 | 0.85 | 55.835 | 19.45 |
| B | risk_aware | 0.95 | 80.576 | 0.0 |
| C | fixed_switch | 1.00 | 71.102 | 7.4 |
| C | threshold tau=0.2 | 1.00 | 90.204 | 0.0 |
| C | threshold tau=0.3 | 1.00 | 71.676 | 0.0 |
| C | threshold tau=0.4 | 0.80 | 57.388 | 31.9 |
| C | risk_aware | 0.95 | 90.257 | 0.0 |

Paired continuous comparisons use the existing `analyze_significance.py` with
temporary concatenation of the two immutable per-strategy `all_runs` CSVs:

- Scene B, risk_aware vs fixed_switch: energy median paired difference
  +36.769 J, rank-biserial +0.943, p=2.67e-05.
- Scene B, risk_aware vs threshold tau=0.2: energy median paired difference
  +19.593 J, rank-biserial +0.476, p=0.06372. The aggregate Pareto conclusion is
  still clear, but this particular energy difference is not significant at 0.05.
- Scene C, risk_aware vs fixed_switch: energy median paired difference
  +18.006 J, rank-biserial +0.895, p=0.0001049.
- Scene C, risk_aware vs threshold tau=0.3: energy median paired difference
  +22.888 J, rank-biserial +0.848, p=0.0003223.

Success is 19/20 for risk_aware versus 20/20 for each selected comparator; the
single discordant seed gives exact McNemar p=1.0. Therefore Batch 2 supports the
DOWNGRADE through the success-energy Pareto result and strong paired energy
evidence in Scene C/fixed-switch comparisons, not through a claim that every
individual success or energy contrast is significant.

Canonical CSVs, raw directories, hashes, and report paths are frozen in
`rescue_mission/results/STAGE0_RESULT_MANIFEST_20260718.md`. Backend for all of
these data is `ideal_set_model_state`.

## Diagnostics to also report (not decisive, for interpretation)

flight_fraction (mode-time based, not distance), terminal-landing-in-risky-zone
rate, and the per-seed success/stuck bimodality, so the *mechanism* of any
win/loss is visible.
