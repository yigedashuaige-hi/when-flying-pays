# Claim A Paper Materials (finalized tables + limitations)

Source: 20-seed paired (Common Random Numbers) batch
`results/runs_paper_hardening_20260609_054657` + Scene D recal, analyzed by
`analyze_significance.py`. All comparisons are paired by seed; continuous metrics
use Wilcoxon signed-rank (effect size = rank-biserial correlation, 95% CI by
bootstrap of the median paired difference); binary metrics use exact McNemar.

This is paper-facing material for the **finalized** results only (Claim A =
risk-aware cross-domain decision; Claim B = safety supervisor). It is not a new
experiment and not the final paper outline.

---

## Table 1 — Main result: risk_aware vs energy_rule (shared perception, 20 seeds)

Fair baseline `energy_rule` and proposed `risk_aware` see the identical perception
stream; they differ only in the decision rule (risk_aware adds the uncertainty
input + two-point CVaR). Lower stuck / higher success is better.

| Scene | metric | energy_rule | risk_aware | effect (rank-biserial) | p |
|-------|--------|-------------|------------|------------------------|---|
| B | stuck_event_count (mean) | 153.2 | **53.5** | −1.00 (lower on 20/20 seeds) | **1.9e-6** |
| B | success_rate | 0.00 | **0.45** (9/20) | McNemar b=9,c=0 | **0.0039** |
| B | timeout_rate | 1.00 | **0.55** | McNemar b=0,c=9 | **0.0039** |
| B | total_energy_j (mean) | 15.7 | 84.8 | +1.00 (cost of flying) | 1.9e-6 |
| C | stuck_event_count (mean) | 144.4 | **44.5** | −1.00 (lower on 20/20 seeds) | **1.9e-6** |
| C | success_rate | 0.00 | **0.50** (10/20) | McNemar b=10,c=0 | **0.0020** |
| C | timeout_rate | 1.00 | **0.50** | McNemar b=0,c=10 | **0.0020** |
| C | total_energy_j (mean) | 14.7 | 92.3 | +1.00 (cost of flying) | 1.9e-6 |

Median paired difference (stuck): B −89.0, 95% CI [−142, −59]; C −95.5,
95% CI [−142, −63.5]. Secondary: slip_ratio lower for risk_aware (B p=0.033,
C p=0.011); total_time shorter (B p=0.014, C p=0.064).

### Table 1b — Success-vs-energy Pareto frontier (20 seeds) — ⚠ CRITICAL HONEST FINDING

All five strategies, mean success vs mean energy (higher success / lower energy
better):

| Scene | strategy | success | energy_j | flight_fraction | Pareto |
|-------|----------|---------|----------|-----------------|--------|
| B | ground_only | 0.00 | 15.2 | 0.00 | frontier |
| B | energy_rule | 0.00 | 15.7 | 0.00 | dominated |
| B | **fixed_switch** | **1.00** | **75.3** | 0.72 | **frontier** |
| B | air_preferred | 0.25 | 64.8 | 0.95 | frontier |
| B | risk_aware | 0.45 | 84.8 | 0.89 | **DOMINATED by fixed_switch** |
| C | fixed_switch | **0.95** | 91.3 | 0.73 | frontier |
| C | risk_aware | 0.50 | 93.7 | 0.92 | **DOMINATED by fixed_switch** |

**`risk_aware` is Pareto-dominated by the naive `fixed_switch` baseline** (which
just alternates GROUND/AIR every 10 s): fixed_switch gets *higher* success at
*lower* energy. This is the opposite of the hoped-for "risk_aware is frontier
optimal" and must be confronted, not hidden.

Diagnosis (per-seed): risk_aware is **bimodal** — on ~half the seeds it flies the
barrier cleanly (stuck=0, success), on the other half it is on the ground during
the barrier crossing, hits a permanent traction-zero trap, and fails (stuck
~90, ends in RETURN). `fixed_switch` is airborne ~50% of the time *on a fixed
schedule*, so it is almost always airborne during the brief barrier crossing and
never gets trapped; it also lands periodically so it can still deliver
(`air_preferred`, which almost never lands, fails to deliver → success 0.25).

Root cause: the stuck model is a **permanent trap** (traction→0 forever once a
risky cell triggers), which makes a single mistimed ground moment in the barrier
catastrophic and rewards blind frequent flying over perception-timed flying. In
this setup risk_aware's selectivity is a liability, not an asset, versus a dumb
periodic switcher.

Implication for Claim A: the *paired, energy-matched* claim (risk_aware vs
energy_rule, Table 1) is real and significant, but the *absolute* claim
("risk_aware is the best multimodal policy") is **false** here. This is a
load-bearing problem; see the decision options recorded with the user
(reframe vs. fix stuck model vs. recalibrate). Do not paper over it.

Reading: with shared perception, risk_aware reduces getting-stuck on **every
seed** and turns a 0% baseline into 45–50% mission success, at a higher (honestly
reported) energy cost. Mean success ~0.5 because on the hardest seeds even
risk_aware gets stuck — an honest distribution, not a ceiling result.

---

## Table 2 — Ablation (20 seeds, vs `full` risk_aware)

| Variant | B stuck | B success | C stuck | C success | role |
|---------|---------|-----------|---------|-----------|------|
| **full** | 53.5 | 0.45 | 44.5 | 0.50 | proposed |
| w/o switch penalty | 57.7 | 0.40 | 38.9 | 0.55 | minor |
| w/o uncertainty (k_unc=0) | 57.4 | 0.50 | 28.9 | 0.75 | see note |
| **w/o CVaR (wcvar=0)** | **146.3** | **0.00** | **145.2** | **0.00** | **essential** |

- **CVaR is the essential mechanism.** Removing it collapses risk_aware to baseline
  behaviour (~145 stuck, 0% success) — it is what produces the entire Claim A
  effect.
- **Uncertainty inflation (k_unc) is not a headline.** `w/o uncertainty` is no
  worse than (Scene B) or better than (Scene C) full. Dedicated test below.

### Table 2b — k_unc under calibrated (heteroscedastic) perception, 20 seeds

The first ablation used homoscedastic noise where u_trav was uninformative
(corr(u_trav, |p_trav−theta_true|)=+0.012). With calibrated perception
(noise scale = u_trav, corr=+0.883):

| variant | stuck (mean±std) | success | paired |
|---------|------------------|---------|--------|
| full (k_unc=1) | 54.4 ± 50.6 | 9/20 | stuck Wilcoxon p=0.45, rbc=−0.23 |
| w/o uncertainty (k_unc=0) | 64.7 ± 55.6 | 8/20 | success McNemar p=1.00 |

Reported as **"no significant benefit (small effect, underpowered)"**, not a
proven null. The constant symmetric inflation helps and hurts on roughly equal
numbers of seeds. → uncertainty-inflation = secondary/auxiliary; future work is a
*directional* uncertainty rule (only raise failure prob where tail cost is high).

---

## Table 3 — Claim B: safety supervisor in Scene D (low-battery), 20 seeds

Two settings reported. **Nominal is the main result; adversarial is a labelled
stress test.**

| Scene D | unguarded success | guarded success | unguarded irreversible | guarded irreversible | supervisor recall | supervisor precision |
|---------|-------------------|-----------------|------------------------|----------------------|-------------------|----------------------|
| **nominal (main)** | 0.00 | 0.00 | 0.20 (4/20) | **0.00** | **1.00** | 0.20 |
| adversarial (stress test, wcvar=2.5) | 0.00 | 0.00 | 1.00 (20/20) | **0.00** | 1.00 | 1.00 |

- Nominal Scene D is a **low-battery dead-end**: both success rates are 0 (the
  mission is unwinnable at this battery whether or not the robot flies). The
  supervisor drives irreversible_failure 4/20 → 0/20 (recall 1.00) at **zero
  mission-success cost**.
- Precision 0.20 (false-block rate 0.80) reflects deliberate conservatism; here it
  loses no successes (the blocked flights would not have completed anyway).
- Significance: nominal irreversible McNemar b=0,c=4, **p=0.125** (rare event —
  reported honestly, not leaned on). Adversarial irreversible b=0,c=20,
  **p=1.9e-6** — demonstrates the mechanism under certain danger, labelled a
  stress test.

Claim B is **catastrophic-failure prevention / safe abort, not safe completion**:
the supervisor mechanically guarantees no irreversible "took off, cannot land"
state; it does not by itself complete the mission.

---

## Limitations (state explicitly in the paper)

1. **Air dynamics are idealized (`set_model_state`, no RotorS).** Air motion is
   prescribed, so energy figures are a proxy and air results assume ideal
   trajectory tracking. Quantitative air-energy and the supervisor's joule
   thresholds are illustrative; RotorS integration is required before any
   "controlled by our flight controller" claim. (Deferred Stage 1.)
2. **Supervisor conservatism.** Precision 0.20 in nominal Scene D: the supervisor
   blocks many takeoffs that would have been survivable. It is safe but not free;
   in a *winnable* low-margin scenario this conservatism could cost real mission
   successes (not observed here because the scene is a dead-end). Flagged as
   scenario-dependent.
3. **Uncertainty term (k_unc) is not validated as beneficial.** Even with a
   calibrated heteroscedastic perception model, constant epistemic inflation shows
   no significant benefit (underpowered, small effect). Presented as an auxiliary
   component / future work (directional uncertainty), not a contribution.
4. **Safe abort ≠ safe completion.** Claim B prevents the catastrophe but the
   guarded robot can still fail the mission (safely, on the ground). The
   contribution is removing the irreversible failure mode, not improving task
   success under low battery.
5. **Simulation realism / self-fulfilling-prophecy guard.** Stuck is a seed-keyed
   Bernoulli on ground-truth theta_true (physics), strategies see only the noisy
   p_trav; fairness is code-level (shared perception node, identical params), not
   only distributional. Still a proxy terrain/perception model, not field data.
6. **Effect-size honesty.** Headline effects (stuck, Claim B irreversible) are
   large and consistent across all 20 seeds; secondary metrics (slip, time) and
   k_unc are reported with their real (sometimes weak/non-significant) effect
   sizes, not cherry-picked.
