# Paper Outline — “When Does Flying Pay Off?”

Proposed title:

> **When Does Flying Pay Off? Mode-Switching Regimes and Safety Constraints for Terrestrial–Aerial Robots**

The paper should be a regime/negative-result paper supported by representative
dynamic validation. It should not be framed as complete RotorS replication of
the Stage 0 policy ranking.

## 1. Introduction

- Motivate terrestrial efficiency versus aerial avoidance of catastrophic or
  costly ground hazards.
- State the central question: under what hazard, recoverability, and flight-cost
  regimes is switching to flight worthwhile?
- Contributions:
  1. paired regime evidence exposing the stuck-model dependence;
  2. an honest negative result for the fixed CVaR design versus simpler rules;
  3. an energy-reachability safety layer;
  4. representative same-model RotorS validation of contact, handoff,
     takeoff safety, and recovery primitives.
- State up front that strategy statistics use ideal tracking and RotorS is a
  separate validation layer.

## 2. Problem formulation

- Mission state, ground/air modes, outbound/return task.
- Traversability truth, noisy perception, seed/cell common random numbers.
- Permanent trap versus recoverable slip.
- Separate energy definitions:
  - Stage 0 legacy distance/hover proxy;
  - RotorS shaft-mechanical proxy.
- Define success, energy, stuck/slip, flight count, safe abort, and irreversible
  airborne-below-landing-reserve failure.

Suggested figure: system and evidence-layer diagram. The upper panel shows
policy→supervisor→mode; the lower panel separates Stage 0 ideal evaluation from
RotorS representative validation. This should be newly drawn from architecture,
not a mixed-energy plot.

## 3. Ground-risk and mode-switching policies

- `energy_rule`/ground baseline, periodic `fixed_switch`, scalar
  `threshold_switch`, and fixed `risk_aware` CVaR policy.
- Explain shared perception and common-random-number fairness.
- Disclose threshold candidate search and fixed risk-aware hyperparameters.
- Treat guarded/unguarded as a safety-layer axis, not another perception method.

Suggested table: policy definitions and which parameters were tuned. Source is
the implementation/config plus the CVaR fairness audit; do not imply equal
tuning budgets.

## 4. Why two-point CVaR becomes threshold-like

- Present the two-point loss and piecewise CVaR formula.
- Prove monotonicity and derive the local boundary `f*` under stated assumptions.
- Explain why distance, terrain, visibility, uncertainty, terminal logic,
  battery gates, minimum-air time, and shield overrides make the full policy a
  state-dependent local threshold rather than one fixed tau.
- Use the sampled boundary agreement only as an implementation diagnostic.

Suggested figure: analytic CVaR versus failure probability with the unsaturated
and saturated regions, plus several state-dependent local boundaries. Source:
`CVAR_THRESHOLD_FAIRNESS_AUDIT_20260731.md`; generate from formulas, not by
relabeling an old result figure.

## 5. Energy-reachability safety layer

- State the invariant: approve takeoff only if remaining budget covers takeoff,
  air reserve, landing, and margin; force land when airborne reserve falls low.
- Stage 0 supervisor uses the legacy ideal ledger.
- RotorS validation uses a separate mechanical ledger and never mixes values.
- Explain safe abort versus mission success.

Suggested figure/table:

- Stage 0 nominal and adversarial guarded/unguarded irreversible outcomes from
  the explicitly named Scene D reports.
- A separate RotorS D2 budget composition table and four/six-case validation
  table. Do not place ideal and RotorS joules on one axis.

## 6. Experimental methodology

- ROS1 Noetic/Gazebo Classic environment and paired seed design.
- Stage 0 scenes, stuck models, policies, sample counts, and statistics:
  Wilcoxon, rank-biserial, bootstrap interval, exact McNemar.
- Explain tuning/evaluation reuse as a limitation and pressure-test design.
- RotorS-native single-model structure, measured feedback, and no
  `set_model_state` in the RotorS branch.
- Separate evidence tiers:
  1. Stage 0 paper-facing regime experiments;
  2. D2 short-envelope calibration/safety validation;
  3. isolated contact/handoff/clearance/reposition matrices;
  4. one frozen Scene A sequence case study.

Suggested table: complete experiment matrix with backend, scene, seeds, stuck
model, and energy model. Source: `FINAL_EVIDENCE_MANIFEST_20260812.md`.

## 7. Stage 0 regime results

- Permanent versus recoverable assumption changes the value of flight.
- Batch 2 recoverable B/C table: fixed, selected thresholds, risk-aware.
- Sparse expensive-flight corner: no-fly baseline wins strongly; CVaR overflies.
- Interpret as a negative result for this fixed implementation, not universal
  rejection of CVaR.

Recommended paper assets:

1. Main success/energy Pareto plot for recoverable B/C, generated directly from
   canonical Batch 2 `all_runs` tables.
2. Regime comparison plot for permanent versus recoverable, labelled as a
   five-seed pilot where applicable.
3. Sparse-corner paired energy plot/table.
4. Paired-effect table with exact source reports.

Do not use the existing permanent `figures_ph_scene_b/c` as Batch 2 figures.

## 8. RotorS dynamic validation

- Begin with scope statement: validation, not regime replication.
- Same-model RotorS-native `uav_v4` and measured motor/odometry/IMU loop.
- Finite-force ground contact, staged ownership handoff, landing/disarm.
- Obstacle-aware takeoff clearance and bounded historical reposition.
- D2 mechanical proxy within its calibration envelope.
- Frozen Scene A sequence: first blocked takeoff recovers, flies, lands,
  disarms, and returns to ground control; second sortie safely rejects due to
  missing recent reverse-valid history; final mission `success=0`.

Recommended assets:

1. State/ownership sequence diagram from E1a-H/C/R contracts.
2. Recovery timeline plot from
   `r4_final_first_reposition_timeline.csv`: XY/height, state, site-valid,
   ground ownership, and motor enable.
3. Four-panel still sequence: blocked site, retreat, vertical takeoff, landing/
   disarm. It must be labelled as one representative trial.
4. D2 measured-versus-predicted held-out plot, explicitly “mechanical proxy”.

## 9. Limitations

- Ideal dynamics underlie all policy-ranking statistics.
- Threshold tuning budget exceeded CVaR tuning budget and reused evaluation
  seeds; no unbiased family-level superiority claim.
- Full CVaR implementation is only locally threshold-shaped.
- RotorS did not complete final Scene A and no Scene B/C RotorS ranking exists.
- Historical backtrack is not a general planner and cannot recover arbitrary
  post-flight landing sites.
- D2 is short-envelope, one model/payload, no wind, and excludes electrical
  losses/battery dynamics; 96.25% of audited B/C sorties exceed its distance
  envelope.
- No hardware experiments.

## 10. Conclusion

- Flying pays when ground failure is sufficiently consequential relative to
  flight cost, but recoverable ground failure can make aggressive flight wasteful.
- Sophisticated risk functionals are not automatically better than transparent
  switching rules; evaluation must include strong, fairly tuned baselines.
- Energy reachability and measured ownership handoffs can prevent unsafe mode
  transitions even when mission completion is impossible.
- Representative RotorS trials show feasibility of the safety primitives while
  defining the remaining planning and calibration boundaries.

## Writing route and optional next experiment

Recommended route is immediate paper/report writing with no additional
engineering. If one experiment is authorized, use new paired tuning seeds and
untouched held-out evaluation seeds to give threshold and CVaR equal candidate
budgets and a preregistered selection rule. Do not spend that single experiment
on a Scene A local planner or RotorS regime batch.

