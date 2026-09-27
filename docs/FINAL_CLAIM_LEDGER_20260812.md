# Final Claim Ledger

**Frozen:** 2026-08-12  
All wording below is bounded by the backend, scene, seed, stuck-model, energy
model, and sample-size metadata in `FINAL_EVIDENCE_MANIFEST_20260812.md`.

| Claim | Evidence | Allowed wording | Forbidden wording |
|---|---|---|---|
| CVaR negative result | Stage 0 Batch 2 B/C, sparse recoverable corner, paired significance reports, fairness audit | “The tested fixed two-point-CVaR implementation did not outperform simpler fixed/threshold switching in these ideal-backend regimes.” | “CVaR is universally inferior,” “CVaR never helps,” or any cross-backend/unseen-regime generalization |
| Two-point CVaR mechanism | Analytic derivation and sampled-boundary audit | “The two-point CVaR core is monotone and threshold-shaped; the complete implementation induces a state-dependent local threshold.” | “The complete policy is exactly equivalent to one fixed threshold,” or “the logged final mode sequence is analytically identical to threshold_switch” |
| Threshold result | Batch 2 selected tau comparisons and sparse corner | “Among the evaluated candidates, selected thresholds matched or exceeded the fixed risk-aware implementation on the observed success-energy tradeoff.” | “Fairly tuned threshold families generally beat fairly tuned CVaR families”; tuning and evaluation were not independently split |
| Permanent/recoverable finding | Stage 0 regime pilot, permanent B/C archive, recoverable Batch 2 and sparse corner | “The assumed recoverability of ground failure materially changes when flight pays off and can change policy ordering.” | “The same ranking holds for every stuck model,” or combining permanent/recoverable rows without labels |
| Stage 0 supervisor | Scene D nominal 4/20→0/20 irreversible failures and adversarial 20/20→0/20 | “The decision-layer guard prevented observed irreversible airborne-below-landing-reserve states, converting them to conservative safe aborts.” | “The supervisor guarantees mission completion,” “the nominal reduction is statistically conclusive” (nominal McNemar p=0.125), or presenting the adversarial stress test as nominal |
| RotorS integration | C2 finite scans/hover/hybrid smoke; E1a-H and E1a-R evidence | “RotorS representative closed-loop trials validate the feasibility of the ground-air handoff and safety primitives.” | “RotorS experiments reproduce the full Stage-0 strategy ranking,” “the full RotorS mission stack is validated,” or treating stock Firefly as final hybrid evidence |
| RotorS mechanical energy | D1/D2 calibration and source audit | “Rotor reaction-torque shaft mechanical energy was recorded using `P=k_f k_m Σ|ω|^3`, with physical-model aggregate rotor speed in rad/s.” | “Battery energy,” “electrical consumption,” or mixing joules numerically with the Stage 0 legacy proxy |
| Mechanical-energy supervisor | D2 12/6 split, held-out max underprediction, replay, E0 closed-loop cases | “The mechanical-energy supervisor was validated within its calibration envelope.” | “The supervisor guarantees battery-safe mission completion,” “the bound covers Scene A/B/C,” or “zero empirical violations proves probabilistic safety” |
| Takeoff clearance | E1a-C 40/40 matrix, old regressions, blocked Scene A bag | “A local swept-volume observer prevented arming at geometrically blocked takeoff sites and supported measured abort/land/disarm.” | “General collision avoidance,” “global route feasibility,” “sensor-realistic hardware clearance,” or “all valid sites are reachable” |
| Ground contact | E1a 15/15 finite-force contact microtests and Scene A before/after | “Finite-force ground ownership removed the hard-velocity contact-energy injection in the tested head-on, oblique, and corner cases.” | “All ground contacts are stable,” “the full Scene A task was thereby solved,” or hiding the later independent handoff/planning failures |
| Reposition recovery | E1a-R final 15/15 expected outcomes and frozen Scene A first cycle | “A bounded historical-valid-site backtrack restored a blocked takeoff and enabled one complete recovery-flight-land-disarm cycle.” | “A general local planner,” “arbitrary landing-site recovery,” “Scene A mission success,” or “recovery always finds a path” |
| Final Scene A case | Final E1a-R bag/log/CSV, seed 20260720 | “The first blocked takeoff recovered and completed a full cycle; a later sortie was fail-safe rejected for lack of recent ground-valid history.” | “Scene A DONE,” “mission success,” or omitting `success=0` and the second-sortie boundary |
| Ideal versus RotorS relation | Stage 0 manifest plus RotorS validation reports | “Stage 0 provides ideal-backend policy/regime evidence; RotorS provides separate representative dynamic and safety validation.” | “RotorS confirms the Stage 0 quantitative energy values/ranking,” any pooled backend metric, or calling ideal pose-setting real flight dynamics |
| Real-world relevance | Simulation architecture and limitation record | “The results motivate hardware evaluation and expose interfaces required for a real terrestrial-aerial system.” | “Real-world validated,” “hardware proven,” or “deployment-ready” |

## Mandatory qualifiers

- Say `ideal_set_model_state` wherever Stage 0 dynamics matter.
- Say `rotors_aero_shaft_mechanical_proxy_v1` or “shaft-mechanical proxy” for
  RotorS energy.
- Label `permanent` versus `recoverable` in every regime table.
- Describe the selected threshold result as a pressure test with unequal tuning
  budget, not an unbiased family-level contest.
- Describe RotorS as representative closed-loop validation, not regime
  replication.

