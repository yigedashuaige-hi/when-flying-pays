# Post-F1 Paper Outline

**Frozen:** 2026-08-22  
**Working title:** *When Does Flying Pay Off? Mode-Switching Regimes and Safety Constraints for Terrestrial–Aerial Robots*

Alternative titles retained for editorial choice:

1. *No Universal Switching Rule: Regime-Dependent Flight Decisions for Terrestrial–Aerial Robots*
2. *When to Leave the Ground: Recoverability, Risk, and Safe Mode Switching for Terrestrial–Aerial Robots*

The working title is preferred because it states the scientific question without implying a universal policy winner and leaves room for both regime and safety evidence.

## One-sentence thesis

Flight pays only in particular combinations of ground-failure recoverability, perceived risk, and switching structure; equal-budget held-out evaluation finds no universal winner among threshold, risk-aware, and fixed switching, while a separate RotorS evidence layer validates representative safety and handoff primitives.

## Contributions

1. **Regime characterization.** A controlled `ideal_set_model_state` study shows that recoverability of ground failure changes both the value of flight and policy ordering.
2. **Mechanism analysis.** For the implemented two-point loss, CVaR is monotone and threshold-shaped, but the full policy creates a state-dependent local threshold rather than one globally fixed cutoff.
3. **Fair family comparison.** A preregistered equal-budget search with disjoint tuning and held-out paired seeds shows equality in Scene B and a nonsignificant success-rate point advantage plus lower joint-success task proxy for risk-aware in Scene C; fixed switching remains a strong reference.
4. **Safety and dynamic feasibility.** An energy-reachability supervisor and representative RotorS-native closed-loop cases validate safe rejection, forced landing, ground–air ownership handoff, blocked-takeoff recovery, landing, and disarm within explicitly bounded conditions.

## Manuscript structure

### 1. Abstract

- Research question: when is flight worth its transition and exposure cost?
- Distinguish Stage 0 `ideal_set_model_state` evaluation from RotorS representative validation.
- Lead with regime dependence and F1 fair held-out result.
- State Scene C rates with McNemar `p=0.125`; do not imply significance.
- Close with supervisor and representative feasibility, not full mission success.

### 2. Introduction

- Terrestrial–aerial robots have a choice unavailable to ground-only risk planners: escape the ground mode.
- Flight is costly and introduces irreversible landing-reserve risk.
- The scientific unit is therefore a switching regime, not “flight is better.”
- Explain why fixed, scalar threshold, and state-dependent risk-aware rules may trade sortie count, distance, slip, and completion differently.
- List four contributions.

### 3. Related Work

- Terrestrial–aerial multimodal platforms and cross-domain planning.
- Traversability and risk-aware ground planning.
- CVaR and coherent tail-risk planning/control.
- Hybrid-mode safety, reachability, and supervisory shielding.
- RotorS/Gazebo validation boundary.
- Mark unverified or incomplete literature coverage in `REFERENCE_GAPS.md`.

### 4. Problem Formulation

- Modes `GROUND`, `TAKEOFF`, `AIR`, `LAND`; shared perception and paired cell-keyed randomness.
- Recoverable versus permanent stuck processes.
- Task objectives and metrics.
- Two noncommensurate energy ledgers: Stage 0 legacy task proxy versus RotorS shaft-mechanical proxy.

### 5. Mode-Switching Policies and Two-Point CVaR Analysis

- Ground-only/energy rule, fixed switching, threshold family, risk-aware family.
- Piecewise two-point CVaR formula for nominal cost and stuck loss.
- Derive local failure-probability boundary.
- Explain state dependence from distance, visibility, terrain cost, uncertainty, saturation, and mode gates.
- Use mechanism figure; avoid exact global equivalence.

### 6. Energy-Reachability Safety Supervisor

- Separate policy preference from admissibility.
- Ideal ledger invariant and RotorS mechanical-proxy ledger.
- D2 equation and units.
- Frozen `31,400 J` envelope and one-sided `900 J` margin.
- Safe abort is not mission success.

### 7. Experimental Setup

- ROS 1 Noetic/Gazebo Classic; Scene B/C recoverable and historical permanent/sparse regimes.
- `ideal_set_model_state` canonical backend.
- F1 equal-budget protocol: 5 candidates/family × 10 seeds × 2 scenes; held-out 20 paired seeds/scene; fixed reference.
- Primary success and conditional-effort estimands; exact McNemar, Wilcoxon, rank-biserial, paired bootstrap.
- RotorS representative cases are separate and do not reproduce policy ranking.

### 8. Regime and Fair-Tuning Results

- Stage 0 regime plot: permanent versus recoverable and sparse corner.
- F1 tuning selections and equal budget.
- Scene B: 20/20 versus 20/20; no detected task-proxy difference.
- Scene C: 20/20 versus 16/20, `p=0.125`; on 16 joint successes risk-aware `-11.267 J`, CI `[-14.002,-8.085]`, `p=.0002136`.
- Structural tradeoffs: longer air distance, more slip, fewer sorties.
- Fixed 20/20 in both; descriptive, not tuned.
- Synthesis: no universal winner.

### 9. RotorS Representative Dynamic Validation

- RotorS-native single rigid-body `uav_v4`, ownership gating, measured feedback, no `set_model_state` client.
- Contact 15/15; isolated handoff 20/20; clearance 40/40; reposition 15/15.
- Detailed final representative timeline.
- Explicitly report final exact Scene A `success=0` after the first successful cycle.
- D2 predicted/measured mechanical proxy as supplementary evidence.

### 10. Discussion and Limitations

- Recoverability is a modeling choice with causal consequences for strategy ranking.
- CVaR core complexity versus full-policy state dependence.
- Fixed switching can be strong when scene structure is stable.
- Small permanent pilot and only 20 held-out pairs limit precision.
- Scene C success point difference is not significant.
- Stage 0 ideal dynamics and proxy effort.
- RotorS limited to representative cases; no full Scene A success, local planner, hardware, battery, or universal safety guarantee.
- D2 envelope is much shorter than most Stage 0 sorties and cannot be extrapolated.

### 11. Conclusion

- Reiterate regime dependence and evidence separation.
- State most defensible conclusion: mode-switching structure should be selected and supervised for the anticipated failure regime, not assumed universally optimal.

## Planned main figures and tables

1. System and evidence-layer architecture.
2. Stage 0 recoverability-regime result.
3. F1 held-out success and joint-success task-proxy comparison.
4. Two-point CVaR and local-threshold mechanism.
5. Supervisor nominal/adversarial table or compact figure.
6. RotorS representative recovery/handoff timeline.
7. Supplementary D2 measured-versus-predicted shaft-mechanical proxy.

Tables: F1 protocol/selections; F1 paired statistics and structural tradeoffs; supervisor outcomes; RotorS validation matrix; evidence/backend contract.

