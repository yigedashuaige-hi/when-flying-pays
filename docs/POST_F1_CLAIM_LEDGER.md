# Post-F1 Claim Ledger

**Frozen:** 2026-08-22  
**Scope:** paper claims after Phase F1; no new simulation evidence is introduced.  
**Supersedes for manuscript wording:** `FINAL_CLAIM_LEDGER_20260812.md` while preserving that file as historical evidence.

## Claim boundary

The paper's central conclusion is now:

> **No single switching rule dominates across regimes.** Whether flight pays depends on ground-failure recoverability, the risk regime, and the structure of mode switching.

The legacy Stage 0 task-energy proxy and the RotorS aerodynamic shaft-mechanical proxy are separate quantities. They must not share a quantitative energy axis or be interpreted as battery energy.

## Frozen claims

| Topic | Evidence | Allowed manuscript claim | Prohibited overclaim |
|---|---|---|---|
| Recoverability changes the value of flight | Stage 0 permanent/recoverable Scene B pilot; recoverable Batch 2 and sparse corner | Ground-failure recoverability materially changes when flight is useful and changes the observed ordering of switching strategies. | Universal ordering over environments, hardware, or untested dynamics. |
| Fair threshold–risk-aware comparison, Scene B | F1 preregistered held-out seeds 200–219 | Both selected families succeeded on 20/20 paired runs. No difference in mission success was observed, and no paired legacy-energy difference was detected (mean risk-aware minus threshold `-2.339 J`, bootstrap 95% CI `[-9.112, 3.815]`, Wilcoxon `p=0.9273`). | “Risk-aware is better” or “threshold is better” in Scene B. |
| Fair threshold–risk-aware comparison, Scene C | F1 preregistered held-out seeds 200–219 | Risk-aware succeeded on 20/20 and threshold on 16/20. The point estimate favors risk-aware, but exact McNemar `p=0.125`; the success difference is not statistically significant at 0.05. | “Risk-aware significantly improves Scene C success,” or a population-level dominance claim. |
| Scene C joint-success effort | F1 paired analysis restricted to the 16 seeds where both methods succeeded | On joint-success pairs, risk-aware used less legacy task-proxy energy: mean paired difference `-11.267 J`, bootstrap 95% CI `[-14.002, -8.085]`, Wilcoxon `p=0.0002136`, rank-biserial `-0.9412`. | Comparing all-seed energy as efficiency evidence, because threshold's four failures terminate early. |
| Structural trade-offs in F1 | F1 held-out paired statistics | Risk-aware flew farther and experienced more slip, but used fewer sorties; this is a different switching structure, not a free improvement. | Treating longer flight or additional slip as uniformly good or bad without task context. |
| Fixed switching | F1 held-out descriptive reference | Fixed switching remained a strong reference (20/20 success in both scenes) and must be shown. It was not tuned and did not affect family selection. | Calling fixed switching the statistically established winner or hiding it because it is simple. |
| Historical CVaR negative result | Stage 0 Batch 2 and sparse recoverable corner | The historically fixed CVaR configuration did not outperform selected simple rules in those frozen experiments; in the sparse expensive-flight corner it over-flew substantially. | Generalizing the historical fixed-configuration result to fairly tuned CVaR/risk-aware families after F1. |
| Two-point CVaR mechanism | Analytic derivation plus sampled boundary audit | The two-point CVaR core is monotone and threshold-shaped. The complete implementation induces a state-dependent local threshold through distance, visibility, terrain cost, uncertainty, saturation, and state-machine gates. | “Risk-aware is exactly equivalent to one fixed threshold,” or presenting the local threshold analysis as proof of identical trajectories. |
| Stage 0 dynamics | Stage 0 manifest and code path | Stage 0 is an `ideal_set_model_state` task-level evaluation with a legacy energy/effort proxy. It supports regime and policy comparisons under that backend. | Calling Stage 0 dynamically faithful flight, RotorS, battery-energy evaluation, or hardware validation. |
| Energy-reachability supervisor | Stage 0 Scene D nominal/adversarial results; D2 replay and E0 closed-loop tests | The supervisor prevents the tested irreversible landing-reserve violations and converts unsafe continuations into safe aborts. In nominal Stage 0, irreversible failures changed 4/20 to 0/20 (`p=0.125`); in adversarial stress, 20/20 to 0/20 (`p=1.907e-6`). | Equating safe abort with mission success, or claiming universal safety. |
| RotorS energy model | D1/D2 calibration | `rotors_aero_shaft_mechanical_proxy_v1` is a rotor aerodynamic shaft-mechanical proxy derived from physical motor speed and RotorS coefficients. Held-out maximum underprediction was `847.70 J`; the frozen one-sided margin is `900 J`. | Battery electrical energy, whole-vehicle energy, endurance, or hardware energy prediction. |
| D2 budget | D2 and E0 frozen manifests | Within the registered envelope, the `31,400 J` takeoff gate is `4,900` takeoff + `21,900` air + `3,700` landing + `900` margin. The air term is hover `6,202.014` + outbound `5,927.564` + return `5,927.564` + settle `3,776.764 = 21,833.905 J`, rounded conservatively. | Applying the envelope outside height/time/distance/speed limits or double-counting it as measured battery energy. |
| RotorS representative validation | C2, E1a contact/H/C/R reports | A single RotorS-native `uav_v4` demonstrated finite ground/air ownership handoff, blocked-takeoff detection, repositioning to recent valid ground history, vertical takeoff, horizontal air control, landing, disarm, and ground reacquisition in representative closed-loop trials, without `set_model_state`. | Full Scene A success, Stage 0 policy-ranking replication, general planning, arbitrary landing recovery, or deployment readiness. |
| Scene A RotorS outcome | E1a-R final exact-scene evidence | The representative first recovery/handoff cycle completed, but the complete Scene A mission ended with `success=0`; the next blocked takeoff had no admissible recent valid history and was rejected fail-safe. | Any statement that the final RotorS Scene A mission succeeded. |
| Cross-backend relation | Stage 0 and RotorS evidence manifests | RotorS evidence establishes feasibility of representative execution and safety primitives; Stage 0 establishes the frozen policy/regime evidence. | Direct numerical comparison of their energy values or claiming dynamic replication of all Stage 0 conclusions. |

## Paper-facing contributions frozen after F1

1. A regime characterization showing that ground-failure recoverability changes when flight is worthwhile and can change policy ordering.
2. An analytic explanation that the two-point CVaR core is threshold-shaped while the implemented decision rule realizes a state-dependent local threshold.
3. A preregistered, equal-budget, new-seed held-out comparison showing regime-dependent relative advantages rather than a universal winner.
4. An energy-reachability supervisor and representative RotorS closed-loop validation showing that safe ground–air handoff, takeoff rejection, abort, reposition, landing, disarm, and ownership transfer are dynamically realizable in the tested cases.

## Mandatory manuscript qualifiers

- Use `ideal_set_model_state` whenever naming the Stage 0 backend.
- Use “legacy task-energy/effort proxy” for Stage 0 joule-labelled accounting.
- Use “aerodynamic shaft-mechanical energy proxy” for RotorS D1/D2 accounting.
- Use “representative closed-loop validation” for RotorS.
- Report Scene C McNemar `p=0.125` next to the `20/20` versus `16/20` rates.
- Report Scene C energy only on joint-success pairs for method-efficiency interpretation.
- State that fixed switching achieved 20/20 in both F1 scenes and was an untuned descriptive reference.

