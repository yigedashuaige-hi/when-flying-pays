# Project Final Engineering Status

**Frozen:** 2026-08-12  
**Workspace:** `$CATKIN_WS`  
**Status:** engineering scope closed; paper preparation may proceed  
**Authoritative companions:** `FINAL_CLAIM_LEDGER_20260812.md`,
`FINAL_EVIDENCE_MANIFEST_20260812.md`, and `PAPER_OUTLINE_20260812.md`

## A. Final project summary

This project studies when a terrestrial-aerial rescue robot should switch from
ground locomotion to flight under uncertain traversability. The paper-facing
strategy/regime/statistical evidence is the frozen Stage 0
`ideal_set_model_state` archive. It rejects the original CVaR-first hypothesis:
in the tested configurations, a tuned scalar threshold and often a periodic
fixed switch match or dominate the fixed CVaR implementation, while the assumed
stuck model materially changes the value of flight. RotorS is deliberately
narrower evidence. It validates one dynamically simulated `uav_v4`, finite-force
ground contact, measured ground/air ownership handoff, landing/disarm, a
shaft-mechanical energy proxy and short-envelope supervisor, obstacle-aware
takeoff clearance, fail-safe takeoff deferral, and historical valid-site
reposition. In frozen Scene A it completed one full
GROUND→blocked TAKEOFF→REPOSITION→TAKEOFF→AIR→LAND→DISARM→GROUND cycle, then
safely rejected a second takeoff because no recent ground-valid reverse path
existed. Full RotorS mission completion and strategy ranking are not claimed;
the project will not add a local planner merely to force Scene A DONE.

## B. Complete phase timeline

| Phase | Goal | Verdict | Core result | Problem and resolution | Paper evidence? | Final artifact |
|---|---|---|---|---|---|---|
| Stage 0 | Establish policies, shared uncertainty, stuck/slip regimes, paired statistics, and ideal safety supervisor | **PASS / frozen** | 20-seed paired B/C robustness, sparse corner, permanent/recoverable axis, Claim B nominal and stress evidence | CVaR-first headline rejected; threshold tuning fairness later narrowed the wording | **Yes**, primary strategy/regime evidence, always labelled ideal backend | `rescue_mission/results/STAGE0_RESULT_MANIFEST_20260718.md` |
| Phase A | Close Batch 2 and freeze provenance | **PASS** | Verified 10 B/C configurations ×20 seeds, created paired reports and immutable checksums | Stale “running” prose and ambiguous Scene D naming documented additively | **Yes**, provenance layer | `docs/superpowers/specs/2026-07-18-stage0-archive-change-record.md` |
| Phase B | Prove installed RotorS infrastructure independently | **PASS, diagnostic** | Stock Firefly takeoff, hover, 2 m move, return and controlled touchdown; topic/frame contract established | Stock vehicle did not disarm and is not the final hybrid robot | Infrastructure only | `rescue_worlds/results/rotors_smoke_20260718/PHASE_B_ROTORS_SMOKE_REPORT_20260718.md` |
| Phase C | Retrofit RotorS into the detailed-CAD `uav_v4` | **FAIL / stopped** | Topic/plugin/controller chain reached lift-off testing | Multi-link CAD/contact tree produced ODE NaN at high thrust; gain tuning stopped | Failure/architecture motivation only | `rescue_worlds/results/uav_v4_rotors_hybrid_smoke_20260718/PHASE_C_UAV_V4_ROTORS_REPORT_20260718.md` |
| Phase C2 | Rebuild one stable RotorS-native `uav_v4` | **PASS** | Single-primary-body model preserved 3.0440996088 kg aggregate properties; finite scans, Lee hover, and same-model ground→flight→land→ground smoke passed | Complex CAD collisions/joints replaced with stable proxies; failed Phase C retained | **Yes**, representative integration validation | `rescue_worlds/results/uav_v4_rotors_native_c2_20260718/PHASE_C2_ROTORS_NATIVE_REPORT_20260718.md` |
| Phase D1 | Add phase-resolved RotorS energy logging and initial calibration | **PASS with scope limit** | Shaft-mechanical proxy logged; 6 calibration +3 held-out; held-out MAE 526.37 J, RMSE 643.55 J, MAPE 2.40% | Not electrical/battery energy; small data set | Supporting methods/diagnostic evidence | `rescue_worlds/results/uav_v4_rotors_energy_d1_20260718/PHASE_D1_ROTORS_ENERGY_REPORT_20260718.md` |
| Phase D2 | Audit speed semantics, expand calibration, and add a conservative RotorS supervisor | **PASS within short envelope** | 12 calibration +6 held-out; staged model; max held-out underprediction 847.70 J; 31,400 J conservative floor; guarded validation had zero reserve violations | Distance/time collinearity rejected joint model; landing tail and short distance envelope remain limits | **Yes**, only as short-envelope mechanical-proxy validation | `rescue_worlds/results/uav_v4_rotors_energy_d2_20260719/PHASE_D2_ROTORS_ENERGY_SUPERVISOR_REPORT_20260719.md` |
| Phase E0 | Freeze D2 and exercise real closed-loop supervisor cases | **PASS** | Six RotorS closed-loop cases matched declared guarded/unguarded outcomes; no added tolerance/hysteresis | None inside declared smoke scope | **Yes**, representative safety validation | `rescue_worlds/results/phase_e0_e1_rotors_20260720/PHASE_E0_E1_REPORT_20260720.md` |
| Phase E1 | Backend Scene A regression | **PARTIAL / stopped** | Ideal reached DONE; RotorS also reached nominal DONE but was invalidated by a GROUND barrier launch | Hard planar velocity ownership injected contact energy; sent to E1a | Failure evidence, not a valid RotorS task result | same E0/E1 report |
| Phase E1a | Diagnose and minimally repair ground contact | **PARTIAL** | Finite-force servo contact microtests passed 15/15; ideal regression remained behaviorally consistent | Scene A then exposed an independent simultaneous ground/air handoff transient | Ground-contact validation only | `rescue_worlds/results/phase_e1a_contact_20260731/PHASE_E1A_REPORT_20260731.md` |
| Phase E1a-H | Implement measured staged ground/air ownership handoff | **PARTIAL** | Isolated handoff 20/20 and contact 15/15; first Scene A cycle worked | Second takeoff released horizontal motion beside a barrier before target height and inverted | Handoff validation yes; failed Scene A diagnostic only | `rescue_worlds/results/phase_e1a_h_handoff_20260731/PHASE_E1A_H_HANDOFF_REPORT_20260731.md` |
| Phase E1a-C | Add obstacle-aware swept-volume takeoff validity and abort | **PARTIAL** | Clearance matrix 40/40, handoff 20/20, contact 15/15; invalid Scene A takeoff stayed GROUND/motor zero | Original route continued into a wall because no deferred-takeoff reposition existed | Clearance/abort validation yes; mission timeout diagnostic only | `rescue_worlds/results/phase_e1a_c_takeoff_clearance_20260812/PHASE_E1A_C_TAKEOFF_CLEARANCE_REPORT_20260812.md` |
| Phase E1a-R | Add minimal backtrack to recent stable valid takeoff pose | **PARTIAL / final freeze** | Final 15/15 recovery matrix; one complete frozen Scene A recovery-flight-land-disarm cycle | Second sortie had no recent reverse-ground-valid history; correctly fail-safe rejected; planner expansion intentionally declined | **Yes**, representative recovery primitive; not full-task success | `rescue_worlds/results/phase_e1a_r_takeoff_reposition_20260812/PHASE_E1A_R_TAKEOFF_REPOSITION_REPORT_20260812.md` |

`PARTIAL` means its isolated engineering acceptance passed but its broader
Scene A end-to-end objective did not. It must not be rewritten as a full mission
pass.

## C. Final system capability matrix

| Capability | Status | Frozen interpretation |
|---|---|---|
| Single-model RotorS flight | **PASS** | One RotorS-native `uav_v4`, not a ground robot plus Firefly |
| Ground finite-force dynamics | **PASS** | Head-on/oblique/corner contact microtests; no hard velocity teleport |
| Measured ground/air handoff | **PASS** | Brake, zero-wrench release, arm, vertical climb, measured clearance |
| Landing/disarm/ground reacquisition | **PASS** | Measured touchdown, motor-zero disarm, then ground ownership |
| Mechanical energy logging | **PASS** | `rotors_aero_shaft_mechanical_proxy_v1`; not battery energy |
| Short-envelope supervisor | **PASS** | Validated only within D2 height/hover/distance/speed envelope |
| Obstacle-aware takeoff clearance | **PASS** | Local conservative Gazebo collision-AABB observer contract |
| Blocked-takeoff abort/defer | **PASS** | Invalid site holds GROUND and motor zero; ascent fault lands/disarms |
| Local historical reposition | **PASS** | Bounded backtrack to recently traversed, stable-valid ground pose |
| One complete recovery-flight-land cycle | **PASS** | Frozen Scene A, fixed_switch, seed 20260720, recoverable, RotorS |
| Full Scene A RotorS mission success | **NOT CLAIMED** | Final run `success=0`; second sortie safely rejected |
| Scene B/C RotorS strategy ranking | **NOT CLAIMED** | No RotorS Scene B/C regime batch was run |
| Full-task RotorS energy guarantee | **NOT CLAIMED** | 96.25% of audited B/C sorties exceed D2 1.2 m distance envelope |
| Battery-energy safety | **NOT CLAIMED** | No electrical model, voltage/current, ESC efficiency, or battery dynamics |
| General local planning | **NOT CLAIMED** | Recovery is finite historical backtrack, not A*/DWA/local mapping |
| Arbitrary landing-site recovery | **NOT CLAIMED** | No guarantee after landing without a recent reverse-valid ground path |
| Real-hardware validation | **NOT CLAIMED** | All dynamic validation is ROS/Gazebo/RotorS simulation |

## D. Paper-usable and non-usable evidence

| Evidence | Use | Restrictions |
|---|---|---|
| Stage 0 Batch 2 B/C and sparse corner | **Paper-facing primary evidence** | State `backend=ideal_set_model_state`; label stuck model; apply fairness-qualified threshold wording |
| Stage 0 permanent/recoverable pilot | **Paper-facing regime evidence** | Five-seed pilot; do not present as a powered universal ranking |
| Stage 0 Claim B | **Paper-facing safety evidence** | Nominal result is conservative safe abort; adversarial result is a mechanism stress test |
| Two-point CVaR derivation and boundary audit | **Paper-facing analysis** | Claim threshold-shaped/state-dependent local boundary, not exact equality to one fixed tau |
| C2 same-model RotorS smoke | **Paper-facing validation** | Demonstrates feasibility, not policy ranking |
| D2 mechanical proxy/supervisor | **Paper-facing validation** | Short calibrated envelope only; never call battery energy or mission guarantee |
| E1a contact, H handoff, C clearance, R reposition matrices | **Paper-facing engineering validation** | Present acceptance counts and contracts, not full mission efficacy |
| Final E1a-R Scene A sequence | **Paper-facing case study** | One representative seed/cycle; disclose contact during initial retreat and final `success=0` |
| Phase C NaN, E0 barrier launch, H/C/R failed Scene A attempts | **Diagnostic/limitations only** | Preserve as engineering audit trail; do not pool with accepted trials |
| Firefly Phase B | **Infrastructure only** | Never call it `uav_v4` hybrid validation |
| E1a-R `dev`, arrival-fix, yaw-recheck directories | **Diagnostic only** | Explicitly non-acceptance development evidence |
| Cross-backend energy comparison | **Forbidden quantitatively** | Ideal legacy energy and RotorS mechanical proxy are different metrics |

## Frozen blockers and scope decision

The remaining RotorS mission blocker is not basic flight, handoff, contact,
clearance, or local backtracking. It is recovery after a flight lands in an
invalid area with no recent ground-traversed valid path. General solution would
require persistent safe-site semantics plus route feasibility or a local
planner. That capability is outside the frozen contribution and will not be
implemented unless a future submission explicitly requires full RotorS task
completion.

Scientific blockers are separate: threshold and CVaR did not receive equal
tuning budgets or independent held-out evaluation; the two-point derivation
does not make the full implementation identical to a fixed threshold; D2 does
not cover normal B/C sortie distances; and no real-hardware data exist.

## Two allowed future routes — planning only

### Route A: finish now — recommended

Stop engineering and all new simulation batches. Write the paper or student
project report around Stage 0 regime findings plus representative RotorS
validation. Lower the claim from complete task-level RotorS replication to
validated dynamic/safety primitives, and state the planner, energy-envelope,
tuning-fairness, and hardware limitations explicitly.

### Route B: continue research

Priority is fixed:

1. Equal-budget CVaR/threshold tuning on new tuning seeds and one untouched
   held-out paired evaluation set.
2. Formalize the two-point CVaR threshold proposition and its assumptions.
3. Add a lightweight regime-map formalization over hazard severity,
   recoverability, and flight cost.
4. Extend the RotorS energy envelope only if task-level RotorS safety is an
   explicit publication requirement.
5. Validate on hardware last.

Do not resume local-planner work by default.

## Final recommendation

Freeze engineering now and write. The most defensible headline is:

> Under the tested ideal-tracking regimes, the value of flight depends strongly
> on hazard density/severity and whether ground failure is permanent or
> recoverable; a fixed CVaR policy did not outperform simpler switching rules.
> Representative RotorS closed-loop trials independently validate that the
> same-model ground-air handoff and safety primitives are dynamically feasible,
> while exposing explicit limits on planning and energy-envelope generality.

If exactly one additional scientific experiment is permitted, run the
equal-budget, independently held-out paired CVaR-versus-threshold comparison.
It resolves the largest threat to the paper's algorithmic interpretation
without reopening the RotorS engineering stack.

