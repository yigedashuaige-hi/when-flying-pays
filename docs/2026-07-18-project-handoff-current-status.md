# Project Handoff — Current State, Evidence, and Next Work

**Last reconciled:** 2026-07-18  
**Workspace:** `$CATKIN_WS`  
**Primary packages:** `rescue_mission`, `rescue_worlds`  
**Purpose:** This is the first document a new GPT/Claude agent should read. It
summarizes the current truth without erasing the older pre-registrations and
failed hypotheses that explain how the project arrived here.

---

## 0. Instructions to the next agent

Do not start by redesigning the algorithm or launching another overnight batch.
First verify the claims and file paths in this handoff against the workspace.
Treat the older documents as an audit trail: several contain conclusions that
were later refuted, and their historical wording must not be mistaken for the
current paper claim.

The immediate project boundary is:

- ROS 1 Noetic + Gazebo Classic 11, normally run in Docker container
  `uav_noetic`.
- Keep the current ROS 1/Gazebo Classic paper track. Do not migrate to ROS 2,
  PX4, or Gazebo Harmonic during RotorS integration.
- No RL or PEEK work in this phase.
- Preserve all existing strategies, CSV columns, result directories, and
  pre-registration documents.
- Prefer additive changes and a selectable backend. The existing ideal
  `set_model_state` backend must remain available as a regression reference.
- Do not claim final flight-controller or calibrated energy results until a
  real RotorS dynamics/controller loop has replaced ideal air motion.

Recommended reading order:

1. **This handoff**.
2. `2026-06-11-paper-pivot-framework.md` — current scientific framing, but its
   statements that Batch 2 is still running are stale.
3. `2026-06-11-claim-a-regime-prereg.md` — pre-registered Claim A decision and
   the sparse-corner result.
4. `2026-06-10-claim-a-paper-materials.md` — finalized pre-pivot tables,
   limitations, and safety-supervisor evidence. Read with the pivot warning.
5. `top-tier-experiment-and-ablation-plan.md` — implementation/history ledger.
6. `2026-06-08-risk-aware-terrestrial-aerial-rescue-EXEC-v3.1.md` — original
   execution specification. Its mechanics and Stage 0/1/2 rule remain useful;
   its CVaR-first contribution framing has been superseded.
7. `ros-gazebo-environment-notes.md` — runtime environment.

---

## 1. One-paragraph project summary

This project studies mode selection for a terrestrial-aerial bimodal rescue
robot under uncertain ground traversability. A shared perception node publishes
ground-truth and noisy perceived traversability; policies choose GROUND/AIR;
terrain physics applies seed-keyed stuck/slip outcomes; a mission state machine
runs an outbound delivery and return task; and a logger records energy, success,
stuck/slip, decisions, and safety-supervisor metrics. The original hypothesis
was that a two-point CVaR `risk_aware` policy would be the best way to use flight
as tail-risk avoidance. Controlled experiments rejected that headline: a tuned
scalar threshold on perceived risk, and often even a periodic fixed switch,
matches or dominates CVaR. The paper has therefore pivoted to a regime study of
*when flying pays off*, the decisive effect of permanent-trap versus recoverable-
slip modeling, an honest negative result on sophisticated risk functionals, and
a decision-layer energy-reachability supervisor. All present flight experiments
still use ideal pose-setting, so RotorS integration is the next engineering gate.

---

## 2. Runtime architecture as it exists today

Main launch:

```text
rescue_phase1_demo.launch
  -> rescue_worlds/disaster_gazebo.launch
       -> Gazebo Classic disaster_<scene>.world
       -> spawn uav_v4 URDF as model "uav_v4"
  -> rescue_mission/rescue_mission.launch
       -> mission_state_machine.py
       -> scenario_sensor_sim.py
       -> mode_switcher.py
       -> ground_goal_controller.py
       -> air_goal_bridge.py
       -> air_motion_executor.py       [CURRENT IDEAL AIR BACKEND]
       -> rescue_metrics_logger.py
```

Key data flow:

```text
seeded terrain truth + noisy perception
             |
             v
      shared /rescue/* topics
             |
             v
        mode_switcher
   proposed mode -> safety supervisor -> approved /rescue/mode
          |                              |
          | GROUND                       | TAKEOFF/AIR/LAND
          v                              v
ground_goal_controller             air_goal_bridge
          |                              |
 /mecanum/cmd_vel               /rescue/air_goal
                                         |
                                         v
                              air_motion_executor
                              /gazebo/set_model_state
```

Important modules:

- `scenario_sensor_sim.py`: shared perception and seed-keyed terrain signal.
- `terrain_field.py`: deterministic common-random-number field by
  `(seed, cell, salt)`.
- `mode_switcher.py`: strategies and decision costs.
- `ground_goal_controller.py`: ground tracking plus permanent/recoverable
  stuck/slip model.
- `safety_supervisor.py`: ROS-free decision-layer reachability guard.
- `rescue_metrics_logger.py`: sole task-level battery owner and per-row CSV
  logger.
- `run_phase1_experiments.py`: repeated paired-seed Gazebo runner.
- `summarize_results.py`, `analyze_significance.py`, plotting scripts: analysis.

Supported strategy family includes at least:

- `ground_only`
- `air_preferred`
- `fixed_switch`
- `energy_rule`
- `threshold_switch`
- `risk_aware`
- `risk_aware_safety`
- experimental `risk_aware_active`

Supported scene configs/worlds include `scene_a`, `scene_b`, `scene_c`,
`scene_d`, `scene_e`, and `scene_sparse`.

---

## 3. Work already completed

### PR-1 — shared perception and fairness logging: complete

The perception node publishes shared traversability mean/uncertainty, obstacle
density, terrain cost, visibility, and stuck-risk signals. The logger carries
them into CSV, and `check_shared_inputs.py` exists. In position-scripted B/C
scenes, policies visit different cells, so distribution-level equality is not a
valid fairness test; fairness there is code-level and seed/cell-level CRN.

### PR-2 — energy and risk-aware decisions: complete

`energy_rule`, `risk_aware`, decision-cost logging, two-point CVaR, and terminal
landing behavior are implemented. `threshold_switch` was later added as the
decisive tuned baseline.

### PR-3 — stuck/slip physics and Scene B/C: complete

Ground truth is separated from perceived traversability. Stuck outcomes are
deterministic per seed/cell for fair paired comparisons. Both permanent-trap and
recoverable-slip variants are available. Scene B and C produce meaningful
hazard crossings and multi-seed variation.

### PR-4 — decision-layer safety supervisor: complete

The logger owns the joule budget. The supervisor monitors all policies and
enforces only for guarded variants. It blocks takeoff when the remaining energy
cannot cover takeoff, air reserve, landing, and margin; it also detects the
irreversible airborne-below-landing-energy state.

### Statistical hardening and ablations: complete

- Common Random Numbers by seed/cell.
- 20-seed paired batches.
- Wilcoxon signed-rank with rank-biserial effect size and bootstrap CI.
- Exact McNemar for binary outcomes.
- Full / without uncertainty / without CVaR / without switch penalty ablations.
- Calibrated heteroscedastic perception diagnostic.
- Result and paper-figure tooling.

### Active-recoverability pilot: NO-GO, deferred

The pre-registered Scene E pilot did not improve mission success. All conditions
had zero success, partly because the permanent-trap model caught the vehicle in
an earlier hazard before the late-hazard foresight mechanism could be tested.
Per the pre-registration, do not promote this as a contribution and do not tune
Scene E to rescue it. A recoverable-slip re-test is optional future work, not the
current priority.

---

## 4. Current scientific conclusion

The old headline “CVaR risk-aware switching is the superior multimodal policy”
is false in the tested setup and must not be revived by scene/parameter tuning.

Current four-part paper direction:

1. Perceived-risk-based mode switching is valuable in dense-hazard and/or
   catastrophic stuck regimes, but is not universally necessary.
2. A tuned scalar threshold on `(1 - p_trav)` matches or dominates the richer
   CVaR formulation. This is an honest negative result with a mechanistic reason:
   for a one-dimensional two-point outcome, CVaR induces a monotone boundary that
   is effectively threshold-like, while the current uncertainty inflation adds
   little validated information.
3. The stuck-model assumption is a decisive experimental axis. Permanent traps
   reward frequent/blind flight; recoverable slips reward selective switching;
   sparse recoverable hazards can make flying unnecessary.
4. A decision-layer energy-reachability supervisor prevents the irreversible
   “airborne without enough energy to land” state. This is safe-abort/catastrophe
   prevention, not guaranteed mission completion.

Do not use CVaR in the title or as the primary contribution. Do not hide that
`fixed_switch` and tuned thresholds beat it.

---

## 5. Experiment record and final known results

### 5.1 Earlier paper-hardening result

In the original permanent-trap 20-seed Scene B/C experiments, `risk_aware`
substantially beat the non-flying `energy_rule` on stuck and success, but was
Pareto-dominated by `fixed_switch`. That finding triggered the pre-registered
regime sweep and paper pivot.

### 5.2 Claim A regime sweep and sparse corner

The recoverable Scene B pilot showed 100% success for `fixed_switch`, tuned
threshold, and `risk_aware`, but CVaR used much more energy. The permanent model
also failed to produce a CVaR win.

The one possible NARROW corner was then run:

`scene_sparse`, recoverable slip, `fly_k1=3.0`, 10 paired seeds.

| strategy | success | mean energy |
|---|---:|---:|
| energy_rule | 1.00 | 7.3 J |
| tuned threshold, tau=0.6 | 1.00 | 91.0 J |
| fixed_switch | 1.00 | 135.5 J |
| risk_aware | 1.00 | 258.4 J |

This locked the pre-registered DOWNGRADE verdict. In this sparse+recoverable
corner, flying was unnecessary; CVaR over-flew by the widest margin.

### 5.3 Batch 2 robustness — complete, despite stale docs saying “running”

Source directories:

- `results/runs_claim_a_airtight/scene_b_recoverable_*`
- `results/runs_claim_a_airtight/scene_c_recoverable_*`
- `results/airtight_run_batch2.log`

Every Batch 2 configuration contains 20 run CSVs. The log ends with
`Claim A airtight batches DONE` and the final process return code is zero. The
many `ROSInterruptException` tracebacks in the log occur during normal roslaunch
shutdown and do not invalidate the completed CSVs.

Final aggregate values:

| scene | strategy | n | success | energy J | air distance m | stuck events |
|---|---|---:|---:|---:|---:|---:|
| B | fixed_switch | 20 | 1.00 | 48.306 | 9.912 | 0.0 |
| B | threshold tau=0.2 | 20 | 1.00 | 67.161 | 11.605 | 0.0 |
| B | threshold tau=0.3 | 20 | 0.95 | 49.687 | 9.077 | 5.3 |
| B | threshold tau=0.4 | 20 | 0.85 | 55.835 | 9.080 | 19.45 |
| B | risk_aware | 20 | 0.95 | 80.576 | 15.093 | 0.0 |
| C | fixed_switch | 20 | 1.00 | 71.102 | 12.978 | 7.4 |
| C | threshold tau=0.2 | 20 | 1.00 | 90.204 | 15.569 | 0.0 |
| C | threshold tau=0.3 | 20 | 1.00 | 71.676 | 11.923 | 0.0 |
| C | threshold tau=0.4 | 20 | 0.80 | 57.388 | 8.919 | 31.9 |
| C | risk_aware | 20 | 0.95 | 90.257 | 17.716 | 0.0 |

Interpretation:

- Scene B: `fixed_switch` strictly dominates `risk_aware` in aggregate
  success/energy. Threshold tau=0.2 also has higher success and lower energy.
- Scene C: `fixed_switch` and threshold tau=0.3 both have higher success and
  roughly 19 J lower mean energy than `risk_aware`.
- The 20-seed robustness batch strengthens the DOWNGRADE conclusion; it does not
  merely reproduce the 5-seed pilot.
- Do not rerun this batch unless a code change directly affects its semantics.

### 5.4 Claim B safety-supervisor evidence

Nominal Scene D, paired 20 seeds:

- unguarded success: 0/20
- guarded success: 0/20
- unguarded irreversible failure: 4/20
- guarded irreversible failure: 0/20
- recall: 1.00; precision: 0.20
- McNemar for irreversible reduction: p=0.125

Adversarial forced-flight stress test:

- irreversible failure: 20/20 -> 0/20
- exact McNemar p about 1.9e-6

Correct wording: the supervisor prevents catastrophic irreversible failure and
converts it into safe abort at no observed success cost in an already unwinnable
scene. It is conservative and does not make the mission succeed.

---

## 6. RotorS status — what exists and what does not

### Already present

- Full `rotors_simulator` source tree in the workspace.
- RotorS packages have build/devel artifacts.
- Gazebo model/plugin paths already mention RotorS.
- `air_goal_bridge.py` can optionally publish `geometry_msgs/PoseStamped` to a
  configurable RotorS command-pose topic.
- `rescue_air_goal_to_rotors.launch` enables that publisher.
- RotorS `lee_position_controller_node` accepts `command/pose`, so the proposed
  message type is compatible.

### Not actually integrated

- `rescue_mission.launch` always starts `air_motion_executor.py`, which directly
  calls `/gazebo/set_model_state`.
- `publish_rotors_command_pose` defaults to false.
- The task world spawns `uav_v4`, not a RotorS vehicle.
- The `uav_v4` URDF currently has the planar ground-move plugin but no RotorS
  motor-model, IMU, odometry, rotor, or multirotor dynamics plugin chain.
- The main task launch does not start a Lee position controller.
- Namespace/remap wiring for `/command/pose`, odometry, and motor commands is not
  part of the task launch.
- TAKEOFF/AIR/LAND completion currently assumes commanded ideal motion rather
  than closed-loop measured flight state.
- Air energy is still a distance/hover proxy, not inferred from motor speed,
  thrust, or electrical power.

Therefore the correct status is: **RotorS is installed and a partial command
bridge exists, but there is no end-to-end RotorS flight loop for the bimodal
robot.** Current results are Stage 0 results under ideal tracking.

The central engineering decision is unresolved: a separate stock Firefly can be
used for a quick standalone smoke test, but it is not a valid final TABV system.
The final integration must use one consistent robot/state. Likely options are:

1. Retrofit RotorS motor/odometry/IMU plugins and controller parameters into the
   `uav_v4` hybrid model while retaining ground locomotion; or
2. Build a RotorS-based hybrid model derived from a stock multirotor and add the
   ground mechanism.

Do not silently spawn a ground `uav_v4` and a separate flying Firefly and report
them as one robot.

---

## 7. Recommended next execution plan

### Phase A — close the Stage 0 record

1. Recompute/persist paired significance and effect-size reports for the Batch 2
   comparisons that will appear in the paper.
2. Update the stale “Batch 2 running” statements in the pivot/prereg documents
   to “complete,” with the final table above. Preserve their historical text or
   clearly label the update date.
3. Create a compact result manifest mapping every paper table/figure to its CSV
   source and simulation backend (`ideal_set_model_state`).

Acceptance: another agent can reproduce every Stage 0 number from a named CSV
without guessing which result directory is canonical.

**Completion addendum (2026-07-18):** Phase A is complete. Batch 2 paired
reports, separate nominal/adversarial Scene D reports, canonical source mapping,
and data caveats are archived in
`rescue_mission/results/STAGE0_RESULT_MANIFEST_20260718.md`. The exact file
changes and pre-edit backup hashes are recorded in
`docs/superpowers/specs/2026-07-18-stage0-archive-change-record.md`. No Stage 1
RotorS/model work was started as part of this closure.

### Phase B — standalone RotorS smoke test

Before changing the hybrid model, prove the installed RotorS stack works in the
current container and disaster world:

1. Spawn one stock RotorS vehicle in a disaster world.
2. Start its odometry plugin and Lee position controller.
3. Send command poses for takeoff, hover, horizontal translation, and landing.
4. Verify command, odometry, actuator/motor-speed topics and stable tracking.
5. Record the exact namespace and topic contract.

This smoke test is infrastructure validation only, not a paper result.

Acceptance: no `/gazebo/set_model_state`; measured altitude reaches the commanded
hover, horizontal error converges, landing is controlled, and all required
topics are documented.

**Completion addendum (2026-07-18):** Phase B passed with a standalone stock
RotorS Firefly in a Scene-A disaster-world copy containing the required RotorS
ROS-interface world plugin. Takeoff, stable hover, a 2 m horizontal move, return,
and position-controlled touchdown all converged under the Lee controller. IMU,
odometry, commanded actuators, measured motor speeds, and Gazebo model state were
observed without a `set_model_state` client or call path. This is infrastructure
evidence only, not a TABV result. The complete command/topic/frame contract,
metrics, caveats, and file changes are frozen in
`rescue_worlds/results/rotors_smoke_20260718/PHASE_B_ROTORS_SMOKE_REPORT_20260718.md`.
The stock Lee controller continues producing balancing thrust after touchdown;
Phase C therefore needs an explicit LAND-complete/disarm or controller-handoff
interface. No Phase C work was started.

### Phase C — selectable backend and single-robot hybrid integration

1. Add a launch argument such as `air_backend:=ideal|rotors`.
2. Keep `ideal` behavior unchanged for regression.
3. Under `rotors`, do not start `air_motion_executor.py`; start the RotorS
   controller/plugin chain and enable `air_goal_bridge` publishing.
4. Use one robot model and one odometry/state source for ground and air.
5. Gate/coordinate the planar ground plugin so it does not fight multirotor
   dynamics in AIR, and ensure zero rotor thrust does not break ground behavior.
6. Use measured altitude/velocity to define TAKEOFF and LAND completion rather
   than elapsed/commanded motion alone.

Acceptance: the same model can ground-drive, take off, fly across a Scene B risk
zone, land, resume ground travel, deliver, return, and finish once under a smoke
strategy. There must be no pose teleportation.

**Stop addendum (2026-07-18):** A direct detailed-CAD `uav_v4` retrofit was
implemented far enough to build, expose the complete `/uav_v4` RotorS topic
chain, drive on the ground, and isolate motor dynamics, but it is **not a Phase-C
pass**. At lift-off thrust the Gazebo ODE state becomes non-finite even under
equal open-loop motor commands, independently of Lee control. Per the explicit
stop rule, controller tuning and the remaining sequence were halted. The
failure evidence, partial file list, exact physical parameters, topic contract,
and two single-robot recovery architectures are in
`rescue_worlds/results/uav_v4_rotors_hybrid_smoke_20260718/PHASE_C_UAV_V4_ROTORS_REPORT_20260718.md`.
The recommended restart is a RotorS-native single-rigid-body `uav_v4` using the
measured aggregate mass/inertia and CAD visuals, not a separate Firefly and not
continued tuning of the unstable multi-link contact tree. Phase D was not
started.

**Phase C2 completion addendum (2026-07-18):** The recommended RotorS-native
single-primary-body architecture was implemented and passed the requested
three gated smoke stages. The model exactly preserves the `3.0440996088045096
kg` aggregate mass, COM, inertia, and four measured rotor locations while using
CAD only as fixed visual geometry, a primitive body collision, and optional
fixed wheel proxies. Pure-flight and wheel-proxy configurations remained finite
through the `1000 rad/s` scan and both achieved stable Lee hover. The same
`uav_v4` then completed ground drive, measured takeoff, hover, horizontal move,
return, measured landing/disarm, and resumed ground drive with no non-finite
sample, no Firefly, final zero motor speed, and no RotorS `set_model_state`
client. The IMU frame is now `uav_v4/base_link`; ideal-backend short regression
also passed and the Stage-0 latest CSV mirror was restored byte-for-byte. Full
evidence and limitations are archived in
`rescue_worlds/results/uav_v4_rotors_native_c2_20260718/PHASE_C2_ROTORS_NATIVE_REPORT_20260718.md`;
the edit/backup record is
`docs/superpowers/specs/2026-07-18-phase-c2-rotors-native-change-record.md`.
Phase D and paper batches were not started.

### Phase D — energy and supervisor calibration

1. Log RotorS actuator/motor-speed/thrust signals and measured flight distance,
   duration, takeoff, hover, and landing phases.
2. Define and document a defensible energy proxy or motor-power model. Keep the
   old proxy columns for compatibility; add new columns rather than renaming.
3. Recalibrate takeoff/air/landing energy estimates and the safety supervisor's
   joule margins from RotorS trials.
4. Test supervisor monitoring/enforcement on winnable and unwinnable low-margin
   cases; retain the honest safe-abort framing.

Acceptance: predicted reserve and measured RotorS energy use are compared on
held-out maneuvers, and the supervisor never approves a maneuver whose measured
landing reserve violates the invariant in the test set.

**Phase D1 completion addendum (2026-07-18):** RotorS mechanical-energy logging
and a small held-out calibration are complete; supervisor recalibration is not.
The logger keeps all 55 historical fields unchanged and appends backend,
energy-model, phase, commanded/actual motor speed, odometry, duration, distance,
and five phase-specific mechanical-energy records. The declared model is the
RotorS reaction-torque shaft proxy
`P = k_f k_m sum(|omega_i|^3)`, using `k_f=1.568e-05`, `k_m=0.06`, and
slowdown-corrected actual motor speed. It is explicitly not battery electrical
energy. Six calibration and three untouched held-out takeoff/hover/translation/
return/landing trials all remained finite and disarmed to zero speed. A compact
descriptor fit achieved held-out MAE `526.37 J`, RMSE `643.55 J`, and MAPE
`2.40%` on the mechanical proxy. Ideal behavior and old fields passed a short
regression, and the Stage-0 latest mirror was restored byte-for-byte. Evidence,
formula limits, CSV schema, and D2 blockers are in
`rescue_worlds/results/uav_v4_rotors_energy_d1_20260718/PHASE_D1_ROTORS_ENERGY_REPORT_20260718.md`;
the backup/edit ledger is
`docs/superpowers/specs/2026-07-18-phase-d1-rotors-energy-change-record.md`.
No supervisor batch, safety regime, Phase E, or paper batch was started.

### Phase E — experiment reruns

Run in increasing cost order:

1. Scene A regression for old strategies, ideal and RotorS backend.
2. Single-seed Scene B mission smoke.
3. Five paired seeds for the smallest decisive strategy set.
4. Only after stability, the paper-facing multi-seed regime and Claim B batches.

The minimum paper-facing RotorS strategy set should follow the pivot, not the old
CVaR-first story: `ground_only`/`energy_rule`, `fixed_switch`, tuned
`threshold_switch`, `risk_aware`, and guarded/unguarded supervisor comparisons
where applicable.

---

## 8. Risks and non-negotiable honesty constraints

- Existing CSV energy values are proxy values under ideal pose control. Never
  mix them in one quantitative table with RotorS energy without a backend label.
- A standalone Firefly smoke test is not evidence for the `uav_v4` hybrid.
- The current `uav_v4` planar plugin may conflict with free-flight dynamics; test
  this explicitly rather than assuming coexistence.
- Permanent-trap and recoverable-slip results must always be labeled. Never
  silently swap the stuck model to improve a policy.
- Threshold tuning is allowed only as the explicitly tuned baseline described in
  the pre-registration. Do not tune scenes or costs to make CVaR win.
- Shared-perception fairness means identical perception mechanism and CRN truth
  for a seed/cell, not identical observed sample distributions after strategies
  choose different trajectories.
- `k_unc` has no demonstrated significant benefit even under calibrated
  heteroscedastic noise; it is auxiliary/future-work material.
- Active recoverability is a recorded NO-GO and is not the next priority.
- Safe abort is not safe completion.
- Preserve all original result directories. New backend runs need distinct,
  timestamped directories and explicit backend metadata.

---

## 9. Suggested GPT/Claude collaboration protocol

Use one agent as implementer and the other as reviewer/experiment auditor; do
not let both edit the same launch/model files concurrently.

For each step:

1. Implementer states the exact acceptance test before editing.
2. Reviewer checks the proposed change against this handoff, the pre-registration,
   and current launch/topic contracts.
3. Implementer makes a small additive change and runs the cheapest smoke test.
4. Reviewer examines logs/CSV schema and checks for hidden `set_model_state`,
   topic namespace mistakes, mixed backends, or changed experiment semantics.
5. Record the outcome and canonical artifact path in a dated progress note.

Good task split for the next iteration:

- **Agent A:** Stage 0 result manifest, Batch 2 significance, stale-doc update.
- **Agent B:** read-only RotorS launch/model/topic audit and standalone smoke-test
  design.
- Then jointly review the single-robot hybrid design before either agent edits
  `uav_v4` or the main launch files.

Avoid launching long batches while either agent is still changing the model,
controller, logger, or experiment semantics.

---

## 10. Current checkpoint in one sentence

**The Stage 0 algorithm/regime/safety experiments are complete and have already
rejected CVaR as the headline; the next task is to close their documentation and
then replace ideal `set_model_state` flight with a verified, single-robot RotorS
dynamics/controller/energy loop before generating final paper results.**

**Checkpoint addendum (2026-07-18 after Phase C2):** the single-robot RotorS
dynamics/controller/ground-air handoff smoke is now verified. The next allowed
phase is Phase D energy/supervisor calibration, but it has not been started.

**Checkpoint addendum (2026-07-18 after Phase D1):** RotorS phase-resolved
shaft-mechanical proxy logging and small held-out calibration are verified. The
next task is a reviewed Phase D2 choice of electrical-versus-mechanical reserve,
uncertainty margin, and supervisor interface; D2 has not been started.

**Checkpoint addendum (2026-07-19 after Phase D2):** RotorS speed/slowdown
semantics are source-confirmed, and a predeclared 18-trial set (12 calibration,
6 held-out) completed with finite phase-resolved mechanical-proxy records and
measured disarm. Translation time and distance were effectively perfectly
collinear, so the joint D1 descriptor was rejected for supervision in favor of
a staged takeoff/hover/translation/landing model. Held-out maximum total
underprediction was `847.70 J`; the conservative RotorS-only budget is
`4900 + 21900 + 3700 + 900 = 31400 J` of shaft-mechanical proxy, with landing
tail handled separately. Guarded decision replay had zero reserve violations,
allowed clearly safe and near-boundary cases, blocked an unaffordable takeoff,
and forced landing for an airborne low-margin case. Ideal supervisor behavior
matched its backup in 448/448 cases, the old 55-column CSV prefix remained
identical, and the Stage-0 canonical mirror hash is unchanged. Full evidence is
in
`rescue_worlds/results/uav_v4_rotors_energy_d2_20260719/PHASE_D2_ROTORS_ENERGY_SUPERVISOR_REPORT_20260719.md`;
the edit/backup ledger is
`docs/superpowers/specs/2026-07-19-phase-d2-rotors-energy-supervisor-change-record.md`.
This proxy is not battery electrical energy, its bound is only validated inside
the short D2 maneuver envelope, and Phase E/final regime study has not started.

**Checkpoint addendum (2026-07-20, Phase E0/E1 stopped during Scene A):** the
D2 configuration and implementation hashes were frozen, and six real
ROS/Gazebo/RotorS supervisor cases passed their declared expectations. Guarded
tests approved a clearly sufficient flight, blocked an insufficient preflight,
forced landing after an airborne low-margin transition, and produced zero
reserve violations. The matching unguarded low-margin case continued AIR and
recorded one conservative-budget violation before the test harness landed and
disarmed it. Exact and 0.1 J-below takeoff-boundary probes behaved correctly;
no tolerance or hysteresis was added. A same-seed Scene A `fixed_switch` ideal
run reached DONE with the legacy backend unchanged. The RotorS run also reached
DONE and remained finite, but failed the ground-dynamics regression: while in
GROUND with measured motor speed zero, contact with `low_barrier` near x=7 m
launched the model to z=2.4766 m (ground speed peaked at 5.004686 m/s). The
mode-aware planar plugin preserves the ODE-generated Z velocity while imposing
XY velocity, so the obstacle impulse is not constrained by the planar ownership
contract. This is not a supervisor-budget failure and was not hidden by scene or
policy tuning. Scene B and the five-seed pre-experiment were not started. The
next allowed action is a separately reviewed, minimal ground-contact/planar
ownership fix followed by a repeat of Scene A; do not enter the RotorS paper
batch before that regression passes. Evidence is in
`rescue_worlds/results/phase_e0_e1_rotors_20260720/PHASE_E0_E1_REPORT_20260720.md`.

**Checkpoint addendum (2026-07-31, Phase E1a/E1a-H stopped during Scene A):**
the original hard planar-contact defect was replaced by the finite-force ground
servo and its head-on/oblique/corner regressions remain 15/15. A centralized
RotorS handoff now implements measured brake, zero-wrench release, latched-XY
vertical takeoff, clearance confirmation, latched-XY vertical landing,
touchdown settle, measured motor-zero disarm, and ground reacquisition. Four
isolated classes x five fresh runs passed 20/20 with no NaN/Inf or dual
ownership. The same-condition Scene A run completed its first full handoff but
failed its second flight: near x=7.317 m, the fixed 0.30 m ground-relative
clearance admitted horizontal tracking at z=0.5665 m beside the 0.70 m barrier;
the return command crossed the barrier, the robot became inverted at z~0.75 m,
and LAND could not establish ground-relative touchdown before timeout. Per the
stop rule, no post-failure tuning or ideal regression was run. E1b, Scene B,
five-seed, formal RotorS, and Claim B runs remain forbidden. Evidence is in
`rescue_worlds/results/phase_e1a_h_handoff_20260731/PHASE_E1A_H_HANDOFF_REPORT_20260731.md`;
the edit ledger is
`docs/superpowers/specs/2026-07-31-phase-e1a-h-handoff-change-record.md`.

---

## Superseded status notice — 2026-08-12 Phase F0

This file remains the chronological handoff and is intentionally not deleted or
rewritten. Its earlier “next phase” statements are historical. The engineering
scope is now frozen after E1a-C and E1a-R: the final RotorS Scene A trial
completed one full blocked-takeoff recovery, flight, landing, disarm, and ground
reacquisition cycle, then safely rejected a later sortie because no recent
ground-valid reverse path existed. The project will not add a local planner to
force full Scene A DONE, and no Scene B/E1b/RotorS regime batch is authorized.

The current authoritative entry point is:

`docs/superpowers/specs/2026-08-12-project-final-engineering-status.md`

Its companion frozen claim, evidence, and paper-planning documents are:

- `docs/superpowers/specs/FINAL_CLAIM_LEDGER_20260812.md`
- `docs/superpowers/specs/FINAL_EVIDENCE_MANIFEST_20260812.md`
- `docs/superpowers/specs/PAPER_OUTLINE_20260812.md`

The final E1a-R engineering evidence remains at
`rescue_worlds/results/phase_e1a_r_takeoff_reposition_20260812/PHASE_E1A_R_TAKEOFF_REPOSITION_REPORT_20260812.md`.
