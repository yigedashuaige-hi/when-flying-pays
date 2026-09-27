# Phase E1a-C takeoff-clearance change record

Date: 2026-08-12 (Asia/Shanghai)

Scope: RotorS-only local takeoff-site validity, target-height horizontal
release, blocked-takeoff defer/retry, and measured abort/land/disarm recovery.
No Scene B, E1b, five-seed, formal regime, or Claim B work was authorized.

## Pre-edit evidence and backup

Gate C0 was completed before production changes and is preserved at:

`rescue_worlds/results/phase_e1a_c_takeoff_clearance_20260812/c0_prechange/C0_SECOND_TAKEOFF_DIAGNOSIS_20260812.md`

Same-day rollback copies are under:

`docs/superpowers/specs/backups/2026-08-12-phase-e1a-c/`

They include the coordinator, handoff YAML, native xacro, uav_v4 build/package
files, logger, handoff harness, and the two relevant launch files. The main
mission launch and handoff-test launch remained byte-identical after work.
Original hashes are recorded in:

`rescue_worlds/results/phase_e1a_c_takeoff_clearance_20260812/PRE_CHANGE_SHA256.txt`

Audit limitation: `rescue_mission/CMakeLists.txt` and
`rescue_mission/package.xml` received only the later-discovered
`diagnostic_msgs` dependency additions and were not included in the
before-edit copy set. Their exact pre-edit forms have been reconstructed as
`rescue_mission_CMakeLists.prechange-reconstructed.txt` and
`rescue_mission_package.prechange-reconstructed.xml.txt`. Those names
deliberately distinguish reconstruction from an original timestamped backup.

## Production changes

### New file

- `src/uav_v4/src/takeoff_clearance_plugin.cpp`
  - Gazebo model observer using all uav_v4 collision AABBs and every external
    non-support collision AABB.
  - Full vertical swept-volume test to base z 1.2 m with a 0.02 m margin.
  - Atomic timestamped site-status contract plus separate logging topics.
  - Non-support obstacle contact and maximum contact-force publication.
  - No force application, pose mutation, model-state service, obstacle name,
    Scene A coordinate, or semantic traversability input.

### Modified files

- `src/uav_v4/CMakeLists.txt`
  - Build/install `takeoff_clearance_plugin`; add `diagnostic_msgs`.
- `src/uav_v4/package.xml`
  - Add `diagnostic_msgs` dependency.
- `src/uav_v4/urdf/uav_v4_rotors_native.urdf.xacro`
  - Load the observer with target z 1.2 m, margin 0.02 m, support tolerance
    0.01 m, update rate 50 Hz, and explicit topic names.
- `src/rescue_mission/CMakeLists.txt`, `src/rescue_mission/package.xml`
  - Declare `diagnostic_msgs` used by the coordinator/logger.
- `src/rescue_mission/scripts/rotors_handoff_coordinator.py`
  - Consume the atomic site-status message.
  - Hold invalid requests in public GROUND with ground ownership and no motor.
  - Latch/log blocked position, duration, path distance, and approved pose.
  - Gate retry on stable new geometry or footprint-derived displacement.
  - Require actual target z and the complete measured release predicate.
  - Abort on lost site, sustained obstacle contact, or measured height stall;
    retain measured touchdown/disarm/ground-reacquire handshake.
- `src/rescue_mission/config/rotors_handoff_e1a_h.yaml`
  - Append target-z tolerance, site freshness/hold, contact hold, and height
    progress/grace/timeout parameters. Legacy 0.30 m remains a minimum only.
- `src/rescue_mission/scripts/rescue_metrics_logger.py`
  - Append site measurement/validity/clearance/reason, decision, blocked
    duration/distance, obstacle contact/force, and handoff state fields.
  - No old Stage-0 field was deleted or renamed.
- `src/rescue_worlds/scripts/uav_v4_rotors_handoff_test.py`
  - Add eight clearance/abort cases and their result/timeline observations;
    retain the original four handoff classes.

### New execution and analysis files

- `src/rescue_worlds/scripts/run_takeoff_clearance_matrix.sh`
- `src/rescue_worlds/scripts/run_phase_e1a_c_scene_a.sh`
- `src/rescue_worlds/results/phase_e1a_c_takeoff_clearance_20260812/summarize_validation.py`
- `src/rescue_worlds/results/phase_e1a_c_takeoff_clearance_20260812/analyze_c5_bag.py`
- C0 diagnostic-only worlds/launch/analyzers under `c0_prechange/`

## Development evidence retained

Non-acceptance development runs were not deleted:

- `c4_dev_matrix`: a Gazebo SpawnModel fixture supplied an initial pose that
  unintentionally overrode the SDF pose, placing a far obstacle at the origin.
  The test fixture was corrected; production logic was not weakened.
- `c4_dev_matrix_v2`: independent scalar topics exposed a transient
  cross-message race. The decision contract was changed to one atomic
  `DiagnosticArray` and a stable preflight hold.
- `c4_dev_contact_v4`: an overlapping ceiling made Gazebo's delete service
  deadlock. The final injected-contact fixture uses a side post and requires
  real contact plus complete abort/disarm recovery; no production parameter was
  changed for this fixture issue.

## Verification and results

- C0 classification: both invalid/no-margin site and early release.
- C4 isolated matrix: 8 classes × 5 fresh Gazebo runs = 40/40.
- Existing handoff regression: 4 classes × 5 = 20/20.
- Existing ground-contact regression: 3 classes × 5 = 15/15.
- Python byte compilation: pass.
- XML parse for native xacro and relevant launches: pass.
- `catkin_make --pkg uav_v4 rescue_mission rescue_worlds`: pass.
- Static RotorS-path audit: no `set_model_state` call; ideal-only
  `air_motion_executor.py` remains excluded by the existing launch condition.
- Residual-process audit: clean.

Exact C5 (`scene_a`, `fixed_switch`, seed 20260720, recoverable, RotorS, frozen
D2) failed mission acceptance: the generic swept volume blocked the first
takeoff before brake/arm, the robot remained safely GROUND with motor speed
zero, then the unchanged ground route stalled at the left wall and timed out.
Per the stop rule, no post-C5 ideal or second handoff/contact rerun was started.

Full report:

`rescue_worlds/results/phase_e1a_c_takeoff_clearance_20260812/PHASE_E1A_C_TAKEOFF_CLEARANCE_REPORT_20260812.md`

## Frozen artifacts

Unchanged at final audit:

- `rotors_supervisor_d2.yaml`:
  `a49e5f585d700ac4b031c1511f7979227443d6eaefa4ec9eb0d3565ff1ca4a40`
- Stage-0 canonical `rescue_metrics.csv`:
  `e070dbfa608b4fdf52cf8f526b4b25f67c1a8b9dcf83134f849e1746121cad32`
- Stage-0 result manifest:
  `149c509a9e66a5ccbf0ef0c9834e70b7a41b9e0359cfbc7b68fdba570de86124`

Final source/result hashes are in `POST_CHANGE_SHA256.txt`.

## Stop status

Phase E1a-C stops here with C5 failed. E1b is not authorized, and Scene B,
five-seed, formal RotorS regime, and Claim B remain prohibited. The next design
question is a generic high-level ground reposition/escape behavior for deferred
takeoff; it must not be a Scene A coordinate or obstacle-name special case.
