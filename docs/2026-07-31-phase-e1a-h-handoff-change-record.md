# Phase E1a-H handoff change record

Date: 2026-07-31 (Asia/Shanghai)

## Scope and backups

Work was limited to the same-model RotorS ground/flight ownership handshake,
its isolated tests, contact regression, and the permitted same-condition Scene A
attempt. Backups made before core edits are under
`docs/superpowers/specs/backups/2026-07-31-phase-e1a-h/`. Pre/post hashes are in
`rescue_worlds/results/phase_e1a_h_handoff_20260731/`.

No Lee gain, mass, inertia, rotor parameter, finite ground-servo gain, scene,
route, strategy, CVaR/threshold parameter, D2 supervisor parameter, energy
model, or canonical Stage-0 result was modified.

## Existing files modified

- `rescue_mission/scripts/mode_switcher.py`: RotorS publishes supervisor-approved
  requests on `/rescue/requested_mode` and consumes coordinator-approved public
  `/rescue/mode`; ideal retains the original direct publication path.
- `rescue_mission/scripts/rotors_motor_gate.py`: optional managed mode arms only
  from `/rescue/rotors/motor_enable`; legacy unmanaged behavior remains.
- `rescue_mission/launch/rescue_mission.launch`: RotorS loads the E1a-H config
  and coordinator and does not start the old RotorS pose bridge; ideal launch is
  unchanged and still starts `air_goal_bridge` plus `air_motion_executor`.
- `rescue_mission/CMakeLists.txt`: installs the new coordinator.
- `uav_v4/src/mode_aware_planar_move_plugin.cpp`: accepts an explicit ownership
  command and publishes active/zero-wrench evidence. With no ownership publisher
  it retains the E1a `GROUND && disarmed` fallback used by contact microtests.
- `uav_v4/urdf/uav_v4_rotors_native.urdf.xacro`: adds only the ownership and
  telemetry topic names; all dynamics/contact/servo values are unchanged.
- `rescue_worlds/launch/uav_v4_rotors_hybrid_smoke.launch` and
  `rescue_worlds/scripts/uav_v4_rotors_hybrid_smoke_test.py`: route the old smoke
  through the coordinator and use the main 1.2 m altitude contract.

`air_goal_bridge.py` was backed up but is byte-identical before/after.

## New files

- `rescue_mission/config/rotors_handoff_e1a_h.yaml`
- `rescue_mission/scripts/rotors_handoff_coordinator.py`
- `rescue_worlds/launch/uav_v4_rotors_handoff_test.launch`
- `rescue_worlds/scripts/uav_v4_rotors_handoff_test.py`
- `rescue_worlds/scripts/run_handoff_matrix.sh`
- `rescue_worlds/scripts/run_contact_regression_e1a_h.sh`
- Gate H0 and Scene A offline analyzers and all evidence below
  `rescue_worlds/results/phase_e1a_h_handoff_20260731/`.

## Validation and stop event

- Python compilation, XML parsing, and `catkin_make --pkg uav_v4 rescue_mission
  -j2` passed.
- Isolated handoff matrix: 4 classes x 5 fresh Gazebo runs = 20/20 passed.
- Original contact regression: head-on, oblique, corner = 5/5 each.
- Same-condition Scene A RotorS: failed and timed out in the second landing.
  Per the requested stop condition, no ideal regression, E1b, Scene B, five-seed
  run, or further controller/threshold trial was started.

The task logger temporarily wrote its normal noncanonical latest mirror. That
file was preserved as
`scene_a_rotors/noncanonical_latest_mirror_before_restore.csv`, then
`rescue_mission/results/rescue_metrics.csv` was restored to the frozen Stage-0
hash `e070dbfa608b...` from the identical canonical source. D2 remains
`a49e5f585d70...`; the Stage-0 result manifest is `149c509a9e66...`.
