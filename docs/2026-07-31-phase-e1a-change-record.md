# Phase E1a Change Record

Date: 2026-07-31 (Asia/Shanghai)

## Scope and integrity

Work was limited to Scene A contact diagnosis/minimal repair and two read-only
scientific audits. No Scene B, five-seed RotorS run, formal RotorS batch, or
Claim B multi-seed experiment was started. Git metadata was unavailable, so
pre-change copies are under
`docs/superpowers/specs/backups/2026-07-31-phase-e1a/`; pre/post hashes are in
`rescue_worlds/results/phase_e1a_contact_20260731/{PRE,POST}_CHANGE_SHA256.txt`.

Frozen D2 YAML, Stage-0 canonical CSV, safety/mode/logger/motor/bridge code, and
main launches retain their frozen hashes. Scene worlds, policy, risk model,
supervisor, route, and canonical results were not edited.

## Existing files modified

- `uav_v4/src/mode_aware_planar_move_plugin.cpp`: replaced per-step
  `SetLinearVel/SetAngularVel` ownership with gated, bounded force/torque
  ownership. It publishes an optional exact drive-work diagnostic CSV when
  `GROUND_DRIVE_DIAG_CSV` is set. In TAKEOFF/AIR/LAND or armed state it applies
  zero force/torque.
- `uav_v4/urdf/uav_v4_rotors_native.urdf.xacro`: explicit servo parameters;
  wheel-proxy friction=0.05 applied only by the plugin to fixed wheel collision
  proxies. No collision was removed.
- `uav_v4/CMakeLists.txt`: builds the isolated diagnostic world plugin.

## New implementation/test files

- `uav_v4/src/ground_contact_diagnostic_plugin.cpp`
- `rescue_worlds/worlds/ground_contact_microtest.world`
- `rescue_worlds/launch/ground_contact_microtest.launch`
- `rescue_worlds/scripts/ground_contact_microtest_driver.py`
- `rescue_worlds/scripts/analyze_ground_contact_microtest.py`
- `rescue_mission/scripts/audit_d2_stage0_envelope.py`
- `rescue_mission/scripts/audit_cvar_threshold_equivalence.py`

Reports and generated CSVs live only below
`rescue_worlds/results/phase_e1a_contact_20260731/`.

## Finite ownership parameters

Total mass is 3.0440996088 kg. Planar acceleration is capped at 1.5 m/s^2, hence
force at 4.56615 N; velocity-error gain is 32 s^-1. Vertical spring/damping is
800 N/m and 50 N s/m with a 29.83 N cap (one vehicle weight). Leveling is
1.0 N m/rad plus 0.15 N m s/rad damping, capped at 0.6 N m, below the approximate
`m g 0.04 = 1.19 N m` wheel-support moment scale. Yaw torque is capped at
0.06 N m. The controller never sets pose or velocity.

The fixed wheel proxies cannot roll, so default mu=1 made the finite 4.566 N
drive behave like four locked brakes. Evidence from the first Gate-1 trial led
to setting only `wheel_*` proxy mu/mu2 to 0.05; body and rotor surfaces remain
at their original values. The capped servo supplies the intended mecanum
traction. Normal contact and all collisions remain.

## Commands/tests

- Build: `catkin_make --pkg uav_v4 -j2` (passed after final source state).
- Microtest template and exact geometry variants are in the Gate-0 report.
- Final Gate 2: head-on, 10-degree oblique, and side-corner approach, five runs
  each; one flat control; one TAKEOFF/armed release test.
- Scene A RotorS: exact E0/E1 command with `scene_a`, `fixed_switch`, seed
  20260720, mission timeout 55, recoverable, D2, isolated output directory.
- Scene A ideal: exact E0/E1 command with mission timeout 45 and isolated output.
- Python compile and XML parsing passed; no raw/canonical CSV was overwritten.

## Stop event

The contact microtests passed, but the Scene A RotorS run exposed a distinct
GROUND-to-TAKEOFF control transient before reaching the barrier. TAKEOFF did not
clear the ground, roll grew, LAND never contacted/disarmed, and the wall guard
stopped the run. Per the requested stop condition, no further gain/route/model
trial was attempted. Scene B remains forbidden.
