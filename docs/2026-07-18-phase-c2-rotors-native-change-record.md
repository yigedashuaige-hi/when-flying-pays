# Phase C2 RotorS-native uav_v4 Change Record

**Date:** 2026-07-18  
**Outcome:** A, B, and C smoke gates passed; Phase D not started

## Backup and preserved failure evidence

Git metadata is unavailable at `$CATKIN_WS`, so existing files were
copied before editing to:

`docs/superpowers/specs/backups/2026-07-18-phase-c2-rotors-native/`

The backup tree retains the original relative paths and contains 11 `.before`
files: the failed detailed-CAD xacro, both controller configs, three launch
files, motor gate, hybrid test, Stage-0 latest CSV mirror, and the handoff.

The Phase C failure model and evidence were not overwritten:

```text
35014a21c6c64bd56d4c3e05f86386fd759640a1573301375823ca7d66b80628  uav_v4_rotors.urdf.xacro
2ed66fe89fcbd7c03d1820affa081449c1241fb08cd0a93811604eb330d2614e  Phase-C flight_metrics.json
```

## Existing files modified

- `rescue_mission/launch/rescue_mission.launch`
- `rescue_mission/scripts/rotors_motor_gate.py`
- `rescue_worlds/launch/disaster_gazebo.launch`
- `rescue_worlds/launch/uav_v4_rotors_hybrid_smoke.launch`
- `rescue_worlds/scripts/uav_v4_rotors_hybrid_smoke_test.py`
- `docs/superpowers/specs/2026-07-18-project-handoff-current-status.md`

## Files added

- `uav_v4/urdf/uav_v4_rotors_native.urdf.xacro`
- `uav_v4/config/uav_v4_rotors_native.yaml`
- `rescue_worlds/launch/uav_v4_rotors_native_stage.launch`
- `rescue_worlds/scripts/uav_v4_rotors_native_flight_test.py`
- `rescue_worlds/results/uav_v4_rotors_native_c2_20260718/` test JSONs and report
- this change record

## Behavioral changes

- Only `air_backend:=rotors` now spawns the native model and loads its measured
  aggregate controller configuration.
- `air_backend:=ideal` still spawns `uav_v4.urdf` and starts the unchanged
  `air_motion_executor.py`.
- The motor gate now rejects non-finite Lee vectors and clamps each command to
  `[0, 1100] rad/s` before forwarding.
- The hybrid smoke logger now treats any non-finite odometry, IMU, motor, or
  Gazebo model-state sample as an immediate failure.

No policy, cost, risk, stuck model, Stage-0 analysis, or experiment semantics
were modified.

## Data hygiene

The ideal regression updated the legacy latest CSV mirror as expected. It was
restored byte-for-byte from the pre-edit backup. Final and backup SHA-256 are:

```text
e070dbfa608b4fdf52cf8f526b4b25f67c1a8b9dcf83134f849e1746121cad32
```

Canonical Stage-0 run directories and reports were not written. Test artifacts
are isolated under `rescue_worlds/results/uav_v4_rotors_native_c2_20260718/`.
