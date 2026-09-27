# Phase C uav_v4 RotorS Change Record

**Date:** 2026-07-18  
**Outcome:** stopped after reproducible non-finite dynamics at lift-off

## Pre-edit backup

Existing files were copied before modification to:

`docs/superpowers/specs/backups/2026-07-18-phase-c-uav-v4-rotors/`

The backup tree contains `.before` copies, retaining paths, for the original
URDF/SDF, uav package metadata, both main mission launch files, mission mode and
air bridge scripts, mission package metadata, disaster launch, and all six
original disaster worlds. SHA-256 values were printed before edits; the
canonical original URDF hash is:

```text
894aeb015512d5b716c22d7bfde9c2ea86bc40d1ca2fe51f37a930bff26ceada  uav_v4.urdf.before
```

Git metadata was unavailable (`$CATKIN_WS` is not a Git worktree), so
this backup is the rollback source.

## Existing files modified

- `uav_v4/CMakeLists.txt`
- `uav_v4/package.xml`
- `rescue_mission/CMakeLists.txt`
- `rescue_mission/package.xml`
- `rescue_mission/launch/rescue_phase1_demo.launch`
- `rescue_mission/launch/rescue_mission.launch`
- `rescue_mission/scripts/mode_switcher.py`
- `rescue_worlds/launch/disaster_gazebo.launch`
- `rescue_worlds/worlds/disaster_scene_a.world`
- `rescue_worlds/worlds/disaster_scene_b.world`
- `rescue_worlds/worlds/disaster_scene_c.world`
- `rescue_worlds/worlds/disaster_scene_d.world`
- `rescue_worlds/worlds/disaster_scene_e.world`
- `rescue_worlds/worlds/disaster_scene_sparse.world`
- `docs/superpowers/specs/2026-07-18-project-handoff-current-status.md`

The world changes only add the RotorS ROS-interface bridge plugin. The original
`uav_v4.urdf`, historical `uav_v4.sdf`, `air_goal_bridge.py`, policy costs,
risk model, stuck model, and Stage-0 analysis code were not modified.

## Files added

- `uav_v4/src/mode_aware_planar_move_plugin.cpp`
- `uav_v4/urdf/uav_v4_rotors.urdf.xacro`
- `uav_v4/config/lee_controller_uav_v4.yaml`
- `uav_v4/config/uav_v4_rotors.yaml`
- `rescue_mission/scripts/rotors_motor_gate.py`
- `rescue_mission/scripts/rotors_odometry_bridge.py`
- `rescue_worlds/launch/uav_v4_rotors_hybrid_smoke.launch`
- `rescue_worlds/scripts/uav_v4_rotors_hybrid_smoke_test.py`
- `rescue_worlds/results/uav_v4_rotors_hybrid_smoke_20260718/flight_metrics.json`
- `rescue_worlds/results/uav_v4_rotors_hybrid_smoke_20260718/PHASE_C_UAV_V4_ROTORS_REPORT_20260718.md`
- this change record

## Build and data hygiene

- `catkin_make --pkg uav_v4 rescue_mission rescue_worlds` passed.
- Xacro expansion and Gazebo URDF-to-SDF conversion passed.
- New RotorS files contain no `set_model_state` call.
- The ideal regression caused the existing logger to overwrite its documented
  non-canonical `results/rescue_metrics.csv` latest mirror. It was restored from
  the final canonical Batch-2 run
  `runs_claim_a_airtight/scene_c_recoverable_threshold_t0.4/threshold_switch__20260611_223734.csv`.
  Both files then had SHA-256:

```text
e070dbfa608b4fdf52cf8f526b4b25f67c1a8b9dcf83134f849e1746121cad32
```

No canonical Stage-0 CSV, aggregate, significance report, policy algorithm, or
risk/stuck semantics was changed.
