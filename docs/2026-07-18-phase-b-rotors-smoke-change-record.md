# Phase B RotorS Smoke Test — Change Record

**Date:** 2026-07-18  
**Scope:** standalone stock-RotorS infrastructure validation  
**Phase C:** not started

## Pre-change backup

Before appending the Phase B completion status, the handoff was copied to:

`docs/superpowers/specs/backups/2026-07-18-phase-b-rotors-smoke/2026-07-18-project-handoff-current-status.md.before`

Both original and backup had SHA-256:

`ce1a12d25a7692195675e08e1f83f830db8c84a34ea7d2d1a45e67f673819227`

## Added files

- `rescue_worlds/worlds/disaster_scene_a_rotors_smoke.world`
  - Scene A geometry/physics copy with only the RotorS ROS-interface world
    plugin added.
- `rescue_worlds/launch/rotors_firefly_smoke_disaster.launch`
  - Starts Gazebo, stock Firefly model/plugins, generic odometry, and Lee
    controller under `/firefly`.
- `rescue_worlds/scripts/rotors_firefly_smoke_test.py`
  - Publishes five position stages and records convergence/topic/frame evidence.
- `rescue_worlds/results/rotors_smoke_20260718/flight_metrics.json`
  - Canonical machine-readable metrics.
- `rescue_worlds/results/rotors_smoke_20260718/ros_logs/`
  - Copied ROS logs from the smoke launch.
- `rescue_worlds/results/rotors_smoke_20260718/PHASE_B_ROTORS_SMOKE_REPORT_20260718.md`
  - Full commands, contract, acceptance matrix, and Phase C interface list.
- this change record.

## Modified file

- `docs/superpowers/specs/2026-07-18-project-handoff-current-status.md`
  - Added a dated Phase B completion addendum only.

## Runtime state changes

- Existing stopped container `uav_noetic` was started.
- One headless roslaunch/Gazebo/Lee-controller smoke session was run.
- The session was terminated with SIGINT to its exact roslaunch PID after the
  Docker exec PTY failed to forward Ctrl-C.
- Exact child PIDs were verified absent afterward.
- The `uav_noetic` container was returned to its original stopped state.

## Explicit non-changes

- No file under `uav_v4/` changed.
- No `rescue_mission` algorithm, launch, config, world, or result changed.
- No existing disaster world changed.
- No RotorS source/controller/model/plugin code changed.
- No paper experiment or batch was launched.
- No `/gazebo/set_model_state` client or call path was used.

## Result

All automated smoke acceptance flags are true. The stock Firefly completed
takeoff, stable hover, 2 m horizontal movement, return, and stationary touchdown
with working odometry, IMU, controller output, motor measurements, and Gazebo
model state. Landing is position-controlled but not disarmed; this is recorded
as a required Phase C interface rather than patched into stock RotorS here.
