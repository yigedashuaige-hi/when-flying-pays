# Phase E1a-R takeoff-reposition change record

Date: 2026-08-12  
Scope: RotorS-only deferred-takeoff recovery. Scene B, E1b, policy, world,
route, vehicle dynamics, Lee gains, clearance geometry, D2, and Stage 0 were
not changed.

## Pre-change evidence and backup

Gate R0 was completed read-only against the frozen E1a-C C5 bag before source
changes. Its report and timeline are under
`rescue_worlds/results/phase_e1a_r_takeoff_reposition_20260812/r0_readonly_audit/`.

Pre-change copies are in
`docs/superpowers/specs/backups/2026-08-12-phase-e1a-r/`:

| File | Pre-change SHA-256 |
|---|---|
| `rotors_handoff_coordinator.py` | `9c8e7e4ac91fd8aa4998a8cf3432d5d0e97509272946a91543f274643d860799` |
| `ground_goal_controller.py` | `5923b3f80dc4f1e56fce281b7d872843f6241653f1cbdce779223f73570c31c2` |
| `rescue_metrics_logger.py` | `c4175882b6085612155fbce138c65c345d92b7452d6eec23f8fcc4683c3f2305` |
| `rotors_handoff_e1a_h.yaml` | `5592720f2e429ae0eb8c70a26f1f316f2c96e4435779d2a8921dcd89d5c3ed7b` |
| `uav_v4_rotors_handoff_test.py` | `59aa2649e32da5d5499de9245c1ab4a0522638ca36beaeb4921f9968f8876f84` |

The launch preimage was already preserved before E1a-C at
`docs/superpowers/specs/backups/2026-08-12-phase-e1a-c/uav_v4_rotors_handoff_test.launch`
(SHA-256 `e45e5e3119a9e15d4090a2433bb59b5293145ba3e988ab1e205668aa396da1a6`).

## Modified source/configuration

- `rescue_mission/scripts/rotors_handoff_coordinator.py`
  - finite fresh GROUND valid-site history;
  - internal `TAKEOFF_REPOSITION` and bounded failure state;
  - temporary recovery goal/result/metrics topics;
  - measured position, speed, fresh stable validity arrival gate;
  - motor-off/ground-owner output during recovery.
- `rescue_mission/scripts/ground_goal_controller.py`
  - RotorS recovery-goal override while retaining the sole `/mecanum/cmd_vel`
    publisher;
  - recovery-only historical yaw restoration attempt.
- `rescue_mission/config/rotors_handoff_e1a_h.yaml`
  - added all history/recovery thresholds. Final recovery-only XY tolerance is
    0.03 m and progress epsilon is 0.005 m, both justified by retained R4 bags.
- `rescue_mission/scripts/rescue_metrics_logger.py`
  - append-only recovery columns and subscribers; no old column was deleted or
    renamed.
- `rescue_worlds/scripts/uav_v4_rotors_handoff_test.py`
  - five recovery fixtures and result/timeline capture.
- `rescue_worlds/launch/uav_v4_rotors_handoff_test.launch`
  - optional launch of the existing ground controller for recovery tests.

## New scripts/evidence helpers

- `rescue_worlds/scripts/run_takeoff_reposition_matrix.sh`
- `rescue_worlds/scripts/run_contact_sample_e1a_r.sh`
- `rescue_worlds/scripts/run_phase_e1a_r_scene_a.sh`
- `rescue_worlds/results/phase_e1a_r_takeoff_reposition_20260812/r0_readonly_audit/analyze_pre_takeoff_history.py`
- `rescue_worlds/results/phase_e1a_r_takeoff_reposition_20260812/analyze_r4_reposition.py`

## Final source SHA-256

| File | SHA-256 |
|---|---|
| `rotors_handoff_coordinator.py` | `1a18d4188deeea8f3756129e2fd43515845316c4ffc18d546dac9fba02acf7e8` |
| `ground_goal_controller.py` | `abfd32fe761badffa99832dcdd6d1d6f0eeda776e3094b5e17d546c1c783303b` |
| `rescue_metrics_logger.py` | `1c5a9aa725568a5a6327f41a262b582cd6f29e6fb87a10fc9e4370243b8f2b48` |
| `rotors_handoff_e1a_h.yaml` | `2d512e8f0ddd68ae18834f1c42a97faa2de987021ae5efbb1e35ba33e4f186a0` |
| `uav_v4_rotors_handoff_test.py` | `eace77b3523847cf15f396e26235086c6309519342ee634c815a2c3dc9caa184` |
| `uav_v4_rotors_handoff_test.launch` | `047e77fef3dacbe55c2bc804ff8dda9b73df86822b334540fd9eb28798fcf447` |
| `run_takeoff_reposition_matrix.sh` | `46c3dd52115796854ea0704fc56e565db92c5cef215ec9e12f8ca671683cda55` |
| `run_contact_sample_e1a_r.sh` | `1e9d6d7707af4d3b49c79b23401853101b3f5a33faef32129292adbe7d2e919b` |
| `run_phase_e1a_r_scene_a.sh` | `fc6cf43acb1420fc40166e72dca1d6c355c17639c76e4ca53e2ad1e54fc86c3f` |

## Validation and retained failures

- Python compilation: pass.
- `catkin_make --pkg rescue_mission rescue_worlds`: pass; only existing
  Gazebo Classic/Eigen deprecation warnings.
- Final-config new matrix: 15/15 expected outcomes, split between
  `r3_final_progress_recheck_unchanged_cases/` and
  `r3_final_progress_recheck/`.
- Old sampled handoff/contact/clearance regressions: all pass; directories are
  listed in the Phase E1a-R report.
- Failed development and R4 attempts were retained, including the 0.10 m
  arrival-tolerance failure and the 0.025 m progress-epsilon failure. They were
  not overwritten or relabelled as success.

The final Scene A run completed one full recovery/flight/landing/disarm cycle,
then safely rejected a later sortie because no recent ground-valid history
existed. No further architecture was added after that stop condition.

