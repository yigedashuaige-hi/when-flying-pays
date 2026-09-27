# Phase E0/E1 change record — 2026-07-20

## Scope

Freeze/audit D2, add a dedicated real-dynamics supervisor test harness, run
closed-loop tests and Scene A backend regressions, and stop on the first invalid
regression. No policy algorithm, risk model, scene, controller gain, robot model,
Stage 0 result, or D2 parameter was changed.

## Backup created before editing an existing document

- `docs/superpowers/specs/backups/2026-07-20-phase-e0-e1/2026-07-18-project-handoff-current-status.md`

## New files

- `rescue_worlds/launch/uav_v4_rotors_supervisor_closed_loop.launch`
- `rescue_worlds/scripts/uav_v4_rotors_supervisor_closed_loop.py`
- `rescue_worlds/results/phase_e0_e1_rotors_20260720/README.md`
- `rescue_worlds/results/phase_e0_e1_rotors_20260720/D2_FROZEN_CONFIG_MANIFEST_20260720.md`
- `rescue_worlds/results/phase_e0_e1_rotors_20260720/PHASE_E0_E1_REPORT_20260720.md`
- six `supervisor_closed_loop/<case>/result.json` files and six matching
  `metrics.csv` files
- isolated Scene A ideal and RotorS run CSVs plus their local `latest.csv`
  mirrors
- this change record

## Existing file updated

- `docs/superpowers/specs/2026-07-18-project-handoff-current-status.md`
  received a dated append-only checkpoint. Earlier history was retained.

## Validation

- Python syntax: dedicated harness passed `python3 -m py_compile`.
- Launch XML: dedicated launch passed `xmllint --noout` and `roslaunch --nodes`.
- All six supervisor JSON results report their declared expectation as passed;
  all matching CSVs contain zero textual NaN/Inf rows.
- Scene A ideal and RotorS CSVs contain zero textual NaN/Inf rows.
- Stage 0 canonical hash and frozen D2 core hashes were rechecked unchanged.
- Post-run ROS/Gazebo process audit was empty.

## Stop record

The RotorS Scene A run reached DONE but was not accepted: in GROUND/disarmed with
zero motor speed, collision with the x=7 m low barrier launched the model to
2.4766 m. Scene B and five-seed runs were deliberately not started. No attempted
fix was made in this phase, so the frozen baseline and the failure evidence stay
reproducible.
