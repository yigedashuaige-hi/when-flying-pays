# Phase D1 RotorS Energy Logging and Calibration Change Record

**Date:** 2026-07-18  
**Outcome:** D1 passed; D2 and paper experiments not started

## Pre-edit backup

Git metadata remains unavailable, so five existing files were copied before
editing to:

`docs/superpowers/specs/backups/2026-07-18-phase-d1-rotors-energy/`

The tree contains `.before` copies of the metrics logger, mission launch,
mission CMake file, project handoff, and Stage-0 latest CSV mirror. Important
pre-edit hashes:

```text
c6a9b8117eb53c6a2ffc91d960008db5d23c0ff3008aa7ed219e023a4cfb9e56  rescue_metrics_logger.py.before
e070dbfa608b4fdf52cf8f526b4b25f67c1a8b9dcf83134f849e1746121cad32  rescue_metrics.csv.before
```

## Existing files modified

- `rescue_mission/scripts/rescue_metrics_logger.py`
- `rescue_mission/launch/rescue_mission.launch`
- `rescue_mission/CMakeLists.txt`
- `docs/superpowers/specs/2026-07-18-project-handoff-current-status.md`

The logger retains the original 55 CSV fields in the original order and with
the original energy/battery semantics. Forty-one D1 fields are appended. The
main launch only passes the already-existing `air_backend` selection into the
logger. `mav_msgs` was added to the CMake dependency list; it was already in
`package.xml`.

## Files added

- `rescue_worlds/launch/uav_v4_rotors_energy_calibration.launch`
- `rescue_worlds/scripts/uav_v4_rotors_energy_calibration.py`
- `rescue_worlds/scripts/analyze_rotors_energy_calibration.py`
- `rescue_worlds/results/uav_v4_rotors_energy_d1_20260718/` raw CSVs, JSONs,
  trial summary, ideal compatibility result, and report
- this change record

## Data hygiene

The D1 calibration launch directs both `csv_path` and `latest_csv_path` to its
own result file and never writes Stage-0 directories. The ideal compatibility
regression intentionally exercised the normal legacy logger path, which updated
the non-canonical latest mirror; it was restored byte-for-byte from the D1
pre-edit backup. Its final SHA-256 is again:

```text
e070dbfa608b4fdf52cf8f526b4b25f67c1a8b9dcf83134f849e1746121cad32
```

No canonical Stage-0 run, aggregate, significance output, policy, scene,
stuck/risk model, supervisor threshold, or experiment semantics were modified.
