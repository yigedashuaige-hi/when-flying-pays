# Phase D2 RotorS energy/supervisor change record

Date: 2026-07-19  
Scope: small calibration, conservative mechanical-proxy supervisor wiring, and
explicit guarded/unguarded validation only.

## Backups made before editing

Directory:
`docs/superpowers/specs/backups/2026-07-19-phase-d2-rotors-energy/`

- `uav_v4_rotors_energy_calibration.py`, SHA-256
  `6c7961a7c000b4260ef8ada98be74e05d736314e6ee94aea0778bbe9d4b290cc`
- `rescue_metrics_logger.py`, SHA-256
  `543b8fa3f007f2ee2e2118aa623b4efcd5def6be24c7ebbb68ef176aeb98071c`
- `mode_switcher.py`, SHA-256
  `f33d16e6b2a9625429c3c588628e2598ee74e6dd2a9dd2b699b11fdbad519116`
- `safety_supervisor.py`, SHA-256
  `feaebd6d20f211fad3429ce5b5127fe7813467f0a7b97bd01a9dc54e4173e2f7`
- `rescue_mission.launch`, SHA-256
  `16acac11a19f42a1c5f52616d662b8770e3eb35de248868a3095c3cff0c2f173`
- `rescue_phase1_demo.launch`, SHA-256
  `78ccb7fcbb483e6c3aee92445d312d2bc2889101b63970049b8670f94c5ea70c`
- `2026-07-18-project-handoff-current-status.md`, SHA-256
  `20dead2cb975b2a5cd504c7cc05d527692d6c100dc6b750e9602de88c69bf894`

Git metadata was not used because `$CATKIN_WS` and its `src`
directory are not valid Git worktrees in this environment.

## Existing files modified

- `rescue_worlds/scripts/uav_v4_rotors_energy_calibration.py`
  - added per-trial `height_m`, defaulting to the D1 value 1.0 m.
- `rescue_mission/scripts/safety_supervisor.py`
  - added an explicit energy backend/model, generic remaining-energy input, and
    predicted/reserve/block snapshots; default ideal decision behavior remains
    identical.
- `rescue_mission/scripts/mode_switcher.py`
  - selects old ideal parameters or separate RotorS mechanical parameters and
    publishes the supervisor accounting contract.
- `rescue_mission/scripts/rescue_metrics_logger.py`
  - appended eight D2 columns and publishes the separate RotorS mechanical
    consumed/remaining ledger; old Stage-0/D1 columns were not renamed/deleted.
- `rescue_mission/launch/rescue_mission.launch`
  - conditionally loads the RotorS-only D2 config and forwards backend metadata;
    also permits an optional noncanonical latest-CSV path for safe regression.
- `rescue_mission/launch/rescue_phase1_demo.launch`
  - forwards the optional latest-CSV path; default behavior is unchanged.
- `docs/superpowers/specs/2026-07-18-project-handoff-current-status.md`
  - appended a dated D2 completion checkpoint; historical text retained.

## New source/config files

- `rescue_worlds/scripts/uav_v4_rotors_energy_d2_calibration.py`
- `rescue_worlds/scripts/analyze_rotors_energy_d2.py`
- `rescue_worlds/scripts/validate_rotors_supervisor_d2.py`
- `rescue_worlds/scripts/check_ideal_supervisor_parity.py`
- `rescue_mission/config/rotors_supervisor_d2.yaml`

## Result artifacts

Directory: `rescue_worlds/results/uav_v4_rotors_energy_d2_20260719/`

- `calibration_samples.csv` — canonical final raw D2 data, SHA-256
  `23d87d4b7a3e52daf8fc671d4e3ab67a015e8aa7398e4eb4eb8370a58a48c105`
- `calibration_run.json` — declared plan/completion, SHA-256
  `e0fb355fee3673ee4229f78497f0cbb5b054f5ce0c13d8abae47d439f20dfb42`
- `calibration_trial_summary.csv`
- `d1_joint_analysis.json`
- `calibration_analysis_d2.json` — selected model/budget, SHA-256
  `62c71098bf0e036e5929eca54994ce8c8defdf8ad2d9eaa0a9439d94a88f6fe6`
- `supervisor_case_results.{json,csv}` — JSON SHA-256
  `9716a050a82464fc4e31c26c11aff30230d6669c6c30673c9d94fa72fe32af30`
- `ideal_supervisor_parity.json` — SHA-256
  `593d82e9dbc7e28e8423244b2da2ce829bf865177283786ab5f4588cabbfaaa3`
- `ideal_regression/` and `rotors_wiring/` — short runtime CSV evidence.
- `calibration_run_first_pass.json` and
  `calibration_samples_corrupted_after_stale_process.csv` — preserved first-pass
  incident evidence; explicitly noncanonical and unused by final analysis.
- `PHASE_D2_ROTORS_ENERGY_SUPERVISOR_REPORT_20260719.md`

## Verification performed

- 18/18 final trials passed; no NaN/Inf; every trial landed/disarmed.
- 12 calibration and 6 held-out samples remained separated.
- D1-style joint and selected staged models both evaluated on held-out data.
- Conservative margin uses maximum held-out underprediction, not average error;
  landing has an independent tail reserve.
- Guarded replay: zero reserve violations; sufficient and near-boundary cases
  allowed; insufficient and airborne-low-margin cases handled safely.
- Ideal supervisor: 448/448 behavior cases matched the backup.
- Ideal runtime: historical 55-column prefix and legacy energy model preserved.
- RotorS runtime: new topics/config/CSV metadata present; no ideal executor.
- Stage-0 canonical mirror unchanged at SHA-256 `e070dbfa...`.
- Python compilation and ideal/rotors roslaunch graph parsing passed.

No algorithm policy, risk model, robot dynamics/model, scene semantics, Stage-0
canonical result, or paper batch was changed.
