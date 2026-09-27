# Phase F1 Equal-Budget Tuning Change Record

Phase F1 added experiment-only infrastructure and evidence. It did not modify
`mode_switcher.py`, perception, terrain/stuck physics, scenes B/C, the energy
model, supervisor, RotorS, robot models, or Stage 0 canonical data.

## Added configuration

- `rescue_mission/config/ablations/f1_risk_wcvar_{025,050,075,100,125}.yaml`

Each file overrides only `mode_switcher.risk_aware.wcvar`.

## Added experiment scripts

- `rescue_mission/scripts/run_phase_f1_fair.py`
- `rescue_mission/scripts/select_phase_f1_configs.py`
- `rescue_mission/scripts/analyze_phase_f1.py`
- `rescue_mission/scripts/generate_phase_f1_manifest.py`

The runner isolates `runs_dir` and `latest_csv_path` per cell, validates the
frozen ideal backend and common numeric fields, records command/CSV hashes, and
supports non-overlapping task shards. The selection and analysis scripts
implement the preregistered rules.

## Added results

The independent result tree is
`rescue_mission/results/phase_f1_fair_tuning_20260821/`. It contains the
preregistration, amendments/incidents, candidate and run configs, 200 valid
tuning cells, selected-config freeze, 120 valid held-out cells, logs,
summaries, paired statistics, plots, environment metadata, final report, and
full SHA-256 manifest. Technical-invalid pre-simulation attempts remain
retained and explicitly excluded.

No existing canonical result file was intentionally edited. The final report
supersedes only the fairness-limited interpretation of the old CVaR/threshold
claim; the historical evidence and claim ledger remain preserved as dated
records.

