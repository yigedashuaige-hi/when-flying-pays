# Final Evidence Manifest

**Frozen:** 2026-08-12  
**Path convention:** paths are relative to `ros_ws/src` unless
otherwise stated. “Paper-facing” means usable with the wording and limitations
in `FINAL_CLAIM_LEDGER_20260812.md`; it does not mean every artifact belongs in
the main paper.

## 1. Frozen integrity anchors

| Artifact | Verified SHA-256 | Check result |
|---|---|---|
| Stage 0 canonical/latest mirror `rescue_mission/results/rescue_metrics.csv` | `e070dbfa608b4fdf52cf8f526b4b25f67c1a8b9dcf83134f849e1746121cad32` | matches all prior freezes |
| Stage 0 manifest `rescue_mission/results/STAGE0_RESULT_MANIFEST_20260718.md` | `149c509a9e66a5ccbf0ef0c9834e70b7a41b9e0359cfbc7b68fdba570de86124` | unchanged |
| D2 YAML `rescue_mission/config/rotors_supervisor_d2.yaml` | `a49e5f585d700ac4b031c1511f7979227443d6eaefa4ec9eb0d3565ff1ca4a40` | unchanged |
| D2 raw `rescue_worlds/results/uav_v4_rotors_energy_d2_20260719/calibration_samples.csv` | `23d87d4b7a3e52daf8fc671d4e3ab67a015e8aa7398e4eb4eb8370a58a48c105` | canonical final rerun |
| D2 analysis `.../calibration_analysis_d2.json` | `62c71098bf0e036e5929eca54994ce8c8defdf8ad2d9eaa0a9439d94a88f6fe6` | unchanged |
| D2 cases `.../supervisor_case_results.json` | `9716a050a82464fc4e31c26c11aff30230d6669c6c30673c9d94fa72fe32af30` | unchanged |

All 18 Batch 2/sparse immutable `all_runs` files passed
`sha256sum -c rescue_mission/results/stage0_archive_20260718/SHA256SUMS.txt` on
2026-08-12.

## 2. Important result-to-source map

| Number/result | Source report | CSV/JSON or raw directory | Backend | Scene/seed | Stuck model | Energy model | Classification |
|---|---|---|---|---|---|---|---|
| Batch 2 B fixed: success 1.00, energy 48.306 J | Stage 0 manifest §2 | `rescue_mission/results/all_runs_airtight_scene_b_recoverable_fixed_switch.csv` | ideal | B, seeds 0–19 | recoverable | legacy task proxy | **Paper-facing** |
| Batch 2 B risk-aware: success .95, energy 80.576 J | Stage 0 manifest §2 | `.../all_runs_airtight_scene_b_recoverable_risk_aware.csv` | ideal | B, 0–19 | recoverable | legacy task proxy | **Paper-facing** |
| Batch 2 B threshold .2: success 1.00, energy 67.161 J | Stage 0 manifest §2 | `.../all_runs_airtight_scene_b_recoverable_threshold_t0.2.csv` | ideal | B, 0–19 | recoverable | legacy task proxy | **Paper-facing, tuning caveat** |
| Batch 2 C fixed: success 1.00, energy 71.102 J | Stage 0 manifest §2 | `.../all_runs_airtight_scene_c_recoverable_fixed_switch.csv` | ideal | C, 0–19 | recoverable | legacy task proxy | **Paper-facing** |
| Batch 2 C risk-aware: success .95, energy 90.257 J | Stage 0 manifest §2 | `.../all_runs_airtight_scene_c_recoverable_risk_aware.csv` | ideal | C, 0–19 | recoverable | legacy task proxy | **Paper-facing** |
| Batch 2 C threshold .3: success 1.00, energy 71.676 J | Stage 0 manifest §2 | `.../all_runs_airtight_scene_c_recoverable_threshold_t0.3.csv` | ideal | C, 0–19 | recoverable | legacy task proxy | **Paper-facing, tuning caveat** |
| Sparse: all success; energy_rule 7.292 J, threshold .6 91.044 J, risk-aware 258.380 J; paired p=.001953 | Stage 0 manifest §3 | `.../all_runs_airtight_scene_sparse_recoverable_{energy_rule,threshold_t0.6,risk_aware}.csv`; `.../stage0_archive_20260718/significance_sparse_*.txt` | ideal | sparse, 0–9 | recoverable | legacy task proxy, fly_k1=3 | **Paper-facing, tuning caveat** |
| Permanent versus recoverable ordering changes | Stage 0 manifest §§4–5 | `rescue_mission/results/runs_claim_a_regime/`; `runs_paper_hardening_20260609_054657/scene_{b,c}/` | ideal | B/C, documented seeds | labelled per directory | legacy task proxy | **Paper-facing**, pilot/permanent archive |
| Nominal guard irreversible 4/20→0/20, p=.125 | Stage 0 manifest §7 | `.../all_runs_ph_scene_d.csv`; explicit nominal significance report | ideal | D, 0–19 | permanent | legacy ledger | **Paper-facing**, conservative/non-significant |
| Adversarial guard irreversible 20/20→0/20, p=1.907e-6 | Stage 0 manifest §7 | `.../all_runs_ph_scene_d_recal.csv`; explicit adversarial report | ideal | D, 0–19 | permanent | legacy ledger | **Paper-facing stress test** |
| Two-point core is threshold-shaped; fixed tau agreement B 70.25%, C 49.80% | CVaR fairness audit | `rescue_worlds/results/phase_e1a_contact_20260731/readonly_audit_cvar/sampled_decision_boundary_comparison.csv` | ideal | B/C, 0–19 | recoverable Batch 2 logs | decision score, not physical energy | **Paper-facing analysis** |
| RotorS infrastructure takeoff/hover/2 m/return/touchdown | Phase B report | `rescue_worlds/results/rotors_smoke_20260718/flight_metrics.json` | rotors stock Firefly | Scene-A world copy, no policy seed | n/a | none | **Diagnostic only** |
| Detailed-CAD ODE non-finite lift-off | Phase C report | `rescue_worlds/results/uav_v4_rotors_hybrid_smoke_20260718/flight_metrics.json` | rotors | isolated smoke | n/a | none | **Diagnostic failure** |
| RotorS-native pure/wheel hover and same-model hybrid smoke pass | Phase C2 report | `rescue_worlds/results/uav_v4_rotors_native_c2_20260718/stage_a_fixed_speed_scan.json`, `stage_a_lee_hover.json`, `stage_b_fixed_speed_scan.json`, `stage_b_lee_hover.json`, `stage_c_hybrid_smoke.json` | rotors | isolated | n/a | none | **Paper-facing validation** |
| D1 held-out MAE 526.37 J, RMSE 643.55 J, MAPE 2.40% | D1 report | `rescue_worlds/results/uav_v4_rotors_energy_d1_20260718/` calibration/analysis artifacts | rotors | calibration maneuvers | n/a | shaft mechanical proxy | Supporting/diagnostic |
| D2 18 trials, 12 calibration/6 held-out; staged max underprediction 847.70 J | D2 report §§4–5 | D2 `calibration_samples.csv`, `calibration_run.json`, `calibration_analysis_d2.json` | rotors | isolated maneuvers | n/a | `rotors_aero_shaft_mechanical_proxy_v1` | **Paper-facing validation** |
| D2 4,900+21,900+3,700+900=31,400 J floor | D2 report §6 | D2 YAML and analysis JSON | rotors | <=1.5 m height, <=5 s hover, <=1.2 m one-way, >=.25 m/s | n/a | same mechanical proxy | **Paper-facing only inside envelope** |
| D2 guarded replay: zero violations; two unguarded counterfactual violations | D2 report §8 | `.../supervisor_case_results.{json,csv}` | replay keyed to rotors | four declared cases | n/a | same mechanical proxy | Supporting validation |
| E0 six real closed-loop supervisor cases pass; unguarded airborne-low records one violation | E0/E1 report | `rescue_worlds/results/phase_e0_e1_rotors_20260720/supervisor_closed_loop/*/{result.json,metrics.csv}` | rotors | isolated cases | n/a | same mechanical proxy | **Paper-facing validation** |
| B/C 616/640 sorties (96.25%) exceed D2 1.2 m distance envelope | D2-envelope audit | `.../readonly_audit_d2/stage0_sortie_flight_demand.csv` and group/run tables | ideal demand audited against rotors envelope | B/C, 0–19 | recoverable | cross-model comparison prohibited | **Paper-facing limitation** |
| Finite contact regression 15/15; max z .049989 m, pitch .017183 rad | E1a report | `rescue_worlds/results/phase_e1a_contact_20260731/gate2_final_microtests/` | rotors | isolated contact geometries | n/a | none | **Paper-facing validation** |
| Staged handoff matrix 20/20; contact regression 15/15 | E1a-H report | `rescue_worlds/results/phase_e1a_h_handoff_20260731/gate_h3_matrix/` and `contact_regression/` | rotors | isolated | n/a | none | **Paper-facing validation** |
| Clearance matrix 40/40; handoff 20/20; contact 15/15 | E1a-C report | `rescue_worlds/results/phase_e1a_c_takeoff_clearance_20260812/VALIDATION_SUMMARY.json`, `c4_matrix_8x5/`, and regression dirs | rotors | isolated | n/a | none | **Paper-facing validation** |
| C5 invalid takeoff: only GROUND, motor=0, final timeout | E1a-C report | `.../c5_scene_a_rotors/scene_a_e1a_c.bag`, `C5_BAG_AUDIT.json` | rotors | A, seed 20260720 | recoverable | frozen D2 proxy | Diagnostic safety case |
| E1a-R final recovery matrix expected outcomes 15/15 | E1a-R report | `.../r3_final_progress_recheck_unchanged_cases/` and `r3_final_progress_recheck/` | rotors | isolated | n/a | none | **Paper-facing validation** |
| Final Scene A first recovery: 6.220 s, path .391072 m, full cycle; run success=0 | E1a-R report | `.../r4_scene_a_rotors_final/scene_a_e1a_r.bag`, `roslaunch.log`, `runs/fixed_switch__20260812_044737.csv`, `r4_final_first_reposition_timeline.csv` | rotors | A, seed 20260720 | recoverable | frozen D2 proxy | **Paper-facing case study with failure disclosed** |

“ideal” in the table means `ideal_set_model_state` and
`legacy_task_distance_hover_proxy_v1`. RotorS energy rows use a different
mechanical proxy and must not share a quantitative energy axis with ideal rows.

## 3. Current RotorS implementation hashes

| Source/config | SHA-256 |
|---|---|
| `uav_v4/src/mode_aware_planar_move_plugin.cpp` | `83ab8e12e825257b1cf9c55c1c6e110618b8634c0c3f05785888ea61a63e7d09` |
| `uav_v4/src/takeoff_clearance_plugin.cpp` | `c69dc7c5f2f8b4d3690b0592da7e527978aa79c9e3a3f9edb707becf7186d564` |
| `uav_v4/urdf/uav_v4_rotors_native.urdf.xacro` | `3db6170beed16354ce2372a2875e75b732030c9c0195efae423788d6a1c7b5b4` |
| `uav_v4/config/uav_v4_rotors_native.yaml` | `e14fa2256910391ba26be6981e97f9e4b9676d6cbfa669fc3bb95e7a39b96932` |
| `rescue_mission/scripts/rotors_handoff_coordinator.py` | `1a18d4188deeea8f3756129e2fd43515845316c4ffc18d546dac9fba02acf7e8` |
| `rescue_mission/scripts/rotors_motor_gate.py` | `e4fde185ef59be2f6d88afb4061ab2487fe6eca613a5f385f99b0ce0efa54a7b` |
| `rescue_mission/scripts/rotors_odometry_bridge.py` | `f555bfefca4bc4a0c9c9c2420bb20fc76ea04738d858da45b87876bcc0fca48d` |
| `rescue_mission/scripts/ground_goal_controller.py` | `abfd32fe761badffa99832dcdd6d1d6f0eeda776e3094b5e17d546c1c783303b` |
| `rescue_mission/scripts/rescue_metrics_logger.py` | `1c5a9aa725568a5a6327f41a262b582cd6f29e6fb87a10fc9e4370243b8f2b48` |
| `rescue_mission/scripts/mode_switcher.py` | `c9d84ab2dac41d24465529ab9ae2e7817ae3e7617847a679bf5f75a3de5e5f57` |
| `rescue_mission/scripts/safety_supervisor.py` | `8760ec32faa0c45975772f695ba90aefeeb829b1488681c771cd1c9032d57e65` |
| `rescue_mission/config/rotors_handoff_e1a_h.yaml` | `2d512e8f0ddd68ae18834f1c42a97faa2de987021ae5efbb1e35ba33e4f186a0` |
| `rescue_mission/launch/rescue_mission.launch` | `7ed62dd635e86af0bd9fc17e6522fb45eecfcb323a0f157f7d804c241cc91b1e` |
| `rescue_mission/launch/rescue_phase1_demo.launch` | `67bef96b3677b92f0074290ffb19118f4f0448ca73a08413cdaa47b12e4c16fe` |

## 4. Diagnostic/failure retention audit

- Phase C non-finite detailed-CAD evidence remains under
  `uav_v4_rotors_hybrid_smoke_20260718/` and its failed model is backed up.
- D2 corrupted first-pass CSV remains explicitly named
  `calibration_samples_corrupted_after_stale_process.csv` and is excluded.
- E1a-H `gate_h3_dev/yaw_generation_failed_run1` remains diagnostic.
- E1a-C `dev`, `c4_dev_*`, C5 failed bag/log, and C0 diagnosis remain present.
- E1a-R `r3_dev_*`, `r3_yaw_restore_recheck`, both failed R4 attempts, and the
  intermediate final-matrix failure remain present. Only the two explicitly
  named final-config directories constitute the 15-run acceptance set.
- Final E1a-R Scene A CSV records `success=0`; no report calls it DONE.

No canonical Stage 0 data or D2 artifact was modified during Phase F0.
