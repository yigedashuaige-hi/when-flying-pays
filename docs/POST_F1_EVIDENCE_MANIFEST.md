# Post-F1 Evidence Manifest

**Frozen:** 2026-08-22  
**Purpose:** manuscript-level provenance after the fair-tuning study. Paths are relative to `ros_ws/src` unless absolute. Original evidence remains in place and is read-only for paper derivation.

## Primary paper evidence

| Paper item | Frozen source | Backend / energy semantics | Use |
|---|---|---|---|
| F1 protocol | `rescue_mission/results/phase_f1_fair_tuning_20260821/PHASE_F1_PREREGISTRATION_20260821.md`; amendment; `configs/candidate_space.json`; `configs/run_protocol.json` | `ideal_set_model_state`; legacy task proxy | Equal candidate budget, seed split, selection rule, stop rules. |
| F1 selected configurations | `.../SELECTED_CONFIGS_BEFORE_EVALUATION.json`; `.../SELECTED_CONFIG_FREEZE_SHA256.txt` | Same | B: threshold `tau=.2`, risk `w_cvar=1.25`; C: threshold `tau=.4`, risk `w_cvar=1.0`. |
| F1 tuning results | `.../tuning_candidate_summary.csv`; all files under `.../runs/tuning/` | Same | All 200 valid tuning runs; never combine with held-out inference. |
| F1 held-out per-run data | `.../heldout_run_summary.csv`; all files under `.../runs/evaluation/` | Same | 120 runs, paired seeds 200–219, including fixed reference. Primary success/effort figure. |
| F1 inferential statistics | `.../heldout_paired_statistics.json`; `.../PHASE_F1_FINAL_REPORT_20260822.md` | Same | Exact McNemar, paired Wilcoxon, rank-biserial effect sizes, paired bootstrap intervals. |
| F1 integrity | `.../FINAL_INTEGRITY_AUDIT.json`; `.../SHA256_MANIFEST.sha256`; `.../SEED_AUDIT.json` | n/a | Completeness, no seed overlap, archive hashes. |
| Stage 0 provenance | `rescue_mission/results/STAGE0_RESULT_MANIFEST_20260718.md` | `ideal_set_model_state`; legacy task proxy | Canonical source map and backend semantics. |
| Permanent/recoverable regime pilot | `rescue_mission/results/summary_regime_scene_b_{permanent,recoverable}_*.csv`; matching `all_runs_*.csv`; `docs/superpowers/specs/2026-06-11-claim-a-regime-prereg.md` | Same; 5 paired seeds per configuration | Regime figure. Mark as a small preregistered pilot, not F1 held-out inference. |
| Recoverable Batch 2 | `rescue_mission/results/summary_airtight_scene_{b,c}_recoverable_*.csv`; matching `all_runs_*.csv` | Same; seeds 0–19 | Historical context and recoverable policy landscape. |
| Sparse expensive-flight corner | `rescue_mission/results/summary_airtight_scene_sparse_recoverable_*.csv`; matching `all_runs_*.csv`; `stage0_archive_20260718/significance_sparse_*.txt` | Same; `fly_k1=3`; seeds 0–9 | Boundary case showing that unnecessary flight can dominate task effort. |
| CVaR mechanism audit | `rescue_worlds/results/phase_e1a_contact_20260731/readonly_audit_cvar/CVAR_THRESHOLD_FAIRNESS_AUDIT_20260731.md`; `sampled_decision_boundary_{comparison,summary}.csv`; `rescue_mission/scripts/mode_switcher.py` | Analytic decision score; not physical energy | Piecewise CVaR and state-dependent local-threshold figure. |
| Stage 0 supervisor nominal/adversarial | `rescue_mission/results/stage0_archive_20260718/significance_scene_d_{nominal,adversarial}_risk_aware_safety_vs_risk_aware.txt`; named source CSVs in Stage 0 manifest | `ideal_set_model_state`; legacy ledger | Safe-abort versus mission-success table. Do not use the ambiguous generic Scene D report. |
| RotorS D2 calibration | `rescue_worlds/results/uav_v4_rotors_energy_d2_20260719/calibration_trial_summary.csv`; `calibration_analysis_d2.json`; D2 report | RotorS; `rotors_aero_shaft_mechanical_proxy_v1` | Measured-versus-predicted supplement and parameter/held-out error table. |
| RotorS D2 supervisor replay | `.../supervisor_case_results.csv`; `.../supervisor_case_results.json` | RotorS mechanical proxy ledger | Small replay case table; guarded zero reserve violations, unguarded two. |
| RotorS supervisor closed loop | `rescue_worlds/results/phase_e0_e1_rotors_20260720/supervisor_closed_loop/*/{metrics.csv,result.json}`; report | RotorS mechanical proxy ledger | Sufficient, preflight block, forced landing, unguarded violation, boundary behavior. |
| RotorS contact validation | `rescue_worlds/results/phase_e1a_contact_20260731/PHASE_E1A_REPORT_20260731.md`; contact matrix CSV/log evidence | RotorS/Gazebo dynamics | Finite-force ground contact, 15/15 cases; ground plugin releases in air. |
| RotorS handoff validation | `rescue_worlds/results/phase_e1a_h_handoff_20260731/PHASE_E1A_H_HANDOFF_REPORT_20260731.md`; H3/H4 result files | RotorS/Gazebo dynamics | Ownership state machine, 20/20 isolated cases, first exact-scene cycle; full task failed. |
| RotorS takeoff clearance | `rescue_worlds/results/phase_e1a_c_takeoff_clearance_20260812/PHASE_E1A_C_TAKEOFF_CLEARANCE_REPORT_20260812.md`; C4/C5 evidence | RotorS/Gazebo dynamics | 40/40 clearance matrix and fail-safe blocked Scene A takeoff; no planner claim. |
| RotorS takeoff reposition | `rescue_worlds/results/phase_e1a_r_takeoff_reposition_20260812/r4_final_first_reposition_timeline.csv`; `PHASE_E1A_R_TAKEOFF_REPOSITION_REPORT_20260812.md`; final run CSV/bag | RotorS/Gazebo dynamics | Representative blocked→reposition→flight→land→disarm timeline. Full mission `success=0`. |
| Final engineering scope | `docs/superpowers/specs/2026-08-12-project-final-engineering-status.md`; F0 change record | n/a | Frozen boundary between canonical evaluation and representative validation. |

## Derived paper artifacts

All paper-derived figures and tables live under `paper_when_flying_pays_20260822/`. Their generation script reads the sources above and writes only into that paper directory. Derived artifacts are not new experimental evidence.

## Evidence exclusions and cautions

- Do not use technically invalid F1 attempt directories (`invalid_preflight_schema_sentinel`, `technical_invalid_*`) as scientific observations.
- Do not treat old `latest.csv` symlinks or mirrors as independent runs.
- Do not use all-seed Scene C energy to argue efficiency; four threshold failures ended early.
- Do not merge Stage 0 task-proxy joules with RotorS shaft-mechanical joules.
- Do not claim final Scene A RotorS mission completion: the final run is `success=0`.
- Do not call the RotorS proxy battery energy.
- Preserve the original F0 claim/evidence/outline documents as the historical pre-F1 position.

