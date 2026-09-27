# Phase E1a-R2 Persistent Safe-Site Change Record

**Date:** 2026-08-28  
**Scope:** minimal cross-sortie relaunch-site persistence and one frozen Scene A RotorS validation.

## Pre-change preservation

Preimages were copied before modification to:

`docs/superpowers/specs/backups/2026-08-28-phase-e1a-r2/`

The workspace is not a reliable Git worktree, so this dated backup and the R2 SHA256 manifest are the change-control record. The earlier E1a-R report/bag/log/CSV hashes were rechecked and remain unchanged.

## Production and configuration changes

- `rescue_mission/scripts/safe_site_memory.py` — new bounded, deduplicated, measurement-only persistent site registry with per-episode unreachable/reached state.
- `rescue_mission/scripts/rotors_handoff_coordinator.py` — records stable-valid and exact approved sites; selects recent history first and persistent memory second; requires fresh current revalidation before arming; preserves bounded failure and ownership behavior; publishes audit metadata.
- `rescue_mission/config/rotors_handoff_e1a_h.yaml` — adds only memory capacity 32 and deduplication distance 0.15 m. Existing timeouts, gains, and safety geometry are unchanged.
- `rescue_mission/scripts/rescue_metrics_logger.py` — append-only addition of nine R2 metadata columns; no old Stage 0 column was removed or renamed.
- `rescue_mission/CMakeLists.txt` — installs the new pure Python module.

## Test and archival changes

- `rescue_mission/tests/test_safe_site_memory.py`
- `rescue_mission/tests/test_rotors_handoff_persistent_contract.py`
- `rescue_worlds/scripts/uav_v4_rotors_handoff_test.py`
- `rescue_worlds/scripts/run_takeoff_reposition_matrix.sh`
- `rescue_worlds/scripts/run_phase_e1a_r2_scene_a.sh`
- `rescue_worlds/results/phase_e1a_r2_full_scene_a_20260828/analyze_scene_a_r2_bag.py`

The result tree preserves all development isolation attempts, accepted isolation/regression runs, the single frozen mission bag/logs, bag-derived CSV/JSON, report, and SHA256 manifest. The runner now resolves its output directory to an absolute path; this fixes archival location only and was not followed by another mission run.

## Paper-only post-success changes

The RAS abstract, RotorS section, Fig. 5/caption, discussion, limitations, conclusion, SI, evidence provenance, claim audit, reviewer-risk register, and internal IROS compact text were updated. Fig. 5 is regenerated from the R2 bag-derived trace/event CSV plus unchanged Scene A SDF geometry. No Stage 0/F1 number, statistical result, policy method, D2 model, or canonical evidence file was changed.

## Verification summary

- `pytest`: 10/10 host memory/contract tests passed.
- Accepted reposition isolation matrix: 6/6.
- Sampled handoff/clearance/contact regressions: 2/2, 3/3, 3/3.
- Frozen Scene A bag audit: all acceptance flags true; `DONE@80.000 s`; final disarm 83.482 s; final motor 0.
- Static contract: no coordinator `cmd_vel` publisher, no RotorS `set_model_state`, and no Scene A coordinates/barrier-name special cases.
- Frozen hashes rechecked: Scene A YAML `b7b87d9c...`, Scene A world `73428576...`, mode switcher `c9d84ab2...`, D2 config `a49e5f58...`.
