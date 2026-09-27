# Phase F0 Project Freeze Change Record

**Date:** 2026-08-12  
**Scope:** documentation, integrity checks, evidence closure, and paper planning
only. No ROS/Gazebo experiment was run.

## Read-only checks

- Stage 0 canonical/latest mirror SHA-256:
  `e070dbfa608b4fdf52cf8f526b4b25f67c1a8b9dcf83134f849e1746121cad32`.
- Stage 0 manifest SHA-256:
  `149c509a9e66a5ccbf0ef0c9834e70b7a41b9e0359cfbc7b68fdba570de86124`.
- D2 YAML SHA-256:
  `a49e5f585d700ac4b031c1511f7979227443d6eaefa4ec9eb0d3565ff1ca4a40`.
- D2 raw/analysis/supervisor hashes matched the D2 change record.
- All 18 immutable Stage 0 Batch 2/sparse `all_runs` sources passed the frozen
  `SHA256SUMS.txt` check.
- Main host and `uav_noetic` container had no residual ROS master, roslaunch,
  Gazebo, rosbag recorder, or rescue mission process. The only host matches were
  the audit shell/search commands themselves.
- Failed and development evidence remains present under the Phase C, D2,
  E1a-H, E1a-C, and E1a-R result trees.
- The final E1a-R task CSV is explicitly `success=0`; it is not labelled DONE.

## New documents

- `2026-08-12-project-final-engineering-status.md`
- `FINAL_CLAIM_LEDGER_20260812.md`
- `FINAL_EVIDENCE_MANIFEST_20260812.md`
- `PAPER_OUTLINE_20260812.md`
- this change record

## Updated historical document

- `2026-07-18-project-handoff-current-status.md`
  - pre-append SHA-256:
    `e1b5e2be7997c5e554d6bca8143be9a5b06e59aee3847e9457937d961b2ba0c4`;
  - post-append SHA-256:
    `cb25706159f50a070d4a63326334150d425b31bab3658c01c8cb7aae5ad110ab`;
  - appended a dated superseded-status notice and pointer to the final status;
  - no historical paragraph was deleted or rewritten.

## Explicit non-changes

No ground planner, handoff code, Lee controller, robot model, plugin, scene,
route, safety volume, policy, CVaR/threshold parameter, D2 config, Stage 0
canonical data, experiment result, or analysis output was changed. No Scene B,
E1b, five-seed, Claim B RotorS, formal regime, or hardware run was started.

## Freeze decision

Engineering stops at representative RotorS validation. Full Scene A RotorS
mission success, RotorS B/C policy ranking, task-envelope energy guarantee,
battery safety, general planning, arbitrary landing recovery, and hardware
validation remain explicitly unclaimed.
