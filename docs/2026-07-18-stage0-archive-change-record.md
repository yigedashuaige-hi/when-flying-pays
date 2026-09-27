# Stage 0 Archive — Change Record

**Date:** 2026-07-18  
**Scope:** result verification and documentation only  
**Explicitly not changed:** algorithms, parameters, robot models, launch
semantics, worlds, experiment runner, CSV source data, RotorS integration

## Requested work completed

1. Verified Batch 2 final outputs and generated missing paired significance
   reports from the existing immutable CSVs.
2. Added dated archival notes next to stale Batch 2 `running` history without
   deleting or rewriting the original record.
3. Created `rescue_mission/results/STAGE0_RESULT_MANIFEST_20260718.md` mapping
   paper tables/figures to sources, scripts, strategies, scenes, stuck models,
   and `backend=ideal_set_model_state`.
4. Created and checksum-verified pre-edit backups because Git metadata is not
   reliable in the current environment.
5. Recorded data/provenance inconsistencies rather than silently fixing names or
   overwriting old artifacts.

## Pre-edit backups

Directory:

`docs/superpowers/specs/backups/2026-07-18-stage0-archive/`

| original | backup | pre-edit SHA-256 (both copies matched) |
|---|---|---|
| `2026-06-11-paper-pivot-framework.md` | `2026-06-11-paper-pivot-framework.md.before` | `7a45fb6645f0eb12ac2c6f7a25a2061c84615ae2172571a7c0b82f62f5e5a19d` |
| `2026-06-11-claim-a-regime-prereg.md` | `2026-06-11-claim-a-regime-prereg.md.before` | `0ea6456a5b416990d836748943f7fe74441f0d97b9f6568ab84eba193c3038dc` |
| `2026-07-18-project-handoff-current-status.md` | `2026-07-18-project-handoff-current-status.md.before` | `343e302cfc61a948743b87c6ddbf44fee0bf7c19a0cfaaaeb83228791990aedf` |

## Modified documentation

### `2026-06-11-paper-pivot-framework.md`

Added a dated archival block immediately after the original “Batch (2) is still
running” paragraph. The old paragraph remains intact. The addendum records final
Scene B/C values and points to the canonical manifest.

### `2026-06-11-claim-a-regime-prereg.md`

Appended a dated completion section after the original `Remaining: ... is
running` line. It records all ten final aggregate rows, selected paired tests,
the exact limitation on the nonsignificant Scene B threshold energy contrast,
and the immutable-source manifest.

### `2026-07-18-project-handoff-current-status.md`

Added a Phase A completion note pointing to the result manifest and this change
record. No Phase B/RotorS work was started.

## New result artifacts

Directory: `rescue_mission/results/stage0_archive_20260718/`

- `significance_batch2_scene_b_risk_aware_vs_fixed_switch.txt`
- `significance_batch2_scene_b_risk_aware_vs_threshold_t0.2.txt`
- `significance_batch2_scene_c_risk_aware_vs_fixed_switch.txt`
- `significance_batch2_scene_c_risk_aware_vs_threshold_t0.3.txt`
- `significance_sparse_risk_aware_vs_threshold_t0.6.txt`
- `significance_sparse_risk_aware_vs_energy_rule.txt`
- `significance_scene_d_nominal_risk_aware_safety_vs_risk_aware.txt`
- `significance_scene_d_adversarial_risk_aware_safety_vs_risk_aware.txt`
- `SHA256SUMS.txt`

For Batch 2/sparse comparisons, two source `all_runs` files were mechanically
concatenated in `/tmp`, analyzed with the unchanged
`scripts/analyze_significance.py`, and the temporary file was deleted. Scene D
inputs already contained both strategies.

## New manifest

`rescue_mission/results/STAGE0_RESULT_MANIFEST_20260718.md`

The manifest separates:

- recoverable Batch 2 from earlier permanent-trap figures;
- sparse expensive-flight evidence from default-flight evidence;
- nominal Scene D from adversarial Scene D;
- canonical 20-seed uncertainty files from earlier 10-seed files;
- paper-facing evidence from the Scene E NO-GO audit trail.

Every experiment entry is explicitly labelled
`backend=ideal_set_model_state`.

## Verification findings

### Passed

- Batch 2 has 10/10 expected configurations.
- Each configuration contains 20 rows and exactly seeds 0--19.
- Scene tags match filenames.
- Runner log has a completion marker and final code zero.
- Stored summary means match recomputation from per-run tables.
- Sparse corner has eight expected configurations with ten runs each.

### Inconsistencies/caveats found

1. `significance_ph_scene_d.txt` contains the adversarial 20/20 -> 0/20 result,
   although its generic name can be mistaken for nominal. It was preserved; new
   explicit nominal/adversarial files were created.
2. `figures_ph_scene_d/plot_manifest.md` lists output names but not its input
   summary. Timing strongly associates it with the adversarial run, but the
   provenance is insufficient for a canonical claim; the result manifest marks
   it noncanonical.
3. `figures_ph_scene_b/` and `figures_ph_scene_c/` are permanent-trap
   paper-hardening figures, not recoverable Batch 2 figures.
4. Batch 2 had no paired significance text before this archive because treatment
   and baseline were split across per-strategy tables.
5. Scene B risk_aware versus threshold tau=0.2 has a clear aggregate Pareto loss
   but paired energy p=0.06372; documentation now avoids claiming significance
   for that individual contrast.
6. Exact success contrasts in the selected Batch 2 pairs are 19/20 versus 20/20
   and McNemar p=1.0. The scientific conclusion rests on Pareto/energy evidence,
   not a statistically significant success difference.

## Stop point

Stage 0 result closure is complete. No `uav_v4`, RotorS, controller, world,
algorithm, or experiment-semantic work was started after this archive.

