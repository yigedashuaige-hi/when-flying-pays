# Phase F1 Scene C Censoring-Sensitivity Replay

Date: 2026-08-29  
Status: **complete; 4/4 valid replays; label flips = 0/4**

## Scope and interpretation boundary

This is a **post-hoc stopping-sensitivity analysis**, not F1b and not a new
confirmatory comparison. It answers one question only: whether the four
wall-clock-censored Scene C `threshold_switch` runs from Phase F1 change their
completion label when they receive the complete 85 s `/clock` horizon.

The original F1 files and labels remain frozen. These replays must not be used
to replace the preregistered F1 outcomes, recompute the original exact McNemar
test, retune `tau`, or make a new method-ranking claim. No paper file was
modified.

## Frozen conditions and stopping rule

All valid replays used:

- Scene C, `stuck_model=recoverable`;
- `strategy=threshold_switch`, `switch_tau=0.40`;
- seeds 204, 207, 209, and 212 only;
- `air_backend=ideal` (`ideal_set_model_state`);
- the frozen Scene C world, physics, robot, mission, perception, policy,
  ablation, energy, and launch parameters;
- `heteroscedastic=false`, `initial_battery=-1`, `fly_k1=-1`, GUI disabled,
  headless and unpaused execution.

The only scientific execution change was the outer stopping mechanism:

1. `DONE` observed at `/clock < 85.0 s` is a success;
2. otherwise, reaching `/clock = 85.0 s` is a timeout/failure;
3. the monitor observes 0.05 s beyond the boundary only to drain queued ROS
   callbacks; a later `DONE` cannot change the label;
4. the 900 s wall watchdog is infrastructure-only. If it fires, the run gets
   no completion label.

The valid runs stopped at `/clock=85.055--85.094 s`; none triggered the wall
watchdog.

## Pre-run drift audit

The following F1 freeze anchors matched byte for byte:

| Artifact | Frozen/current SHA256 prefix | Match |
|---|---:|---:|
| `mode_switcher.py` | `c9d84ab2dac41d24` | yes |
| `mode_switch_params.yaml` | `9ca954e22c3244ce` | yes |
| `scene_c.yaml` | `747ca8dd268de86b` | yes |
| `rescue_mission.launch` | `7ed62dd635e86af0` | yes |
| `rescue_phase1_demo.launch` | `67bef96b3677b92f` | yes |
| Docker image | `85592064297803dd` | yes |

The environment also matched ROS Noetic `roslaunch 1.17.4`, Gazebo Classic
11.15.1, and Python 3.8.10. The current metrics logger contains append-only R2
safe-site fields and subscriptions relative to the pre-R2 version. Those
fields are passive in `air_backend=ideal`, have no publishers in this launch,
and do not enter the policy, perception, controller, or physics path. This
non-scientific schema extension is recorded in
`PRE_RUN_CONDITION_AUDIT_20260829.json`.

As a dynamic reproduction check, replay pose at each old wall cutoff differed
from the old F1 terminal pose by only 0.00094--0.01073 m in XY.

## Result

| Seed | Old label at wall cutoff | Old sim time (s) | Replay label at full horizon | Stop `/clock` (s) | State / mode at horizon | Final XY (m) | Slip | Poor-motion events | AIR requests |
|---:|---|---:|---|---:|---|---:|---:|---:|---:|
| 204 | failure | 42.6 | failure | 85.062 | `NAVIGATE / GROUND` | (3.6352, 0.7875) | 0.8973 | 125 | 0 |
| 207 | failure | 52.4 | failure | 85.090 | `NAVIGATE / GROUND` | (3.6339, 0.7916) | 0.8971 | 124 | 0 |
| 209 | failure | 50.0 | failure | 85.055 | `NAVIGATE / GROUND` | (3.6307, 0.8001) | 0.8977 | 124 | 0 |
| 212 | failure | 67.0 | failure | 85.094 | `NAVIGATE / GROUND` | (3.6345, 0.7899) | 0.8979 | 125 | 0 |

**Label flips: 0/4.** All four old F1 labels remain unchanged under the full
simulation-time horizon.

## What the robot was doing at 85 s

The four robots were not close to full mission completion. Each had completed
only the first of five configured outbound waypoints and remained in
`NAVIGATE` while approaching the second waypoint `(4.5, 1.2)`. Their distance
to that current waypoint was still 0.9569--0.9581 m, outside the 0.5 m mission
transition tolerance. None had reached later outbound waypoints, `DELIVER`,
`RETURN`, or `DONE`.

All mode histories contained `GROUND` only. All switch-reason histories were
`threshold_switch:ground(tau=0.40)`, with zero AIR requests, zero switches,
and zero air distance. The trajectories repeatedly made poor ground progress
inside the configured slope zone (`x=3.0--5.2 m`), accumulating 124--125
poor-motion events and approximately 0.897 slip. Thus the added sim time did
not reveal a nearly completed mission censored by wall time; it revealed the
same persistent ground-progress bottleneck.

## Wall/simulation timing

The valid replays were run sequentially. Their last recorded
sim-time/wall-time ratios were 0.914--0.932, and total wall elapsed time,
including orderly bag and roslaunch shutdown, was 94.7--96.5 s. This contrasts
with the heavily parallel original F1 execution, where the 130 s wall cutoff
occurred at only 42.6--67.0 s of simulation time for these seeds. The replay
therefore isolates the intended censoring mechanism without changing the
85 s scientific horizon.

## Technical-invalid attempts

Two initial seed-204 infrastructure attempts are retained and excluded. The
external monitor's own `rospy` process initially used the default ROS master
instead of the isolated per-run master, so its stopping loop never became
active. No completion label was assigned to either attempt. The cause was
fixed by setting the same `ROS_MASTER_URI` and `GAZEBO_MASTER_URI` in the
monitor process before `rospy` initialization and by removing provenance
subprocesses from the run-time critical path. Both attempts, logs, CSVs, bags,
incident records, and protocol amendments are preserved under
`technical_invalid_attempts/`; they do not contribute to the four-row replay
dataset.

## Evidence inventory

- `replay_run_summary.csv`: authoritative four-run replay summary, histories,
  final metrics, bag paths, and hashes.
- `old_vs_replay_comparison.csv`: per-seed old-to-replay labels and matched old
  cutoff reproduction checks.
- `audit.json`: machine-readable final integrity audit.
- `runs/seed_*/csv/`: complete logger CSVs; old F1 paths are untouched.
- `runs/seed_*/bags/`: indexed ROS bags spanning approximately 0.17--85.09 s
  `/clock`, 102,862--102,905 messages each.
- `runs/seed_*/progression/`: sim/wall/RTF, XY/velocity, waypoint, mission-state,
  mode, and switch-reason histories.
- `runs/seed_*/logs/` and `metadata/`: full roslaunch/rosbag logs, exact launch
  command, stop decision, and infrastructure status.
- `REPLAY_PROTOCOL_FROZEN_BEFORE_RUNS.json`, two protocol amendments, and two
  technical incident records: stopping semantics and transparent execution
  provenance.
- `SHA256_MANIFEST.txt` and `SHA256_MANIFEST.sha256`: final result freeze.

## Does this require F1b?

**Not for the narrow censoring question.** The 0/4 flip result, together with
the unchanged `NAVIGATE/GROUND` histories and close old-cutoff trajectory
reproduction, shows that the four F1 Scene C threshold failures were not
created merely by receiving less than 85 s of simulation time.

**A genuine F1b would still be required for a stronger confirmatory claim.**
The original Scene C success contrast remains 20/20 versus 16/20 with exact
McNemar `p=0.125`; this post-hoc replay cannot increase its inferential sample
size or change that p-value. A separately preregistered, fresh paired-seed F1b
would be justified only if the research objective is to obtain more precise
evidence about the Scene C success difference or generalization beyond the
original held-out seeds. It is not needed solely to repair the stopping-rule
ambiguity diagnosed here.
