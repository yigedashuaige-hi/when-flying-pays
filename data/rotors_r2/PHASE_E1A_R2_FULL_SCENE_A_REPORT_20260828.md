# Phase E1a-R2 Full Scene A Closure Report

**Date:** 2026-08-28  
**Status:** PASS — the single authorized frozen Scene A RotorS case reached `DONE`; no further simulator experiments were run.  
**Evidence role:** post-freeze engineering validation extension, not Stage 0/F1 policy evidence.

## Scope and frozen condition

The run retained Scene A, `fixed_switch`, seed `20260720`, recoverable ground physics, `air_backend=rotors`, the 120-s mission timeout, the frozen D2 supervisor configuration, the existing Lee gains, safety volume, route, and success criterion. No Stage 0/F1, Scene B/C, CVaR/threshold, D2, or additional RotorS experiment was started.

The earlier E1a-R failure is preserved unchanged. Its report, bag, launch log, and CSV still hash to:

- report: `17ecc3ae3ffaf84d12f306a24ea9a153bba488d5fc009f6c0c29bb71be2689da`
- bag: `c83e8d08d0614ad6167cd3d2fdb1961e7f81821eb13e82d9b8fa2f2969580ffe`
- launch log: `e95b8cd4d8cae0435c279135c38a6d668fd4f0e6ec2225611133d3ffd7db5fcd`
- CSV: `f0b461aca0f389a57b7ee7b49bb6cd54f7f47f3e0f4618193cd9f3b1ed2b1e19`

## Minimal generic repair

R2 adds a bounded `SafeSiteMemory` independent of Scene A. A record is accepted only from finite, measured, stable-valid ground evidence and contains:

- measured pose and yaw;
- observation and last-validation timestamps;
- available/required clearance, nearest obstacle, and recheck distance;
- evidence source (`stable_ground` or exact `approved_takeoff`);
- ground epoch, validation count, and per-episode reachability state.

The existing recent reverse-ground history remains first priority. If it has expired, the coordinator may select the nearest admissible persistent site as a navigation target. A stored record never authorizes flight: after arrival, current measurements must again satisfy the unchanged validity, stability, clearance, and freshness checks. No-progress or timeout marks the target unreachable for the current episode. In `TAKEOFF_REPOSITION_FAILED`, the system can continue only through request withdrawal or a newly measured fresh-valid current site.

This is not a Scene A hack: production code contains no Scene A coordinates, `low_barrier` name, route identifiers, obstacle names, or `set_model_state`; it does not publish `/mecanum/cmd_vel`. The existing ground controller retains the sole planar command publisher, while the coordinator only gates ownership and a reposition goal.

## Isolation and regression checks

- Host tests: 10/10 passed (`test_safe_site_memory.py`, `test_rotors_handoff_persistent_contract.py`).
- Reposition matrix: 6/6 passed: current-valid, just-invalid, invalid-after-move, no-history fail-safe, unreachable fail-safe, and cross-sortie persistent recovery.
- In the cross-sortie isolation case, recent history was deliberately allowed to age beyond 12 s; an `approved_takeoff` record about 20.246 s old was selected, reposition used zero motor speed, current evidence was revalidated, and the second sortie completed.
- Sampled handoff regression: 2/2 passed (static and moving).
- Sampled clearance regression: 3/3 passed (open, contact-stall abort, landing handshake).
- Sampled contact regression: 3/3 completed (head-on, oblique, corner).
- Stale/invalid records are rejected by the memory contract; unreachable records are excluded only for the current episode.

Development-only failed isolation attempts are retained under `isolated/persistent_cross_sortie_dev*`: the first exposed a catkin Python import-wrapper issue, and the second showed that the test had not yet aged out recent history. The final `final_reposition_6x1` directory is the accepted matrix.

## Frozen Scene A command

The run was launched once through:

```bash
src/rescue_worlds/scripts/run_phase_e1a_r2_scene_a.sh \
  src/rescue_worlds/results/phase_e1a_r2_full_scene_a_20260828/scene_a_frozen/run_1
```

The script expands to `roslaunch rescue_mission rescue_phase1_demo.launch` with `gui:=false`, `headless:=true`, `strategy:=fixed_switch`, `scene:=scene_a`, `seed:=20260720`, `mission_timeout:=120`, `stuck_model:=recoverable`, and `air_backend:=rotors`. The outer 140-s shell timeout returned status 124 after the mission and post-DONE landing had completed; it does not indicate mission timeout. The bag spans simulation time 2.139--133.354 s and is complete.

## Complete mission timeline

| Simulation time (s) | Event and evidence |
|---:|---|
| 10.020 | Takeoff request blocked by current swept-volume check; motors remain off. |
| 10.022--16.140 | Recent reverse-ground history selected; reposition succeeds. |
| 16.501 | First motor enable; current measurement is fresh and site-valid. |
| 18.780 | First horizontal AIR phase. |
| 20.021--24.772 | LAND, touchdown, motor off, disarm. |
| 25.222 | Ground ownership/control restored. |
| 30.020 | Second requested site invalid; recent history has expired. |
| 30.023 | Persistent site 21 (`approved_takeoff`, age 13.572 s) selected as an unverified cross-sortie navigation target; motor remains zero. |
| 38.022 | Unchanged 8-s reposition bound expires after approximately 1.416 m ground motion; site marked unreachable for this episode; no takeoff. |
| 40.023 | Request withdrawn and nominal ground travel resumes. |
| 50.021--50.581 | Current location passes fresh site/clearance validation; second takeoff arms without using memory as authorization. |
| 52.880 | Second AIR phase. |
| 60.021--62.102 | Second LAND and disarm. |
| 62.541 | Ground control restored. |
| 70.002--75.083 | Third blocked request uses recent reverse history and is freshly revalidated. |
| 75.441 | Third takeoff; site evidence is fresh and valid. |
| 77.721 | Third AIR phase. |
| 80.000 | Mission state reaches `DONE`; `success=1`, `timeout=0`. |
| 80.021--83.482 | Final LAND and disarm. |
| 83.921 | Final `GROUND_DRIVE`; measured mean motor speed is 0 rad/s. |

## Acceptance audit

`R2_BAG_AUDIT.json`, derived directly from the frozen bag, reports:

- `success=1`, `timeout=0`, final mission state `DONE`;
- 3 takeoffs, all with measurement-valid, site-valid evidence aged about 0.013 s;
- 3 landing episodes, all followed by disarm;
- final handoff `GROUND_DRIVE`, final disarmed true, final mean motor speed 0 rad/s;
- exactly one `uav_v4` throughout;
- zero non-finite odometry/IMU/motor/model-state samples;
- zero dual-ownership samples;
- zero invalid-site takeoff-motor samples;
- maximum motor speed during reposition 0 rad/s;
- no `/gazebo/set_model_state` client in the RotorS coordinator/branch.

All acceptance flags are true.

## Archival note

The original run script passed a relative logger output path into a container whose ROS working directory was ephemeral. Consequently, the logger's high-rate task CSV did not survive container removal. The complete bag and launch/rosbag logs did survive. R2 therefore derives and saves `R2_MISSION_SUMMARY.csv`, `R2_EVENT_TIMELINE.csv`, and the deterministic 10-Hz `R2_PAPER_TRACE_10HZ.csv` from that bag; the analyzer also audits the full-rate odometry, IMU, motor, command, and Gazebo model-state streams. The script now canonicalizes its result path for reproducible future use, but the frozen mission was not rerun.

## Interpretation

R2 closes the known cross-sortie execution/recovery interface gap and demonstrates one end-to-end unchanged Scene A case. It does not show that every persistent site is reachable, provide a local planner, estimate a mission success rate, rank switching policies, validate hardware, or extend the D2 short-flight mechanical-energy envelope.

