# Phase D2 RotorS energy/supervisor report

Date: 2026-07-19  
Status: **complete for small-scale D2 validation; not a battery-safety proof**  
Backend: `rotors`  
Energy model: `rotors_aero_shaft_mechanical_proxy_v1`

## 1. Outcome

The RotorS speed/slowdown data path was confirmed from source, a predeclared
18-trial calibration/held-out set was completed without NaN/Inf, a staged model
was selected, and a one-sided conservative mechanical-proxy budget was wired as
an optional RotorS supervisor backend. Guarded replay approved the clearly safe
and near-boundary-safe cases, blocked the unaffordable takeoff, forced landing
for an airborne low-margin case, and produced zero guarded reserve violations.

This report never interprets the proxy as battery electrical energy. No paper
batch, regime study, policy retuning, scene change, or Stage-0 data rewrite was
performed.

## 2. Motor speed and slowdown source audit

The source-level data flow is:

1. Lee publishes desired physical rotor angular speed as
   `mav_msgs/Actuators` on `/uav_v4/command/motor_speed_raw` (rad/s).
2. `rotors_motor_gate.py` checks finiteness/range and publishes
   `/uav_v4/command/motor_speed` (rad/s), or four zeros while disarmed.
3. The RotorS controller interface forwards the command to the motor plugins.
4. Each motor plugin sets its simulated joint speed to
   `omega_command / rotorVelocitySlowdownSim` (`gazebo_motor_model.cpp:480-482`).
5. For force calculation the same plugin restores physical-model speed once:
   `omega_real = omega_joint * rotorVelocitySlowdownSim`
   (`gazebo_motor_model.cpp:417-424`), then uses
   `T_i = k_f * omega_real^2` (`:428-431`) and
   `Q_i = T_i * k_m` (`:458-459`).
6. The aggregate multirotor plugin publishes joint velocity multiplied by the
   slowdown once (`gazebo_multirotor_base_plugin.cpp:115-120`). Therefore
   `/uav_v4/motor_speed` is the restored physical-model rotor speed in rad/s.
7. The individual `/uav_v4/motor_speed/{0..3}` Float32 topics are published
   directly from `joint_->GetVelocity(0)` (`gazebo_motor_model.cpp:34-37`) and
   are the slowed simulated joint speed, approximately aggregate speed / 10.

The native model config uses `k_f=1.568e-05 N/(rad/s)^2`, `k_m=0.06 m`, and
`rotorVelocitySlowdownSim=10` in both the motor and aggregate plugins
(`uav_v4_rotors_native.urdf.xacro:139-146,160-165`). The logger consumes only
the aggregate `/uav_v4/motor_speed`; it must **not** multiply by 10 again.

As an independent consistency check over 1,160 D2 hover rows:

- mean aggregate rotor speed: `690.737 rad/s`;
- analytic hover speed: `sqrt(mg/(4*k_f)) = 690.019 rad/s`;
- modeled total thrust: `29.9277 N`;
- robot weight (`3.0440996088045096 kg * 9.81 m/s^2`): `29.8626 N`;
- difference: `+0.218%`.

This agreement rejects both a missing slowdown factor and a repeated factor of
10 at hover.

## 3. Mechanical proxy

The recorded aerodynamic shaft/reaction-torque proxy remains:

```text
P_proxy(t) = k_f * k_m * sum_i(|omega_i(t)|^3)       [W mechanical proxy]
E_proxy    = integral P_proxy(t) dt                   [J mechanical proxy]
```

It models rotor reaction torque times angular speed. It excludes motor/ESC
losses, efficiency maps, battery voltage sag, avionics/compute loads, thermal
effects, and regenerative behavior. It is not battery electrical power or
energy and provides no battery-level safety guarantee.

## 4. Controlled data set

Final canonical D2 raw data is `calibration_samples.csv`: 5,879 rows, 104
columns, all checked energy values finite, all flight rows valid. The plan and
completion record is `calibration_run.json`.

| Dimension | Levels / repeats |
|---|---|
| Split | 12 calibration, 6 held-out |
| Takeoff/landing | 18 each; every trial ended with measured disarm |
| Height | 1.0 m x 9; 1.5 m x 9 |
| Hover | 2 s x 4; 3 s x 10; 5 s x 4 |
| Horizontal distance | 0 m x 10; 0.6 m x 2; 0.9 m x 4; 1.2 m x 2 |
| Target speed | 0.25 m/s x 2; 0.30 m/s x 4; 0.45 m/s x 2 |

The six held-out samples were labeled before collection and were not used for
fitting. All 18 trials completed takeoff, their declared action, landing, and
disarm, with no non-finite odometry or motor vector.

During the first collection pass, a roslaunch child survived its parent exec
session. A later ideal check opened the same raw path and created a sparse-file
hole. That invalid raw file is preserved as
`calibration_samples_corrupted_after_stale_process.csv`; it is not used by any
final number. The identical predeclared plan was rerun from a clean container,
then the container was restarted to close all ROS/Gazebo children. The final
raw file has SHA-256
`23d87d4b7a3e52daf8fc671d4e3ab67a015e8aa7398e4eb4eb8370a58a48c105`.

## 5. Regression review and selected model

For the four moving calibration trials, planned translation time and round-trip
distance have Pearson correlation `1.0`; the corresponding two-predictor VIF is
effectively infinite. Actual translation time versus distance is `0.999616`.
Consequently, the old joint time-plus-distance form is retained only as a
post-flight comparison, not used by the supervisor.

| Held-out result | D1-style joint model | Selected staged model |
|---|---:|---:|
| MAE | 236.94 J | 245.22 J |
| RMSE | 444.28 J | 373.81 J |
| MAPE | 1.118% | 1.113% |
| maximum underprediction | 1,070.59 J | 847.70 J |

The selected preflight-usable stage model is:

```text
E_hat = (a0 + a_h*h)
      + P_hover*t_hover
      + P_translation*(2*d/v + t_settle)
      + E_land_base
```

Parameters fitted only on the 12 calibration trials:

- `a0 = 3679.979 J`, `a_h = 758.016 J/m`;
- `P_hover = 1240.403 W`;
- `P_translation = 1234.909 W`;
- controller settling allowance `t_settle = 3.058 s`;
- robust base landing estimate `E_land_base = 2584.233 J`.

In 1,982 valid deterministic bootstrap resamples, the 5-95% ranges were:

- hover power: `1239.313-1241.867 W`;
- translation power: `1234.690-1235.097 W`;
- takeoff height slope: `756.607-759.029 J/m`;
- settling allowance: `2.700-3.467 s` (largest relative instability, CV 10.7%);
- landing base median: `2576.866-2586.747 J`.

Landing is handled separately because the held-out tail reached `3690.055 J`,
far above the `2584.233 J` base. The held-out maximum total underprediction was
`847.700 J`; the maximum non-landing underprediction was only `2.472 J`, which
confirms that the dominant tested tail was landing/contact behavior.

## 6. Conservative one-sided budget

The RotorS-only config is `rescue_mission/config/rotors_supervisor_d2.yaml`.
All values are rounded upward to 100 J:

| Component | Value | Construction |
|---|---:|---|
| `E_takeoff_j` | 4,900 J | observed maximum 4,817.603 J, rounded up |
| `E_air_reserve_j` | 21,900 J | declared envelope: 5 s hover plus 1.2 m each way at 0.25 m/s plus settling; prediction 21,833.905 J |
| `E_land_j` | 3,700 J | observed landing tail maximum 3,690.055 J, rounded up |
| `E_margin_j` | 900 J | held-out maximum total underprediction 847.700 J, rounded up; not mean error |
| takeoff approval floor | 31,400 J | sum of the four components |

Validated envelope: target height <= 1.5 m, hover <= 5 s, one-way distance <=
1.2 m, and translation speed >= 0.25 m/s. Extrapolation outside this envelope
is not validated and must not inherit this guarantee.

## 7. Supervisor backend and CSV contract

`air_backend:=ideal` continues to instantiate the historical supervisor against
`/rescue/battery_j` with the original 11/12/12/8 J parameters and legacy model.
`air_backend:=rotors` loads the separate D2 YAML and evaluates the mechanical
remaining ledger `/rescue/rotors_mechanical_remaining_j`. Policy costs and the
Stage-0 ledger are not replaced or numerically mixed.

Eight fields were appended after all Stage-0 and D1 columns:

```text
supervisor_energy_backend
supervisor_energy_model
supervisor_predicted_consumption_j
supervisor_actual_consumption_j
supervisor_reserve_j
supervisor_block_reason
rotors_mechanical_budget_j
rotors_mechanical_remaining_j
```

The existing `safe_landing_margin_j` remains for compatibility; analyses must
key any cross-backend supervisor value by `supervisor_energy_backend` and
`supervisor_energy_model`.

## 8. Guarded/unguarded cases

These are deterministic decision-layer replays using the fitted budget and
held-out observed energies, not new physics flights and not a paper batch.

| Case | Available | Guarded | Unguarded | Guarded violation |
|---|---:|---|---|---:|
| sufficient takeoff | 36,500 J | TAKEOFF | TAKEOFF | 0 |
| near boundary safe | 31,401 J | TAKEOFF | TAKEOFF | 0 |
| below full-envelope/landing floor | 31,300 J | GROUND, `takeoff_unsafe_low_battery` | TAKEOFF | 0 (unguarded: 1) |
| airborne low margin | 4,500 J | LAND, `air_unsafe_force_land` | AIR | 0 (unguarded: 1) |

Across all guarded cases: zero reserve violations. The counterfactual
unguarded decisions contain two conservative-reserve violations. Clearly safe
actions were not blanket-blocked.

## 9. Compatibility and infrastructure checks

- Ideal supervisor parity: 448 state/action/budget cases against the backed-up
  pre-D2 implementation, zero mismatches.
- Clean ideal runtime: 79 rows, old 55-column prefix identical, backend/model
  `ideal`/`legacy_task_distance_hover_proxy_v1`, legacy energy changed normally.
- Stage-0 latest mirror SHA-256 remains
  `e070dbfa608b4fdf52cf8f526b4b25f67c1a8b9dcf83134f849e1746121cad32`.
- RotorS wiring runtime: D2 params loaded, remaining budget topic was 37,000 J,
  new CSV metadata was correct, and ground-only mechanical consumption remained
  zero.
- RotorS launch graph contained Lee, motor gate, odometry bridge, and logger,
  but no `air_motion_executor`. The RotorS code path contains no
  `SetModelState` client; `/gazebo/set_model_state` remains only a Gazebo-owned
  advertised service.

## 10. Main commands

```bash
roslaunch rescue_worlds uav_v4_rotors_energy_calibration.launch \
  output_csv:=.../calibration_samples.csv gui:=false headless:=true

python3 src/rescue_worlds/scripts/uav_v4_rotors_energy_d2_calibration.py \
  --output .../calibration_run.json

python3 src/rescue_worlds/scripts/analyze_rotors_energy_d2.py \
  --csv .../calibration_samples.csv --plan .../calibration_run.json \
  --output .../calibration_analysis_d2.json

python3 src/rescue_worlds/scripts/validate_rotors_supervisor_d2.py \
  --config src/rescue_mission/config/rotors_supervisor_d2.yaml \
  --analysis .../calibration_analysis_d2.json \
  --output-json .../supervisor_case_results.json \
  --output-csv .../supervisor_case_results.csv
```

The commands were executed inside the existing `uav_noetic` ROS1 Noetic /
Gazebo Classic container where ROS was required.

## 11. Limits before a final regime study

1. The data cover only one model/payload, no wind, two heights, short distances,
   and 18 trials. The maximum residual is an empirical bound for this set, not
   a population-level or probabilistic guarantee.
2. Landing/contact energy is the dominant tail and needs more repetitions,
   surfaces, lateral residual velocities, and quantile/tolerance-bound work.
3. Electrical motor constants, ESC efficiency, voltage/current telemetry, and
   battery dynamics remain unavailable; no battery-state guarantee is possible.
4. The fixed air reserve is valid only for the declared maneuver envelope. A
   route-conditioned predictor and explicit out-of-envelope rejection are still
   needed before mission-scale experiments.
5. The guarded/unguarded evidence here is decision replay plus wiring smoke, not
   a statistically powered closed-loop regime study.

Phase D2 stops here. Phase E/final regime study and paper batches were not
started.

