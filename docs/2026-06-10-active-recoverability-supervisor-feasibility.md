# Active Recoverability Supervisor — Feasibility Assessment (evaluation only)

**Status: assessment + effort estimate only. No implementation in this pass.**

Question: in the current sim, what would it take to upgrade the safety supervisor
from the present *reactive one-step gate* to an *active recoverability* guard that
(i) maintains, at every step, the invariant "there exists an energy-reachable safe
exit", and (ii) shapes *earlier* decisions so the robot never enters an
unrecoverable dead-end?

---

## 1. What the current supervisor already does (the reactive half)

`safety_supervisor.py` is a one-step filter on `(current_mode, battery_j,
landing_area_safety)`:

- **TAKEOFF gate**: block takeoff unless
  `battery_j >= E_takeoff + E_air_reserve + E_land + E_margin`. This is already a
  recoverability check *at the moment of entering air*.
- **AIR-continuation gate**: while airborne, if `battery_j < E_land + E_margin`,
  force LAND. This already maintains a crude "always able to land" invariant.
- **LAND gate** + `critical_warning` for the no-safe-area case.

So the *passive/reactive* recoverability check exists. What is missing is the
**forward-looking** part: a real reachable-set test over the mission horizon, and
using the recoverability margin to influence decisions *before* the last safe
moment.

## 2. What "active recoverability" adds

1. **Recoverable set R, not a one-step energy sum.** Define
   `R = { states with an energy-reachable safe exit }` and require the state to
   stay in `R` after every committed action. "Safe exit" = land in place (if the
   local area is safe) or reach the nearest safe-landing candidate and land.
2. **Worst-case / uncertainty-aware energy.** Use a conservative (e.g. CVaR or
   upper-quantile) estimate of energy-to-exit rather than a fixed
   `E_air_reserve`, so margin survives perception/energy noise.
3. **Point-of-no-return detection + earlier action.** Detect the last step at
   which a return/land still keeps the state in `R`, and act there — including
   during ground cruise (force RETURN/conserve before committing further), not
   only at the takeoff instant.
4. **Recoverability margin fed into the decision** (the genuinely "active" part):
   add the margin (distance to leaving `R`) as a term in `J_ground/J_air` so the
   policy proactively avoids low-margin excursions. (This crosses into
   decision-logic territory — see risk note in §5.)

## 3. Recoverable-set approximation in this sim

The state that matters for irreversibility is low-dimensional:
`(mode, battery_j, robot_xy, set-of-safe-landing-zones, home_xy)`. A full
reachability computation is unnecessary; a tractable approximation:

- **Safe-exit candidates**: a small set of designated safe-landing positions
  `{L_k}` (plus "land in place" when `landing_area_safety >= eta_land`).
- **Energy-to-exit**: `E_exit(s) = min_k [ E_fly(dist(xy, L_k)) + E_land ]`, using
  the existing per-metre air-energy model (`air_energy_per_m`, already in
  `risk_cost_decision`).
- **Membership**: `s in R  <=>  battery_j >= E_exit_worstcase(s) + E_margin`.
- **Invariant check**: before committing an action, verify the *resulting* state
  is still in `R`; if not, override to the action that maximizes the margin
  (return / land / stay ground).

This is still energy arithmetic over a handful of candidates — cheap, deterministic,
unit-testable, and consistent with the existing joule-budget model.

## 4. Modules to touch (scope)

| Module | Change | Size |
|--------|--------|------|
| `safety_supervisor.py` | Add recoverable-set membership (multi-exit `E_exit`, worst-case energy, margin output, point-of-no-return). Keep the one-step gates as the floor. | **Moderate** |
| world / `scenario_sensor_sim.py` | Provide spatial **safe-landing zones** (positions + a landing-safety map), not just a scalar `landing_area_safety`. | **Small–moderate** |
| energy model | A distance→energy *predictor* to candidate exits (reuse `risk_cost_decision`'s `e_air_pred`). | Small (reuse) |
| `mode_switcher.py` | (a) pass full state (xy, home, zones) to the supervisor; (b) *optional active* — add recoverability margin to `J_ground/J_air`. | (a) small / (b) **algorithm-logic, careful** |
| `rescue_metrics_logger.py` + summarize | New metrics: `recoverability_margin` over time, `point_of_no_return` events, `entered_unrecoverable_set` (target 0 under active). | Small |

## 5. Can Scene D be the passive-vs-active experiment? — **No, as-is.**

Scene D starts with the battery *already* too low (25.5 J) — it is an inherent
dead-end from t=0. No *earlier* decision can preserve recoverability, so an active
supervisor cannot beat the passive one there (both end at success 0). Scene D
tests "block the unsafe action", which is exactly the reactive property already
demonstrated.

A meaningful passive-vs-active comparison needs a scenario where **foresight has
something to preserve**:

- **Proposed Scene E (recoverability-foresight):** *moderate* starting battery,
  a long out-and-return mission, and a risky mid-route region. A myopic/passive
  policy spends energy reaching/exploring, then the reactive gate blocks the final
  return-takeoff too late → stuck far from home, mission fails (and possibly
  irreversible if it over-commits). An active supervisor forces an earlier
  conserve/return while still in `R` → returns safely and ideally completes.
- Expected contrast: **active > passive on success and on "entered unrecoverable
  set"**, in a scenario where the battery is winnable-with-foresight but not
  blindly.

This is the honest finding: **the active contribution must be shown in a new
foresight scenario; Scene D cannot distinguish passive from active.**

## 6. Effort estimate + risk

- Recoverable-set supervisor (multi-exit, worst-case energy, point-of-no-return):
  **~1–2 days**.
- Spatial safe-landing zones + landing-safety map in sim: **~0.5–1 day**.
- New Scene E (world/config) + multi-seed batches: **~1 day code + a few hours
  Gazebo per batch**.
- Optional decision-cost shaping (active margin term): **~0.5 day code**, but it
  is decision-logic and risks conflating with the CVaR contribution — frame as a
  *separate* safety mechanism ("active recoverability filter") distinct from the
  risk-aware *objective*.
- **Total: ~3–5 days**, gated by a **go/no-go pilot**.

**Risk (learn from k_unc):** the active benefit must be *demonstrable* — active
completes/returns where passive fails — or it is not worth a contribution. Do a
**cheap pilot first**: implement only `E_exit` + point-of-no-return + the
`entered_unrecoverable_set` metric, build Scene E, run ~5 seeds passive vs active.
If active does not change outcomes, stop (as we did with `k_unc`); if it does,
commit to the full version + 20-seed batch.

## 7. Recommendation (for the user's decide-later call)

- The current supervisor is a *defensible reactive* safety filter (Claim B as
  written). It already enforces the recoverability invariant at the one-step
  level.
- "Active recoverability" is a **real, paper-worthy second-tier safety
  contribution** *iff* a foresight scenario demonstrates value — which Scene D
  cannot, so it requires **new Scene E + a pilot** before any commitment.
- Cheapest decision path: a ~1-day pilot (E_exit + Scene E + 5-seed passive-vs-
  active) → go/no-go, *before* investing in the full active supervisor or RotorS.
- If the pilot is negative, the two-leg paper (Claim A risk-aware + Claim B
  reactive supervisor) stands on its own and RotorS becomes the next priority.
