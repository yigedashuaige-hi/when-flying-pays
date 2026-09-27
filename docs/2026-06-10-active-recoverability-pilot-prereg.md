# Active Recoverability — Pilot Pre-Registration (go/no-go)

**Written before running. The go/no-go is judged mechanically against the
criteria in §3; no post-hoc reinterpretation. If the criteria are not met, the
leg is dropped (as `k_unc` was).**

Scope: a ~1-day pilot only. No full implementation, no RotorS, no 20-seed batch,
no decision-cost shaping. Passive-vs-active, a few battery levels, 5 seeds each.

## 0. Prerequisite (answered)

**Q: can a stuck ground robot take off and fly out (ground→air recovery)?
A: YES.** `air_motion_executor.py` drives TAKEOFF/AIR via `set_model_state` from
the mode decision alone; ground stuck only zeroes `/mecanum/cmd_vel`. There is no
code path blocking takeoff when stuck. So cross-domain recovery exists (air
idealized — see §6). The pilot proceeds.

## 1. Hypothesis

A *passive* (current one-step) supervisor blocks the unsafe takeoff only at the
instant it is proposed — which can be too late: by the time the robot is stuck in
a late-mission risky region, its battery may already be below the fly-out
threshold, so it is stuck with no safe exit (entered the unrecoverable set). An
*active* supervisor maintains, while still mobile, the invariant
`battery_j >= E_fly_out(upcoming risky region) + E_land + margin`, forcing an
earlier fly-over / conserve so the fly-out option is preserved → fewer
unrecoverable states and higher success.

## 2. Conditions

- **passive** = `risk_aware_safety` (existing reactive supervisor).
- **active**  = `risk_aware_active` (new): same decision + reactive gates PLUS the
  forward-looking recoverability invariant (reserve fly-out energy for the
  upcoming risky region; act before the point of no return).
- **air_preferred** = fly-heavy reference (distinctness control, §3b).
- Identical seeds (CRN) per battery level; paired comparison. 3 conditions x 4
  battery levels x 5 seeds = 60 runs.

## 3. Pre-registered go/no-go criteria (judged after running)

"Active is clearly effective" requires **all three**:

1. **Unrecoverable states removed**: mean `entered_unrecoverable_set` for active
   <= 0.10, AND active < passive on it at >= 3 of 4 battery levels.
2. **Success improvement**: mean `success_rate` (averaged over battery levels) for
   active >= passive + **0.20** (20 percentage points absolute).
3. **Not a knife-edge**: active `success_rate` >= passive at >= 3 of the 4 battery
   levels (benefit is broad, not a single configuration).

If any of the three fails → **NO-GO**: report the null, keep the two-leg paper,
do not implement the full active supervisor. (Also NO-GO if active merely shifts
failures from irreversible to stuck with no success gain — that is just the
existing safe-abort, not foresight value.)

GO → report the minimal change list + risks; the user decides whether to do the
full version (after RotorS).

## 3b. Distinctness guard (against "active = disguised air_preferred")

The three criteria above can be satisfied trivially by a policy that just **flies
everything** — that is not foresight, it is `air_preferred`. To rule this out:

- **Add `air_preferred` as a third Scene E condition**, run identically (4 battery
  levels x 5 seeds, same CRN seeds 0-4) alongside passive and active.
- **Diagnostic metrics (reported, NOT part of the go/no-go):** `total_energy_j`,
  `air_distance`, `flight_fraction` (= air_distance / path_length).
- **Distinctness criterion (a gate, judged after):** active counts as a *distinct*
  contribution only if it is selectively-flying, i.e. **both**:
  - **success parity with air_preferred**: active mean `success_rate`
    >= air_preferred mean `success_rate` − 0.10 (active is roughly as capable), AND
  - **clearly less flying / energy than air_preferred**: active mean
    `flight_fraction` <= 0.70 x air_preferred mean `flight_fraction`
    **OR** active mean `total_energy_j` <= 0.75 x air_preferred mean
    `total_energy_j` (active flies only when needed to preserve the fly-out
    option).
- If active is ~indistinguishable from air_preferred on success **and** on
  energy/flight (it flies about as much), then **NO-GO: not a distinct
  contribution** — report honestly and stop, even if criteria 1-3 pass.

The defensible active story is: **"success close to air_preferred, but
significantly lower energy/flight — it flies only to keep the fly-out option."**
Anything else is not a separate contribution.

## 4. Scene E design (cross-domain coupling, not generic out-and-return)

The risk must be a **late-mission ground hazard reachable only after spending
ground energy**, so foresight (reserving fly-out battery early) is what matters —
not "keep some battery to go home".

- Long mission: waypoints out to a far goal through a **late** barrier region
  (placed near the far end), then return.
- **Medium battery**: enough to finish *if* the robot flies the late barrier once
  while it still can; a myopic policy that does not reserve fly-out energy reaches
  the barrier, gets stuck, and lacks battery to fly out → unrecoverable / fail.
- The barrier is far enough that ground energy spent reaching it materially
  lowers battery before the fly-out decision (the coupling).

Scene E is **frozen before the runs** and is **not tuned to make active win**
(requirement #4). Terrain uses the same seeded Bernoulli stuck as B/C.

## 5. Battery levels (avoid single-point wins — requirement #4)

Run passive vs active at **4 initial battery levels** spanning "comfortably
enough" to "clearly too little", e.g. capacity fixed, `initial_battery in
{0.55, 0.65, 0.75, 0.85}`, **5 seeds each** (seeds 0-4). 4 levels x 2 conditions
x 5 seeds = 40 runs. The benefit must appear across levels, not at one.

## 6. Honesty caveat (requirement #5)

Recoverability here is computed on the **proxy energy model** (`set_model_state`
air, no RotorS). The fly-out energy `E_exit` uses the same per-metre air-energy
proxy as the policy. A trustworthy active-recoverability result needs RotorS-level
energy; the pilot only establishes whether the *mechanism* changes outcomes in
sim. This caveat is stated regardless of go/no-go.

## 7. Metrics reported (requirement #6 — only these)

Go/no-go metrics: `success_rate`, `irreversible_failure`,
`entered_unrecoverable_set`, `safe_abort_rate`, `final_distance`,
`recoverability_margin_min`. Per battery level, for air_preferred / passive /
active, mean over 5 seeds. No other metric is used to argue go/no-go.

Distinctness diagnostics (reported, NOT in go/no-go — see §3b):
`total_energy_j`, `air_distance`, `flight_fraction`.

## 8. Minimal implementation needed for the pilot

- `safety_supervisor.py`: an `active` mode adding the forward `E_exit` invariant
  and point-of-no-return action (keep reactive gates as the floor).
- `mode_switcher.py`: register `risk_aware_active`; pass the extra state
  (stuck-risk / upcoming-hazard proxy) to the supervisor.
- `rescue_metrics_logger.py`: log `entered_unrecoverable_set`,
  `recoverability_margin_min`, `safe_abort_rate` (additive columns).
- `config/scenes/scene_e.yaml` + world wiring; `scene_e` in the launches.

If the pilot is NO-GO, these stay as an experimental branch and are not promoted.

## 9. RESULT (2026-06-11) — NO-GO

Ran 3 conditions x 4 battery levels x 5 seeds on the frozen Scene E
(`runs_pilot_scene_e`). Outcome judged mechanically against §3:

| metric (mean over levels) | air_preferred | passive (risk_aware_safety) | active (risk_aware_active) |
|---------------------------|---------------|------------------------------|----------------------------|
| success_rate | 0.00 | 0.00 | 0.00 |
| irreversible_failure | 0.00 | 0.00 | 0.00 |
| entered_unrecoverable_set | ~0.85 | ~0.15 | ~0.10 |
| flight_fraction | 0.86 | ~0.45 | ~0.48 |

- **CRIT2 fails**: active success (0.00) is not >= passive (0.00) + 0.20. **NO-GO.**
  Per the pre-registration we report the null and do not promote the active leg
  (the k_unc precedent).
- The active supervisor *does* reduce `entered_unrecoverable_set` (0.10 vs 0.15)
  and is selective vs air_preferred (flies less; distinctness on flight holds),
  but with zero mission success there is no foresight value to claim.

**Confound (honest):** all risk_aware_(safety/active) runs end in NAVIGATE with
mean max-x ~7.2 (goal x=13) — they are permanently trapped in the EARLY slope
zone (~x5-7) and never reach the late barrier the pilot was built to test. This
is the **same permanent-trap stuck-model artifact** under investigation in the
Claim A regime sweep (2026-06-11). So the NO-GO is partly a stuck-model artifact,
not a clean refutation of active recoverability.

**Disposition:** active-recoverability leg is NOT validated -> dropped for now
(two-leg paper). The only *principled* way to give it a fair test would be a
re-run under the **recoverable-slip** stuck model (an existing explicit axis, not
a tune-to-win), and only if the Claim A regime sweep establishes recoverable-slip
as the realistic model. Deferred to the user's decision after the regime sweep;
NOT retuning Scene E geometry to rescue it.
