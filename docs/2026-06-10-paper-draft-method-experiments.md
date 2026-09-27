# Paper Draft — Method & Experiments (v0)

> ⚠️ **SUPERSEDED claim structure (2026-06-11).** §4.3's "risk_aware on the
> efficient frontier" and §4.4's "CVaR is the essential mechanism" are REFUTED by
> the regime sweep (DOWNGRADE: a tuned scalar threshold dominates CVaR). The new
> claim structure lives in `2026-06-11-paper-pivot-framework.md`. The **Method
> mechanics** here (§3.2 shared perception, §3.4 supervisor, §3.5 energy) and the
> Related-Work note remain valid and are reused; do not cite §4.3/§4.4 framing.

Draft of the Method and Experiments sections from the finalized Claim A/B
materials. Intro / Related Work are deferred. Final framing (two-leg vs three-leg)
waits on the active-recoverability pilot. Numbers come from
`2026-06-10-claim-a-paper-materials.md`.

> **DO NOT DROP — Related Work is the load-bearing wall for Claim A's novelty.**
> The contribution (risk-aware CVaR cross-domain mode switching) is only defensible
> against: (a) cross-domain/terrestrial-aerial energy-optimal planning that ignores
> traversability *uncertainty* (treats it as deterministic/benign); (b) risk-aware
> off-road planning (CVaR / probabilistic traversability) that is *single-mode
> ground* (stuck = failure, no "fly over" action); (c) safety-shielded RL that
> shields *control-layer* obstacle avoidance, not *decision-layer* mode
> feasibility. The novelty is the empty intersection: using a cross-domain action
> to dissolve ground-traversability tail risk, plus a decision-layer reachability
> supervisor for irreversibility. Related Work must establish this gap explicitly,
> with citations, before submission. Deferred, not optional.

---

## 3. Method

### 3.1 Problem setup

A single robot must complete a navigation/delivery mission over terrain whose
traversability is uncertain. At each decision step the robot chooses a locomotion
mode — GROUND or AIR (via TAKEOFF/LAND transitions) — to trade off energy, time,
and the risk of getting irrecoverably stuck on the ground. Flying is treated not
as "faster" but as an active mechanism to **avoid the tail risk of getting stuck**,
at an energy cost. A secondary safety concern is the **irreversible** state of
being airborne with too little energy to land safely.

### 3.2 Shared perception (truth vs estimate)

All policies subscribe to the identical perception stream, so they differ only in
decision logic (fairness is enforced at the code level: one perception node,
identical parameters). The environment maintains a ground-truth per-cell
traversability `theta_true(cell)` (used only by the physics and the logger);
policies observe a *noisy estimate* `p_trav = theta_true + noise` plus an
uncertainty signal `u_trav`. Getting stuck is a Bernoulli event governed by
`theta_true` (not by the policy's estimate), so a policy cannot make the ground
safer by mis-perceiving it. Randomness is keyed by `(seed, cell)`, giving Common
Random Numbers: under one seed every policy faces the identical terrain field and
differs only in its path/mode choices.

### 3.3 Baselines and the proposed policy

- **ground_only / air_preferred / fixed_switch**: single-mode and naive
  multimodal baselines.
- **energy_rule** (fair baseline): uses the shared `p_trav` plus energy/time and a
  *mean* stuck-risk term; does **not** use `u_trav` and does not compute CVaR.
- **risk_aware** (proposed): same shared inputs, but models ground traversal as a
  two-point (Bernoulli) outcome and scores the ground option by the **Conditional
  Value-at-Risk (CVaR)** of that outcome, capturing the stuck tail. It optionally
  inflates the failure probability by `k_unc * u_trav` (epistemic inflation).

For each option the ground cost is, schematically,
`J_ground = w_E E_ground + w_cvar * CVaR_alpha(stuck outcome) + ...`, with the air
cost `J_air = w_E (E_takeoff + E_air + E_land) + ...`; the robot picks the
lower-cost mode. The CVaR term is what makes risk_aware fly to avoid the stuck
tail when the mean would still prefer ground.

### 3.4 Decision-layer safety supervisor (reactive recoverability)

Independent of low-level control, a decision-layer supervisor enforces an
energy-reachability invariant in joules against a battery budget:

- **TAKEOFF** allowed iff `battery_j >= E_takeoff + E_air_reserve + E_land + E_margin`.
- **AIR** continues iff `battery_j >= E_land + E_margin`, else force LAND.
- **LAND** gated by a landing-area-safety threshold; the out-of-energy-and-no-safe-
  area case is flagged.

The supervisor *monitors* always and *enforces* overrides for the guarded policy
`risk_aware_safety`. Its purpose is to mechanically prevent the irreversible
"airborne, cannot land" state — a safe abort, not improved task completion.

### 3.5 Energy budget

Battery is a task-level joule budget owned by the metrics logger:
`battery_j = initial_battery * capacity - energy_consumed`, where energy is
integrated across all modes. (Air dynamics are prescribed via `set_model_state`;
energy is therefore a proxy — see Limitations.)

---

## 4. Experiments

### 4.1 Simulation, scenes, protocol

ROS 1 Noetic + Gazebo Classic 11. Scenes:

- **Scene B / C**: a risky barrier region (C harsher) where ground traversal can
  get stuck; the cross-domain trade-off (Claim A).
- **Scene D**: a low-battery scenario for the safety supervisor (Claim B).

Protocol: 20 seeds per (scene, strategy), one run per seed, paired by seed (CRN).
Continuous metrics use Wilcoxon signed-rank (effect size = rank-biserial, 95% CI by
bootstrap of the median paired difference); binary metrics use exact McNemar. A
run that ends without reaching the goal is recorded as `timeout = 1` (robust to
Gazebo's variable real-time factor). Tuning and final runs are kept separate; new
results never overwrite originals.

### 4.2 Metrics

`success`, `timeout`, `stuck_event_count`, `slip_ratio`, `total_energy_j`,
transition energies, `switch_count`; for the supervisor `shield_override_count`,
`irreversible_failure`, `safe_landing_margin_j`. Ground-truth `theta_true` is
logged for analysis only (never a policy input).

### 4.3 Claim A — risk-aware cross-domain decision

With shared perception, `risk_aware` reduces stuck on **every one of 20 seeds**
in both scenes (Scene B median paired diff −89, 95% CI [−142, −59], p=1.9e-6;
Scene C −95.5 [−142, −63.5], p=1.9e-6) and converts a 0% baseline success into
0.45 (B) / 0.50 (C) (McNemar p=0.0039 / 0.0020), at a higher, honestly reported
energy cost. A success-vs-energy **Pareto frontier** over all baselines
(ground_only, air_preferred, fixed_switch, energy_rule, risk_aware) places
risk_aware on the efficient frontier (Table 1 / Fig. frontier), so the result is
not an artifact of one weak baseline. [frontier numbers pending baseline batch]

### 4.4 Ablation

`w/o CVaR` collapses risk_aware to baseline behaviour (~145 stuck, 0% success):
the CVaR objective is the essential mechanism. `w/o switch penalty` is a minor
effect. The uncertainty-inflation term `k_unc` shows **no significant benefit**
even under a calibrated heteroscedastic perception model where `u_trav` predicts
the perception error (corr 0.88), Wilcoxon p=0.45, small effect — it is reported
as an auxiliary component, not a contribution (the constant symmetric inflation
helps and hurts in equal measure; a directional rule is future work).

### 4.5 Claim B — safety supervisor

In the nominal low-battery Scene D the supervisor drives `irreversible_failure`
from 4/20 to 0/20 (recall 1.00 over would-crash takeoffs, precision 0.20),
**at zero mission-success cost** because the scene is a low-battery dead-end
(both policies' success rate is 0). It is a *safe-abort* / catastrophic-failure-
prevention guarantee, not improved completion. An adversarial stress test (forced
flight) shows the mechanism under certain danger (20/20 → 0, p=1.9e-6) and is
labelled as such.

### 4.6 Limitations

Idealized air dynamics (`set_model_state`, no RotorS) make energy a proxy;
supervisor conservatism (precision 0.20) is safe but could cost successes in a
winnable scene; `k_unc` is not validated as beneficial; safe-abort is not safe-
completion; the terrain/perception model is a proxy with code-level (not merely
distributional) fairness. See the dedicated limitations list in the materials doc.
