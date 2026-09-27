# Paper Pivot — Framework & Contributions (post-DOWNGRADE)

Supersedes the framing of `2026-06-10-paper-draft-method-experiments.md` (which
put CVaR "on the efficient frontier" as the headline — now refuted). The Method
**mechanics** there (3.2 shared perception, 3.4 supervisor, 3.5 energy) are still
correct and reused; only the *claim structure* changes.

Trigger: the pre-registered Claim A regime sweep (`2026-06-11-claim-a-regime-prereg.md`)
returned **DOWNGRADE** — `risk_aware`/CVaR is Pareto-dominated by a *tuned scalar
threshold* on `(1 - p_trav)` in both stuck models. We do not death-grip CVaR or
redesign scenes to rescue it. Instead the paper pivots to what the evidence
actually supports: a systematic regime study of when cross-domain mode switching
pays off, an honest negative on CVaR, and a reachability safety supervisor.

> **Pre-registration honesty — corner now resolved.** Batch (1), the expensive-
> flight + sparse-hazard corner (`scene_sparse`, `--fly-k1 3.0`, recoverable — the
> only place Claim A could have been NARROW), **ran and confirmed DOWNGRADE**:
> risk_aware 100%/258 J vs tuned threshold τ=0.6 100%/91 J (paired +167 J/seed,
> n=10), and flying turned out *unnecessary* there (energy_rule 100%/7.3 J). The
> NARROW fallback is closed. Batch (2) — the 20-seed Scene B + C recoverable
> robustness confirmation — is still running; it only tightens statistics, the
> direction is locked.

> **ARCHIVAL STATUS ADDENDUM (2026-07-18; original text above retained):**
> Batch (2) completed on 2026-06-12 local time. All ten Scene B/C
> recoverable-slip configurations contain the full seeds 0--19, and the batch
> log ends with `Claim A airtight batches DONE`. The final result strengthens
> DOWNGRADE: in Scene B, `fixed_switch` is 1.00/48.3 J and the success-matched
> threshold tau=0.2 is 1.00/67.2 J versus `risk_aware` 0.95/80.6 J; in Scene C,
> `fixed_switch` is 1.00/71.1 J and threshold tau=0.3 is 1.00/71.7 J versus
> `risk_aware` 0.95/90.3 J. Thus every later `running` label in this historical
> document is superseded by this dated addendum. Canonical sources and paired
> reports are listed in
> `rescue_mission/results/STAGE0_RESULT_MANIFEST_20260718.md`.

---

## New thesis

> For a terrestrial-aerial bimodal vehicle (TABV) under traversability
> uncertainty, **switching modes on perceived risk is what matters — not the
> sophistication of the risk functional, and not any single mode.** A simple
> perceived-risk threshold already captures the benefit; a CVaR/uncertainty
> formulation does not beat it. Crucially, *which* policy looks best is decided by
> a modeling assumption that prior TABV work leaves implicit — whether getting
> stuck is a permanent trap or a recoverable slip. We map this regime dependence
> and add a decision-layer reachability supervisor that provably prevents the one
> irreversible failure (airborne with no energy to land).

Working title: *"When Does Flying Pay Off? A Regime Study of Risk-Based Mode
Switching for Terrestrial-Aerial Rescue Robots, with a Reachability Safety
Supervisor."* (CVaR is **not** in the title.)

---

## Contributions (exactly four; CVaR is not a headline)

**(i) Perceived-risk-based mode switching is necessary and sufficient.**
*Necessary:* single-mode and energy-greedy policies fail — `ground_only` and
`energy_rule` get 0% success (stuck on the barrier), `air_preferred` fails to
deliver (flies into the risky zone on return, ~0.05–0.25 success). *Sufficient:*
any policy that switches on the shared perceived risk `p_trav` — whether a scalar
threshold or CVaR — converts that 0% into ~100% in the recoverable regime. The
load-bearing variable is "switch on perceived risk," full stop.

**(ii) A simple scalar threshold matches CVaR — an honest negative, with a
mechanistic explanation.** Sweeping the threshold `τ` in `fly iff (1−p_trav) > τ`
yields a policy that is on (or dominates) the success-vs-energy Pareto frontier
and **matches `risk_aware`'s success at strictly lower energy** in every realistic
regime tested. The mechanistic reason — not just an empirical loss — is that with
a one-dimensional perceived-risk signal and a non-informative uncertainty term
(`k_unc`, separately shown null, §ablation), **CVaR over a two-point Bernoulli
outcome reduces to a monotone threshold on `(1−p_trav)`**: `CVaR_α` of a
two-mass distribution is itself a monotone function of the tail mass, so the
decision boundary is a level set of `p_trav` — exactly what `τ` parameterizes.
`risk_aware`'s extra machinery only makes it *over-fly* (higher energy, no extra
success). We report this as a *tested, over-engineered* component: a useful
negative result for the TABV community, which has been adding risk functionals
without testing them against a tuned threshold.

**(iii) The stuck-model assumption is decisive — a methodological caution for
TABV research.** The ranking of policies *flips* with one modeling choice that
prior work leaves implicit:
  - **Permanent trap** (traction→0 forever once a risky cell fires): a single
    mistimed ground step is catastrophic, so blind periodic flying (`fixed_switch`)
    or always-flying beats perception-timed flying; selectivity is a *liability*.
  - **Recoverable slip** (low traction → slow, robot crawls out): selective
    risk-based switching reaches 100% success cheaply; periodic flying just wastes
    energy.
  We show this with the stuck model as an explicit, side-by-side experimental
  axis (not a silent swap). The takeaway: **any TABV mode-switching result is only
  interpretable relative to its stuck/recoverability model**, and papers that fix
  one implicitly may be reporting an artifact. This regime characterization is the
  paper's primary scientific contribution.

**(iv) A decision-layer reachability safety supervisor that provably prevents
irreversible failure.** Independent of the mode policy, an energy-reachability
invariant in joules (`battery_j ≥ E_takeoff + E_air + E_land + margin` to permit
TAKEOFF; force LAND while still affordable) mechanically eliminates the one
irreversible state — airborne with insufficient energy to land. In the nominal
low-battery Scene D it drives irreversible failures 4/20 → 0/20 at zero
mission-success cost (the scene is a dead-end; this is a **safe-abort /
catastrophic-failure-prevention** guarantee, explicitly *not* safe-completion).
Reported with its honest confusion matrix (recall 1.00, precision 0.20) and an
adversarial forced-flight stress test (20/20 → 0).

---

## Where CVaR lives in the paper

A subsection of Experiments titled approximately *"Does a richer risk functional
help? (No.)"* — present CVaR as a natural hypothesis, test it head-to-head against
the tuned threshold, report the loss + the reduction-to-threshold argument + the
null `k_unc`. This is a *strength*: pre-registered, mechanically-judged negative
results are credibility, not weakness. CVaR appears nowhere in the title, abstract
headline, or contribution (i)/(iii)/(iv).

## Evidence map (what backs each contribution today)

| Contribution | Evidence (status) |
|---|---|
| (i) necessary | ground_only/energy_rule 0%, air_preferred ~0.05–0.25 (frontier batch, **done**) |
| (i) sufficient | threshold & risk_aware ~100% recoverable (regime sweep n=5 **done**; n=20 B+C **running**) |
| (ii) threshold ≥ CVaR | regime sweep: threshold τ=0.2 100%/58.9J vs risk_aware 100%/94.5J (**done**, n=5); n=20 B+C + sparse corner (**running**) |
| (ii) mechanism | CVaR-of-Bernoulli = monotone threshold (analytic, **done**); k_unc null (Wilcoxon p=0.45, **done**) |
| (iii) stuck-model decisive | permanent vs recoverable side-by-side flip (regime sweep, **done**) |
| (iv) supervisor | Scene D 4/20→0/20, recall 1.0/prec 0.2, adversarial 20/20→0 (**done**) |

## NARROW fallback — CLOSED (corner ran, DOWNGRADE held)

The expensive-flight + sparse-hazard corner was run (2026-06-11) and did **not**
overturn DOWNGRADE — risk_aware lost to the tuned threshold by a *wider* margin
(+167 J/seed, success tied), and in that regime flying was unnecessary altogether
(energy_rule 100%/7.3 J). So contribution (ii) stays as the honest negative; no
NARROW carve-out. Bonus: this sharpens (iii) — whether *any* flying is needed is
itself regime-dependent (recoverable+sparse needs none), so risk-based switching's
*necessity* (contribution i) is also regime-scoped, not universal: it is necessary
in dense-hazard and/or permanent-trap regimes, not in sparse+recoverable.

## Related Work (still load-bearing — do not drop)

The contribution survives the pivot but its *shape* shifts. Novelty now rests on
the empty intersection of: (a) cross-domain/TABV energy-optimal planning that
treats traversability as deterministic/benign; (b) risk-aware off-road planning
(CVaR/probabilistic traversability) that is **single-mode ground** (no "fly over"
action); (c) safety-shielded RL that shields *control-layer* obstacle avoidance,
not *decision-layer* mode feasibility. Our added angles vs the old draft: we are
the first (to our knowledge) to (1) test these risk functionals against a tuned
*threshold* baseline in the TABV mode-switching setting and report the negative,
and (2) expose the stuck/recoverability model as the decisive, usually-implicit
variable. Citations required before submission.

## Out of scope / deferred (per user)

- No new algorithm legs. No scene redesign to make CVaR win.
- RotorS-level energy is follow-up engineering to harden credibility (energy is a
  proxy via `set_model_state`), **not** a novelty source.
- Active-recoverability supervisor leg: NO-GO under the permanent-trap artifact
  (`2026-06-10-active-recoverability-pilot-prereg.md` §9); only re-testable under
  recoverable-slip, deferred to the user.

## Revised section outline

1. Intro — TABV under traversability uncertainty; the "when does flying pay off"
   question; four contributions.
2. Related Work — the empty intersection (above).
3. Method — problem setup; shared perception (truth vs estimate, CRN); the policy
   family (single-mode, energy-greedy, threshold, CVaR) as a *spectrum of risk
   use*; the reachability supervisor; energy budget. (Reuse v0 §3.2/3.4/3.5.)
4. Experiments — (4.1) protocol; (4.2) necessity & sufficiency of risk-based
   switching; (4.3) **the regime study: stuck-model × flight-cost × hazard-density**
   (the centerpiece); (4.4) does a richer risk functional help? (CVaR vs tuned
   threshold; the negative + mechanism + k_unc null); (4.5) the safety supervisor
   (Claim B, safe-abort framing); (4.6) limitations.
5. Conclusion — switch on perceived risk; the functional's sophistication is
   second-order; the stuck model decides the ranking; a supervisor closes the
   irreversibility gap.
