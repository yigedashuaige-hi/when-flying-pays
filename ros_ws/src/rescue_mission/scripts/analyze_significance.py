#!/usr/bin/env python3
"""Paired significance tests for a multi-seed batch (paper-hardening).

Reads an ``all_runs_*.csv`` (one row per run, with a ``seed`` column), pairs a
treatment strategy against a baseline by seed (Common Random Numbers), and runs:

- Wilcoxon signed-rank on continuous metrics (stuck_event_count, energy_j, ...),
  with the matched-pairs rank-biserial effect size and a bootstrap 95% CI of the
  median paired difference.
- McNemar (exact) on binary metrics (success, timeout, irreversible_failure).

Usage:
    rosrun rescue_mission analyze_significance.py all_runs_pr5_scene_b.csv \
        --scene scene_b --treatment risk_aware --baseline energy_rule
"""
import argparse
import csv
import os
import random
import statistics

try:
    from scipy import stats as scipy_stats
except Exception:  # pragma: no cover - scipy optional at import time
    scipy_stats = None

CONT_METRICS = ["stuck_event_count", "energy_j", "total_energy_j", "slip_ratio", "completion_time_s", "total_time_s"]
BIN_METRICS = ["success", "timeout", "irreversible_failure"]


def load_runs(path, scene):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            if scene and r.get("scene") != scene:
                continue
            rows.append(r)
    return rows


def to_float(v, default=float("nan")):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def by_seed(rows, strategy, metric):
    out = {}
    for r in rows:
        if r.get("strategy") != strategy:
            continue
        if metric not in r:
            continue
        out[r.get("seed")] = to_float(r.get(metric))
    return out


def paired(rows, treatment, baseline, metric):
    t = by_seed(rows, treatment, metric)
    b = by_seed(rows, baseline, metric)
    seeds = sorted(set(t) & set(b), key=lambda s: (len(str(s)), str(s)))
    pairs = [(t[s], b[s]) for s in seeds if t[s] == t[s] and b[s] == b[s]]
    return pairs


def rank_biserial(pairs):
    diffs = [a - b for a, b in pairs if a != b]
    if not diffs:
        return 0.0
    import math
    ranks = sorted(range(len(diffs)), key=lambda i: abs(diffs[i]))
    # average ranks
    abs_sorted = sorted(abs(d) for d in diffs)
    rank_of = {}
    i = 0
    while i < len(abs_sorted):
        j = i
        while j + 1 < len(abs_sorted) and abs_sorted[j + 1] == abs_sorted[i]:
            j += 1
        avg = (i + j) / 2.0 + 1.0
        rank_of[abs_sorted[i]] = avg
        i = j + 1
    pos = sum(rank_of[abs(d)] for d in diffs if d > 0)
    neg = sum(rank_of[abs(d)] for d in diffs if d < 0)
    total = pos + neg
    return (pos - neg) / total if total else 0.0


def bootstrap_median_ci(pairs, n=5000, alpha=0.05, seed=12345):
    diffs = [a - b for a, b in pairs]
    if not diffs:
        return (float("nan"), float("nan"))
    rng = random.Random(seed)
    meds = []
    k = len(diffs)
    for _ in range(n):
        sample = [diffs[rng.randrange(k)] for _ in range(k)]
        meds.append(statistics.median(sample))
    meds.sort()
    lo = meds[int((alpha / 2) * n)]
    hi = meds[int((1 - alpha / 2) * n) - 1]
    return (lo, hi)


def wilcoxon_report(pairs, metric):
    if len(pairs) < 2:
        return f"  {metric:<20} n={len(pairs)} (insufficient pairs)"
    t_vals = [a for a, _ in pairs]
    b_vals = [b for _, b in pairs]
    t_mean, b_mean = statistics.mean(t_vals), statistics.mean(b_vals)
    diffs = [a - b for a, b in pairs]
    if all(d == 0 for d in diffs):
        return (f"  {metric:<20} n={len(pairs)} identical (treat={t_mean:.3f} base={b_mean:.3f}) "
                f"p=1.000 (no difference)")
    if scipy_stats is None:
        return f"  {metric:<20} n={len(pairs)} treat={t_mean:.3f} base={b_mean:.3f} (scipy missing)"
    try:
        stat, p = scipy_stats.wilcoxon(t_vals, b_vals, zero_method="wilcox")
    except ValueError as exc:
        return f"  {metric:<20} n={len(pairs)} wilcoxon error: {exc}"
    rbc = rank_biserial(pairs)
    lo, hi = bootstrap_median_ci(pairs)
    # Per-seed consistency: how many seeds favour treatment (lower value) vs
    # baseline. Guards against a result driven by a few seeds pulling the mean.
    n_lower = sum(1 for d in diffs if d < 0)
    n_higher = sum(1 for d in diffs if d > 0)
    n_tie = sum(1 for d in diffs if d == 0)
    return (f"  {metric:<20} n={len(pairs)} treat={t_mean:.3f} base={b_mean:.3f} "
            f"median_diff={statistics.median(diffs):+.3f} 95%CI[{lo:+.3f},{hi:+.3f}] "
            f"rank_biserial={rbc:+.3f} p={p:.4g} "
            f"[treat_lower:{n_lower} higher:{n_higher} tie:{n_tie}]")


def mcnemar_report(pairs, metric):
    # pairs are (treatment_binary, baseline_binary)
    b = sum(1 for t, base in pairs if t == 1 and base == 0)  # treat success, base fail
    c = sum(1 for t, base in pairs if t == 0 and base == 1)  # treat fail, base success
    n = len(pairs)
    t_rate = statistics.mean([t for t, _ in pairs]) if pairs else float("nan")
    base_rate = statistics.mean([base for _, base in pairs]) if pairs else float("nan")
    if scipy_stats is None or (b + c) == 0:
        return (f"  {metric:<20} n={n} treat_rate={t_rate:.2f} base_rate={base_rate:.2f} "
                f"discordant(b={b},c={c}) p=1.000")
    # exact McNemar = two-sided binomial on discordant pairs
    p = scipy_stats.binom_test(min(b, c), b + c, 0.5)
    return (f"  {metric:<20} n={n} treat_rate={t_rate:.2f} base_rate={base_rate:.2f} "
            f"discordant(b={b},c={c}) p={p:.4g}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("all_runs_csv")
    ap.add_argument("--scene", default="")
    ap.add_argument("--treatment", default="risk_aware")
    ap.add_argument("--baseline", default="energy_rule")
    ap.add_argument("--output", default="")
    args = ap.parse_args()

    rows = load_runs(args.all_runs_csv, args.scene)
    strategies = sorted(set(r.get("strategy") for r in rows))
    present = set()
    for r in rows:
        present.update(k for k in r.keys())

    lines = []
    lines.append(f"Paired significance: {args.treatment} (treatment) vs {args.baseline} (baseline)"
                 + (f"  scene={args.scene}" if args.scene else ""))
    lines.append(f"strategies in file: {strategies}")
    lines.append("")
    lines.append("Wilcoxon signed-rank (continuous, paired by seed):")
    for m in CONT_METRICS:
        if m in present:
            pairs = paired(rows, args.treatment, args.baseline, m)
            if pairs:
                lines.append(wilcoxon_report(pairs, m))
    lines.append("")
    lines.append("McNemar (binary, paired by seed):")
    for m in BIN_METRICS:
        if m in present:
            pairs = paired(rows, args.treatment, args.baseline, m)
            if pairs:
                lines.append(mcnemar_report(pairs, m))

    text = "\n".join(lines) + "\n"
    print(text)
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, "w") as f:
            f.write(text)


if __name__ == "__main__":
    main()
