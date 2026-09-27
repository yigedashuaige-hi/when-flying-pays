#!/usr/bin/env python3
"""Diagnose whether the reported traversability_uncertainty (u_trav) is
informative — i.e. whether it actually predicts the perception error
|p_trav - theta_true| — and whether k_unc saturates p_fail_eff.

This is a *diagnostic*: it explains why the w_o_uncertainty ablation is no worse
than full, without tuning anything. Reads the per-row run CSVs (which log both
the noisy estimate p_trav = traversability_mean and the ground truth
theta_true = traversability_true).

Usage:
    rosrun rescue_mission analyze_uncertainty.py <runs_dir_or_glob> [--k-unc 1.0]
"""
import argparse
import csv
import glob
import math
import os
import statistics


def fget(r, k, d=float("nan")):
    try:
        return float(r[k])
    except (KeyError, TypeError, ValueError):
        return d


def pearson(xs, ys):
    pts = [(x, y) for x, y in zip(xs, ys) if x == x and y == y]
    if len(pts) < 3:
        return float("nan")
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    mx, my = statistics.mean(xs), statistics.mean(ys)
    sx = statistics.pstdev(xs)
    sy = statistics.pstdev(ys)
    if sx == 0 or sy == 0:
        return float("nan")
    cov = sum((x - mx) * (y - my) for x, y in pts) / len(pts)
    return cov / (sx * sy)


def collect(paths):
    rows = []
    for p in paths:
        with open(p) as f:
            for r in csv.DictReader(f):
                rows.append(r)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", help="dir or glob of run CSVs")
    ap.add_argument("--k-unc", type=float, default=1.0)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    if os.path.isdir(args.runs):
        paths = sorted(glob.glob(os.path.join(args.runs, "*.csv")))
    else:
        paths = sorted(glob.glob(args.runs))
    rows = collect(paths)
    if not rows:
        print("no rows found")
        return

    p_trav = [fget(r, "traversability_mean") for r in rows]
    theta = [fget(r, "traversability_true") for r in rows]
    u_trav = [fget(r, "traversability_uncertainty") for r in rows]
    obst = [fget(r, "obstacle_density") for r in rows]
    cvar = [fget(r, "risk_cvar") for r in rows]
    err = [abs(a - b) if (a == a and b == b) else float("nan") for a, b in zip(p_trav, theta)]
    # p_fail_eff = clip((1 - p_trav) + k_unc * u_trav, 0, 1)  (mode_switcher logic)
    pfe = [max(0.0, min(1.0, (1 - pt) + args.k_unc * ut))
           if (pt == pt and ut == ut) else float("nan")
           for pt, ut in zip(p_trav, u_trav)]
    # p_fail without inflation (energy_rule's view)
    pf0 = [max(0.0, min(1.0, 1 - pt)) if pt == pt else float("nan") for pt in p_trav]

    def desc(name, xs):
        xs2 = [x for x in xs if x == x]
        if not xs2:
            return f"  {name:<14} (no data)"
        return (f"  {name:<14} mean={statistics.mean(xs2):.4f} std={statistics.pstdev(xs2):.4f} "
                f"min={min(xs2):.4f} max={max(xs2):.4f}")

    print("=" * 70)
    print("UNCERTAINTY DIAGNOSIS" + (f"  [{args.label}]" if args.label else ""))
    print(f"rows={len(rows)} from {len(paths)} run(s); k_unc={args.k_unc}")
    print("-" * 70)
    print(desc("p_trav", p_trav))
    print(desc("theta_true", theta))
    print(desc("u_trav", u_trav))
    print(desc("|p-theta| err", err))
    print(desc("p_fail (no inf)", pf0))
    print(desc("p_fail_eff", pfe))
    print(desc("risk_cvar", cvar))
    print("-" * 70)
    print("Is u_trav informative about the ACTUAL perception error?")
    print(f"  corr(u_trav, |p_trav-theta_true|) = {pearson(u_trav, err):+.3f}")
    print(f"  corr(u_trav, obstacle_density)    = {pearson(u_trav, obst):+.3f}")
    print(f"  corr(|err|, obstacle_density)     = {pearson(err, obst):+.3f}")
    print("  (u_trav is informative only if corr(u_trav,|err|) is clearly > 0)")
    print("-" * 70)
    # Saturation: how often does k_unc push p_fail_eff to the ceiling?
    valid_pfe = [x for x in pfe if x == x]
    sat = sum(1 for x in valid_pfe if x >= 0.99) / len(valid_pfe) if valid_pfe else float("nan")
    # How often does the inflation actually change p_fail meaningfully?
    delta = [a - b for a, b in zip(pfe, pf0) if a == a and b == b]
    print(f"  p_fail_eff saturated (>=0.99): {sat*100:.1f}% of rows")
    print(f"  mean inflation (p_fail_eff - p_fail) = {statistics.mean(delta):.4f}" if delta else "")
    print("=" * 70)


if __name__ == "__main__":
    main()
