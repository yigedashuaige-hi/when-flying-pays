#!/usr/bin/env python3
"""Recompute F1 held-out paired statistics from the public row-level CSV."""
from __future__ import annotations
import argparse, csv, json, random, statistics, sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
DATA = PROJECT / "data" / "f1"
MISSION_SCRIPTS = PROJECT / "ros_ws" / "src" / "rescue_mission" / "scripts"
sys.path.insert(0, str(MISSION_SCRIPTS))
from analyze_phase_f1 import (METRICS, METHODS, SCENES, bootstrap_mean_ci,
                              exact_mcnemar, wilcoxon_and_effect)  # noqa: E402

NUMERIC = {"value", "seed", "success", "timeout", "energy_j", "air_distance_m",
           "switch_count", "stuck_event_count", "slip_ratio", "final_distance", "sortie_count"}

def load_rows(path):
    rows = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            for key in NUMERIC:
                if key in row and row[key] != "": row[key] = float(row[key])
            row["seed"], row["success"], row["timeout"] = int(row["seed"]), int(row["success"]), int(row["timeout"])
            rows.append(row)
    return rows

def compute(records):
    rng = random.Random(20260821)
    output = {"difference_direction": "risk_aware minus threshold_switch", "scenes": {}}
    for scene in SCENES:
        scene_rows = [r for r in records if r["scene"] == scene]
        by_method = {m: {r["seed"]: r for r in scene_rows if r["family"] == m} for m in METHODS}
        for method in METHODS:
            if sorted(by_method[method]) != list(range(200, 220)):
                raise RuntimeError(f"incomplete held-out cell: {scene}/{method}")
        threshold, risk = by_method["threshold_switch"], by_method["risk_aware"]
        b = sum(threshold[s]["success"] == 1 and risk[s]["success"] == 0 for s in threshold)
        c = sum(threshold[s]["success"] == 0 and risk[s]["success"] == 1 for s in threshold)
        common = [s for s in threshold if threshold[s]["success"] and risk[s]["success"]]
        result = {
            "success": {m: sum(by_method[m][s]["success"] for s in by_method[m]) for m in METHODS},
            "mcnemar": {"threshold_only_success": b, "risk_only_success": c,
                        "discordant_total": b + c, "exact_two_sided_p": exact_mcnemar(b, c)},
            "both_success_n": len(common), "paired_both_success": {}, "fixed_reference_descriptive": {},
        }
        for metric in METRICS:
            differences = [risk[s][metric] - threshold[s][metric] for s in common]
            stats = {"n": len(differences), "mean_difference": statistics.mean(differences),
                     "median_difference": statistics.median(differences),
                     "bootstrap_95pct_ci_mean_difference": bootstrap_mean_ci(differences, rng)}
            stats.update(wilcoxon_and_effect(differences))
            result["paired_both_success"][metric] = stats
        for method in METHODS:
            rows = list(by_method[method].values())
            target = "fixed_reference_descriptive" if method == "fixed_reference" else "paired_both_success"
            result[target].setdefault(method, {"n": len(rows),
                "success_rate": sum(r["success"] for r in rows) / len(rows),
                "energy_all_seeds_mean_descriptive": statistics.mean(r["energy_j"] for r in rows)})
        output["scenes"][scene] = result
    return output

def close(a, b, tol=1e-10):
    if isinstance(a, dict): return a.keys() == b.keys() and all(close(a[k], b[k], tol) for k in a)
    if isinstance(a, list): return len(a) == len(b) and all(close(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, (int, float)) and isinstance(b, (int, float)): return abs(float(a) - float(b)) <= tol
    return a == b

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=PROJECT / "reproduced" / "heldout_paired_statistics.json")
    args = parser.parse_args()
    result = compute(load_rows(DATA / "heldout_run_summary.csv"))
    reference = json.loads((DATA / "heldout_paired_statistics.json").read_text())
    if not close(result, reference): raise RuntimeError("recomputed statistics differ from the archived reference")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(f"PASS: reproduced F1 statistics match {DATA / 'heldout_paired_statistics.json'}")
    return 0

if __name__ == "__main__": raise SystemExit(main())
