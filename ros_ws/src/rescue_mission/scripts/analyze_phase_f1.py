#!/usr/bin/env python3
"""Generate the preregistered Phase F1 held-out paired statistics and plots."""

import csv
import hashlib
import json
import math
import pathlib
import random
import statistics
import sys


PKG = pathlib.Path(__file__).resolve().parents[1]
ROOT = PKG / "results" / "phase_f1_fair_tuning_20260821"
SCENES = ("scene_b", "scene_c")
METHODS = ("threshold_switch", "risk_aware", "fixed_reference")
METRICS = ("energy_j", "air_distance_m", "sortie_count", "stuck_event_count", "slip_ratio")


def exact_mcnemar(b, c):
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / (2 ** n)
    return min(1.0, 2.0 * tail)


def average_ranks(values):
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    start = 0
    while start < len(order):
        end = start + 1
        while end < len(order) and values[order[end]] == values[order[start]]:
            end += 1
        rank = ((start + 1) + end) / 2.0
        for pos in range(start, end):
            ranks[order[pos]] = rank
        start = end
    return ranks


def wilcoxon_and_effect(differences):
    nonzero = [d for d in differences if d != 0]
    if not nonzero:
        return {"n_nonzero": 0, "statistic": 0.0, "p_two_sided": 1.0,
                "matched_rank_biserial": 0.0}
    ranks = average_ranks([abs(d) for d in nonzero])
    w_plus = sum(rank for rank, diff in zip(ranks, nonzero) if diff > 0)
    w_minus = sum(rank for rank, diff in zip(ranks, nonzero) if diff < 0)
    total = w_plus + w_minus
    try:
        from scipy.stats import wilcoxon
        try:
            result = wilcoxon(nonzero, alternative="two-sided", zero_method="wilcox", method="auto")
        except TypeError:
            # SciPy 1.5 (the frozen ROS container) names this argument `mode`.
            result = wilcoxon(nonzero, alternative="two-sided", zero_method="wilcox", mode="auto")
        statistic, p_value = float(result.statistic), float(result.pvalue)
    except Exception:
        statistic, p_value = min(w_plus, w_minus), None
    return {"n_nonzero": len(nonzero), "statistic": statistic, "p_two_sided": p_value,
            "matched_rank_biserial": (w_plus - w_minus) / total}


def bootstrap_mean_ci(differences, rng, repetitions=10000):
    if not differences:
        return [None, None]
    n = len(differences)
    samples = []
    for _ in range(repetitions):
        samples.append(sum(differences[rng.randrange(n)] for _ in range(n)) / n)
    samples.sort()
    return [samples[round(0.025 * (repetitions - 1))],
            samples[round(0.975 * (repetitions - 1))]]


def sortie_count(csv_path):
    with open(csv_path, newline="") as stream:
        modes = [row["mode"] for row in csv.DictReader(stream)]
    count = 0
    previous = None
    for mode in modes:
        if mode == "TAKEOFF" and previous != "TAKEOFF":
            count += 1
        previous = mode
    return count


def load_evaluation():
    records = []
    for path in sorted((ROOT / "metadata" / "evaluation").glob("*.json")):
        record = json.loads(path.read_text())
        if record.get("status") != "valid":
            raise RuntimeError("non-valid evaluation record: %s" % path)
        final = record["validation"]["final"]
        item = {key: record[key] for key in ("scene", "family", "candidate", "value", "seed")}
        item["success"] = int(float(final["success"]))
        item["timeout"] = int(float(final["timeout"]))
        for field in ("energy_j", "air_distance_m", "switch_count", "stuck_event_count", "slip_ratio", "final_distance"):
            item[field] = float(final[field])
        item["sortie_count"] = sortie_count(record["csv"])
        item["csv"] = record["csv"]
        item["csv_sha256"] = record["csv_sha256"]
        records.append(item)
    if len(records) != 120:
        raise RuntimeError("expected 120 evaluation records, found %d" % len(records))
    return records


def main():
    records = load_evaluation()
    raw_path = ROOT / "heldout_run_summary.csv"
    with raw_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)

    rng = random.Random(20260821)
    output = {"difference_direction": "risk_aware minus threshold_switch", "scenes": {}}
    for scene in SCENES:
        scene_records = [r for r in records if r["scene"] == scene]
        by_method = {method: {r["seed"]: r for r in scene_records if r["family"] == method}
                     for method in METHODS}
        for method in METHODS:
            if sorted(by_method[method]) != list(range(200, 220)):
                raise RuntimeError("incomplete held-out cell %s/%s" % (scene, method))
        threshold = by_method["threshold_switch"]
        risk = by_method["risk_aware"]
        b = sum(threshold[s]["success"] == 1 and risk[s]["success"] == 0 for s in threshold)
        c = sum(threshold[s]["success"] == 0 and risk[s]["success"] == 1 for s in threshold)
        both_success = [s for s in threshold if threshold[s]["success"] and risk[s]["success"]]
        result = {
            "success": {
                method: sum(by_method[method][s]["success"] for s in by_method[method])
                for method in METHODS
            },
            "mcnemar": {"threshold_only_success": b, "risk_only_success": c,
                         "discordant_total": b + c, "exact_two_sided_p": exact_mcnemar(b, c)},
            "both_success_n": len(both_success),
            "paired_both_success": {},
            "fixed_reference_descriptive": {},
        }
        for metric in METRICS:
            differences = [risk[s][metric] - threshold[s][metric] for s in both_success]
            stats = {
                "n": len(differences),
                "mean_difference": statistics.mean(differences) if differences else None,
                "median_difference": statistics.median(differences) if differences else None,
                "bootstrap_95pct_ci_mean_difference": bootstrap_mean_ci(differences, rng),
            }
            stats.update(wilcoxon_and_effect(differences))
            result["paired_both_success"][metric] = stats
        for method in METHODS:
            rows = list(by_method[method].values())
            result["fixed_reference_descriptive" if method == "fixed_reference" else "paired_both_success"].setdefault(
                method, {"n": len(rows), "success_rate": sum(r["success"] for r in rows) / len(rows),
                         "energy_all_seeds_mean_descriptive": statistics.mean(r["energy_j"] for r in rows)})
        output["scenes"][scene] = result

    analysis_path = ROOT / "heldout_paired_statistics.json"
    analysis_path.write_text(json.dumps(output, indent=2, sort_keys=True, allow_nan=False) + "\n")

    figures = ROOT / "plots"
    figures.mkdir(exist_ok=True)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        for scene in SCENES:
            subset = [r for r in records if r["scene"] == scene]
            by_method = {m: {r["seed"]: r for r in subset if r["family"] == m} for m in METHODS}
            fig, axes = plt.subplots(1, 2, figsize=(10, 4))
            axes[0].bar(METHODS, [sum(r["success"] for r in by_method[m].values()) for m in METHODS])
            axes[0].set_ylim(0, 20)
            axes[0].set_ylabel("successful missions / 20")
            axes[0].tick_params(axis="x", rotation=20)
            common = [s for s in range(200, 220) if by_method["threshold_switch"][s]["success"] and by_method["risk_aware"][s]["success"]]
            for seed in common:
                axes[1].plot([0, 1], [by_method["threshold_switch"][seed]["energy_j"], by_method["risk_aware"][seed]["energy_j"]], color="0.65", alpha=0.7)
            axes[1].set_xticks([0, 1], ["threshold", "risk-aware"])
            axes[1].set_ylabel("energy_j (both-success pairs)")
            fig.suptitle("Phase F1 %s held-out" % scene)
            fig.tight_layout()
            fig.savefig(figures / (scene + "_heldout.png"), dpi=180)
            plt.close(fig)
    except Exception as exc:
        (figures / "PLOT_ERROR.txt").write_text(str(exc) + "\n")

    print(json.dumps(output, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
