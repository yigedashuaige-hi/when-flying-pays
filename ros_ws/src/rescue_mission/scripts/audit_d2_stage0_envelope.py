#!/usr/bin/env python3
"""Read-only Phase E1a audit of Stage-0 flight demand versus the D2 envelope."""

import csv
import glob
import math
import statistics
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1] / "results"
OUT = (Path(__file__).resolve().parents[2] / "rescue_worlds" / "results" /
       "phase_e1a_contact_20260731" / "readonly_audit_d2")

CANONICAL = {
    "scene_b": ["fixed_switch", "risk_aware", "threshold_t0.2",
                "threshold_t0.3", "threshold_t0.4"],
    "scene_c": ["fixed_switch", "risk_aware", "threshold_t0.2",
                "threshold_t0.3", "threshold_t0.4"],
}
AIR_MODES = {"TAKEOFF", "AIR", "LAND"}


def quantile(values, q):
    values = sorted(values)
    if not values:
        return math.nan
    position = (len(values) - 1) * q
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return values[low]
    return values[low] * (high - position) + values[high] * (position - low)


def pearson(xs, ys):
    if len(xs) < 2:
        return math.nan
    mx, my = statistics.mean(xs), statistics.mean(ys)
    numerator = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    denominator = math.sqrt(sum((x - mx) ** 2 for x in xs) *
                            sum((y - my) ** 2 for y in ys))
    return numerator / denominator if denominator else math.nan


def extract(path, strategy_tag, evidence_class="stage0_canonical"):
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    # Canonical summarization uses the first terminal sample. Raw loggers can
    # continue writing DONE/timeout rows until roslaunch cleanup; those rows are
    # not additional task demand and would inflate energy/hover time.
    terminal = next((index for index, row in enumerate(rows)
                     if row["success"] == "1" or row["timeout"] == "1"),
                    len(rows) - 1)
    rows = rows[:terminal + 1]
    scene, seed = rows[0]["scene"], int(rows[0]["seed"])
    sorties = []
    current = None
    previous = None
    takeoffs = landings = 0
    for row in rows:
        mode = row["mode"]
        t = float(row["time"])
        point = (float(row["x"]), float(row["y"]), float(row["z"]))
        if mode == "TAKEOFF" and (previous is None or previous["mode"] != "TAKEOFF"):
            takeoffs += 1
        if mode == "LAND" and (previous is None or previous["mode"] != "LAND"):
            landings += 1
        if mode in AIR_MODES and current is None:
            current = {"start_t": t, "end_t": t, "start": point, "end": point,
                       "path_m": 0.0, "hover_s": 0.0, "max_z_m": point[2]}
        if current is not None and mode in AIR_MODES:
            current["end_t"] = t
            current["end"] = point
            current["max_z_m"] = max(current["max_z_m"], point[2])
            if previous is not None and previous["mode"] in AIR_MODES:
                dt = max(0.0, t - float(previous["time"]))
                dx = point[0] - float(previous["x"])
                dy = point[1] - float(previous["y"])
                distance = math.hypot(dx, dy)
                current["path_m"] += distance
                # Stage-0 has no flight_phase column. Count only low-horizontal-
                # speed intervals wholly inside AIR; TAKEOFF/LAND vertical time
                # is excluded so it is not mislabeled as D2 hover demand.
                if (dt > 0.0 and mode == "AIR" and
                        previous["mode"] == "AIR" and distance / dt <= 0.05):
                    current["hover_s"] += dt
        if current is not None and mode not in AIR_MODES:
            sorties.append(current)
            current = None
        previous = row
    if current is not None:
        sorties.append(current)

    sortie_rows = []
    for index, sortie in enumerate(sorties, 1):
        chord = math.hypot(sortie["end"][0] - sortie["start"][0],
                           sortie["end"][1] - sortie["start"][1])
        sortie_rows.append({
            "scene": scene, "strategy": strategy_tag, "seed": seed,
            "sortie_index": index, "one_way_chord_m": chord,
            "total_path_m": sortie["path_m"],
            "duration_s": sortie["end_t"] - sortie["start_t"],
            "hover_proxy_s": sortie["hover_s"], "max_height_m": sortie["max_z_m"],
            "exceeds_one_way_1p2": int(chord > 1.2),
            "exceeds_hover_5": int(sortie["hover_s"] > 5.0),
            "exceeds_height_1p5": int(sortie["max_z_m"] > 1.5),
        })
    final = rows[-1]
    try:
        source_csv = str(path.relative_to(ROOT))
    except ValueError:
        source_csv = str(path)
    run = {
        "scene": scene, "strategy": strategy_tag, "seed": seed,
        "evidence_class": evidence_class, "source_csv": source_csv,
        "total_air_distance_m": float(final["air_distance_m"]),
        "sortie_count": len(sorties), "takeoff_count": takeoffs,
        "landing_count": landings,
        "total_hover_proxy_s": sum(x["hover_proxy_s"] for x in sortie_rows),
        "legacy_energy_j": float(final["energy_j"]),
        "fixed_rotors_takeoff_landing_proxy_j": 8600.0 * len(sorties),
        "success": int(final["success"]),
    }
    return run, sortie_rows


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0])
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    run_rows, sortie_rows = [], []
    for scene, strategies in CANONICAL.items():
        for strategy in strategies:
            directory = ROOT / "runs_claim_a_airtight" / (
                f"{scene}_recoverable_{strategy}")
            files = sorted(directory.glob("*.csv"))
            if len(files) != 20:
                raise RuntimeError(f"expected 20 files in {directory}, got {len(files)}")
            aggregate_path = ROOT / f"all_runs_airtight_{scene}_recoverable_{strategy}.csv"
            with aggregate_path.open(newline="") as handle:
                aggregate = {int(row["seed"]): row for row in csv.DictReader(handle)}
            for path in files:
                run, sorties = extract(path, strategy)
                canonical = aggregate[run["seed"]]
                run["total_air_distance_m"] = float(canonical["air_distance_m"])
                run["legacy_energy_j"] = float(canonical["total_energy_j"])
                run["success"] = int(canonical["success"])
                run_rows.append(run)
                sortie_rows.extend(sorties)
    scene_a = (Path(__file__).resolve().parents[2] / "rescue_worlds" / "results" /
               "phase_e0_e1_rotors_20260720" / "scene_a" / "ideal" / "runs" /
               "fixed_switch__20260719_164430.csv")
    run, sorties = extract(scene_a, "fixed_switch_e0_diagnostic",
                           "frozen_e0_diagnostic_not_stage0_canonical")
    run_rows.append(run)
    sortie_rows.extend(sorties)
    write_csv(OUT / "stage0_run_flight_demand.csv", run_rows)
    write_csv(OUT / "stage0_sortie_flight_demand.csv", sortie_rows)

    groups = defaultdict(list)
    for row in sortie_rows:
        groups[(row["scene"], row["strategy"])].append(row)
    summary = []
    metrics = ["one_way_chord_m", "total_path_m", "duration_s",
               "hover_proxy_s", "max_height_m"]
    run_groups = defaultdict(list)
    for row in run_rows:
        run_groups[(row["scene"], row["strategy"])].append(row)
    for key in sorted(groups):
        items = groups[key]
        runs = run_groups[key]
        result = {"scene": key[0], "strategy": key[1],
                  "runs": len(runs), "sorties": len(items),
                  "sorties_over_one_way_1p2": sum(x["exceeds_one_way_1p2"] for x in items),
                  "sorties_over_hover_5": sum(x["exceeds_hover_5"] for x in items),
                  "sorties_over_height_1p5": sum(x["exceeds_height_1p5"] for x in items),
                  "run_sorties_median": quantile([x["sortie_count"] for x in runs], .5),
                  "run_sorties_p90": quantile([x["sortie_count"] for x in runs], .9),
                  "run_sorties_p95": quantile([x["sortie_count"] for x in runs], .95),
                  "run_sorties_max": max(x["sortie_count"] for x in runs),
                  "run_air_distance_median": quantile([x["total_air_distance_m"] for x in runs], .5),
                  "run_air_distance_p90": quantile([x["total_air_distance_m"] for x in runs], .9),
                  "run_air_distance_p95": quantile([x["total_air_distance_m"] for x in runs], .95),
                  "run_air_distance_max": max(x["total_air_distance_m"] for x in runs)}
        for metric in metrics:
            values = [x[metric] for x in items]
            for label, q in (("median", .5), ("p90", .9), ("p95", .95)):
                result[f"{metric}_{label}"] = quantile(values, q)
            result[f"{metric}_max"] = max(values)
        summary.append(result)
    write_csv(OUT / "stage0_group_envelope_summary.csv", summary)

    correlations = []
    for key in sorted(run_groups):
        items = run_groups[key]
        energy = [x["legacy_energy_j"] for x in items]
        for metric in ("total_air_distance_m", "sortie_count", "total_hover_proxy_s"):
            correlations.append({"scene": key[0], "strategy": key[1],
                                 "metric": metric,
                                 "pearson_r": pearson(energy, [x[metric] for x in items]),
                                 "n": len(items)})
    write_csv(OUT / "legacy_energy_correlations.csv", correlations)


if __name__ == "__main__":
    main()
