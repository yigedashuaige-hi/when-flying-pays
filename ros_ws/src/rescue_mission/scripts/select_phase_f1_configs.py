#!/usr/bin/env python3
"""Apply the preregistered Phase F1 lexicographic selection rule."""

import csv
import datetime as dt
import hashlib
import json
import math
import pathlib
import statistics
import sys


PKG = pathlib.Path(__file__).resolve().parents[1]
ROOT = PKG / "results" / "phase_f1_fair_tuning_20260821"
SCENES = ("scene_b", "scene_c")
FAMILIES = ("threshold_switch", "risk_aware")
DEFAULTS = {"threshold_switch": 0.4, "risk_aware": 0.75}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_records():
    evaluation = ROOT / "metadata" / "evaluation"
    if evaluation.exists() and any(evaluation.glob("*.json")):
        raise RuntimeError("evaluation metadata already exists; blinding boundary violated")
    records = []
    for path in sorted((ROOT / "metadata" / "tuning").glob("*.json")):
        record = json.loads(path.read_text())
        if record.get("status") != "valid":
            raise RuntimeError("non-valid tuning record: %s" % path)
        final = record["validation"]["final"]
        record["success"] = int(float(final["success"])) == 1
        for field in ("energy_j", "air_distance_m", "switch_count"):
            record[field] = float(final[field])
        records.append(record)
    if len(records) != 200:
        raise RuntimeError("expected 200 valid tuning records, found %d" % len(records))
    return records


def candidate_key(summary, family):
    return (
        -summary["success_count"],
        summary["successful_energy_median"],
        summary["successful_air_distance_median"],
        summary["successful_switch_count_median"],
        abs(summary["value"] - DEFAULTS[family]),
        summary["value"],
    )


def main():
    records = load_records()
    summaries = []
    selected = {}
    for scene in SCENES:
        selected[scene] = {}
        for family in FAMILIES:
            subset = [r for r in records if r["scene"] == scene and r["family"] == family]
            candidates = sorted({r["candidate"] for r in subset})
            if len(candidates) != 5:
                raise RuntimeError("%s/%s expected 5 candidates" % (scene, family))
            family_summaries = []
            for candidate in candidates:
                runs = [r for r in subset if r["candidate"] == candidate]
                if len(runs) != 10 or sorted(r["seed"] for r in runs) != list(range(100, 110)):
                    raise RuntimeError("incomplete paired tuning cell %s/%s/%s" % (scene, family, candidate))
                successful = [r for r in runs if r["success"]]
                med = lambda field: statistics.median(r[field] for r in successful) if successful else math.inf
                summary = {
                    "scene": scene,
                    "family": family,
                    "candidate": candidate,
                    "value": float(runs[0]["value"]),
                    "n": len(runs),
                    "success_count": len(successful),
                    "success_rate": len(successful) / len(runs),
                    "successful_energy_median": med("energy_j"),
                    "successful_air_distance_median": med("air_distance_m"),
                    "successful_switch_count_median": med("switch_count"),
                }
                summaries.append(summary)
                family_summaries.append(summary)
            winner = min(family_summaries, key=lambda item: candidate_key(item, family))
            selected[scene][family] = winner

    summary_path = ROOT / "tuning_candidate_summary.csv"
    with summary_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)

    prereg = ROOT / "PHASE_F1_PREREGISTRATION_20260821.md"
    candidate_space = ROOT / "configs" / "candidate_space.json"
    output = {
        "status": "frozen_before_evaluation",
        "frozen_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "selection_rule": "preregistered lexicographic rule",
        "preregistration_sha256": sha256(prereg),
        "candidate_space_sha256": sha256(candidate_space),
        "tuning_summary_sha256": sha256(summary_path),
        "selected": selected,
    }
    selected_path = ROOT / "SELECTED_CONFIGS_BEFORE_EVALUATION.json"
    selected_path.write_text(json.dumps(output, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps(output, indent=2, sort_keys=True, allow_nan=False))
    print("selected_config_sha256=%s" % sha256(selected_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
