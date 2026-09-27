#!/usr/bin/env python3
"""Generate a deterministic SHA-256 listing for Phase F1 evidence."""

import hashlib
import pathlib


WS = pathlib.Path(__file__).resolve().parents[3]
ROOT = WS / "src" / "rescue_mission" / "results" / "phase_f1_fair_tuning_20260821"
OUTPUT = ROOT / "SHA256_MANIFEST.txt"
OUTPUT_CHECKSUM = ROOT / "SHA256_MANIFEST.sha256"
EXTERNAL = [
    *(WS / "src" / "rescue_mission" / "config" / "ablations").glob("f1_risk_wcvar_*.yaml"),
    WS / "src" / "rescue_mission" / "scripts" / "run_phase_f1_fair.py",
    WS / "src" / "rescue_mission" / "scripts" / "select_phase_f1_configs.py",
    WS / "src" / "rescue_mission" / "scripts" / "analyze_phase_f1.py",
    WS / "src" / "rescue_mission" / "scripts" / "generate_phase_f1_manifest.py",
    WS / "src" / "rescue_mission" / "scripts" / "mode_switcher.py",
    WS / "src" / "rescue_mission" / "config" / "mode_switch_params.yaml",
    WS / "src" / "rescue_mission" / "config" / "scenes" / "scene_b.yaml",
    WS / "src" / "rescue_mission" / "config" / "scenes" / "scene_c.yaml",
    WS / "src" / "rescue_mission" / "results" / "rescue_metrics.csv",
    WS / "src" / "rescue_mission" / "results" / "STAGE0_RESULT_MANIFEST_20260718.md",
    WS / "src" / "rescue_mission" / "config" / "rotors_supervisor_d2.yaml",
    WS / "src" / "rescue_worlds" / "results" / "uav_v4_rotors_energy_d2_20260719" / "calibration_samples.csv",
    WS / "src" / "rescue_worlds" / "results" / "uav_v4_rotors_energy_d2_20260719" / "calibration_analysis_d2.json",
    WS / "src" / "rescue_worlds" / "results" / "uav_v4_rotors_energy_d2_20260719" / "supervisor_case_results.json",
    WS / "src" / "docs" / "superpowers" / "specs" / "2026-08-22-phase-f1-fair-tuning-change-record.md",
]


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


paths = [path for path in ROOT.rglob("*") if path.is_file() and path not in {OUTPUT, OUTPUT_CHECKSUM}]
paths.extend(path for path in EXTERNAL if path.is_file())
paths = sorted(set(paths), key=lambda path: str(path.relative_to(WS)))
with OUTPUT.open("w") as stream:
    stream.write("# Phase F1 SHA-256 manifest; paths relative to $CATKIN_WS\n")
    for path in paths:
        stream.write("%s  %s\n" % (digest(path), path.relative_to(WS)))
print("wrote %d hashes to %s" % (len(paths), OUTPUT))
