import math
import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from safe_site_memory import SafeSiteMemory  # noqa: E402


def evidence(x, y=0.0, stamp=1.0, epoch=0, **overrides):
    item = {
        "x": x,
        "y": y,
        "yaw": 0.1,
        "observation_stamp": stamp,
        "last_validated_stamp": stamp,
        "measurement_valid": True,
        "site_valid": True,
        "stable_valid": True,
        "available_m": -1.0,
        "required_m": 1.15,
        "nearest_m": 0.4,
        "recheck_distance_m": 0.43,
        "source": "stable_ground",
        "ground_epoch": epoch,
    }
    item.update(overrides)
    return item


def test_site_persists_across_ground_epoch_but_never_authorizes_takeoff():
    memory = SafeSiteMemory(capacity=8, dedup_distance_m=0.10)
    site_id = memory.upsert(evidence(1.0, stamp=4.0, epoch=0))

    candidate = memory.select(
        current_xy=(2.0, 0.0), episode_id=3, ground_epoch=1,
        min_separation_m=0.30,
    )

    assert candidate["site_id"] == site_id
    assert candidate["ground_epoch"] == 0
    assert candidate["reachability"] == "unverified_cross_sortie"
    assert candidate["authorizes_takeoff"] is False
    assert candidate["last_validated_stamp"] == 4.0


def test_invalid_or_nonfinite_evidence_is_not_registered():
    memory = SafeSiteMemory(capacity=8, dedup_distance_m=0.10)

    assert memory.upsert(evidence(0.0, site_valid=False)) is None
    assert memory.upsert(evidence(0.0, measurement_valid=False)) is None
    assert memory.upsert(evidence(0.0, stable_valid=False)) is None
    assert memory.upsert(evidence(math.nan)) is None
    assert len(memory) == 0


def test_spatial_deduplication_does_not_drag_the_recorded_pose():
    memory = SafeSiteMemory(capacity=8, dedup_distance_m=0.10)
    site_id = memory.upsert(evidence(1.0, stamp=1.0))
    repeated_id = memory.upsert(evidence(1.05, stamp=2.0))

    assert repeated_id == site_id
    assert len(memory) == 1
    site = memory.snapshot()[0]
    assert site["x"] == 1.0
    assert site["last_validated_stamp"] == 2.0
    assert site["validation_count"] == 2
    assert site["source"] == "stable_ground"


def test_approved_takeoff_keeps_its_exact_measured_pose():
    memory = SafeSiteMemory(capacity=8, dedup_distance_m=0.10)
    memory.upsert(evidence(1.0, stamp=1.0))
    approved_id = memory.upsert(
        evidence(1.05, stamp=2.0, source="approved_takeoff"))

    approved = [site for site in memory.snapshot()
                if site["site_id"] == approved_id][0]
    assert len(memory) == 2
    assert approved["x"] == 1.05
    assert approved["source"] == "approved_takeoff"


def test_unreachable_site_is_excluded_only_for_the_current_episode():
    memory = SafeSiteMemory(capacity=8, dedup_distance_m=0.10)
    near = memory.upsert(evidence(1.0, stamp=1.0))
    far = memory.upsert(evidence(2.0, stamp=2.0))

    first = memory.select((0.0, 0.0), episode_id=9, ground_epoch=1,
                          min_separation_m=0.30)
    assert first["site_id"] == near
    memory.mark_unreachable(near, episode_id=9)

    second = memory.select((0.0, 0.0), episode_id=9, ground_epoch=1,
                           min_separation_m=0.30)
    assert second["site_id"] == far

    next_episode = memory.select((0.0, 0.0), episode_id=10, ground_epoch=1,
                                 min_separation_m=0.30)
    assert next_episode["site_id"] == near


def test_capacity_is_bounded_and_selection_is_distance_then_recency():
    memory = SafeSiteMemory(capacity=3, dedup_distance_m=0.05)
    memory.upsert(evidence(1.0, stamp=1.0))
    memory.upsert(evidence(2.0, stamp=2.0))
    memory.upsert(evidence(3.0, stamp=3.0))
    memory.upsert(evidence(4.0, stamp=4.0))

    assert [site["x"] for site in memory.snapshot()] == [2.0, 3.0, 4.0]
    selected = memory.select((3.6, 0.0), episode_id=1, ground_epoch=0,
                             min_separation_m=0.30)
    assert selected["x"] == 4.0
    assert selected["reachability"] == "observed_ground_path"
