#!/usr/bin/env python3
"""ROS-independent persistent safe-site registry for RotorS recovery.

Stored clearance evidence may select a ground navigation target.  It never
authorizes takeoff; the coordinator must revalidate the current footprint with
its fresh atomic clearance measurement after arrival.
"""

import math


_NUMERIC_FIELDS = (
    "x", "y", "yaw", "observation_stamp", "last_validated_stamp",
    "available_m", "required_m", "nearest_m", "recheck_distance_m",
)


class SafeSiteMemory:
    """Bounded, deterministic registry of measured stable-valid launch sites."""

    def __init__(self, capacity=32, dedup_distance_m=0.15):
        if int(capacity) < 1:
            raise ValueError("capacity must be positive")
        if not math.isfinite(float(dedup_distance_m)) or dedup_distance_m < 0.0:
            raise ValueError("dedup_distance_m must be finite and nonnegative")
        self.capacity = int(capacity)
        self.dedup_distance_m = float(dedup_distance_m)
        self._sites = []
        self._next_id = 1
        self._unreachable = set()
        self._reached_epoch = {}

    def __len__(self):
        return len(self._sites)

    @staticmethod
    def _eligible(item):
        if not (item.get("measurement_valid") and item.get("site_valid")
                and item.get("stable_valid")):
            return False
        try:
            return all(math.isfinite(float(item[field])) for field in _NUMERIC_FIELDS)
        except (KeyError, TypeError, ValueError):
            return False

    def upsert(self, measurement):
        """Register eligible evidence and return its generated site id."""
        if not self._eligible(measurement):
            return None
        x = float(measurement["x"])
        y = float(measurement["y"])
        approved_takeoff = measurement.get("source") == "approved_takeoff"
        matched = None
        for site in self._sites:
            distance = math.hypot(x - site["x"], y - site["y"])
            # Preserve the exact pose associated with an approved takeoff.
            # Ordinary stable-ground samples remain spatially deduplicated.
            match_limit = 1e-6 if approved_takeoff else self.dedup_distance_m
            if distance <= match_limit:
                matched = site
                break
        if matched is not None:
            matched["last_validated_stamp"] = max(
                matched["last_validated_stamp"],
                float(measurement["last_validated_stamp"]),
            )
            matched["measurement_valid"] = True
            matched["site_valid"] = True
            matched["stable_valid"] = True
            matched["available_m"] = float(measurement["available_m"])
            matched["required_m"] = float(measurement["required_m"])
            matched["nearest_m"] = float(measurement["nearest_m"])
            matched["recheck_distance_m"] = float(measurement["recheck_distance_m"])
            matched["ground_epoch"] = int(measurement["ground_epoch"])
            matched["validation_count"] += 1
            if measurement.get("source") == "approved_takeoff":
                matched["source"] = "approved_takeoff"
            return matched["site_id"]

        site = {
            "site_id": self._next_id,
            "x": x,
            "y": y,
            "yaw": float(measurement["yaw"]),
            "observation_stamp": float(measurement["observation_stamp"]),
            "last_validated_stamp": float(measurement["last_validated_stamp"]),
            "measurement_valid": True,
            "site_valid": True,
            "stable_valid": True,
            "available_m": float(measurement["available_m"]),
            "required_m": float(measurement["required_m"]),
            "nearest_m": float(measurement["nearest_m"]),
            "recheck_distance_m": float(measurement["recheck_distance_m"]),
            "source": str(measurement.get("source", "stable_ground")),
            "ground_epoch": int(measurement["ground_epoch"]),
            "validation_count": 1,
        }
        self._next_id += 1
        self._sites.append(site)
        if len(self._sites) > self.capacity:
            removed = self._sites.pop(0)
            removed_id = removed["site_id"]
            self._unreachable = {
                pair for pair in self._unreachable if pair[0] != removed_id
            }
            self._reached_epoch.pop(removed_id, None)
        return site["site_id"]

    def mark_unreachable(self, site_id, episode_id):
        self._unreachable.add((int(site_id), int(episode_id)))

    def mark_reached(self, site_id, ground_epoch):
        self._reached_epoch[int(site_id)] = int(ground_epoch)

    def _view(self, site, ground_epoch=None):
        view = dict(site)
        view["authorizes_takeoff"] = False
        if ground_epoch is None:
            view["reachability"] = "stored_unverified"
        elif self._reached_epoch.get(site["site_id"]) == int(ground_epoch):
            view["reachability"] = "verified_current_epoch"
        elif site["ground_epoch"] == int(ground_epoch):
            view["reachability"] = "observed_ground_path"
        else:
            view["reachability"] = "unverified_cross_sortie"
        return view

    def select(self, current_xy, episode_id, ground_epoch, min_separation_m):
        """Return the nearest untried navigation candidate, never an approval."""
        x, y = (float(current_xy[0]), float(current_xy[1]))
        minimum = float(min_separation_m)
        candidates = []
        for site in self._sites:
            if (site["site_id"], int(episode_id)) in self._unreachable:
                continue
            distance = math.hypot(x - site["x"], y - site["y"])
            if distance < minimum:
                continue
            candidates.append((distance, -site["last_validated_stamp"],
                               site["site_id"], site))
        if not candidates:
            return None
        _, _, _, selected = min(candidates)
        return self._view(selected, ground_epoch=ground_epoch)

    def snapshot(self, ground_epoch=None):
        return [self._view(site, ground_epoch=ground_epoch)
                for site in self._sites]
