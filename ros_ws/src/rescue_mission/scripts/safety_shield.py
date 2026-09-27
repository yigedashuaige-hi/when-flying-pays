#!/usr/bin/env python3
"""Safety filter for high-level land-air mode actions.

The shield is intentionally lightweight: it filters a proposed next mode from
the policy without owning ROS publishers/subscribers. This keeps it reusable by
the rule policy today and by an RL policy later.
"""


class SafetyShield:
    def __init__(self, params=None):
        params = params or {}
        self.enabled = bool(params.get("enabled", True))
        self.battery_min_air = float(params.get("battery_min_air", 0.35))
        self.landing_obstacle_max = float(params.get("landing_obstacle_max", 0.75))
        self.terrain_blocked_density = float(params.get("terrain_blocked_density", 0.90))
        self.recent_switch_window_sec = float(params.get("recent_switch_window_sec", 12.0))
        self.max_switches_in_window = int(params.get("max_switches_in_window", 4))

    def filter(self, raw_mode, state):
        if not self.enabled:
            return raw_mode

        current_mode = state.get("current_mode", "")
        battery = float(state.get("battery", 1.0))
        obstacle_density = float(state.get("obstacle_density", 0.0))
        mode_elapsed = float(state.get("mode_elapsed", 0.0))
        min_air_duration = float(state.get("min_air_duration", 0.0))
        switch_count_recent = int(state.get("switch_count_recent", 0))

        if raw_mode in ("TAKEOFF", "AIR") and battery < self.battery_min_air:
            return "GROUND" if current_mode in ("GROUND", "LAND") else "LAND"

        if raw_mode == "LAND" and obstacle_density > self.landing_obstacle_max:
            return "AIR" if current_mode == "AIR" else raw_mode

        if current_mode == "AIR" and raw_mode == "LAND" and mode_elapsed < min_air_duration:
            return "AIR"

        if (
            current_mode == "GROUND"
            and obstacle_density > self.terrain_blocked_density
            and battery >= self.battery_min_air
        ):
            return "TAKEOFF"

        if switch_count_recent > self.max_switches_in_window:
            return current_mode

        return raw_mode
