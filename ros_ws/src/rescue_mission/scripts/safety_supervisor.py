#!/usr/bin/env python3
"""Decision-layer reachability safety supervisor (spec B3).

This is the *decision-layer* irreversibility guard, distinct from the
control-layer ``safety_shield.py`` (which rate-limits switching / avoids
obstacles). The supervisor answers a single question: given a proposed mode and
the current energy budget, would executing it risk entering the irreversible
state "airborne with too little energy to land safely"?

Design notes:
- Pure logic (no ROS): params + state in, approved mode out. ``mode_switcher``
  owns the ROS wiring. This keeps the rules unit-testable.
- Monitor-vs-enforce: the supervisor always *monitors* (counts unsafe proposals,
  tracks ``safe_landing_margin_j``, latches ``irreversible_failure``) but only
  *enforces* overrides when ``enforce=True`` (i.e. strategy ``risk_aware_safety``).
  This lets plain ``risk_aware`` actually experience the irreversible failure in
  Scene D while ``risk_aware_safety`` prevents it, with one shared code path.
- Counters use rising-edge detection so a sustained unsafe proposal is counted
  as a single event rather than once per control tick.
"""

GROUND = "GROUND"
AIR = "AIR"
TAKEOFF = "TAKEOFF"
LAND = "LAND"


class SafetySupervisor:
    def __init__(self, params=None, enforce=False, active=False,
                 energy_backend="ideal"):
        params = params or {}
        self.enforce = bool(enforce)
        self.energy_backend = str(energy_backend)
        self.energy_model = str(params.get(
            "energy_model",
            ("rotors_aero_shaft_mechanical_proxy_v1"
             if self.energy_backend == "rotors"
             else "legacy_task_distance_hover_proxy_v1"),
        ))
        # Active recoverability (pilot): forward-looking. When True, in addition to
        # the reactive gates the supervisor proactively flies out of a risky ground
        # region *while the fly-out option is still energy-reachable*, so the robot
        # never gets stuck with too little battery to recover. active implies
        # enforce. See 2026-06-10-active-recoverability-pilot-prereg.md.
        self.active = bool(active)
        if self.active:
            self.enforce = True

        # Energy thresholds (joules), all compared against /rescue/battery_j.
        self.e_takeoff_j = float(params.get("E_takeoff_j", 11.0))
        self.e_air_reserve_j = float(params.get("E_air_reserve_j", 12.0))
        self.e_land_j = float(params.get("E_land_j", 12.0))
        self.e_margin_j = float(params.get("E_margin_j", 8.0))
        # Landing-area safety gate. First version uses visibility_confidence as a
        # proxy; the param name is kept generic for a future landing-safety map.
        self.eta_land = float(params.get("eta_land", 0.30))
        # Active: a ground cell counts as "risky" (worth preserving fly-out for)
        # when its perceived stuck risk is at or above this threshold.
        self.active_risk_threshold = float(params.get("active_risk_threshold", 0.55))

        # Cumulative per-run counters.
        self.shield_override_count = 0
        self.unsafe_action_count = 0
        self.irreversible_failure = 0  # latched 0/1
        # Recoverability monitoring (all conditions, not just active).
        self.entered_unrecoverable_set = 0  # latched 0/1
        self.recoverability_margin = 0.0
        self.recoverability_margin_min = float("inf")

        # Latest-tick snapshot (published every tick).
        self.override_flag = False
        self.override_reason = "none"
        self.safe_landing_margin_j = 0.0
        self.predicted_consumption_j = 0.0
        self.energy_reserve_j = 0.0
        self.block_reason = "none"

        # Rising-edge bookkeeping.
        self._prev_unsafe = False
        self._prev_override = False

    def takeoff_energy_required(self):
        return self.e_takeoff_j + self.e_air_reserve_j + self.e_land_j + self.e_margin_j

    def filter(self, proposed, state):
        """Return the approved mode for ``proposed`` given ``state``.

        state keys: current_mode, energy_remaining_j (or legacy battery_j),
        landing_area_safety.
        """
        current_mode = state.get("current_mode", GROUND)
        battery_j = float(state.get(
            "energy_remaining_j", state.get("battery_j", 0.0)
        ))
        landing_area_safety = float(state.get("landing_area_safety", 1.0))
        stuck_risk_prior = float(state.get("stuck_risk_prior", 0.0))

        self.safe_landing_margin_j = battery_j - (self.e_land_j + self.e_margin_j)

        approved = proposed
        reason = "ok"
        unsafe_now = False
        override_now = False

        if proposed == TAKEOFF and current_mode == GROUND:
            self.predicted_consumption_j = self.takeoff_energy_required()
        elif proposed == LAND:
            self.predicted_consumption_j = self.e_land_j
        elif current_mode in (AIR, TAKEOFF, LAND):
            self.predicted_consumption_j = self.e_land_j + self.e_margin_j
        else:
            self.predicted_consumption_j = 0.0
        self.energy_reserve_j = battery_j - self.predicted_consumption_j

        # ── recoverability monitoring (all conditions) ────────────────────────
        # Required energy to reach a safe exit from the current state:
        #   airborne -> land in place (E_land); on risky ground -> fly out
        #   (takeoff+cross+land); on safe ground -> none.
        airborne = current_mode in (AIR, TAKEOFF, LAND)
        on_risky_ground = (current_mode == GROUND) and (stuck_risk_prior >= self.active_risk_threshold)
        e_fly_out = self.takeoff_energy_required()
        if airborne:
            e_exit = self.e_land_j
        elif on_risky_ground:
            e_exit = e_fly_out
        else:
            e_exit = 0.0
        self.recoverability_margin = battery_j - e_exit
        self.recoverability_margin_min = min(self.recoverability_margin_min, self.recoverability_margin)
        # Unrecoverable: no energy-reachable safe exit.
        if (current_mode == AIR and battery_j < self.e_land_j) or (on_risky_ground and battery_j < e_fly_out):
            self.entered_unrecoverable_set = 1

        # ── irreversible detection (monitored regardless of enforce) ──────────
        # Airborne with not even enough energy to land => unrecoverable.
        if current_mode == AIR and battery_j < self.e_land_j:
            self.irreversible_failure = 1

        # ── active recoverability (forward-looking, only when active) ─────────
        # On risky ground while the fly-out option is still affordable, fly out
        # NOW to preserve recoverability, rather than risk getting stuck and
        # stranded. Selective: only over risky ground, only when battery >= the
        # fly-out floor (otherwise flying cannot help). This is what distinguishes
        # it from air_preferred (which flies everywhere).
        if self.active and on_risky_ground and battery_j >= e_fly_out and proposed != TAKEOFF:
            approved = TAKEOFF
            override_now = True
            reason = "active_preserve_recoverability"

        # ── TAKEOFF reachability gate ─────────────────────────────────────────
        if proposed == TAKEOFF and current_mode == GROUND:
            if battery_j < self.takeoff_energy_required():
                unsafe_now = True
                reason = "takeoff_unsafe_low_battery"
                if self.enforce:
                    approved = GROUND
                    override_now = True

        # ── AIR continuation gate ─────────────────────────────────────────────
        elif current_mode == AIR and proposed in (AIR, TAKEOFF):
            if battery_j < self.e_land_j + self.e_margin_j:
                unsafe_now = True
                if landing_area_safety < self.eta_land:
                    # Out of energy AND no safe landing area: genuine danger
                    # state. First version forces LAND anyway but flags it so the
                    # conflict is explicit in the data.
                    reason = "critical_warning_unsafe_forced_land"
                else:
                    reason = "air_unsafe_force_land"
                if self.enforce:
                    approved = LAND
                    override_now = True

        # ── LAND gate ─────────────────────────────────────────────────────────
        elif proposed == LAND:
            # Defer landing only if the area is unsafe *and* we still have the
            # energy to keep looking; otherwise landing is the safe action.
            if landing_area_safety < self.eta_land and battery_j >= self.e_land_j + self.e_margin_j:
                unsafe_now = True
                reason = "land_area_unsafe_defer"
                if self.enforce:
                    approved = AIR
                    override_now = True

        # ── rising-edge counting ──────────────────────────────────────────────
        if unsafe_now and not self._prev_unsafe:
            self.unsafe_action_count += 1
        if override_now and not self._prev_override:
            self.shield_override_count += 1
        self._prev_unsafe = unsafe_now
        self._prev_override = override_now

        self.override_flag = override_now
        self.override_reason = reason
        self.block_reason = reason if unsafe_now else "none"
        return approved
