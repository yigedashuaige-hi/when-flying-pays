#!/usr/bin/env python3
import math
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from safety_shield import SafetyShield
from safety_supervisor import SafetySupervisor


def fixed_switch_wants_air(elapsed, first_switch_time, switch_interval):
    """Return the fixed baseline's square-wave request.

    ``first_switch_time`` moves only the initial GROUND->AIR boundary.  Every
    later boundary remains exactly ``switch_interval`` seconds apart.  A
    negative first-switch value preserves the historical schedule whose first
    boundary equals the interval (10, 20, 30, ... seconds by default).
    """
    interval = max(float(switch_interval), 1e-6)
    first = interval if float(first_switch_time) < 0.0 else float(first_switch_time)
    if float(elapsed) < first:
        return False
    return (int((float(elapsed) - first) / interval) % 2) == 0


class ModeSwitcher:
    GROUND = "GROUND"
    AIR = "AIR"
    TAKEOFF = "TAKEOFF"
    LAND = "LAND"

    def __init__(self):
        self.mode = rospy.get_param("~initial_mode", self.GROUND)
        self.strategy = rospy.get_param("~strategy", "rule")
        self.rate_hz = rospy.get_param("~update_rate", 10.0)
        self.thresholds = rospy.get_param("~thresholds", {})
        self.risk_params = rospy.get_param("~risk_aware", {})
        self.air_backend = rospy.get_param("~air_backend", "ideal")
        self.handoff_managed = bool(rospy.get_param(
            "~handoff_managed", self.air_backend == "rotors"
        ))
        # threshold_switch baseline tau override (-1 => use thresholds default).
        self.switch_tau = float(rospy.get_param("~switch_tau", -1.0))
        # fixed_switch timing-sensitivity hook. -1 exactly preserves the
        # historical first boundary at fixed_switch_interval.
        self.fixed_first_switch_time = float(
            rospy.get_param("~fixed_first_switch_time", -1.0))
        self.takeoff_duration = rospy.get_param("~takeoff_duration", 2.0)
        self.land_duration = rospy.get_param("~land_duration", 2.0)
        self.min_air_duration = rospy.get_param("~min_air_duration", 4.0)
        self.flight_feedback_enabled = rospy.get_param("~flight_feedback_enabled", False)
        self.cruise_altitude = float(rospy.get_param("~cruise_altitude", 1.2))
        self.takeoff_altitude_tolerance = float(
            rospy.get_param("~takeoff_altitude_tolerance", 0.15)
        )
        self.takeoff_vertical_speed_tolerance = float(
            rospy.get_param("~takeoff_vertical_speed_tolerance", 0.20)
        )
        self.safety_shield = SafetyShield(rospy.get_param("~safety_shield", {}))
        # Decision-layer reachability supervisor. Always monitors; only enforces
        # overrides for the risk_aware_safety strategy (monitor-vs-enforce).
        supervisor_param_name = (
            "~rotors_safety_supervisor"
            if self.air_backend == "rotors" else "~safety_supervisor"
        )
        self.supervisor = SafetySupervisor(
            rospy.get_param(supervisor_param_name, {}),
            enforce=(self.strategy in ("risk_aware_safety", "risk_aware_active")),
            active=(self.strategy == "risk_aware_active"),
            energy_backend=self.air_backend,
        )
        self.switch_times = []

        self.node_start_time = rospy.Time.now()
        self.mode_enter_time = rospy.Time.now()

        self.odom = None
        self.goal = None
        self.slope_deg = 0.0
        self.obstacle_density = 0.0
        self.traversability_mean = 1.0
        self.traversability_uncertainty = 0.0
        self.terrain_cost = 0.0
        self.visibility_confidence = 1.0
        self.stuck_risk_prior = 0.0
        self.battery = 1.0
        # Energy budget in joules (published by the logger). Initialise from the
        # logger's own params so the supervisor has the correct budget from t=0,
        # before the first /rescue/battery_j message arrives — otherwise an
        # immediate takeoff would bypass the reachability gate.
        _cap = rospy.get_param("/rescue_metrics_logger/battery_capacity_j", 2000.0)
        _init = rospy.get_param("/rescue_metrics_logger/initial_battery", 1.0)
        self.battery_j = float(_init) * float(_cap)
        self.rotors_mechanical_remaining_j = float(rospy.get_param(
            "/rescue_metrics_logger/rotors_mechanical_budget_j", 0.0
        ))
        self.mission_state = ""
        self.rotors_disarmed = True
        self.decision_debug = {
            "J_ground": 0.0,
            "J_air": 0.0,
            "risk_cvar": 0.0,
            "energy_term": 0.0,
            "switch_reason": "init",
        }

        mode_topic = "/rescue/requested_mode" if self.handoff_managed else "/rescue/mode"
        self.mode_pub = rospy.Publisher(mode_topic, String, queue_size=10, latch=True)
        self.j_ground_pub = rospy.Publisher("/rescue/J_ground", Float32, queue_size=10)
        self.j_air_pub = rospy.Publisher("/rescue/J_air", Float32, queue_size=10)
        self.risk_cvar_pub = rospy.Publisher("/rescue/risk_cvar", Float32, queue_size=10)
        self.energy_term_pub = rospy.Publisher("/rescue/energy_term", Float32, queue_size=10)
        self.switch_reason_pub = rospy.Publisher("/rescue/switch_reason", String, queue_size=10)
        self.shield_override_pub = rospy.Publisher("/rescue/shield_override_count", Float32, queue_size=10)
        self.unsafe_action_pub = rospy.Publisher("/rescue/unsafe_action_count", Float32, queue_size=10)
        self.irreversible_pub = rospy.Publisher("/rescue/irreversible_failure", Float32, queue_size=10)
        self.safe_margin_pub = rospy.Publisher("/rescue/safe_landing_margin_j", Float32, queue_size=10)
        self.override_reason_pub = rospy.Publisher("/rescue/override_reason", String, queue_size=10, latch=True)
        self.unrecoverable_pub = rospy.Publisher("/rescue/entered_unrecoverable_set", Float32, queue_size=10)
        self.recov_margin_min_pub = rospy.Publisher("/rescue/recoverability_margin_min", Float32, queue_size=10)
        self.supervisor_backend_pub = rospy.Publisher(
            "/rescue/supervisor_energy_backend", String, queue_size=10, latch=True
        )
        self.supervisor_model_pub = rospy.Publisher(
            "/rescue/supervisor_energy_model", String, queue_size=10, latch=True
        )
        self.supervisor_predicted_pub = rospy.Publisher(
            "/rescue/supervisor_predicted_consumption_j", Float32, queue_size=10
        )
        self.supervisor_reserve_pub = rospy.Publisher(
            "/rescue/supervisor_reserve_j", Float32, queue_size=10
        )
        self.supervisor_block_reason_pub = rospy.Publisher(
            "/rescue/supervisor_block_reason", String, queue_size=10, latch=True
        )
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/rescue/goal", PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber("/rescue/terrain_slope_deg", Float32, self.slope_cb, queue_size=1)
        rospy.Subscriber("/rescue/obstacle_density", Float32, self.obstacle_cb, queue_size=1)
        rospy.Subscriber("/rescue/traversability_mean", Float32, self.traversability_mean_cb, queue_size=1)
        rospy.Subscriber(
            "/rescue/traversability_uncertainty",
            Float32,
            self.traversability_uncertainty_cb,
            queue_size=1,
        )
        rospy.Subscriber("/rescue/terrain_cost", Float32, self.terrain_cost_cb, queue_size=1)
        rospy.Subscriber("/rescue/visibility_confidence", Float32, self.visibility_confidence_cb, queue_size=1)
        rospy.Subscriber("/rescue/stuck_risk_prior", Float32, self.stuck_risk_prior_cb, queue_size=1)
        rospy.Subscriber("/rescue/battery", Float32, self.battery_cb, queue_size=1)
        rospy.Subscriber("/rescue/battery_j", Float32, self.battery_j_cb, queue_size=1)
        rospy.Subscriber(
            "/rescue/rotors_mechanical_remaining_j", Float32,
            self.rotors_mechanical_remaining_cb, queue_size=1
        )
        rospy.Subscriber("/rescue/mission_state", String, self.state_cb, queue_size=1)
        rospy.Subscriber(
            "/rescue/rotors/disarmed", Bool, self.rotors_disarmed_cb, queue_size=1
        )
        if self.handoff_managed:
            rospy.Subscriber("/rescue/mode", String, self.public_mode_cb, queue_size=1)

    def public_mode_cb(self, msg):
        if msg.data in (self.GROUND, self.TAKEOFF, self.AIR, self.LAND):
            self.set_mode(msg.data)

    def odom_cb(self, msg):
        self.odom = msg

    def goal_cb(self, msg):
        self.goal = msg

    def slope_cb(self, msg):
        self.slope_deg = msg.data

    def obstacle_cb(self, msg):
        self.obstacle_density = msg.data

    def traversability_mean_cb(self, msg):
        self.traversability_mean = max(0.0, min(1.0, msg.data))

    def traversability_uncertainty_cb(self, msg):
        self.traversability_uncertainty = max(0.0, msg.data)

    def terrain_cost_cb(self, msg):
        self.terrain_cost = max(0.0, min(1.0, msg.data))

    def visibility_confidence_cb(self, msg):
        self.visibility_confidence = max(0.0, min(1.0, msg.data))

    def stuck_risk_prior_cb(self, msg):
        self.stuck_risk_prior = max(0.0, min(1.0, msg.data))

    def battery_cb(self, msg):
        self.battery = max(0.0, min(1.0, msg.data))

    def battery_j_cb(self, msg):
        self.battery_j = max(0.0, msg.data)

    def rotors_mechanical_remaining_cb(self, msg):
        self.rotors_mechanical_remaining_j = max(0.0, msg.data)

    def state_cb(self, msg):
        self.mission_state = msg.data

    def rotors_disarmed_cb(self, msg):
        self.rotors_disarmed = msg.data

    def takeoff_transition_complete(self):
        if not self.flight_feedback_enabled:
            return self.mode_elapsed() >= self.takeoff_duration
        if self.odom is None:
            return False
        altitude = self.odom.pose.pose.position.z
        vertical_speed = self.odom.twist.twist.linear.z
        return (
            altitude >= self.cruise_altitude - self.takeoff_altitude_tolerance
            and abs(vertical_speed) <= self.takeoff_vertical_speed_tolerance
        )

    def land_transition_complete(self):
        if not self.flight_feedback_enabled:
            return self.mode_elapsed() >= self.land_duration
        return self.rotors_disarmed

    def distance_to_goal(self):
        if self.odom is None or self.goal is None:
            return 0.0
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        return math.hypot(gx - px, gy - py)

    def set_mode(self, mode):
        if mode != self.mode:
            self.mode = mode
            self.mode_enter_time = rospy.Time.now()
            self.switch_times.append(self.mode_enter_time)

    def mode_elapsed(self):
        return max(0.0, (rospy.Time.now() - self.mode_enter_time).to_sec())

    def node_elapsed(self):
        return max(0.0, (rospy.Time.now() - self.node_start_time).to_sec())

    @staticmethod
    def cvar_two_point(p_succ, c_nom, c_stuck, alpha):
        p_succ = max(0.0, min(1.0, p_succ))
        alpha = max(0.0, min(0.999, alpha))
        tail = max(1e-6, 1.0 - alpha)
        p_fail = 1.0 - p_succ
        if p_fail >= tail:
            return c_stuck
        return (p_fail * c_stuck + (tail - p_fail) * c_nom) / tail

    def set_decision_debug(self, j_ground=0.0, j_air=0.0, risk_cvar=0.0, energy_term=0.0, switch_reason=""):
        self.decision_debug = {
            "J_ground": float(j_ground),
            "J_air": float(j_air),
            "risk_cvar": float(risk_cvar),
            "energy_term": float(energy_term),
            "switch_reason": switch_reason,
        }

    def publish_decision_debug(self):
        self.j_ground_pub.publish(Float32(data=self.decision_debug["J_ground"]))
        self.j_air_pub.publish(Float32(data=self.decision_debug["J_air"]))
        self.risk_cvar_pub.publish(Float32(data=self.decision_debug["risk_cvar"]))
        self.energy_term_pub.publish(Float32(data=self.decision_debug["energy_term"]))
        self.switch_reason_pub.publish(String(data=self.decision_debug["switch_reason"]))

    def risk_cost_decision(self, strategy, distance, must_save_energy, battery_takeoff):
        p_trav = max(0.0, min(1.0, self.traversability_mean))
        u_trav = max(0.0, self.traversability_uncertainty)
        alpha = self.risk_params.get("alpha", 0.8)
        k_unc = self.risk_params.get("k_unc", 1.0)
        c_stuck = self.risk_params.get("C_stuck", 60.0)
        ground_speed = max(0.05, self.risk_params.get("ground_speed", 0.45))
        air_speed = max(0.05, self.risk_params.get("air_speed", 0.70))
        ground_energy_per_m = self.risk_params.get("ground_energy_per_m", 0.35)
        air_energy_per_m = self.risk_params.get("air_energy_per_m", 1.2)
        takeoff_energy = self.risk_params.get("takeoff_energy_j", 11.0)
        land_energy = self.risk_params.get("land_energy_j", 12.0)
        switch_penalty = self.risk_params.get("switch_penalty", 2.0)
        landing_risk_weight = self.risk_params.get("landing_risk_weight", 10.0)

        wE_g = self.risk_params.get("wE_g", 1.0)
        wT_g = self.risk_params.get("wT_g", 0.15)
        wR = self.risk_params.get("wR", 0.45)
        wD = self.risk_params.get("wD", 0.10)
        wS = self.risk_params.get("wS", 1.0)
        wE_a = self.risk_params.get("wE_a", 1.0)
        wL = self.risk_params.get("wL", 1.0)
        wcvar = self.risk_params.get("wcvar", 0.75)

        e_ground_pred = ground_energy_per_m * distance
        t_ground_pred = distance / ground_speed
        e_air_pred = air_energy_per_m * distance / air_speed
        detour = self.terrain_cost * distance
        landing_risk = max(0.0, 1.0 - self.visibility_confidence) * landing_risk_weight
        energy_term = e_ground_pred

        j_ground_energy = (
            wE_g * e_ground_pred
            + wT_g * t_ground_pred
            + wR * (1.0 - p_trav) * c_stuck
            + wD * detour
            + wS * switch_penalty
        )
        j_air = (
            wE_a * (takeoff_energy + e_air_pred + land_energy)
            + wS * switch_penalty
            + wL * landing_risk
        )

        risk_cvar = 0.0
        j_ground = j_ground_energy
        if strategy in ("risk_aware", "risk_aware_safety", "risk_aware_active"):
            p_fail_eff = max(0.0, min(1.0, (1.0 - p_trav) + k_unc * u_trav))
            p_succ_eff = 1.0 - p_fail_eff
            risk_cvar = self.cvar_two_point(p_succ_eff, t_ground_pred, c_stuck, alpha)
            j_ground = (
                wE_g * e_ground_pred
                + wcvar * risk_cvar
                + wD * detour
                + wS * switch_penalty
            )

        if must_save_energy or self.battery < battery_takeoff:
            self.set_decision_debug(j_ground, j_air, risk_cvar, energy_term, f"{strategy}:low_battery_ground")
            return False

        want_air = j_air < j_ground
        reason = f"{strategy}:{'air_cost_lower' if want_air else 'ground_cost_lower'}"
        self.set_decision_debug(j_ground, j_air, risk_cvar, energy_term, reason)
        return want_air

    def transition_toward_air_request(self, want_air, must_save_energy):
        if self.mode == self.GROUND:
            return self.TAKEOFF if want_air and not must_save_energy else self.GROUND
        if self.mode == self.TAKEOFF:
            return self.AIR if self.takeoff_transition_complete() else self.TAKEOFF
        if self.mode == self.AIR:
            if not want_air and self.mode_elapsed() >= self.min_air_duration:
                return self.LAND
            return self.AIR
        if self.mode == self.LAND:
            if not self.land_transition_complete():
                return self.LAND
            return self.TAKEOFF if want_air and not must_save_energy else self.GROUND
        return self.mode

    def shield(self, raw_mode):
        now = rospy.Time.now()
        window = self.safety_shield.recent_switch_window_sec
        self.switch_times = [
            t for t in self.switch_times
            if (now - t).to_sec() <= window
        ]
        state = {
            "current_mode": self.mode,
            "battery": self.battery,
            "obstacle_density": self.obstacle_density,
            "mode_elapsed": self.mode_elapsed(),
            "min_air_duration": self.min_air_duration,
            "switch_count_recent": len(self.switch_times),
        }
        return self.safety_shield.filter(raw_mode, state)

    def decide(self):
        if self.mission_state == "DONE":
            self.set_decision_debug(
                self.decision_debug["J_ground"],
                self.decision_debug["J_air"],
                self.decision_debug["risk_cvar"],
                self.decision_debug["energy_term"],
                "mission_done",
            )
            if self.mode in (self.TAKEOFF, self.AIR):
                return self.LAND
            if self.mode == self.LAND:
                return self.GROUND if self.land_transition_complete() else self.LAND
            return self.GROUND

        # ── ground_only ────────────────────────────────────────────────────────
        if self.strategy == "ground_only":
            self.set_decision_debug(switch_reason="ground_only")
            if self.mode in (self.TAKEOFF, self.AIR, self.LAND):
                return self.GROUND if self.land_transition_complete() else self.LAND
            return self.GROUND

        # ── fixed_switch ───────────────────────────────────────────────────────
        # Naive baseline: alternate GROUND / AIR every switch_interval seconds,
        # regardless of terrain or energy.
        if self.strategy == "fixed_switch":
            self.set_decision_debug(switch_reason="fixed_switch")
            switch_interval = self.thresholds.get("fixed_switch_interval", 10.0)
            elapsed = self.node_elapsed()
            want_air = fixed_switch_wants_air(
                elapsed, self.fixed_first_switch_time, switch_interval)

            if want_air:
                if self.mode == self.GROUND:
                    return self.TAKEOFF
                if self.mode == self.TAKEOFF:
                    return self.AIR if self.takeoff_transition_complete() else self.TAKEOFF
                if self.mode == self.LAND:
                    return self.TAKEOFF if self.land_transition_complete() else self.LAND
                return self.AIR
            else:
                if self.mode in (self.TAKEOFF, self.AIR):
                    return self.LAND
                if self.mode == self.LAND:
                    return self.GROUND if self.land_transition_complete() else self.LAND
                return self.GROUND

        # ── shared rule inputs ─────────────────────────────────────────────────
        slope_air = self.thresholds.get("slope_air_deg", 30.0)
        obstacle_air = self.thresholds.get("obstacle_air_density", 0.65)
        obstacle_ground = self.thresholds.get("obstacle_ground_density", 0.85)
        fly_distance = self.thresholds.get("fly_distance", 8.0)
        ground_distance = self.thresholds.get("ground_distance", 2.0)
        slope_fly_min_distance = self.thresholds.get("slope_fly_min_distance", 3.0)
        return_ground_finish_distance = self.thresholds.get("return_ground_finish_distance", 3.5)
        battery_takeoff = self.thresholds.get("battery_takeoff_min", 0.40)
        battery_ground = self.thresholds.get("battery_ground_max", 0.25)

        distance = self.distance_to_goal()
        must_save_energy = self.battery < battery_ground
        obstacle_hazard = self.obstacle_density > obstacle_air
        slope_hazard = self.slope_deg > slope_air
        near_return_finish = (
            self.mission_state == "RETURN"
            and distance < return_ground_finish_distance
        )
        slope_requires_flight = (
            slope_hazard
            and distance > slope_fly_min_distance
            and not near_return_finish
        )
        terrain_hazard = obstacle_hazard or slope_requires_flight
        should_fly = (
            self.battery > battery_takeoff
            and (
                terrain_hazard
                or (distance > fly_distance and self.obstacle_density < obstacle_ground)
            )
        )
        should_land = (
            must_save_energy
            or (
                not terrain_hazard
                and self.mode_elapsed() >= self.min_air_duration
                and (distance < ground_distance or not should_fly)
            )
        )

        if self.strategy in ("energy_rule", "risk_aware", "risk_aware_safety", "risk_aware_active"):
            want_air = self.risk_cost_decision(
                self.strategy,
                distance,
                must_save_energy,
                battery_takeoff,
            )
            # Near the terminal goal the robot must be on the ground to deliver /
            # finish the mission (DELIVER -> RETURN requires GROUND mode, and the
            # round trip ends on the ground). With distance ~ 0 the two-point CVaR
            # is still dominated by the fixed C_stuck tail, which would otherwise
            # keep risk_aware hovering forever and block mission completion.
            near_terminal_goal = (
                self.mission_state in ("DELIVER", "RETURN")
                and distance < ground_distance
            )
            if near_terminal_goal:
                want_air = False
                reason = "%s:near_terminal_goal_land" % self.strategy
                self.set_decision_debug(
                    self.decision_debug["J_ground"],
                    self.decision_debug["J_air"],
                    self.decision_debug["risk_cvar"],
                    self.decision_debug["energy_term"],
                    reason,
                )
            return self.transition_toward_air_request(want_air, must_save_energy)

        # ── threshold_switch ──────────────────────────────────────────────────
        # Tuned naive baseline (Claim A stress test): fly iff (1 - p_trav) > tau,
        # using only the shared perceived traversability. No CVaR, no uncertainty.
        # tau is swept to find the per-regime Pareto-optimal threshold.
        if self.strategy == "threshold_switch":
            tau = self.switch_tau if self.switch_tau >= 0.0 else self.thresholds.get("switch_tau", 0.4)
            want_air = (1.0 - self.traversability_mean) > tau
            near_terminal_goal = (
                self.mission_state in ("DELIVER", "RETURN") and distance < ground_distance
            )
            if near_terminal_goal or must_save_energy:
                want_air = False
            self.set_decision_debug(
                switch_reason="threshold_switch:%s(tau=%.2f)" % ("air" if want_air else "ground", tau)
            )
            return self.transition_toward_air_request(want_air, must_save_energy)

        # ── air_preferred ──────────────────────────────────────────────────────
        if self.strategy == "air_preferred":
            self.set_decision_debug(switch_reason="air_preferred")
            near_terminal_goal = (
                self.mission_state in ("DELIVER", "RETURN")
                and distance < ground_distance
            )
            if self.mode == self.GROUND and near_terminal_goal:
                return self.GROUND
            if self.mode == self.GROUND and not must_save_energy:
                return self.TAKEOFF
            if self.mode == self.TAKEOFF:
                return self.AIR if self.takeoff_transition_complete() else self.TAKEOFF
            if self.mode == self.AIR:
                if near_terminal_goal:
                    return self.LAND
                return self.LAND if must_save_energy else self.AIR
            if self.mode == self.LAND:
                if not self.land_transition_complete():
                    return self.LAND
                if near_terminal_goal:
                    return self.GROUND
                return self.GROUND if must_save_energy else self.TAKEOFF
            return self.mode

        # ── rule / rule_energy_shield (default) ───────────────────────────────
        self.set_decision_debug(
            j_ground=0.0,
            j_air=0.0,
            risk_cvar=0.0,
            energy_term=0.0,
            switch_reason="rule_should_fly" if should_fly else "rule_ground",
        )
        use_shield = self.strategy in ("rule_energy_shield", "rule_shield", "safe_rule")
        if self.mode == self.GROUND and should_fly and not must_save_energy:
            raw_mode = self.TAKEOFF
            return self.shield(raw_mode) if use_shield else raw_mode

        if self.mode == self.TAKEOFF:
            if self.takeoff_transition_complete():
                raw_mode = self.AIR
                return self.shield(raw_mode) if use_shield else raw_mode
            raw_mode = self.TAKEOFF
            return self.shield(raw_mode) if use_shield else raw_mode

        if self.mode == self.AIR and should_land:
            raw_mode = self.LAND
            return self.shield(raw_mode) if use_shield else raw_mode

        if self.mode == self.LAND:
            if not self.land_transition_complete():
                raw_mode = self.LAND
                return self.shield(raw_mode) if use_shield else raw_mode
            if should_fly and not must_save_energy:
                raw_mode = self.TAKEOFF
                return self.shield(raw_mode) if use_shield else raw_mode
            raw_mode = self.GROUND
            return self.shield(raw_mode) if use_shield else raw_mode

        return self.shield(self.mode) if use_shield else self.mode

    def apply_supervisor(self, proposed_mode):
        """Route a proposed mode through the decision-layer safety supervisor.

        Always monitors (counters / margin / irreversible detection); only
        overrides the action when enforcing (risk_aware_safety).
        """
        state = {
            "current_mode": self.mode,
            "battery_j": self.battery_j,
            "energy_remaining_j": (
                self.rotors_mechanical_remaining_j
                if self.air_backend == "rotors" else self.battery_j
            ),
            "landing_area_safety": self.visibility_confidence,
            "stuck_risk_prior": self.stuck_risk_prior,
        }
        return self.supervisor.filter(proposed_mode, state)

    def publish_supervisor_metrics(self):
        self.shield_override_pub.publish(Float32(data=float(self.supervisor.shield_override_count)))
        self.unsafe_action_pub.publish(Float32(data=float(self.supervisor.unsafe_action_count)))
        self.irreversible_pub.publish(Float32(data=float(self.supervisor.irreversible_failure)))
        self.safe_margin_pub.publish(Float32(data=float(self.supervisor.safe_landing_margin_j)))
        self.override_reason_pub.publish(String(data=self.supervisor.override_reason))
        self.unrecoverable_pub.publish(Float32(data=float(self.supervisor.entered_unrecoverable_set)))
        margin_min = self.supervisor.recoverability_margin_min
        if margin_min == float("inf"):
            margin_min = 0.0
        self.recov_margin_min_pub.publish(Float32(data=float(margin_min)))
        self.supervisor_backend_pub.publish(String(data=self.air_backend))
        self.supervisor_model_pub.publish(String(data=self.supervisor.energy_model))
        self.supervisor_predicted_pub.publish(Float32(
            data=float(self.supervisor.predicted_consumption_j)
        ))
        self.supervisor_reserve_pub.publish(Float32(
            data=float(self.supervisor.energy_reserve_j)
        ))
        self.supervisor_block_reason_pub.publish(String(
            data=self.supervisor.block_reason
        ))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            proposed = self.decide()
            approved = self.apply_supervisor(proposed)
            if self.handoff_managed:
                self.mode_pub.publish(String(data=approved))
            else:
                self.set_mode(approved)
                self.mode_pub.publish(String(data=self.mode))
            self.publish_decision_debug()
            self.publish_supervisor_metrics()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mode_switcher")
    ModeSwitcher().spin()
