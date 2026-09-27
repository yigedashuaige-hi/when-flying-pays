#!/usr/bin/env python3
import csv
import datetime
import math
import os

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Twist
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32, String

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def euler_from_quaternion(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class RescueMetricsLogger:
    def __init__(self):
        # ── auto-generate timestamped path under results/runs/ ─────────────────
        strategy_tag = rospy.get_param("~strategy_tag", "unknown")
        self.scene_tag = rospy.get_param("~scene_tag", "scene_a")
        # Seed for the randomized environment (paper-hardening). -1 == legacy
        # deterministic run. Recorded per row so analysis can pair runs by seed.
        self.seed = int(rospy.get_param("~seed", -1))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        # Prefer package results/runs/; fall back to ~/.ros/rescue_runs/ if not writable
        pkg_runs = os.path.join(_PKG_DIR, "results", "runs")
        fallback_runs = os.path.expanduser("~/.ros/rescue_runs")
        default_runs = pkg_runs if os.access(os.path.dirname(pkg_runs), os.W_OK) else fallback_runs
        # Empty string (e.g. unset launch arg) falls back to the default dir.
        runs_dir = rospy.get_param("~runs_dir", "") or default_runs
        os.makedirs(runs_dir, exist_ok=True)
        default_path = os.path.join(runs_dir, f"{strategy_tag}__{timestamp}.csv")
        self.csv_path = rospy.get_param("~csv_path", default_path)
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self.latest_csv_path = rospy.get_param(
            "~latest_csv_path",
            os.path.join(_PKG_DIR, "results", "rescue_metrics.csv"),
        )
        os.makedirs(os.path.dirname(self.latest_csv_path), exist_ok=True)

        # ── energy model params ────────────────────────────────────────────────
        self.model_name = rospy.get_param("~model_name", "uav_v4")
        self.fly_k1 = rospy.get_param(
            "~fly_k1", rospy.get_param("/mode_switcher/energy/fly_k1", 1.0)
        )
        self.ground_k2 = rospy.get_param(
            "~ground_k2", rospy.get_param("/mode_switcher/energy/ground_k2", 0.35)
        )
        self.hover_thrust_proxy = rospy.get_param("~hover_thrust_proxy", 1.0)
        self.vertical_thrust_gain = rospy.get_param("~vertical_thrust_gain", 1.0)
        self.horizontal_thrust_gain = rospy.get_param("~horizontal_thrust_gain", 0.25)
        self.switch_energy_cost = rospy.get_param("~switch_energy_cost", 0.25)

        # Phase D1 additive RotorS energy metadata. The historical columns and
        # battery accounting below deliberately continue to use the legacy task
        # proxy; this mechanical proxy is recorded separately and is not battery
        # electrical energy.
        self.air_backend = rospy.get_param("~air_backend", "ideal")
        self.legacy_energy_model = "legacy_task_distance_hover_proxy_v1"
        self.energy_model = (
            "rotors_aero_shaft_mechanical_proxy_v1"
            if self.air_backend == "rotors"
            else self.legacy_energy_model
        )
        self.rotor_force_constant = float(
            rospy.get_param("~rotor_force_constant_n_per_rad_s_sq", 1.568e-05)
        )
        self.rotor_moment_constant = float(
            rospy.get_param("~rotor_moment_constant_m", 0.06)
        )
        self.rotor_velocity_slowdown = float(
            rospy.get_param("~rotor_velocity_slowdown_sim", 10.0)
        )
        self.rotors_mechanical_budget_j = float(
            rospy.get_param("~rotors_mechanical_budget_j", 0.0)
        )

        # ── battery energy budget (task-level joule budget, sole owner) ─────────
        # The logger is the single source of truth for battery state: it already
        # integrates energy_j across all modes, so battery = budget - consumed.
        # Default capacity is large so legacy scenes stay ~full; Scene D sets a
        # small capacity / low initial_battery to make low-battery flight fail.
        self.battery_capacity_j = rospy.get_param("~battery_capacity_j", 2000.0)
        self.initial_battery = rospy.get_param("~initial_battery", 1.0)

        # ── mission timeout ────────────────────────────────────────────────────
        self.mission_timeout_sec = rospy.get_param("~mission_timeout_sec", 120.0)

        # ── state ──────────────────────────────────────────────────────────────
        self.mode = ""
        self.mission_state = ""
        self.goal = None
        self.odom = None
        self.model_pose = None
        self.cmd = Twist()
        self.nominal_cmd = Twist()
        self.slope = 0.0
        self.obstacle_density = 0.0
        self.traversability_mean = 1.0
        self.traversability_true = 1.0
        self.traversability_uncertainty = 0.0
        self.terrain_cost = 0.0
        self.visibility_confidence = 1.0
        self.stuck_risk_prior = 0.0
        self.j_ground = 0.0
        self.j_air = 0.0
        self.risk_cvar = 0.0
        self.energy_term = 0.0
        self.switch_reason = ""
        # safety supervisor metrics (published by mode_switcher)
        self.shield_override_count = 0
        self.unsafe_action_count = 0
        self.irreversible_failure = 0
        self.safe_landing_margin_j = 0.0
        self.override_reason = "none"
        self.entered_unrecoverable_set = 0
        self.recoverability_margin_min = 0.0
        self.battery = self.initial_battery

        self.path_length = 0.0
        self.ground_distance_m = 0.0
        self.air_distance_m = 0.0
        self.switch_count = 0
        self.last_mode = ""
        self.last_xy = None

        self.energy_j = 0.0
        self.ground_energy_j = 0.0
        self.air_energy_j = 0.0
        self.takeoff_energy_j = 0.0
        self.land_energy_j = 0.0
        self.switch_energy_j = 0.0
        self.power_w = 0.0
        self.last_energy_pose = None
        self.last_energy_time = None

        # RotorS mechanical-energy stream and per-calibration-trial counters.
        self.rotors_motor_command = [0.0] * 4
        self.rotors_motor_actual = [0.0] * 4
        self.rotors_motor_command_received = False
        self.rotors_motor_actual_received = False
        self.rotors_mechanical_power_w = 0.0
        self.rotors_mechanical_energy_j = 0.0
        self.rotors_trial_mechanical_energy_j = 0.0
        self.rotors_trial_flight_duration_s = 0.0
        self.rotors_trial_distance_m = 0.0
        self.rotors_phase_energy_j = {
            "TAKEOFF": 0.0,
            "HOVER": 0.0,
            "TRANSLATION": 0.0,
            "RETURN": 0.0,
            "LANDING": 0.0,
        }
        self.rotors_phase_duration_s = dict.fromkeys(
            self.rotors_phase_energy_j, 0.0
        )
        self.rotors_last_time = None
        self.rotors_last_pose = None
        self.rotors_velocity = (0.0, 0.0, 0.0)
        self.energy_phase_override = ""
        self.calibration_trial_id = ""
        self.calibration_split = ""
        # Phase D2 supervisor accounting. These are append-only metadata and do
        # not replace the historical task-energy/battery ledger above.
        self.supervisor_energy_backend = self.air_backend
        self.supervisor_energy_model = self.energy_model
        self.supervisor_predicted_consumption_j = 0.0
        self.supervisor_reserve_j = 0.0
        self.supervisor_block_reason = "none"
        # Phase E1a-C append-only takeoff-site and recovery telemetry.
        self.takeoff_measurement_valid = False
        self.takeoff_site_valid = False
        self.takeoff_clearance_available_m = float("nan")
        self.takeoff_clearance_required_m = float("nan")
        self.takeoff_nearest_obstacle_m = float("nan")
        self.takeoff_site_reason = "measurement_missing"
        self.takeoff_decision_reason = "init"
        self.takeoff_blocked_duration_s = 0.0
        self.takeoff_blocked_distance_m = 0.0
        self.rotors_obstacle_contact = False
        self.rotors_obstacle_contact_force_n = 0.0
        self.rotors_handoff_state = ""
        # Phase E1a-R append-only deferred-takeoff reposition telemetry.
        self.takeoff_reposition_active = False
        self.takeoff_reposition_goal = None
        self.takeoff_reposition_duration_s = 0.0
        self.takeoff_reposition_distance_m = 0.0
        self.takeoff_reposition_initial_distance_m = 0.0
        self.takeoff_reposition_target_history_age_s = 0.0
        self.takeoff_reposition_result = "none"
        # Phase E1a-R2 append-only persistent safe-site metadata.
        self.persistent_safe_site_count = 0
        self.reposition_target_site_id = 0
        self.reposition_target_source = "none"
        self.reposition_target_reachability = "none"
        self.reposition_target_observation_age_s = 0.0
        self.reposition_target_clearance_available_m = float("nan")
        self.reposition_target_clearance_required_m = float("nan")
        self.reposition_target_nearest_obstacle_m = float("nan")
        self.reposition_target_revalidated = False

        # stuck / collision detection
        self.stuck_duration = 0.0
        self.stuck_count = 0
        self.collision_count = 0
        self.current_slip_ratio = 0.0
        self.slip_ratio_time_sum = 0.0
        self.slip_ratio_weight_sum = 0.0

        self.timed_out = False
        self._last_row = None
        self.start_time = rospy.Time.now()

        # ── CSV setup ──────────────────────────────────────────────────────────
        self.files = [open(self.csv_path, "w", newline="")]
        if os.path.abspath(self.latest_csv_path) != os.path.abspath(self.csv_path):
            self.files.append(open(self.latest_csv_path, "w", newline=""))
        self.writers = [csv.writer(f) for f in self.files]
        self.header = [
            "scene",
            "seed",
            "time",
            "mode",
            "mission_state",
            "x", "y", "z",
            "roll", "pitch", "yaw",
            "goal_x", "goal_y",
            "cmd_vx", "cmd_vy", "cmd_wz",
            "slope_deg",
            "obstacle_density",
            "traversability_mean",
            "traversability_true",
            "traversability_uncertainty",
            "terrain_cost",
            "visibility_confidence",
            "stuck_risk_prior",
            "J_ground",
            "J_air",
            "risk_cvar",
            "energy_term",
            "switch_reason",
            "battery",
            "power_w",
            "energy_j",
            "ground_energy_j",
            "air_energy_j",
            "takeoff_energy_j",
            "land_energy_j",
            "switch_energy_j",
            "path_length",
            "ground_distance_m",
            "air_distance_m",
            "switch_count",
            "collision_count",
            "stuck_count",
            "stuck_event_count",
            "slip_ratio",
            "shield_override_count",
            "override_reason",
            "safe_landing_margin_j",
            "unsafe_action_count",
            "irreversible_failure",
            "entered_unrecoverable_set",
            "recoverability_margin_min",
            "final_distance",
            "success",
            "timeout",
            # Phase D1 columns are append-only: every Stage-0 column above
            # retains its original name, order, and semantics.
            "air_backend",
            "legacy_energy_model",
            "energy_model",
            "flight_phase",
            "calibration_trial_id",
            "calibration_split",
            "rotors_cmd_omega_0_rad_s",
            "rotors_cmd_omega_1_rad_s",
            "rotors_cmd_omega_2_rad_s",
            "rotors_cmd_omega_3_rad_s",
            "rotors_actual_omega_0_rad_s",
            "rotors_actual_omega_1_rad_s",
            "rotors_actual_omega_2_rad_s",
            "rotors_actual_omega_3_rad_s",
            "rotors_cmd_omega_mean_rad_s",
            "rotors_actual_omega_mean_rad_s",
            "rotors_total_thrust_n",
            "rotors_mechanical_power_w",
            "rotors_mechanical_energy_j",
            "rotors_trial_mechanical_energy_j",
            "rotors_trial_flight_duration_s",
            "rotors_trial_distance_m",
            "rotors_takeoff_mechanical_energy_j",
            "rotors_hover_mechanical_energy_j",
            "rotors_translation_mechanical_energy_j",
            "rotors_return_mechanical_energy_j",
            "rotors_landing_mechanical_energy_j",
            "rotors_takeoff_duration_s",
            "rotors_hover_duration_s",
            "rotors_translation_duration_s",
            "rotors_return_duration_s",
            "rotors_landing_duration_s",
            "rotors_altitude_m",
            "rotors_velocity_x_m_s",
            "rotors_velocity_y_m_s",
            "rotors_velocity_z_m_s",
            "rotors_speed_m_s",
            "rotor_force_constant_n_per_rad_s_sq",
            "rotor_moment_constant_m",
            "rotor_velocity_slowdown_sim",
            "rotors_energy_valid",
            # Phase D2 append-only supervisor fields.
            "supervisor_energy_backend",
            "supervisor_energy_model",
            "supervisor_predicted_consumption_j",
            "supervisor_actual_consumption_j",
            "supervisor_reserve_j",
            "supervisor_block_reason",
            "rotors_mechanical_budget_j",
            "rotors_mechanical_remaining_j",
            # Phase E1a-C append-only geometry/decision fields.
            "takeoff_measurement_valid",
            "takeoff_site_valid",
            "takeoff_clearance_available_m",
            "takeoff_clearance_required_m",
            "takeoff_nearest_obstacle_m",
            "takeoff_site_reason",
            "takeoff_decision_reason",
            "takeoff_blocked_duration_s",
            "takeoff_blocked_distance_m",
            "rotors_obstacle_contact",
            "rotors_obstacle_contact_force_n",
            "rotors_handoff_state",
            # Phase E1a-R append-only recovery fields.
            "takeoff_reposition_active",
            "takeoff_reposition_goal_x_m",
            "takeoff_reposition_goal_y_m",
            "takeoff_reposition_goal_yaw_rad",
            "takeoff_reposition_duration_s",
            "takeoff_reposition_distance_m",
            "takeoff_reposition_initial_distance_m",
            "takeoff_reposition_target_history_age_s",
            "takeoff_reposition_result",
            # Phase E1a-R2 append-only persistent safe-site fields.
            "persistent_safe_site_count",
            "takeoff_reposition_target_site_id",
            "takeoff_reposition_target_source",
            "takeoff_reposition_target_reachability",
            "takeoff_reposition_target_observation_age_s",
            "takeoff_reposition_target_clearance_available_m",
            "takeoff_reposition_target_clearance_required_m",
            "takeoff_reposition_target_nearest_obstacle_m",
            "takeoff_reposition_target_revalidated",
        ]
        for writer in self.writers:
            writer.writerow(self.header)
        rospy.loginfo("RescueMetricsLogger: writing to %s", self.csv_path)
        rospy.loginfo("RescueMetricsLogger: latest mirror %s", self.latest_csv_path)

        # ── battery publishers (logger is the sole battery owner) ──────────────
        self.battery_pub = rospy.Publisher("/rescue/battery", Float32, queue_size=10)
        self.battery_j_pub = rospy.Publisher("/rescue/battery_j", Float32, queue_size=10)
        self.rotors_mechanical_remaining_pub = rospy.Publisher(
            "/rescue/rotors_mechanical_remaining_j", Float32, queue_size=10
        )
        self.rotors_mechanical_consumed_pub = rospy.Publisher(
            "/rescue/rotors_mechanical_consumed_j", Float32, queue_size=10
        )

        # ── subscribers ────────────────────────────────────────────────────────
        rospy.Subscriber("/rescue/mode", String, self.mode_cb, queue_size=1)
        rospy.Subscriber("/rescue/mission_state", String, self.state_cb, queue_size=1)
        rospy.Subscriber("/rescue/goal", PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1)
        rospy.Subscriber("/mecanum/cmd_vel", Twist, self.cmd_cb, queue_size=1)
        rospy.Subscriber("/rescue/ground_cmd_nominal", Twist, self.nominal_cmd_cb, queue_size=1)
        rospy.Subscriber("/rescue/terrain_slope_deg", Float32, self.slope_cb, queue_size=1)
        rospy.Subscriber("/rescue/obstacle_density", Float32, self.obstacle_cb, queue_size=1)
        rospy.Subscriber("/rescue/traversability_mean", Float32, self.traversability_mean_cb, queue_size=1)
        rospy.Subscriber("/rescue/traversability_true", Float32, self.traversability_true_cb, queue_size=1)
        rospy.Subscriber(
            "/rescue/traversability_uncertainty",
            Float32,
            self.traversability_uncertainty_cb,
            queue_size=1,
        )
        rospy.Subscriber("/rescue/terrain_cost", Float32, self.terrain_cost_cb, queue_size=1)
        rospy.Subscriber("/rescue/visibility_confidence", Float32, self.visibility_confidence_cb, queue_size=1)
        rospy.Subscriber("/rescue/stuck_risk_prior", Float32, self.stuck_risk_prior_cb, queue_size=1)
        rospy.Subscriber("/rescue/J_ground", Float32, self.j_ground_cb, queue_size=1)
        rospy.Subscriber("/rescue/J_air", Float32, self.j_air_cb, queue_size=1)
        rospy.Subscriber("/rescue/risk_cvar", Float32, self.risk_cvar_cb, queue_size=1)
        rospy.Subscriber("/rescue/energy_term", Float32, self.energy_term_cb, queue_size=1)
        rospy.Subscriber("/rescue/switch_reason", String, self.switch_reason_cb, queue_size=1)
        rospy.Subscriber("/rescue/shield_override_count", Float32, self.shield_override_cb, queue_size=1)
        rospy.Subscriber("/rescue/unsafe_action_count", Float32, self.unsafe_action_cb, queue_size=1)
        rospy.Subscriber("/rescue/irreversible_failure", Float32, self.irreversible_cb, queue_size=1)
        rospy.Subscriber("/rescue/safe_landing_margin_j", Float32, self.safe_margin_cb, queue_size=1)
        rospy.Subscriber("/rescue/override_reason", String, self.override_reason_cb, queue_size=1)
        rospy.Subscriber("/rescue/entered_unrecoverable_set", Float32, self.entered_unrecoverable_cb, queue_size=1)
        rospy.Subscriber("/rescue/recoverability_margin_min", Float32, self.recoverability_margin_min_cb, queue_size=1)
        rospy.Subscriber(
            "/rescue/supervisor_energy_backend", String,
            self.supervisor_energy_backend_cb, queue_size=1
        )
        rospy.Subscriber(
            "/rescue/supervisor_energy_model", String,
            self.supervisor_energy_model_cb, queue_size=1
        )
        rospy.Subscriber(
            "/rescue/supervisor_predicted_consumption_j", Float32,
            self.supervisor_predicted_cb, queue_size=1
        )
        rospy.Subscriber(
            "/rescue/supervisor_reserve_j", Float32,
            self.supervisor_reserve_cb, queue_size=1
        )
        rospy.Subscriber(
            "/rescue/supervisor_block_reason", String,
            self.supervisor_block_reason_cb, queue_size=1
        )
        rospy.Subscriber(
            "/uav_v4/command/motor_speed",
            Actuators,
            self.rotors_motor_command_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            "/uav_v4/motor_speed",
            Actuators,
            self.rotors_motor_actual_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/energy_phase", String, self.energy_phase_cb, queue_size=1
        )
        rospy.Subscriber(
            "/rescue/calibration_trial_id",
            String,
            self.calibration_trial_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/calibration_split",
            String,
            self.calibration_split_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/takeoff_site_status", DiagnosticArray,
            self.takeoff_clearance_cb, queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/takeoff_decision_reason", String,
            lambda msg: setattr(self, "takeoff_decision_reason", msg.data), queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/takeoff_blocked_duration_s", Float32,
            lambda msg: setattr(self, "takeoff_blocked_duration_s", msg.data), queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/takeoff_blocked_distance_m", Float32,
            lambda msg: setattr(self, "takeoff_blocked_distance_m", msg.data), queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/obstacle_contact", Bool,
            lambda msg: setattr(self, "rotors_obstacle_contact", bool(msg.data)), queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/obstacle_contact_force_n", Float32,
            lambda msg: setattr(self, "rotors_obstacle_contact_force_n", msg.data), queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/handoff_state", String,
            lambda msg: setattr(self, "rotors_handoff_state", msg.data), queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_active", Bool,
            lambda msg: setattr(self, "takeoff_reposition_active", bool(msg.data)),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_goal", PoseStamped,
            lambda msg: setattr(self, "takeoff_reposition_goal", msg), queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_duration_s", Float32,
            lambda msg: setattr(self, "takeoff_reposition_duration_s", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_distance_m", Float32,
            lambda msg: setattr(self, "takeoff_reposition_distance_m", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_initial_distance_m", Float32,
            lambda msg: setattr(self, "takeoff_reposition_initial_distance_m", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_target_history_age_s", Float32,
            lambda msg: setattr(self, "takeoff_reposition_target_history_age_s", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_result", String,
            lambda msg: setattr(self, "takeoff_reposition_result", msg.data), queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/persistent_safe_site_count", Int32,
            lambda msg: setattr(self, "persistent_safe_site_count", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_target_site_id", Int32,
            lambda msg: setattr(self, "reposition_target_site_id", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_target_source", String,
            lambda msg: setattr(self, "reposition_target_source", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_target_reachability", String,
            lambda msg: setattr(self, "reposition_target_reachability", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_target_observation_age_s", Float32,
            lambda msg: setattr(self, "reposition_target_observation_age_s", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_target_clearance_available_m", Float32,
            lambda msg: setattr(
                self, "reposition_target_clearance_available_m", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_target_clearance_required_m", Float32,
            lambda msg: setattr(
                self, "reposition_target_clearance_required_m", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_target_nearest_obstacle_m", Float32,
            lambda msg: setattr(
                self, "reposition_target_nearest_obstacle_m", msg.data),
            queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/reposition_target_revalidated", Bool,
            lambda msg: setattr(self, "reposition_target_revalidated", bool(msg.data)),
            queue_size=1,
        )
        # NOTE: battery is no longer subscribed; the logger computes and publishes
        # it from the energy budget (see update_battery()).
        rospy.on_shutdown(self.close)

    # ── callbacks ──────────────────────────────────────────────────────────────

    def mode_cb(self, msg):
        self.mode = msg.data
        if self.last_mode and self.mode != self.last_mode:
            self.switch_count += 1
            self.switch_energy_j += self.switch_energy_cost
            self.energy_j += self.switch_energy_cost
        self.last_mode = self.mode

    def takeoff_clearance_cb(self, msg):
        if not msg.status:
            return
        values = {item.key: item.value for item in msg.status[0].values}
        try:
            self.takeoff_measurement_valid = values["measurement_valid"] == "true"
            self.takeoff_site_valid = values["takeoff_site_valid"] == "true"
            self.takeoff_clearance_available_m = float(
                values["vertical_clearance_available_m"]
            )
            self.takeoff_clearance_required_m = float(
                values["vertical_clearance_required_m"]
            )
            self.takeoff_nearest_obstacle_m = float(
                values["nearest_obstacle_distance_m"]
            )
            self.takeoff_site_reason = msg.status[0].message
        except (KeyError, ValueError):
            self.takeoff_measurement_valid = False
            self.takeoff_site_valid = False
            self.takeoff_site_reason = "measurement_invalid:malformed_atomic_status"

    def state_cb(self, msg):
        self.mission_state = msg.data

    def goal_cb(self, msg):
        self.goal = msg

    def odom_cb(self, msg):
        self.odom = msg
        velocity = msg.twist.twist.linear
        self.rotors_velocity = (velocity.x, velocity.y, velocity.z)
        if self.model_pose is not None:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.update_path_length(x, y)

    def model_states_cb(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            return
        self.model_pose = msg.pose[index]
        if self.odom is None:
            velocity = msg.twist[index].linear
            self.rotors_velocity = (velocity.x, velocity.y, velocity.z)
        x = self.model_pose.position.x
        y = self.model_pose.position.y
        self.update_path_length(x, y)

    def update_path_length(self, x, y):
        if self.last_xy is not None:
            step = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.path_length += step
            if self.mode in ("TAKEOFF", "AIR", "LAND"):
                self.air_distance_m += step
            else:
                self.ground_distance_m += step
        self.last_xy = (x, y)

    def cmd_cb(self, msg):
        self.cmd = msg

    def nominal_cmd_cb(self, msg):
        self.nominal_cmd = msg

    def slope_cb(self, msg):
        self.slope = msg.data

    def obstacle_cb(self, msg):
        self.obstacle_density = msg.data

    def traversability_mean_cb(self, msg):
        self.traversability_mean = msg.data

    def traversability_true_cb(self, msg):
        self.traversability_true = msg.data

    def traversability_uncertainty_cb(self, msg):
        self.traversability_uncertainty = msg.data

    def terrain_cost_cb(self, msg):
        self.terrain_cost = msg.data

    def visibility_confidence_cb(self, msg):
        self.visibility_confidence = msg.data

    def stuck_risk_prior_cb(self, msg):
        self.stuck_risk_prior = msg.data

    def j_ground_cb(self, msg):
        self.j_ground = msg.data

    def j_air_cb(self, msg):
        self.j_air = msg.data

    def risk_cvar_cb(self, msg):
        self.risk_cvar = msg.data

    def energy_term_cb(self, msg):
        self.energy_term = msg.data

    def switch_reason_cb(self, msg):
        self.switch_reason = msg.data

    def shield_override_cb(self, msg):
        self.shield_override_count = int(round(msg.data))

    def unsafe_action_cb(self, msg):
        self.unsafe_action_count = int(round(msg.data))

    def irreversible_cb(self, msg):
        self.irreversible_failure = int(round(msg.data))

    def safe_margin_cb(self, msg):
        self.safe_landing_margin_j = msg.data

    def override_reason_cb(self, msg):
        self.override_reason = msg.data

    def entered_unrecoverable_cb(self, msg):
        self.entered_unrecoverable_set = int(round(msg.data))

    def recoverability_margin_min_cb(self, msg):
        self.recoverability_margin_min = msg.data

    def supervisor_energy_backend_cb(self, msg):
        self.supervisor_energy_backend = msg.data

    def supervisor_energy_model_cb(self, msg):
        self.supervisor_energy_model = msg.data

    def supervisor_predicted_cb(self, msg):
        self.supervisor_predicted_consumption_j = msg.data

    def supervisor_reserve_cb(self, msg):
        self.supervisor_reserve_j = msg.data

    def supervisor_block_reason_cb(self, msg):
        self.supervisor_block_reason = msg.data

    @staticmethod
    def valid_motor_vector(values):
        return len(values) == 4 and all(math.isfinite(value) for value in values)

    def rotors_motor_command_cb(self, msg):
        values = list(msg.angular_velocities)
        if self.valid_motor_vector(values):
            self.rotors_motor_command = values
            self.rotors_motor_command_received = True

    def rotors_motor_actual_cb(self, msg):
        # gazebo_multirotor_base_plugin already multiplies Gazebo joint speed by
        # rotorVelocitySlowdownSim. These are physical-model rotor rad/s, signed
        # by rotation direction; the individual motor topics are joint rad/s.
        values = list(msg.angular_velocities)
        if self.valid_motor_vector(values):
            self.rotors_motor_actual = values
            self.rotors_motor_actual_received = True

    def energy_phase_cb(self, msg):
        self.energy_phase_override = msg.data.strip().upper()

    def calibration_trial_cb(self, msg):
        new_trial = msg.data.strip()
        if new_trial and new_trial != self.calibration_trial_id:
            self.rotors_trial_mechanical_energy_j = 0.0
            self.rotors_trial_flight_duration_s = 0.0
            self.rotors_trial_distance_m = 0.0
            self.rotors_phase_energy_j = dict.fromkeys(
                self.rotors_phase_energy_j, 0.0
            )
            self.rotors_phase_duration_s = dict.fromkeys(
                self.rotors_phase_duration_s, 0.0
            )
            self.rotors_last_time = None
            self.rotors_last_pose = None
        self.calibration_trial_id = new_trial

    def calibration_split_cb(self, msg):
        self.calibration_split = msg.data.strip().lower()

    def update_battery(self):
        """Battery as a task-level joule budget: remaining = budget - consumed."""
        capacity = self.battery_capacity_j
        battery_energy_j = self.initial_battery * capacity - self.energy_j
        if capacity > 0.0:
            self.battery = max(0.0, min(1.0, battery_energy_j / capacity))
        else:
            self.battery = 0.0
        try:
            self.battery_pub.publish(Float32(data=self.battery))
            self.battery_j_pub.publish(Float32(data=max(0.0, battery_energy_j)))
        except rospy.ROSException:
            pass  # publishing may fail during shutdown; battery state still recorded

    def update_rotors_budget(self):
        remaining = max(
            0.0, self.rotors_mechanical_budget_j - self.rotors_mechanical_energy_j
        )
        try:
            self.rotors_mechanical_remaining_pub.publish(Float32(data=remaining))
            self.rotors_mechanical_consumed_pub.publish(Float32(
                data=self.rotors_mechanical_energy_j
            ))
        except rospy.ROSException:
            pass
        return remaining

    # ── energy + collision update ───────────────────────────────────────────────

    def update_energy(self, stamp, x, y, z):
        if self.last_energy_time is None:
            self.last_energy_time = stamp
            self.last_energy_pose = (x, y, z)
            return

        dt = (stamp - self.last_energy_time).to_sec()
        if dt <= 0.0:
            return
        dt = min(dt, 1.0)

        last_x, last_y, last_z = self.last_energy_pose
        vx = (x - last_x) / dt
        vy = (y - last_y) / dt
        vz = (z - last_z) / dt
        vxy = math.hypot(vx, vy)

        if self.mode in ("TAKEOFF", "AIR", "LAND"):
            thrust_proxy = (
                self.hover_thrust_proxy
                + self.vertical_thrust_gain * abs(vz)
                + self.horizontal_thrust_gain * vxy
            )
            self.power_w = self.fly_k1 * thrust_proxy * thrust_proxy
            segment_energy = self.power_w * dt
            if self.mode == "TAKEOFF":
                self.takeoff_energy_j += segment_energy
            elif self.mode == "LAND":
                self.land_energy_j += segment_energy
            else:
                self.air_energy_j += segment_energy
        else:
            actual_cmd_speed = math.hypot(self.cmd.linear.x, self.cmd.linear.y)
            nominal_cmd_speed = math.hypot(self.nominal_cmd.linear.x, self.nominal_cmd.linear.y)
            cmd_speed = max(actual_cmd_speed, nominal_cmd_speed)
            self.power_w = self.ground_k2 * cmd_speed
            segment_energy = self.power_w * dt
            self.ground_energy_j += segment_energy

            if cmd_speed > 0.1:
                self.current_slip_ratio = max(0.0, min(1.0, (cmd_speed - vxy) / cmd_speed))
                self.slip_ratio_time_sum += self.current_slip_ratio * dt
                self.slip_ratio_weight_sum += dt
            else:
                self.current_slip_ratio = 0.0

            # stuck / collision detection: commanded intent but poor motion response.
            if cmd_speed > 0.1 and (vxy < 0.02 or self.current_slip_ratio > 0.85):
                self.stuck_duration += dt
                if self.stuck_duration >= 0.5:
                    self.stuck_count += 1
                    self.collision_count += 1
                    self.stuck_duration = 0.0
            else:
                self.stuck_duration = 0.0

        self.energy_j += segment_energy
        self.last_energy_time = stamp
        self.last_energy_pose = (x, y, z)

    def flight_phase(self):
        explicit = self.energy_phase_override
        aliases = {
            "TRANSLATION_OUT": "TRANSLATION",
            "TRANSLATE": "TRANSLATION",
            "LAND": "LANDING",
        }
        explicit = aliases.get(explicit, explicit)
        if explicit in self.rotors_phase_energy_j or explicit == "GROUND":
            return explicit
        if self.mode == "TAKEOFF":
            return "TAKEOFF"
        if self.mode == "LAND":
            return "LANDING"
        if self.mode == "AIR":
            vx, vy, _ = self.rotors_velocity
            return "HOVER" if math.hypot(vx, vy) < 0.15 else "TRANSLATION"
        return "GROUND"

    def update_rotors_energy(self, stamp, x, y, z):
        self.rotors_mechanical_power_w = 0.0
        if self.air_backend != "rotors" or not self.rotors_motor_actual_received:
            return
        if self.rotors_last_time is None:
            self.rotors_last_time = stamp
            self.rotors_last_pose = (x, y, z)
            return

        dt = (stamp - self.rotors_last_time).to_sec()
        if dt <= 0.0:
            return
        dt = min(dt, 1.0)
        omegas = [abs(value) for value in self.rotors_motor_actual]
        self.rotors_mechanical_power_w = (
            self.rotor_force_constant
            * self.rotor_moment_constant
            * sum(value ** 3 for value in omegas)
        )
        segment_energy = self.rotors_mechanical_power_w * dt
        self.rotors_mechanical_energy_j += segment_energy

        phase = self.flight_phase()
        if phase in self.rotors_phase_energy_j:
            self.rotors_trial_mechanical_energy_j += segment_energy
            self.rotors_trial_flight_duration_s += dt
            self.rotors_phase_energy_j[phase] += segment_energy
            self.rotors_phase_duration_s[phase] += dt
            last_x, last_y, last_z = self.rotors_last_pose
            self.rotors_trial_distance_m += math.sqrt(
                (x - last_x) ** 2 + (y - last_y) ** 2 + (z - last_z) ** 2
            )

        self.rotors_last_time = stamp
        self.rotors_last_pose = (x, y, z)

    # ── main write loop ────────────────────────────────────────────────────────

    def write_row(self):
        if self.odom is None and self.model_pose is None:
            return

        pose = self.model_pose if self.model_pose is not None else self.odom.pose.pose
        stamp = rospy.Time.now()
        elapsed = (stamp - self.start_time).to_sec()

        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        roll, pitch, yaw = euler_from_quaternion(pose.orientation)

        goal_x = self.goal.pose.position.x if self.goal is not None else 0.0
        goal_y = self.goal.pose.position.y if self.goal is not None else 0.0

        self.update_energy(stamp, x, y, z)
        self.update_rotors_energy(stamp, x, y, z)
        self.update_battery()
        rotors_mechanical_remaining_j = self.update_rotors_budget()

        # success / timeout flags
        success = 1 if self.mission_state == "DONE" else 0
        if not self.timed_out and not success and elapsed >= self.mission_timeout_sec:
            self.timed_out = True
        timeout_flag = 1 if self.timed_out else 0

        # final_distance: Euclidean distance to current goal (xy only)
        final_distance = 0.0
        if self.goal is not None:
            final_distance = math.hypot(goal_x - x, goal_y - y)

        slip_ratio = 0.0
        if self.slip_ratio_weight_sum > 0.0:
            slip_ratio = self.slip_ratio_time_sum / self.slip_ratio_weight_sum
        reposition_goal_x = float("nan")
        reposition_goal_y = float("nan")
        reposition_goal_yaw = float("nan")
        if self.takeoff_reposition_goal is not None:
            reposition_goal_x = self.takeoff_reposition_goal.pose.position.x
            reposition_goal_y = self.takeoff_reposition_goal.pose.position.y
            _, _, reposition_goal_yaw = euler_from_quaternion(
                self.takeoff_reposition_goal.pose.orientation)

        row = [
            self.scene_tag,
            self.seed,
            f"{elapsed:.3f}",
            self.mode,
            self.mission_state,
            f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
            f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}",
            f"{goal_x:.4f}", f"{goal_y:.4f}",
            f"{self.cmd.linear.x:.4f}", f"{self.cmd.linear.y:.4f}", f"{self.cmd.angular.z:.4f}",
            f"{self.slope:.2f}",
            f"{self.obstacle_density:.3f}",
            f"{self.traversability_mean:.3f}",
            f"{self.traversability_true:.3f}",
            f"{self.traversability_uncertainty:.3f}",
            f"{self.terrain_cost:.3f}",
            f"{self.visibility_confidence:.3f}",
            f"{self.stuck_risk_prior:.3f}",
            f"{self.j_ground:.4f}",
            f"{self.j_air:.4f}",
            f"{self.risk_cvar:.4f}",
            f"{self.energy_term:.4f}",
            self.switch_reason,
            f"{self.battery:.3f}",
            f"{self.power_w:.4f}",
            f"{self.energy_j:.4f}",
            f"{self.ground_energy_j:.4f}",
            f"{self.air_energy_j:.4f}",
            f"{self.takeoff_energy_j:.4f}",
            f"{self.land_energy_j:.4f}",
            f"{self.switch_energy_j:.4f}",
            f"{self.path_length:.4f}",
            f"{self.ground_distance_m:.4f}",
            f"{self.air_distance_m:.4f}",
            self.switch_count,
            self.collision_count,
            self.stuck_count,
            self.stuck_count,
            f"{slip_ratio:.4f}",
            self.shield_override_count,
            self.override_reason,
            f"{self.safe_landing_margin_j:.4f}",
            self.unsafe_action_count,
            self.irreversible_failure,
            self.entered_unrecoverable_set,
            f"{self.recoverability_margin_min:.4f}",
            f"{final_distance:.4f}",
            success,
            timeout_flag,
            self.air_backend,
            self.legacy_energy_model,
            self.energy_model,
            self.flight_phase(),
            self.calibration_trial_id,
            self.calibration_split,
            *[f"{value:.6f}" for value in self.rotors_motor_command],
            *[f"{value:.6f}" for value in self.rotors_motor_actual],
            f"{sum(abs(value) for value in self.rotors_motor_command) / 4.0:.6f}",
            f"{sum(abs(value) for value in self.rotors_motor_actual) / 4.0:.6f}",
            f"{self.rotor_force_constant * sum(value * value for value in self.rotors_motor_actual):.6f}",
            f"{self.rotors_mechanical_power_w:.6f}",
            f"{self.rotors_mechanical_energy_j:.6f}",
            f"{self.rotors_trial_mechanical_energy_j:.6f}",
            f"{self.rotors_trial_flight_duration_s:.6f}",
            f"{self.rotors_trial_distance_m:.6f}",
            f"{self.rotors_phase_energy_j['TAKEOFF']:.6f}",
            f"{self.rotors_phase_energy_j['HOVER']:.6f}",
            f"{self.rotors_phase_energy_j['TRANSLATION']:.6f}",
            f"{self.rotors_phase_energy_j['RETURN']:.6f}",
            f"{self.rotors_phase_energy_j['LANDING']:.6f}",
            f"{self.rotors_phase_duration_s['TAKEOFF']:.6f}",
            f"{self.rotors_phase_duration_s['HOVER']:.6f}",
            f"{self.rotors_phase_duration_s['TRANSLATION']:.6f}",
            f"{self.rotors_phase_duration_s['RETURN']:.6f}",
            f"{self.rotors_phase_duration_s['LANDING']:.6f}",
            f"{z:.6f}",
            f"{self.rotors_velocity[0]:.6f}",
            f"{self.rotors_velocity[1]:.6f}",
            f"{self.rotors_velocity[2]:.6f}",
            f"{math.sqrt(sum(value * value for value in self.rotors_velocity)):.6f}",
            f"{self.rotor_force_constant:.9g}",
            f"{self.rotor_moment_constant:.9g}",
            f"{self.rotor_velocity_slowdown:.9g}",
            int(
                self.air_backend != "rotors"
                or (
                    self.rotors_motor_command_received
                    and self.rotors_motor_actual_received
                    and self.valid_motor_vector(self.rotors_motor_command)
                    and self.valid_motor_vector(self.rotors_motor_actual)
                )
            ),
            self.supervisor_energy_backend,
            self.supervisor_energy_model,
            f"{self.supervisor_predicted_consumption_j:.6f}",
            f"{(self.rotors_mechanical_energy_j if self.air_backend == 'rotors' else self.energy_j):.6f}",
            f"{self.supervisor_reserve_j:.6f}",
            self.supervisor_block_reason,
            f"{self.rotors_mechanical_budget_j:.6f}",
            f"{rotors_mechanical_remaining_j:.6f}",
            int(self.takeoff_measurement_valid),
            int(self.takeoff_site_valid),
            f"{self.takeoff_clearance_available_m:.6f}",
            f"{self.takeoff_clearance_required_m:.6f}",
            f"{self.takeoff_nearest_obstacle_m:.6f}",
            self.takeoff_site_reason,
            self.takeoff_decision_reason,
            f"{self.takeoff_blocked_duration_s:.6f}",
            f"{self.takeoff_blocked_distance_m:.6f}",
            int(self.rotors_obstacle_contact),
            f"{self.rotors_obstacle_contact_force_n:.6f}",
            self.rotors_handoff_state,
            int(self.takeoff_reposition_active),
            f"{reposition_goal_x:.6f}",
            f"{reposition_goal_y:.6f}",
            f"{reposition_goal_yaw:.6f}",
            f"{self.takeoff_reposition_duration_s:.6f}",
            f"{self.takeoff_reposition_distance_m:.6f}",
            f"{self.takeoff_reposition_initial_distance_m:.6f}",
            f"{self.takeoff_reposition_target_history_age_s:.6f}",
            self.takeoff_reposition_result,
            self.persistent_safe_site_count,
            self.reposition_target_site_id,
            self.reposition_target_source,
            self.reposition_target_reachability,
            f"{self.reposition_target_observation_age_s:.6f}",
            f"{self.reposition_target_clearance_available_m:.6f}",
            f"{self.reposition_target_clearance_required_m:.6f}",
            f"{self.reposition_target_nearest_obstacle_m:.6f}",
            int(self.reposition_target_revalidated),
        ]
        self._last_row = row
        for writer in self.writers:
            writer.writerow(row)
        for file_obj in self.files:
            file_obj.flush()

    def close(self):
        # A run shut down (roslaunch killed at the run budget) without ever
        # reaching DONE is a failure-by-timeout. Re-emit the last row with
        # timeout=1 so failed baselines are explicitly timeout=1, not merely
        # success=0 — robust to Gazebo's variable real-time factor and to rospy
        # shutdown (no rospy calls here: we just copy the cached final row).
        try:
            if self.mission_state != "DONE" and self._last_row is not None:
                final = list(self._last_row)
                final[self.header.index("timeout")] = 1
                final[self.header.index("success")] = 0
                for writer in self.writers:
                    writer.writerow(final)
                for file_obj in self.files:
                    file_obj.flush()
        except Exception as exc:  # pragma: no cover - best effort during shutdown
            rospy.logwarn("RescueMetricsLogger: final timeout row failed: %s", exc)
        for file_obj in self.files:
            if not file_obj.closed:
                file_obj.close()
        rospy.loginfo("RescueMetricsLogger: closed %s", self.csv_path)

    def spin(self):
        rate = rospy.Rate(rospy.get_param("~update_rate", 5.0))
        while not rospy.is_shutdown():
            self.write_row()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rescue_metrics_logger")
    RescueMetricsLogger().spin()
