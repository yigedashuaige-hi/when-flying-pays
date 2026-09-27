#!/usr/bin/env python3
import math
import os
import sys

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import terrain_field


class ScenarioSensorSim:
    def __init__(self):
        self.rate_hz = rospy.get_param("~update_rate", 10.0)
        # Seedable terrain randomness (paper-hardening). seed < 0 disables it and
        # falls back to the legacy deterministic structural perception, so older
        # runs/scenes are unchanged.
        self.seed = int(rospy.get_param("~seed", -1))
        self.cell_size = float(rospy.get_param("~cell_size", 0.5))
        self.theta_noise_scale = float(rospy.get_param("~theta_noise_scale", 0.15))
        self.percept_noise_scale = float(rospy.get_param("~percept_noise_scale", 0.10))
        # Heteroscedastic (calibrated) perception: when true, the actual p_trav
        # noise scale at a cell EQUALS the reported traversability_uncertainty, so
        # u_trav genuinely predicts the perception error |p_trav - theta_true|.
        # When false (default), noise is homoscedastic (constant percept_noise_scale)
        # and u_trav is an uncalibrated obstacle-driven proxy. See
        # analyze_uncertainty.py for the diagnosis motivating this toggle.
        self.heteroscedastic_noise = bool(rospy.get_param("~heteroscedastic_noise", False))
        # Battery is owned and published by rescue_metrics_logger (joule budget);
        # the sensor sim no longer publishes /rescue/battery to avoid a double
        # publisher. initial_battery now lives on the logger.
        self.base_obstacle_density = rospy.get_param("~base_obstacle_density", 0.05)
        self.slope_zone_deg = rospy.get_param("~slope_zone_deg", 35.0)
        self.barrier_obstacle_density = rospy.get_param("~barrier_obstacle_density", 0.85)
        self.barrier_x_min = rospy.get_param("~barrier_x_min", 6.2)
        self.barrier_x_max = rospy.get_param("~barrier_x_max", 7.8)
        self.barrier_y_half_width = rospy.get_param("~barrier_y_half_width", 1.4)
        self.slope_zone_x_min = rospy.get_param("~slope_zone_x_min", 3.5)
        self.slope_zone_x_max = rospy.get_param("~slope_zone_x_max", 4.8)
        self.base_traversability_uncertainty = rospy.get_param("~base_traversability_uncertainty", 0.05)
        self.visibility_obstacle_gain = rospy.get_param("~visibility_obstacle_gain", 0.45)
        self.uncertainty_obstacle_gain = rospy.get_param("~uncertainty_obstacle_gain", 0.30)
        self.uncertainty_slope_gain = rospy.get_param("~uncertainty_slope_gain", 0.20)
        self.use_time_script = rospy.get_param("~use_time_script", True)
        self.use_position_script = rospy.get_param("~use_position_script", False)
        self.slope_event_start = rospy.get_param("~slope_event_start", 8.0)
        self.slope_event_end = rospy.get_param("~slope_event_end", 14.0)
        self.obstacle_event_start = rospy.get_param("~obstacle_event_start", 18.0)
        self.obstacle_event_end = rospy.get_param("~obstacle_event_end", 28.0)

        self.odom = None
        self.start_time = rospy.Time.now()

        self.slope_pub = rospy.Publisher("/rescue/terrain_slope_deg", Float32, queue_size=10)
        self.obstacle_pub = rospy.Publisher("/rescue/obstacle_density", Float32, queue_size=10)
        self.traversability_mean_pub = rospy.Publisher("/rescue/traversability_mean", Float32, queue_size=10)
        # Ground-truth traversability (physics + logger only; never seen by a
        # strategy). Equals the perceived mean when seeding is disabled.
        self.traversability_true_pub = rospy.Publisher("/rescue/traversability_true", Float32, queue_size=10)
        self.traversability_uncertainty_pub = rospy.Publisher(
            "/rescue/traversability_uncertainty", Float32, queue_size=10
        )
        self.terrain_cost_pub = rospy.Publisher("/rescue/terrain_cost", Float32, queue_size=10)
        self.visibility_confidence_pub = rospy.Publisher(
            "/rescue/visibility_confidence", Float32, queue_size=10
        )
        self.stuck_risk_prior_pub = rospy.Publisher("/rescue/stuck_risk_prior", Float32, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)

    def odom_cb(self, msg):
        self.odom = msg

    @staticmethod
    def clamp(value, lo=0.0, hi=1.0):
        return max(lo, min(hi, value))

    def estimate_shared_inputs(self, slope, obstacle_density):
        slope_norm = self.clamp(slope / max(self.slope_zone_deg, 1.0))
        terrain_cost = self.clamp(0.65 * obstacle_density + 0.35 * slope_norm)
        traversability_mean = self.clamp(1.0 - terrain_cost)
        traversability_uncertainty = self.clamp(
            self.base_traversability_uncertainty
            + self.uncertainty_obstacle_gain * obstacle_density
            + self.uncertainty_slope_gain * slope_norm
        )
        visibility_confidence = self.clamp(
            1.0
            - self.visibility_obstacle_gain * obstacle_density
            - 0.30 * traversability_uncertainty
        )
        stuck_risk_prior = self.clamp(
            (1.0 - traversability_mean)
            + 0.50 * traversability_uncertainty
        )
        return (
            traversability_mean,
            traversability_uncertainty,
            terrain_cost,
            visibility_confidence,
            stuck_risk_prior,
        )

    def estimate_inputs(self):
        slope = 0.0
        obstacle_density = self.base_obstacle_density
        elapsed = max(0.0, (rospy.Time.now() - self.start_time).to_sec())

        if self.use_position_script and self.odom is not None:
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            if self.slope_zone_x_min <= x <= self.slope_zone_x_max:
                slope = self.slope_zone_deg
            if self.barrier_x_min <= x <= self.barrier_x_max and abs(y) <= self.barrier_y_half_width:
                obstacle_density = self.barrier_obstacle_density

        if self.use_time_script:
            if self.slope_event_start <= elapsed <= self.slope_event_end:
                slope = max(slope, self.slope_zone_deg)
            if self.obstacle_event_start <= elapsed <= self.obstacle_event_end:
                obstacle_density = max(obstacle_density, self.barrier_obstacle_density)

        shared_inputs = self.estimate_shared_inputs(slope, obstacle_density)
        structural_trav = shared_inputs[0]
        u_trav = shared_inputs[1]

        # Seedable truth/perception split. structural_trav is the noise-free
        # mean; theta_true adds seed-keyed cell noise (ground truth) and the
        # published mean becomes a noisy estimate of it. Under heteroscedastic
        # (calibrated) mode the noise scale equals u_trav, so u_trav predicts the
        # actual error; otherwise it is a constant (homoscedastic) scale.
        if self.seed >= 0 and self.odom is not None:
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            theta = terrain_field.theta_true(
                self.seed, x, y, structural_trav, self.cell_size, self.theta_noise_scale
            )
            noise_scale = u_trav if self.heteroscedastic_noise else self.percept_noise_scale
            p_trav = terrain_field.perceived_trav(
                self.seed, x, y, theta, self.cell_size, noise_scale
            )
        else:
            theta = structural_trav
            p_trav = structural_trav

        return slope, obstacle_density, shared_inputs, theta, p_trav

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            slope, obstacle_density, shared_inputs, theta_true, p_trav = self.estimate_inputs()
            (
                _structural_trav,
                traversability_uncertainty,
                terrain_cost,
                visibility_confidence,
                stuck_risk_prior,
            ) = shared_inputs
            self.slope_pub.publish(Float32(data=slope))
            self.obstacle_pub.publish(Float32(data=obstacle_density))
            # Strategies observe the noisy estimate p_trav; physics/logger get theta_true.
            self.traversability_mean_pub.publish(Float32(data=p_trav))
            self.traversability_true_pub.publish(Float32(data=theta_true))
            self.traversability_uncertainty_pub.publish(Float32(data=traversability_uncertainty))
            self.terrain_cost_pub.publish(Float32(data=terrain_cost))
            self.visibility_confidence_pub.publish(Float32(data=visibility_confidence))
            self.stuck_risk_prior_pub.publish(Float32(data=stuck_risk_prior))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("scenario_sensor_sim")
    ScenarioSensorSim().spin()
