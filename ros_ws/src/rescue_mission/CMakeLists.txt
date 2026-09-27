cmake_minimum_required(VERSION 3.0.2)
project(rescue_mission)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  gazebo_msgs
  geometry_msgs
  mav_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs gazebo_msgs geometry_msgs mav_msgs nav_msgs rospy std_msgs
)

catkin_install_python(PROGRAMS
  scripts/air_goal_bridge.py
  scripts/ground_goal_controller.py
  scripts/air_motion_executor.py
  scripts/air_mode_stub.py
  scripts/mode_switcher.py
  scripts/mission_state_machine.py
  scripts/rescue_metrics_logger.py
  scripts/rotors_motor_gate.py
  scripts/rotors_handoff_coordinator.py
  scripts/safe_site_memory.py
  scripts/rotors_odometry_bridge.py
  scripts/analyze_rule_transitions.py
  scripts/check_shared_inputs.py
  scripts/export_summary_table.py
  scripts/plot_results.py
  scripts/run_phase1_experiments.py
  scripts/safety_shield.py
  scripts/scenario_sensor_sim.py
  scripts/summarize_results.py
  scripts/validate_rescue_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
