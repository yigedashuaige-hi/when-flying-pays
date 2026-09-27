# ROS / Gazebo Environment Notes

## Current Working Stack

The current rescue simulation is running inside Docker:

```text
Container: uav_noetic
ROS: ROS 1 Noetic
OS base: Ubuntu 20.04 / focal packages
Gazebo: Gazebo Classic 11.15.1
ROS-Gazebo bridge: gazebo_ros / gazebo_msgs / gazebo_plugins for Noetic
```

This is the stack used by:

- `roslaunch rescue_mission rescue_phase1_demo.launch`
- `roslaunch rescue_worlds disaster_gazebo.launch`
- `gazebo_ros` spawn model nodes
- `/gazebo/model_states`
- `/gazebo/set_model_state`
- `gazebo_msgs`

The host machine is different:

```text
Host ROS: ROS 2 Jazzy
Host Gazebo: Gazebo Sim 8.11.0 (Harmonic generation)
Host Gazebo Classic: not installed
```

## Why the Project Runs Despite the Host Being ROS 2

The ROS 1 / Gazebo Classic environment is isolated in Docker. The host only
provides:

- file mounting for `$CATKIN_WS`
- display/X11 access for Gazebo GUI when enabled
- GPU/runtime support when available

All ROS 1 packages, catkin builds, Gazebo Classic plugins, and ROS topics live
inside the container.

## Should We Move to Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic Now?

Recommendation: **do not migrate the main branch now**.

Reasons:

- The existing codebase is ROS 1 catkin, not ROS 2 ament.
- `gazebo_ros` Classic APIs are used directly.
- The current SDF/URDF spawning pipeline depends on Gazebo Classic services.
- RotorS and the existing MAV/Noetic stack are ROS 1 oriented.
- Migrating now would delay experiments and paper iteration.

ROS 2 Jazzy + Gazebo Harmonic is more modern, but it would require a separate
porting track:

| Current ROS 1 / Classic | ROS 2 / Harmonic equivalent |
|-------------------------|-----------------------------|
| catkin | colcon + ament |
| `rospy` | `rclpy` |
| `.launch` XML ROS 1 | ROS 2 Python/XML launch |
| `gazebo_ros` Classic plugins | `ros_gz` / Gazebo Sim systems |
| `/gazebo/model_states` | Gazebo Transport + `ros_gz_bridge` |
| `/gazebo/set_model_state` | Gazebo Sim entity pose service / custom system |
| RotorS Classic | PX4 SITL / Gazebo Sim or ROS 2-compatible stack |

## Recommended Strategy

Use two tracks:

1. **Paper/Demo track:** keep ROS 1 Noetic + Gazebo Classic 11 in Docker.
   This is the stable path for Scene A/B/C, metrics, safety shield, and the
   first paper.

2. **Modernization track:** later create a separate ROS 2 Jazzy + Gazebo
   Harmonic branch. Port only after the current method, metrics, and baselines
   are stable.

This avoids mixing research progress with a large middleware migration.
