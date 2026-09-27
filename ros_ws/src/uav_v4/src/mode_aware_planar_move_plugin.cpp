#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <boost/bind.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace gazebo {

class ModeAwarePlanarMovePlugin : public ModelPlugin {
 public:
  ModeAwarePlanarMovePlugin() = default;

  ~ModeAwarePlanarMovePlugin() override {
    update_connection_.reset();
    callback_queue_.clear();
    callback_queue_.disable();
    if (ros_node_) {
      ros_node_->shutdown();
    }
    if (callback_thread_.joinable()) {
      callback_thread_.join();
    }
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    if (!ros::isInitialized()) {
      gzerr << "ModeAwarePlanarMovePlugin requires gazebo_ros_api_plugin.\n";
      return;
    }

    command_topic_ = ReadString(sdf, "commandTopic", "/mecanum/cmd_vel");
    mode_topic_ = ReadString(sdf, "modeTopic", "/rescue/mode");
    disarmed_topic_ = ReadString(sdf, "disarmedTopic", "/rescue/rotors/disarmed");
    ground_ownership_topic_ = ReadString(
        sdf, "groundOwnershipTopic", "/rescue/rotors/ground_ownership");
    ground_active_topic_ = ReadString(
        sdf, "groundActiveTopic", "/rescue/rotors/ground_control_active");
    ground_wrench_topic_ = ReadString(
        sdf, "groundWrenchTopic", "/rescue/rotors/ground_control_wrench");
    command_timeout_ = ReadDouble(sdf, "commandTimeout", 0.5);
    planar_velocity_gain_ = ReadDouble(sdf, "planarVelocityGain", 32.0);
    max_planar_acceleration_ = ReadDouble(sdf, "maxPlanarAcceleration", 1.5);
    ground_height_ = ReadDouble(sdf, "groundHeight", 0.044);
    vertical_kp_ = ReadDouble(sdf, "verticalKp", 800.0);
    vertical_kd_ = ReadDouble(sdf, "verticalKd", 50.0);
    max_vertical_force_ = ReadDouble(sdf, "maxVerticalForce", 29.83);
    level_kp_ = ReadDouble(sdf, "levelKp", 1.0);
    level_kd_ = ReadDouble(sdf, "levelKd", 0.15);
    max_level_torque_ = ReadDouble(sdf, "maxLevelTorque", 0.6);
    yaw_rate_gain_ = ReadDouble(sdf, "yawRateGain", 8.0);
    max_yaw_torque_ = ReadDouble(sdf, "maxYawTorque", 0.06);
    wheel_proxy_friction_ = ReadDouble(sdf, "wheelProxyFriction", 0.05);

    base_link_ = model_->GetLink("uav_v4/base_link");
    if (!base_link_) {
      gzerr << "ModeAwarePlanarMovePlugin cannot find uav_v4/base_link.\n";
      return;
    }
    total_mass_ = 0.0;
    for (const auto& link : model_->GetLinks()) {
      if (link && link->GetInertial()) {
        total_mass_ += link->GetInertial()->Mass();
      }
    }
    if (!(total_mass_ > 0.0)) {
      gzerr << "ModeAwarePlanarMovePlugin resolved non-positive mass.\n";
      return;
    }
    for (const auto& collision : base_link_->GetCollisions()) {
      if (!collision || collision->GetName().find("wheel_") == std::string::npos ||
          !collision->GetSurface() ||
          !collision->GetSurface()->FrictionPyramid()) {
        continue;
      }
      collision->GetSurface()->FrictionPyramid()->SetMuPrimary(
          wheel_proxy_friction_);
      collision->GetSurface()->FrictionPyramid()->SetMuSecondary(
          wheel_proxy_friction_);
    }

    const char* diagnostic_path = std::getenv("GROUND_DRIVE_DIAG_CSV");
    if (diagnostic_path && diagnostic_path[0] != '\0') {
      diagnostic_csv_.open(diagnostic_path, std::ios::out | std::ios::trunc);
      diagnostic_csv_ << std::setprecision(12)
                      << "sim_time_s,active,mode,disarmed,cmd_x_m_s,"
                         "cmd_y_m_s,cmd_yaw_rad_s,force_x_n,force_y_n,"
                         "force_z_n,torque_x_nm,torque_y_nm,torque_z_nm,"
                         "power_proxy_w,cumulative_work_proxy_j,"
                         "cumulative_positive_work_proxy_j\n";
    }

    ros_node_.reset(new ros::NodeHandle(""));
    ground_active_pub_ = ros_node_->advertise<std_msgs::Bool>(
        ground_active_topic_, 1);
    ground_wrench_pub_ = ros_node_->advertise<geometry_msgs::WrenchStamped>(
        ground_wrench_topic_, 1);
    ros::SubscribeOptions command_options = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        command_topic_, 1,
        boost::bind(&ModeAwarePlanarMovePlugin::CommandCallback, this, _1),
        ros::VoidPtr(), &callback_queue_);
    ros::SubscribeOptions mode_options = ros::SubscribeOptions::create<std_msgs::String>(
        mode_topic_, 1,
        boost::bind(&ModeAwarePlanarMovePlugin::ModeCallback, this, _1),
        ros::VoidPtr(), &callback_queue_);
    ros::SubscribeOptions disarmed_options = ros::SubscribeOptions::create<std_msgs::Bool>(
        disarmed_topic_, 1,
        boost::bind(&ModeAwarePlanarMovePlugin::DisarmedCallback, this, _1),
        ros::VoidPtr(), &callback_queue_);
    ros::SubscribeOptions ownership_options = ros::SubscribeOptions::create<std_msgs::Bool>(
        ground_ownership_topic_, 1,
        boost::bind(&ModeAwarePlanarMovePlugin::OwnershipCallback, this, _1),
        ros::VoidPtr(), &callback_queue_);
    command_sub_ = ros_node_->subscribe(command_options);
    mode_sub_ = ros_node_->subscribe(mode_options);
    disarmed_sub_ = ros_node_->subscribe(disarmed_options);
    ownership_sub_ = ros_node_->subscribe(ownership_options);

    callback_thread_ = std::thread(&ModeAwarePlanarMovePlugin::QueueThread, this);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModeAwarePlanarMovePlugin::OnUpdate, this,
                  std::placeholders::_1));
    gzmsg << "ModeAwarePlanarMovePlugin loaded for " << model_->GetName()
          << "; finite-force ground ownership requires GROUND and disarmed; "
          << "mass=" << total_mass_ << " kg, max planar force="
          << total_mass_ * max_planar_acceleration_ << " N.\n";
  }

 private:
  static std::string ReadString(const sdf::ElementPtr& sdf,
                                const std::string& name,
                                const std::string& fallback) {
    return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
  }

  static double ReadDouble(const sdf::ElementPtr& sdf,
                           const std::string& name,
                           double fallback) {
    return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
  }

  void CommandCallback(const geometry_msgs::TwistConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = *msg;
    last_command_time_ = ros::Time::now();
  }

  void ModeCallback(const std_msgs::StringConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = msg->data;
  }

  void DisarmedCallback(const std_msgs::BoolConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    disarmed_ = msg->data;
  }

  void OwnershipCallback(const std_msgs::BoolConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    ownership_override_received_ = true;
    ground_ownership_allowed_ = msg->data;
  }

  void QueueThread() {
    static const double kTimeoutSeconds = 0.01;
    while (ros_node_ && ros_node_->ok()) {
      callback_queue_.callAvailable(ros::WallDuration(kTimeoutSeconds));
    }
  }

  static double Clamp(double value, double limit) {
    return std::max(-limit, std::min(limit, value));
  }

  static ignition::math::Vector3d ClampPlanar(
      const ignition::math::Vector3d& value, double limit) {
    const double norm = std::hypot(value.X(), value.Y());
    if (norm <= limit || norm <= 0.0) {
      return value;
    }
    return ignition::math::Vector3d(
        value.X() * limit / norm, value.Y() * limit / norm, value.Z());
  }

  void WriteDiagnostic(const common::Time& sim_time, bool active,
                       const std::string& mode, bool disarmed,
                       const geometry_msgs::Twist& command,
                       const ignition::math::Vector3d& force,
                       const ignition::math::Vector3d& torque,
                       double power, double dt) {
    if (!diagnostic_csv_.is_open()) {
      return;
    }
    if (active && dt > 0.0) {
      cumulative_work_proxy_ += power * dt;
      cumulative_positive_work_proxy_ += std::max(0.0, power * dt);
    }
    diagnostic_csv_ << sim_time.Double() << ',' << (active ? 1 : 0) << ','
                    << mode << ',' << (disarmed ? 1 : 0) << ','
                    << command.linear.x << ',' << command.linear.y << ','
                    << command.angular.z << ',' << force.X() << ','
                    << force.Y() << ',' << force.Z() << ',' << torque.X()
                    << ',' << torque.Y() << ',' << torque.Z() << ',' << power
                    << ',' << cumulative_work_proxy_ << ','
                    << cumulative_positive_work_proxy_ << '\n';
  }

  void OnUpdate(const common::UpdateInfo& info) {
    geometry_msgs::Twist command;
    bool owns_ground = false;
    bool disarmed = false;
    std::string mode;
    ros::Time last_command_time;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      owns_ground = ownership_override_received_
                        ? ground_ownership_allowed_
                        : (mode_ == "GROUND" && disarmed_);
      mode = mode_;
      disarmed = disarmed_;
      command = command_;
      last_command_time = last_command_time_;
    }
    double dt = 0.0;
    if (last_update_time_ != common::Time::Zero) {
      dt = (info.simTime - last_update_time_).Double();
    }
    last_update_time_ = info.simTime;
    if (!owns_ground || !model_ || !base_link_) {
      PublishGroundTelemetry(info.simTime, false,
                             ignition::math::Vector3d::Zero,
                             ignition::math::Vector3d::Zero);
      WriteDiagnostic(info.simTime, false, mode, disarmed, command,
                      ignition::math::Vector3d::Zero,
                      ignition::math::Vector3d::Zero, 0.0, dt);
      return;
    }

    if (last_command_time.isZero() ||
        (ros::Time::now() - last_command_time).toSec() > command_timeout_) {
      command = geometry_msgs::Twist();
    }

    const double yaw = model_->WorldPose().Rot().Yaw();
    const double world_x = std::cos(yaw) * command.linear.x -
                           std::sin(yaw) * command.linear.y;
    const double world_y = std::sin(yaw) * command.linear.x +
                           std::cos(yaw) * command.linear.y;
    const ignition::math::Vector3d current_linear = model_->WorldLinearVel();
    ignition::math::Vector3d planar_force(
        total_mass_ * planar_velocity_gain_ * (world_x - current_linear.X()),
        total_mass_ * planar_velocity_gain_ * (world_y - current_linear.Y()),
        0.0);
    planar_force = ClampPlanar(
        planar_force, total_mass_ * max_planar_acceleration_);

    // Ground-plane ownership is finite and local: vertical spring/damping about
    // the nominal wheel-supported height, plus an attitude restoring torque.
    // No pose or velocity is projected. Gravity and collision constraints remain
    // active, and the upward correction cannot exceed one vehicle weight.
    const auto pose = model_->WorldPose();
    const auto angular = model_->WorldAngularVel();
    const double vertical_force = Clamp(
        -vertical_kp_ * (pose.Pos().Z() - ground_height_) -
            vertical_kd_ * current_linear.Z(),
        max_vertical_force_);
    ignition::math::Vector3d force(
        planar_force.X(), planar_force.Y(), vertical_force);

    const ignition::math::Vector3d body_up =
        pose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);
    ignition::math::Vector3d level_torque =
        level_kp_ * body_up.Cross(ignition::math::Vector3d::UnitZ) -
        level_kd_ * ignition::math::Vector3d(angular.X(), angular.Y(), 0.0);
    level_torque = ClampPlanar(level_torque, max_level_torque_);
    ignition::math::Vector3d torque(
        level_torque.X(), level_torque.Y(),
        Clamp(yaw_rate_gain_ * (command.angular.z - angular.Z()),
              max_yaw_torque_));

    base_link_->AddForce(force);
    base_link_->AddTorque(torque);
    PublishGroundTelemetry(info.simTime, true, force, torque);
    const double power = force.Dot(current_linear) + torque.Dot(angular);
    WriteDiagnostic(info.simTime, true, mode, disarmed, command, force, torque,
                    power, dt);
  }

  void PublishGroundTelemetry(const common::Time& sim_time, bool active,
                              const ignition::math::Vector3d& force,
                              const ignition::math::Vector3d& torque) {
    std_msgs::Bool active_msg;
    active_msg.data = active;
    ground_active_pub_.publish(active_msg);
    geometry_msgs::WrenchStamped wrench;
    wrench.header.stamp.fromSec(sim_time.Double());
    wrench.header.frame_id = "uav_v4/base_link";
    wrench.wrench.force.x = force.X();
    wrench.wrench.force.y = force.Y();
    wrench.wrench.force.z = force.Z();
    wrench.wrench.torque.x = torque.X();
    wrench.wrench.torque.y = torque.Y();
    wrench.wrench.torque.z = torque.Z();
    ground_wrench_pub_.publish(wrench);
  }

  physics::ModelPtr model_;
  physics::LinkPtr base_link_;
  event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Subscriber command_sub_;
  ros::Subscriber mode_sub_;
  ros::Subscriber disarmed_sub_;
  ros::Subscriber ownership_sub_;
  ros::Publisher ground_active_pub_;
  ros::Publisher ground_wrench_pub_;
  ros::CallbackQueue callback_queue_;
  std::thread callback_thread_;
  std::mutex mutex_;
  geometry_msgs::Twist command_;
  ros::Time last_command_time_;
  std::string mode_ = "GROUND";
  bool disarmed_ = true;
  bool ownership_override_received_ = false;
  bool ground_ownership_allowed_ = false;
  std::string command_topic_;
  std::string mode_topic_;
  std::string disarmed_topic_;
  std::string ground_ownership_topic_;
  std::string ground_active_topic_;
  std::string ground_wrench_topic_;
  double command_timeout_ = 0.5;
  double total_mass_ = 0.0;
  double planar_velocity_gain_ = 32.0;
  double max_planar_acceleration_ = 1.5;
  double ground_height_ = 0.044;
  double vertical_kp_ = 800.0;
  double vertical_kd_ = 50.0;
  double max_vertical_force_ = 29.83;
  double level_kp_ = 1.0;
  double level_kd_ = 0.15;
  double max_level_torque_ = 0.6;
  double yaw_rate_gain_ = 8.0;
  double max_yaw_torque_ = 0.06;
  double wheel_proxy_friction_ = 0.05;
  common::Time last_update_time_;
  std::ofstream diagnostic_csv_;
  double cumulative_work_proxy_ = 0.0;
  double cumulative_positive_work_proxy_ = 0.0;
};

GZ_REGISTER_MODEL_PLUGIN(ModeAwarePlanarMovePlugin)

}  // namespace gazebo
