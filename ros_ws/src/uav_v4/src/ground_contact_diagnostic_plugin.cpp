#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <boost/bind.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Inertial.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace gazebo {

namespace {

std::string CsvSafe(const std::string& value) {
  std::string result = value;
  std::replace(result.begin(), result.end(), ',', ';');
  std::replace(result.begin(), result.end(), '\n', ' ');
  return result;
}

struct Snapshot {
  ignition::math::Pose3d pose;
  ignition::math::Vector3d linear;
  ignition::math::Vector3d angular;
  double kinetic = 0.0;
  double potential = 0.0;
  double motor_max = 0.0;
  bool valid = false;
};

}  // namespace

class GroundContactDiagnosticPlugin : public WorldPlugin {
 public:
  GroundContactDiagnosticPlugin() = default;

  ~GroundContactDiagnosticPlugin() override {
    pre_connection_.reset();
    post_connection_.reset();
    end_connection_.reset();
    callback_queue_.clear();
    callback_queue_.disable();
    if (ros_node_) {
      ros_node_->shutdown();
    }
    if (callback_thread_.joinable()) {
      callback_thread_.join();
    }
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
    if (surface_csv_.is_open()) {
      surface_csv_.flush();
      surface_csv_.close();
    }
  }

  void Load(physics::WorldPtr world, sdf::ElementPtr) override {
    world_ = world;
    physics_ = world_->Physics();
    contact_manager_ = physics_->GetContactManager();
    const char* path_env = std::getenv("GROUND_CONTACT_DIAG_CSV");
    output_path_ = path_env ? path_env : "/tmp/ground_contact_diagnostic.csv";
    csv_.open(output_path_, std::ios::out | std::ios::trunc);
    surface_csv_.open(output_path_ + ".surfaces.csv",
                      std::ios::out | std::ios::trunc);
    csv_ << std::setprecision(12);
    surface_csv_ << std::setprecision(12);
    WriteHeader();
    WritePhysicsMetadata();

    if (!ros::isInitialized()) {
      gzerr << "GroundContactDiagnosticPlugin requires gazebo_ros_api_plugin.\n";
      return;
    }
    ros_node_.reset(new ros::NodeHandle(""));
    Subscribe<geometry_msgs::Twist>(
        "/mecanum/cmd_vel", &GroundContactDiagnosticPlugin::CommandCallback,
        command_sub_);
    Subscribe<std_msgs::String>(
        "/rescue/mode", &GroundContactDiagnosticPlugin::ModeCallback,
        mode_sub_);
    Subscribe<std_msgs::Bool>(
        "/rescue/rotors/disarmed",
        &GroundContactDiagnosticPlugin::DisarmedCallback, disarmed_sub_);
    Subscribe<std_msgs::Bool>(
        "/rescue/rotors/armed", &GroundContactDiagnosticPlugin::ArmedCallback,
        armed_sub_);
    callback_thread_ = std::thread(
        &GroundContactDiagnosticPlugin::QueueThread, this);

    // This connection is made by the world plugin before the model is spawned,
    // so it samples before the model's planar plugin at WorldUpdateBegin.
    pre_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GroundContactDiagnosticPlugin::OnPreBegin, this));
    end_connection_ = event::Events::ConnectWorldUpdateEnd(
        std::bind(&GroundContactDiagnosticPlugin::OnEnd, this));
  }

 private:
  template <typename MessageT, typename CallbackT>
  void Subscribe(const std::string& topic, CallbackT callback,
                 ros::Subscriber& subscriber) {
    ros::SubscribeOptions options = ros::SubscribeOptions::create<MessageT>(
        topic, 1, boost::bind(callback, this, _1), ros::VoidPtr(),
        &callback_queue_);
    subscriber = ros_node_->subscribe(options);
  }

  void CommandCallback(const geometry_msgs::TwistConstPtr& msg) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    command_ = *msg;
  }

  void ModeCallback(const std_msgs::StringConstPtr& msg) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    mode_ = msg->data;
  }

  void DisarmedCallback(const std_msgs::BoolConstPtr& msg) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    disarmed_ = msg->data;
  }

  void ArmedCallback(const std_msgs::BoolConstPtr& msg) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    armed_ = msg->data;
  }

  void QueueThread() {
    while (ros_node_ && ros_node_->ok()) {
      callback_queue_.callAvailable(ros::WallDuration(0.002));
    }
  }

  bool ResolveModel() {
    if (model_) {
      return true;
    }
    model_ = world_->ModelByName("uav_v4");
    if (!model_) {
      return false;
    }
    base_link_ = model_->GetLink("uav_v4/base_link");
    if (!base_link_) {
      gzerr << "GroundContactDiagnosticPlugin cannot find base link.\n";
      model_.reset();
      return false;
    }
    std::vector<std::string> collisions;
    for (const auto& link : model_->GetLinks()) {
      for (const auto& collision : link->GetCollisions()) {
        collisions.push_back(collision->GetScopedName());
        model_collision_names_.insert(collision->GetScopedName());
      }
    }
    if (!collisions.empty()) {
      contact_manager_->CreateFilter("phase_e1a_uav_v4", collisions);
    }
    WriteAllSurfaces();

    // Registered only after the spawned model and its planar plugin have
    // registered their callbacks. It samples immediately after that callback
    // and before ODE contact resolution. In Gate 0 this exposed the hard
    // SetLinearVel/SetAngularVel delta; after Gate 1 a finite AddForce has no
    // instantaneous state delta and its exact work proxy is logged separately.
    post_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GroundContactDiagnosticPlugin::OnPostBegin, this));
    gzmsg << "GroundContactDiagnosticPlugin logging to " << output_path_ << "\n";
    return true;
  }

  Snapshot Capture() const {
    Snapshot snapshot;
    if (!model_ || !base_link_) {
      return snapshot;
    }
    snapshot.pose = model_->WorldPose();
    snapshot.linear = model_->WorldLinearVel();
    snapshot.angular = model_->WorldAngularVel();
    const double gravity = std::abs(world_->Gravity().Z());
    for (const auto& link : model_->GetLinks()) {
      const auto inertial = link->GetInertial();
      if (!inertial) {
        continue;
      }
      const double mass = inertial->Mass();
      const auto linear = link->WorldCoGLinearVel();
      const auto omega_world = link->WorldAngularVel();
      const auto omega_body =
          link->WorldPose().Rot().RotateVectorReverse(omega_world);
      const auto inertia = inertial->MOI();
      const auto inertia_omega = inertia * omega_body;
      snapshot.kinetic += 0.5 * mass * linear.SquaredLength();
      snapshot.kinetic += 0.5 * omega_body.Dot(inertia_omega);
      snapshot.potential += mass * gravity * link->WorldCoGPose().Pos().Z();
    }
    for (int index = 0; index < 4; ++index) {
      const std::string joint_name =
          "uav_v4/rotor_" + std::to_string(index) + "_joint";
      const auto joint = model_->GetJoint(joint_name);
      if (joint) {
        snapshot.motor_max = std::max(
            snapshot.motor_max, std::abs(joint->GetVelocity(0)) * 10.0);
      }
    }
    snapshot.valid = true;
    return snapshot;
  }

  void OnPreBegin() {
    if (!ResolveModel()) {
      return;
    }
    pre_ = Capture();
    post_ = Snapshot();
    pre_time_ = world_->SimTime();
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      row_command_ = command_;
      row_mode_ = mode_;
      row_disarmed_ = disarmed_;
      row_armed_ = armed_;
    }
    const double yaw = pre_.pose.Rot().Yaw();
    expected_world_x_ = std::cos(yaw) * row_command_.linear.x -
                        std::sin(yaw) * row_command_.linear.y;
    expected_world_y_ = std::sin(yaw) * row_command_.linear.x +
                        std::cos(yaw) * row_command_.linear.y;
  }

  void OnPostBegin() {
    if (!model_) {
      return;
    }
    post_ = Capture();
  }

  void OnEnd() {
    if (!model_ || !pre_.valid) {
      return;
    }
    const Snapshot end = Capture();
    if (!end.valid) {
      return;
    }
    ContactSummary contacts = SummarizeContacts();
    const double plugin_delta = post_.valid
                                    ? (post_.kinetic + post_.potential) -
                                          (pre_.kinetic + pre_.potential)
                                    : std::numeric_limits<double>::quiet_NaN();
    const double physics_delta = post_.valid
                                     ? (end.kinetic + end.potential) -
                                           (post_.kinetic + post_.potential)
                                     : std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(plugin_delta)) {
      cumulative_plugin_work_ += plugin_delta;
      cumulative_positive_plugin_work_ += std::max(0.0, plugin_delta);
    }
    WriteRow(end, contacts, plugin_delta, physics_delta);
    ++step_;
  }

  struct ContactSummary {
    int pair_count = 0;
    int point_count = 0;
    double max_depth = 0.0;
    double max_force = 0.0;
    ignition::math::Vector3d total_force;
    std::string pairs;
    std::string points;
    std::string normals;
    std::string depths;
  };

  ContactSummary SummarizeContacts() const {
    ContactSummary summary;
    std::ostringstream pair_stream;
    std::ostringstream point_stream;
    std::ostringstream normal_stream;
    std::ostringstream depth_stream;
    const auto contacts = contact_manager_->GetContacts();
    for (const auto* contact : contacts) {
      if (!contact || !contact->collision1 || !contact->collision2) {
        continue;
      }
      const std::string name1 = contact->collision1->GetScopedName();
      const std::string name2 = contact->collision2->GetScopedName();
      if (model_collision_names_.count(name1) == 0 &&
          model_collision_names_.count(name2) == 0) {
        continue;
      }
      if (summary.pair_count++) {
        pair_stream << '|';
      }
      pair_stream << name1 << "--" << name2;
      for (int index = 0; index < contact->count; ++index) {
        if (summary.point_count++) {
          point_stream << '|';
          normal_stream << '|';
          depth_stream << '|';
        }
        const auto& point = contact->positions[index];
        const auto& normal = contact->normals[index];
        const double depth = contact->depths[index];
        const auto force = contact->wrench[index].body1Force;
        point_stream << point.X() << ':' << point.Y() << ':' << point.Z();
        normal_stream << normal.X() << ':' << normal.Y() << ':' << normal.Z();
        depth_stream << depth;
        summary.max_depth = std::max(summary.max_depth, depth);
        summary.max_force = std::max(summary.max_force, force.Length());
        summary.total_force += force;
      }
    }
    summary.pairs = pair_stream.str();
    summary.points = point_stream.str();
    summary.normals = normal_stream.str();
    summary.depths = depth_stream.str();
    return summary;
  }

  void WriteSnapshot(const Snapshot& snapshot) {
    const auto rpy = snapshot.pose.Rot().Euler();
    csv_ << ',' << snapshot.pose.Pos().X() << ',' << snapshot.pose.Pos().Y()
         << ',' << snapshot.pose.Pos().Z() << ',' << rpy.X() << ',' << rpy.Y()
         << ',' << rpy.Z() << ',' << snapshot.linear.X() << ','
         << snapshot.linear.Y() << ',' << snapshot.linear.Z() << ','
         << snapshot.angular.X() << ',' << snapshot.angular.Y() << ','
         << snapshot.angular.Z() << ',' << snapshot.kinetic << ','
         << snapshot.potential << ',' << snapshot.motor_max;
  }

  void WriteRow(const Snapshot& end, const ContactSummary& contacts,
                double plugin_delta, double physics_delta) {
    csv_ << step_ << ',' << pre_time_.Double() << ','
         << physics_->GetMaxStepSize() << ',' << CsvSafe(row_mode_) << ','
         << (row_disarmed_ ? 1 : 0) << ',' << (row_armed_ ? 1 : 0) << ','
         << row_command_.linear.x << ',' << row_command_.linear.y << ','
         << row_command_.angular.z << ',' << expected_world_x_ << ','
         << expected_world_y_;
    WriteSnapshot(pre_);
    WriteSnapshot(post_.valid ? post_ : pre_);
    WriteSnapshot(end);
    csv_ << ',' << plugin_delta << ',' << physics_delta << ','
         << cumulative_plugin_work_ << ',' << cumulative_positive_plugin_work_
         << ',' << contacts.pair_count << ',' << contacts.point_count << ','
         << contacts.max_depth << ',' << contacts.max_force << ','
         << contacts.total_force.X() << ',' << contacts.total_force.Y() << ','
         << contacts.total_force.Z() << ',' << CsvSafe(contacts.pairs) << ','
         << CsvSafe(contacts.points) << ',' << CsvSafe(contacts.normals) << ','
         << CsvSafe(contacts.depths) << '\n';
    csv_.flush();
  }

  void WriteHeader() {
    csv_ << "step,sim_time_s,max_step_size_s,mode,disarmed,armed,"
            "cmd_body_x_m_s,cmd_body_y_m_s,cmd_yaw_rad_s,"
            "planar_target_world_x_m_s,planar_target_world_y_m_s";
    for (const std::string stage : {"pre", "post", "end"}) {
      csv_ << ',' << stage << "_x_m," << stage << "_y_m," << stage
           << "_z_m," << stage << "_roll_rad," << stage << "_pitch_rad,"
           << stage << "_yaw_rad," << stage << "_vx_m_s," << stage
           << "_vy_m_s," << stage << "_vz_m_s," << stage << "_wx_rad_s,"
           << stage << "_wy_rad_s," << stage << "_wz_rad_s," << stage
           << "_kinetic_j," << stage << "_potential_j," << stage
           << "_motor_max_rad_s";
    }
    csv_ << ",plugin_delta_mechanical_j,physics_delta_mechanical_j,"
            "cumulative_plugin_work_proxy_j,"
            "cumulative_positive_plugin_work_proxy_j,contact_pair_count,"
            "contact_point_count,max_contact_depth_m,max_contact_force_n,"
            "contact_total_force_x_n,contact_total_force_y_n,"
            "contact_total_force_z_n,contact_pairs,contact_points_xyz,"
            "contact_normals_xyz,contact_depths_m\n";
  }

  void WritePhysicsMetadata() {
    surface_csv_ << "record_type,name,max_step_size_s,"
                    "real_time_update_rate,max_contacts,mu,mu2,fdir1_x,"
                    "fdir1_y,fdir1_z,kp,kd,cfm,erp,max_vel,min_depth,"
                    "bounce,restitution_threshold\n";
    surface_csv_ << "physics,world," << physics_->GetMaxStepSize() << ','
                 << physics_->GetRealTimeUpdateRate()
                 << ",,,,,,,,,,,,,,,\n";
  }

  void WriteCollisionSurface(const physics::CollisionPtr& collision) {
    const auto surface = collision->GetSurface();
    const auto friction = surface ? surface->FrictionPyramid() : nullptr;
    const auto ode = boost::dynamic_pointer_cast<physics::ODESurfaceParams>(surface);
    surface_csv_ << "collision," << CsvSafe(collision->GetScopedName()) << ",,,"
                 << collision->GetMaxContacts() << ',';
    if (friction) {
      surface_csv_ << friction->MuPrimary() << ',' << friction->MuSecondary()
                   << ',' << friction->direction1.X() << ','
                   << friction->direction1.Y() << ','
                   << friction->direction1.Z();
    } else {
      surface_csv_ << ",,,,";
    }
    if (ode) {
      surface_csv_ << ',' << ode->kp << ',' << ode->kd << ',' << ode->cfm
                   << ',' << ode->erp << ',' << ode->maxVel << ','
                   << ode->minDepth << ',' << ode->bounce << ','
                   << ode->bounceThreshold;
    } else {
      surface_csv_ << ",,,,,,,,";
    }
    surface_csv_ << '\n';
  }

  void WriteModelSurfaces(const physics::ModelPtr& model) {
    if (!model) {
      return;
    }
    for (const auto& link : model->GetLinks()) {
      for (const auto& collision : link->GetCollisions()) {
        WriteCollisionSurface(collision);
      }
    }
  }

  void WriteAllSurfaces() {
    WriteModelSurfaces(model_);
    WriteModelSurfaces(world_->ModelByName("low_barrier"));
    WriteModelSurfaces(world_->ModelByName("ground_plane"));
    surface_csv_.flush();
  }

  physics::WorldPtr world_;
  physics::PhysicsEnginePtr physics_;
  physics::ContactManager* contact_manager_ = nullptr;
  physics::ModelPtr model_;
  physics::LinkPtr base_link_;
  event::ConnectionPtr pre_connection_;
  event::ConnectionPtr post_connection_;
  event::ConnectionPtr end_connection_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Subscriber command_sub_;
  ros::Subscriber mode_sub_;
  ros::Subscriber disarmed_sub_;
  ros::Subscriber armed_sub_;
  ros::CallbackQueue callback_queue_;
  std::thread callback_thread_;
  std::mutex input_mutex_;
  geometry_msgs::Twist command_;
  geometry_msgs::Twist row_command_;
  std::string mode_ = "GROUND";
  std::string row_mode_ = "GROUND";
  bool disarmed_ = true;
  bool armed_ = false;
  bool row_disarmed_ = true;
  bool row_armed_ = false;
  std::unordered_set<std::string> model_collision_names_;
  Snapshot pre_;
  Snapshot post_;
  common::Time pre_time_;
  double expected_world_x_ = 0.0;
  double expected_world_y_ = 0.0;
  double cumulative_plugin_work_ = 0.0;
  double cumulative_positive_plugin_work_ = 0.0;
  uint64_t step_ = 0;
  std::string output_path_;
  std::ofstream csv_;
  std::ofstream surface_csv_;
};

GZ_REGISTER_WORLD_PLUGIN(GroundContactDiagnosticPlugin)

}  // namespace gazebo
