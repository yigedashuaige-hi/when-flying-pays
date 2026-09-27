#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/physics.hh>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

namespace gazebo {

namespace {

double AxisGap(double a_min, double a_max, double b_min, double b_max) {
  if (a_max < b_min) {
    return b_min - a_max;
  }
  if (b_max < a_min) {
    return a_min - b_max;
  }
  return -std::min(a_max, b_max) + std::max(a_min, b_min);
}

bool FiniteBox(const ignition::math::AxisAlignedBox& box) {
  const auto& lo = box.Min();
  const auto& hi = box.Max();
  return std::isfinite(lo.X()) && std::isfinite(lo.Y()) &&
         std::isfinite(lo.Z()) && std::isfinite(hi.X()) &&
         std::isfinite(hi.Y()) && std::isfinite(hi.Z()) &&
         lo.X() <= hi.X() && lo.Y() <= hi.Y() && lo.Z() <= hi.Z();
}

double BoxDistance(const ignition::math::AxisAlignedBox& a,
                   const ignition::math::AxisAlignedBox& b) {
  const double dx = std::max(0.0, AxisGap(a.Min().X(), a.Max().X(),
                                          b.Min().X(), b.Max().X()));
  const double dy = std::max(0.0, AxisGap(a.Min().Y(), a.Max().Y(),
                                          b.Min().Y(), b.Max().Y()));
  const double dz = std::max(0.0, AxisGap(a.Min().Z(), a.Max().Z(),
                                          b.Min().Z(), b.Max().Z()));
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

// Simulator implementation of a replaceable takeoff-site measurement
// contract. It never commands the model. It conservatively checks the union of
// all robot collision AABBs, expanded by a configurable safety margin, through
// the complete vertical translation to the requested target altitude.
class TakeoffClearancePlugin : public ModelPlugin {
 public:
  ~TakeoffClearancePlugin() override {
    update_connection_.reset();
    if (ros_node_) {
      ros_node_->shutdown();
    }
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    world_ = model_->GetWorld();
    if (!ros::isInitialized()) {
      gzerr << "TakeoffClearancePlugin requires gazebo_ros_api_plugin.\n";
      return;
    }
    target_altitude_ = ReadDouble(sdf, "targetAltitude", 1.2);
    safety_margin_ = ReadDouble(sdf, "safetyMargin", 0.02);
    support_tolerance_ = ReadDouble(sdf, "supportSurfaceTolerance", 0.01);
    update_rate_ = ReadDouble(sdf, "updateRate", 50.0);
    status_topic_ = ReadString(
        sdf, "statusTopic", "/rescue/rotors/takeoff_clearance");
    atomic_status_topic_ = ReadString(
        sdf, "atomicStatusTopic", "/rescue/rotors/takeoff_site_status");
    valid_topic_ = ReadString(
        sdf, "validTopic", "/rescue/rotors/takeoff_site_valid");
    measurement_valid_topic_ = ReadString(
        sdf, "measurementValidTopic", "/rescue/rotors/takeoff_measurement_valid");
    reason_topic_ = ReadString(
        sdf, "reasonTopic", "/rescue/rotors/takeoff_blocked_reason");
    recheck_topic_ = ReadString(
        sdf, "recheckDistanceTopic", "/rescue/rotors/takeoff_recheck_distance");
    contact_topic_ = ReadString(
        sdf, "contactTopic", "/rescue/rotors/obstacle_contact");
    contact_force_topic_ = ReadString(
        sdf, "contactForceTopic", "/rescue/rotors/obstacle_contact_force_n");

    for (const auto& link : model_->GetLinks()) {
      for (const auto& collision : link->GetCollisions()) {
        if (collision) {
          model_collision_names_.insert(collision->GetScopedName());
          model_collisions_.push_back(collision);
        }
      }
    }
    if (!model_collisions_.empty()) {
      std::vector<std::string> names(model_collision_names_.begin(),
                                     model_collision_names_.end());
      contact_manager_ = world_->Physics()->GetContactManager();
      contact_manager_->CreateFilter(model_->GetName() + "_takeoff_clearance",
                                     names);
    }

    ros_node_.reset(new ros::NodeHandle(""));
    status_pub_ = ros_node_->advertise<geometry_msgs::Vector3Stamped>(
        status_topic_, 1);
    atomic_status_pub_ = ros_node_->advertise<diagnostic_msgs::DiagnosticArray>(
        atomic_status_topic_, 1);
    valid_pub_ = ros_node_->advertise<std_msgs::Bool>(valid_topic_, 1);
    measurement_valid_pub_ = ros_node_->advertise<std_msgs::Bool>(
        measurement_valid_topic_, 1);
    reason_pub_ = ros_node_->advertise<std_msgs::String>(reason_topic_, 1);
    recheck_pub_ = ros_node_->advertise<std_msgs::Float32>(recheck_topic_, 1);
    contact_pub_ = ros_node_->advertise<std_msgs::Bool>(contact_topic_, 1);
    contact_force_pub_ = ros_node_->advertise<std_msgs::Float32>(
        contact_force_topic_, 1);
    update_connection_ = event::Events::ConnectWorldUpdateEnd(
        std::bind(&TakeoffClearancePlugin::OnUpdate, this));
    gzmsg << "TakeoffClearancePlugin loaded for " << model_->GetName()
          << ": full-collision swept AABB, target z=" << target_altitude_
          << " m, margin=" << safety_margin_ << " m.\n";
  }

 private:
  static std::string ReadString(const sdf::ElementPtr& sdf,
                                const std::string& name,
                                const std::string& fallback) {
    return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
  }

  static double ReadDouble(const sdf::ElementPtr& sdf,
                           const std::string& name, double fallback) {
    return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
  }

  bool RobotBounds(ignition::math::AxisAlignedBox* result) const {
    bool found = false;
    for (const auto& collision : model_collisions_) {
      const auto bounds = collision->BoundingBox();
      if (!FiniteBox(bounds)) {
        continue;
      }
      if (!found) {
        *result = bounds;
        found = true;
      } else {
        result->Min().X(std::min(result->Min().X(), bounds.Min().X()));
        result->Min().Y(std::min(result->Min().Y(), bounds.Min().Y()));
        result->Min().Z(std::min(result->Min().Z(), bounds.Min().Z()));
        result->Max().X(std::max(result->Max().X(), bounds.Max().X()));
        result->Max().Y(std::max(result->Max().Y(), bounds.Max().Y()));
        result->Max().Z(std::max(result->Max().Z(), bounds.Max().Z()));
      }
    }
    return found;
  }

  bool IsSupportSurface(const ignition::math::AxisAlignedBox& obstacle,
                        const ignition::math::AxisAlignedBox& robot) const {
    // Geometry-only support classification: a collision wholly at or below the
    // robot's lowest collision surface cannot obstruct upward translation.
    return obstacle.Max().Z() <= robot.Min().Z() + support_tolerance_;
  }

  void ContactState(const ignition::math::AxisAlignedBox& robot,
                    bool* obstacle_contact, double* maximum_force) const {
    *obstacle_contact = false;
    *maximum_force = 0.0;
    if (!contact_manager_) {
      return;
    }
    for (const auto* contact : contact_manager_->GetContacts()) {
      if (!contact || !contact->collision1 || !contact->collision2) {
        continue;
      }
      const std::string first = contact->collision1->GetScopedName();
      const std::string second = contact->collision2->GetScopedName();
      const bool first_ours = model_collision_names_.count(first) != 0;
      const bool second_ours = model_collision_names_.count(second) != 0;
      if (first_ours == second_ours) {
        continue;
      }
      const auto* other = first_ours ? contact->collision2 : contact->collision1;
      const auto other_bounds = other->BoundingBox();
      if (!FiniteBox(other_bounds) || IsSupportSurface(other_bounds, robot)) {
        continue;
      }
      *obstacle_contact = true;
      for (int index = 0; index < contact->count; ++index) {
        *maximum_force = std::max(
            *maximum_force, contact->wrench[index].body1Force.Length());
      }
    }
  }

  static void AddValue(diagnostic_msgs::DiagnosticStatus* status,
                       const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue item;
    item.key = key;
    item.value = value;
    status->values.push_back(item);
  }

  void PublishAtomic(const common::Time& now, bool measurement_valid,
                     bool site_valid, double available, double required,
                     double nearest, double recheck,
                     const std::string& reason) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp.fromSec(now.Double());
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "takeoff_site_clearance";
    status.hardware_id = model_ ? model_->GetName() : "unknown";
    status.level = !measurement_valid
                       ? diagnostic_msgs::DiagnosticStatus::ERROR
                       : (site_valid ? diagnostic_msgs::DiagnosticStatus::OK
                                     : diagnostic_msgs::DiagnosticStatus::WARN);
    status.message = reason;
    AddValue(&status, "measurement_valid", measurement_valid ? "true" : "false");
    AddValue(&status, "takeoff_site_valid", site_valid ? "true" : "false");
    AddValue(&status, "vertical_clearance_available_m", std::to_string(available));
    AddValue(&status, "vertical_clearance_required_m", std::to_string(required));
    AddValue(&status, "nearest_obstacle_distance_m", std::to_string(nearest));
    AddValue(&status, "recheck_distance_m", std::to_string(recheck));
    array.status.push_back(status);
    atomic_status_pub_.publish(array);
  }

  void PublishInvalid(const common::Time& now, const std::string& reason) {
    geometry_msgs::Vector3Stamped status;
    status.header.stamp.fromSec(now.Double());
    status.header.frame_id = "world";
    status.vector.x = 0.0;
    status.vector.y = std::numeric_limits<float>::quiet_NaN();
    status.vector.z = std::numeric_limits<float>::quiet_NaN();
    status_pub_.publish(status);
    std_msgs::Bool flag;
    flag.data = false;
    valid_pub_.publish(flag);
    measurement_valid_pub_.publish(flag);
    std_msgs::String text;
    text.data = reason;
    reason_pub_.publish(text);
    PublishAtomic(now, false, false, 0.0,
                  std::numeric_limits<double>::quiet_NaN(),
                  std::numeric_limits<double>::quiet_NaN(), 0.0, reason);
  }

  void OnUpdate() {
    const common::Time now = world_->SimTime();
    if (update_rate_ > 0.0 && last_publish_ != common::Time::Zero &&
        (now - last_publish_).Double() < 1.0 / update_rate_) {
      return;
    }
    last_publish_ = now;
    ignition::math::AxisAlignedBox robot;
    if (!RobotBounds(&robot) || !FiniteBox(robot)) {
      PublishInvalid(now, "measurement_invalid:no_finite_robot_collision_bounds");
      return;
    }

    const double required = std::max(
        0.0, target_altitude_ - model_->WorldPose().Pos().Z());
    ignition::math::AxisAlignedBox swept = robot;
    swept.Min().X(swept.Min().X() - safety_margin_);
    swept.Min().Y(swept.Min().Y() - safety_margin_);
    swept.Min().Z(swept.Min().Z() - safety_margin_);
    swept.Max().X(swept.Max().X() + safety_margin_);
    swept.Max().Y(swept.Max().Y() + safety_margin_);
    swept.Max().Z(swept.Max().Z() + required + safety_margin_);

    bool blocked = false;
    double available = std::numeric_limits<double>::infinity();
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto& other_model : world_->Models()) {
      if (!other_model || other_model == model_) {
        continue;
      }
      for (const auto& link : other_model->GetLinks()) {
        for (const auto& collision : link->GetCollisions()) {
          if (!collision) {
            continue;
          }
          const auto obstacle = collision->BoundingBox();
          if (!FiniteBox(obstacle) || IsSupportSurface(obstacle, robot)) {
            continue;
          }
          nearest = std::min(nearest, BoxDistance(robot, obstacle));
          const bool xy_overlap =
              AxisGap(swept.Min().X(), swept.Max().X(), obstacle.Min().X(),
                      obstacle.Max().X()) <= 0.0 &&
              AxisGap(swept.Min().Y(), swept.Max().Y(), obstacle.Min().Y(),
                      obstacle.Max().Y()) <= 0.0;
          if (!xy_overlap) {
            continue;
          }
          if (obstacle.Min().Z() > robot.Max().Z() + safety_margin_) {
            available = std::min(
                available,
                obstacle.Min().Z() - robot.Max().Z() - safety_margin_);
          } else if (obstacle.Max().Z() >= robot.Min().Z() - safety_margin_) {
            available = 0.0;
          }
          const bool z_overlap =
              AxisGap(swept.Min().Z(), swept.Max().Z(), obstacle.Min().Z(),
                      obstacle.Max().Z()) <= 0.0;
          blocked = blocked || z_overlap;
        }
      }
    }

    bool obstacle_contact = false;
    double contact_force = 0.0;
    ContactState(robot, &obstacle_contact, &contact_force);
    const double footprint_x = robot.XLength() + 2.0 * safety_margin_;
    const double footprint_y = robot.YLength() + 2.0 * safety_margin_;
    const double recheck_distance = std::max(footprint_x, footprint_y);

    geometry_msgs::Vector3Stamped status;
    status.header.stamp.fromSec(now.Double());
    status.header.frame_id = "world";
    status.vector.x = std::isfinite(available) ? available : -1.0;
    status.vector.y = required;
    status.vector.z = std::isfinite(nearest) ? nearest : -1.0;
    status_pub_.publish(status);
    std_msgs::Bool valid;
    valid.data = !blocked;
    valid_pub_.publish(valid);
    std_msgs::Bool measurement_valid;
    measurement_valid.data = true;
    measurement_valid_pub_.publish(measurement_valid);
    std_msgs::String reason;
    reason.data = blocked ? "swept_volume_blocked" : "clear";
    reason_pub_.publish(reason);
    std_msgs::Float32 recheck;
    recheck.data = recheck_distance;
    recheck_pub_.publish(recheck);
    std_msgs::Bool contact;
    contact.data = obstacle_contact;
    contact_pub_.publish(contact);
    std_msgs::Float32 force;
    force.data = contact_force;
    contact_force_pub_.publish(force);
    PublishAtomic(now, true, !blocked,
                  std::isfinite(available) ? available : -1.0, required,
                  std::isfinite(nearest) ? nearest : -1.0,
                  recheck_distance, reason.data);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::ContactManager* contact_manager_ = nullptr;
  std::vector<physics::CollisionPtr> model_collisions_;
  std::unordered_set<std::string> model_collision_names_;
  event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher status_pub_;
  ros::Publisher atomic_status_pub_;
  ros::Publisher valid_pub_;
  ros::Publisher measurement_valid_pub_;
  ros::Publisher reason_pub_;
  ros::Publisher recheck_pub_;
  ros::Publisher contact_pub_;
  ros::Publisher contact_force_pub_;
  std::string status_topic_;
  std::string atomic_status_topic_;
  std::string valid_topic_;
  std::string measurement_valid_topic_;
  std::string reason_topic_;
  std::string recheck_topic_;
  std::string contact_topic_;
  std::string contact_force_topic_;
  double target_altitude_ = 1.2;
  double safety_margin_ = 0.02;
  double support_tolerance_ = 0.01;
  double update_rate_ = 50.0;
  common::Time last_publish_;
};

GZ_REGISTER_MODEL_PLUGIN(TakeoffClearancePlugin)

}  // namespace gazebo
