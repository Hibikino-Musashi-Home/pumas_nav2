/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#include "collision_environment_server.hpp"

#include <limits>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tmc_utils/qos.hpp>

#include "utils.hpp"

namespace {
constexpr double kTransformTimeout = 0.2;
constexpr double kDefaultPublishRate = 10.0;

bool ValidateSize(const rclcpp::Logger& logger,
                  const moveit_msgs::msg::CollisionObject& msg) {
  if (msg.primitives.size() != msg.primitive_poses.size()) {
    RCLCPP_ERROR(logger, "The sizes of primitives and primitive_poses do not match.");
    return false;
  }
  if (msg.meshes.size() != msg.mesh_poses.size()) {
    RCLCPP_ERROR(logger, "The sizes of meshes and mesh_poses do not match.");
    return false;
  }
  if (msg.planes.size() != msg.plane_poses.size()) {
    RCLCPP_ERROR(logger, "The sizes of planes and plane_poses do not match.");
    return false;
  }
  return true;
}

}  // namespace

namespace tmc_collision_environment {

CollisionEnvironmentServer::CollisionEnvironmentServer(const rclcpp::NodeOptions& options)
    : Node("collision_environment_server", options), marker_lifetime_(0, 0) {
  // Prepare the color of the marker
  // SEED is fixed because it feels difficult to understand even if the color changes every time
  colors_ = GenerateColors();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  origin_frame_id_ = std::make_shared<tmc_utils::DynamicParameter<std::string>>(this, "origin_frame_id", "");
  set_param_handlers_.emplace_back(this->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<std::string>::SetParameterCallback, origin_frame_id_,
                std::placeholders::_1)));

  collision_object_sub_ = this->create_subscription<moveit_msgs::msg::CollisionObject>(
      "~/collision_object", tmc_utils::ReliableVolatileQoS(10),
      std::bind(&CollisionEnvironmentServer::CollisionObjectCallback, this, std::placeholders::_1));

  environment_pub_ = create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
      "~/environment", tmc_utils::ReliableVolatileQoS());
  transformed_environment_pub_ = create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
      "~/transformed_environment", tmc_utils::ReliableVolatileQoS());
  marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/marker_array", tmc_utils::BestEffortQoS());

  auto publish_rate = tmc_utils::GetParameter(this, "publish_rate", kDefaultPublishRate);
  if (publish_rate < std::numeric_limits<double>::min()) {
    RCLCPP_WARN(this->get_logger(),
                "The parameter publish_rate must be positive."
                " Since the value was %f, use the default value %f", publish_rate, kDefaultPublishRate);
    publish_rate = kDefaultPublishRate;
  }
  marker_lifetime_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);
  timer_ = this->create_wall_timer(marker_lifetime_.to_chrono<std::chrono::duration<double>>(),
                                   std::bind(&CollisionEnvironmentServer::TimerCallback, this));
}

void CollisionEnvironmentServer::TimerCallback() {
  moveit_msgs::msg::PlanningSceneWorld env;
  env.collision_objects.reserve(objects_.size());
  for (const auto& [name, collision_object] : objects_) {
    env.collision_objects.push_back(collision_object);
  }

  marker_array_pub_->publish(ConvertToMarkerArray(env.collision_objects, colors_, marker_lifetime_));
  environment_pub_->publish(env);

  if (!origin_frame_id_->value().empty()) {
    auto it = env.collision_objects.begin();
    while (it != env.collision_objects.end()) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = it->header;
      pose_stamped.pose = it->pose;
      try {
        const auto res = tf_buffer_->transform(pose_stamped, origin_frame_id_->value(),
                                               tf2::durationFromSec(kTransformTimeout));
        it->header = res.header;
        it->pose = res.pose;
        ++it;
      } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
        it = env.collision_objects.erase(it);
      }
    }
    transformed_environment_pub_->publish(env);
  }
}

void CollisionEnvironmentServer::CollisionObjectCallback(const moveit_msgs::msg::CollisionObject::SharedPtr msg) {
  if (msg->id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Object id is empty.");
    return;
  }

  switch (msg->operation) {
    case moveit_msgs::msg::CollisionObject::ADD:
      if (ValidateSize(this->get_logger(), *msg)) {
        objects_[msg->id] = *msg;
      }
      break;
    case moveit_msgs::msg::CollisionObject::REMOVE:
      if (objects_.find(msg->id) != objects_.end()) {
        objects_.erase(msg->id);
      } else {
        RCLCPP_ERROR(this->get_logger(), "The operation REMOVE cannot be completed because the %s does not exist.",
                     msg->id.c_str());
      }
      break;
    case moveit_msgs::msg::CollisionObject::APPEND:
      // # Append to an object that already exists in the planning scene. If the object does not exist, it is added.
      if (!ValidateSize(this->get_logger(), *msg)) {
        return;
      }
      if (objects_.find(msg->id) != objects_.end()) {
        objects_[msg->id].primitives.insert(
            objects_[msg->id].primitives.end(), msg->primitives.begin(), msg->primitives.end());
        objects_[msg->id].primitive_poses.insert(
            objects_[msg->id].primitive_poses.end(), msg->primitive_poses.begin(), msg->primitive_poses.end());
        objects_[msg->id].meshes.insert(
            objects_[msg->id].meshes.end(), msg->meshes.begin(), msg->meshes.end());
        objects_[msg->id].mesh_poses.insert(
            objects_[msg->id].mesh_poses.end(), msg->mesh_poses.begin(), msg->mesh_poses.end());
        objects_[msg->id].planes.insert(
            objects_[msg->id].planes.end(), msg->planes.begin(), msg->planes.end());
        objects_[msg->id].plane_poses.insert(
            objects_[msg->id].plane_poses.end(), msg->plane_poses.begin(), msg->plane_poses.end());
      } else {
        objects_[msg->id] = *msg;
      }
      break;
    case moveit_msgs::msg::CollisionObject::MOVE:
      // # If an object already exists in the scene, new poses can be sent (the geometry arrays must be left empty)
      // # if solely moving the object is desired
      if (objects_.find(msg->id) != objects_.end()) {
        objects_[msg->id].pose = msg->pose;

        if (objects_[msg->id].primitives.size() == msg->primitive_poses.size()) {
          objects_[msg->id].primitive_poses = msg->primitive_poses;
        }
        if (objects_[msg->id].meshes.size() == msg->mesh_poses.size()) {
          objects_[msg->id].mesh_poses = msg->mesh_poses;
        }
        if (objects_[msg->id].planes.size() == msg->plane_poses.size()) {
          objects_[msg->id].plane_poses = msg->plane_poses;
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "The operation MOVE cannot be completed because the %s does not exist.",
                     msg->id.c_str());
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "The requested operation %d is not yet implemented.", msg->operation);
  }
}

}  // end of namespace tmc_collision_environment

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(tmc_collision_environment::CollisionEnvironmentServer)
