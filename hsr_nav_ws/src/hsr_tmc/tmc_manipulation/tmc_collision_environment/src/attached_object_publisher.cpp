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

#include "attached_object_publisher.hpp"

#include <limits>

#include <tf2_eigen/tf2_eigen.hpp>

#include <tmc_utils/parameters.hpp>
#include <tmc_utils/qos.hpp>

#include "utils.hpp"

namespace {
constexpr double kDefaultPublishRate = 10.0;

bool GetRefToTarget(const std::unique_ptr<tf2_ros::Buffer>& tf_buffer,
                    const rclcpp::Logger& logger,
                    const std::string& ref_frame_id,
                    const std::string& target_frame_id,
                    Eigen::Affine3d& ref_to_target) {
  try {
    const auto transform = tf_buffer->lookupTransform(ref_frame_id, target_frame_id, tf2::TimePointZero);
    ref_to_target = tf2::transformToEigen(transform);
    return true;
  } catch (const tf2::TransformException& e) {
    RCLCPP_ERROR(logger, "Could not transform %s to %s: %s",
                 ref_frame_id.c_str(), target_frame_id.c_str(), e.what());
    return false;
  }
}

}  // namespace

namespace tmc_collision_environment {

AttachedObjectServer::AttachedObjectServer(const rclcpp::NodeOptions& options)
    : Node("attached_object_publisher", options), marker_lifetime_(0, 0), colors_(GenerateColors(1)) {
  grasping_frame_id_ = tmc_utils::GetParameter<std::string>(this, "grasping_frame_id", "");
  if (grasping_frame_id_.empty()) {
    RCLCPP_FATAL(this->get_logger(), "grasping_frame_id is required.");
    std::exit(EXIT_FAILURE);
  }
  releasing_frame_id_ = tmc_utils::GetParameter<std::string>(this, "releasing_frame_id", "odom");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  attaching_object_info_sub_ = this->create_subscription<moveit_msgs::msg::AttachedCollisionObject>(
      "~/attaching_object_info", tmc_utils::ReliableVolatileQoS(),
      std::bind(&AttachedObjectServer::AttachingObjectInfoCallback, this, std::placeholders::_1));
  attaching_object_name_sub_ = this->create_subscription<std_msgs::msg::String>(
      "~/attaching_object_name", tmc_utils::ReliableVolatileQoS(),
      std::bind(&AttachedObjectServer::AttachingObjectNameCallback, this, std::placeholders::_1));
  releasing_object_name_sub_ = this->create_subscription<std_msgs::msg::String>(
      "~/releasing_object_name", tmc_utils::ReliableVolatileQoS(),
      std::bind(&AttachedObjectServer::ReleasingObjectNameCallback, this, std::placeholders::_1));

  attached_object_pub_ = create_publisher<moveit_msgs::msg::RobotState>(
      "~/attached_object", tmc_utils::ReliableVolatileQoS());
  marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/marker_array", tmc_utils::BestEffortQoS());

  environment_sub_ = this->create_subscription<moveit_msgs::msg::PlanningSceneWorld>(
      "collision_environment_server/environment", tmc_utils::BestEffortQoS(),
      std::bind(&AttachedObjectServer::EnvironmentCallback, this, std::placeholders::_1));
  collision_object_pub_ = create_publisher<moveit_msgs::msg::CollisionObject>(
      "collision_environment_server/collision_object", tmc_utils::ReliableVolatileQoS());

  auto publish_rate = tmc_utils::GetParameter(this, "publish_rate", kDefaultPublishRate);
  if (publish_rate < std::numeric_limits<double>::min()) {
    RCLCPP_WARN(this->get_logger(),
                "The parameter publish_rate must be positive."
                " Since the value was %f, use the default value %f", publish_rate, kDefaultPublishRate);
    publish_rate = kDefaultPublishRate;
  }
  marker_lifetime_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);
  timer_ = this->create_wall_timer(marker_lifetime_.to_chrono<std::chrono::duration<double>>(),
                                   std::bind(&AttachedObjectServer::TimerCallback, this));
}

void AttachedObjectServer::TimerCallback() {
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  moveit_msgs::msg::RobotState robot_state;
  // I'm worried about the copy cost, but it's okay because there are at most a few ingredients.
  for (const auto& [name, attached_object] : attached_objects_) {
    collision_objects.push_back(attached_object.object);
    robot_state.attached_collision_objects.push_back(attached_object);
  }
  marker_array_pub_->publish(ConvertToMarkerArray(collision_objects, colors_, marker_lifetime_));
  attached_object_pub_->publish(robot_state);
}

void AttachedObjectServer::AttachingObjectInfoCallback(
    const moveit_msgs::msg::AttachedCollisionObject::SharedPtr msg) {
  // Since it is the idea of ​​creating a node for each knowledge frame, if Link_name does not match, it will not be processed.
  if (msg->link_name != grasping_frame_id_) {
    RCLCPP_ERROR(this->get_logger(), "Link name %s is not equal node's handling frame %s.",
                 msg->link_name.c_str(), grasping_frame_id_.c_str());
    return;
  }

  // Moveit_msgs/Msg/AttachedCollisionObject response (below)
  // If action is remove and no object.id is set,
  // all objects attached to the link indicated by link_name will be removed
  if (msg->object.id.empty() && msg->object.operation == moveit_msgs::msg::CollisionObject::REMOVE) {
    // Attached_objects_ is operated in ReleaseIMPL, so divide the loop
    std::vector<std::string> names;
    for (const auto& [name, attached_object] : attached_objects_) {
      names.push_back(name);
    }
    for (const auto& name : names) {
      ReleaseImpl(name);
    }
    return;
  }

  // Add/update/update
  Eigen::Affine3d ref_to_object;
  tf2::fromMsg(msg->object.pose, ref_to_object);

  Eigen::Affine3d ref_to_frame;
  if (!GetRefToTarget(tf_buffer_, this->get_logger(),
                      msg->object.header.frame_id, grasping_frame_id_, ref_to_frame)) {
    return;
  }

  auto attached_object = *msg;
  attached_object.object.header.frame_id = grasping_frame_id_;
  attached_object.object.pose = tf2::toMsg(ref_to_frame.inverse() * ref_to_object);

  attached_objects_[msg->object.id] = attached_object;

  // Remove the knowledge from the environment
  for (const auto collision_object : environments_) {
    if (collision_object.id != msg->object.id) {
      continue;
    }
    moveit_msgs::msg::CollisionObject command;
    command.id = collision_object.id;
    command.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    collision_object_pub_->publish(command);
  }
}

void AttachedObjectServer::AttachingObjectNameCallback(const std_msgs::msg::String::SharedPtr msg) {
  if (attached_objects_.find(msg->data) != attached_objects_.end()) {
    RCLCPP_ERROR(this->get_logger(), "%s has already been included in attached objects.", msg->data.c_str());
    return;
  }

  for (const auto collision_object : environments_) {
    if (collision_object.id != msg->data) {
      continue;
    }
    // Make grid information
    Eigen::Affine3d ref_to_object;
    tf2::fromMsg(collision_object.pose, ref_to_object);

    Eigen::Affine3d ref_to_frame;
    if (!GetRefToTarget(tf_buffer_, this->get_logger(),
                        collision_object.header.frame_id, grasping_frame_id_, ref_to_frame)) {
      return;
    }

    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = grasping_frame_id_;
    attached_object.object = collision_object;
    attached_object.object.header.frame_id = grasping_frame_id_;
    attached_object.object.pose = tf2::toMsg(ref_to_frame.inverse() * ref_to_object);

    attached_objects_[collision_object.id] = attached_object;

    // Remove the knowledge from the environment
    moveit_msgs::msg::CollisionObject command;
    command.id = collision_object.id;
    command.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    collision_object_pub_->publish(command);

    return;
  }
  RCLCPP_ERROR(this->get_logger(), "%s is not in collision environment.", msg->data.c_str());
}

void AttachedObjectServer::ReleasingObjectNameCallback(const std_msgs::msg::String::SharedPtr msg) {
  ReleaseImpl(msg->data);
}

void AttachedObjectServer::EnvironmentCallback(const moveit_msgs::msg::PlanningSceneWorld::SharedPtr msg) {
  environments_ = msg->collision_objects;
}

void AttachedObjectServer::ReleaseImpl(const std::string& name) {
  if (attached_objects_.find(name) == attached_objects_.end()) {
    RCLCPP_ERROR(this->get_logger(), "%s is not in attached objects.", name.c_str());
    return;
  }
  const auto& attached_object = attached_objects_[name];

  Eigen::Affine3d frame_to_object;
  tf2::fromMsg(attached_object.object.pose, frame_to_object);

  Eigen::Affine3d ref_to_frame;
  if (GetRefToTarget(tf_buffer_, this->get_logger(), releasing_frame_id_, grasping_frame_id_, ref_to_frame)) {
    auto command = attached_object.object;
    command.header.frame_id = releasing_frame_id_;
    command.pose = tf2::toMsg(ref_to_frame * frame_to_object);
    command.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_object_pub_->publish(command);
  }

  attached_objects_.erase(name);
}


}  // end of namespace tmc_collision_environment

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(tmc_collision_environment::AttachedObjectServer)
