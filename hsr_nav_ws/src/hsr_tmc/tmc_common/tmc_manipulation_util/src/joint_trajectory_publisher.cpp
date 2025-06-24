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
#include <angles/angles.h>

#include <tmc_manipulation_util/joint_trajectory_publisher.hpp>

#include <tmc_manipulation_util/joint_trajectory_configuration.hpp>
#include <tmc_utils/parameters.hpp>
#include <tmc_utils/qos.hpp>

namespace {

bool GetJointPosition(const sensor_msgs::msg::JointState& joint_state,
                      const std::string& joint,
                      double& angle_out) {
  for (unsigned int i = 0; i < joint_state.name.size(); ++i) {
    if ((joint_state.name[i] == joint) && i < joint_state.position.size()) {
      angle_out = joint_state.position[i];
      return true;
    }
  }
  return false;
}

bool GetJointIndex(const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                   const std::string& joint,
                   uint32_t& index_out) {
  for (unsigned int i = 0; i < joint_trajectory.joint_names.size(); ++i) {
    if (joint_trajectory.joint_names[i] == joint) {
      index_out = i;
      return true;
    }
  }
  return false;
}

void OffsetContinuousJoint(trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                           const std::vector<std::string>& continuous_joints,
                           const sensor_msgs::msg::JointState& joint_state) {
  if (joint_trajectory.points.empty()) {
    return;
  }
  for (const auto& name : continuous_joints) {
    double current_position;
    if (!GetJointPosition(joint_state, name, current_position)) {
      continue;
    }

    uint32_t index;
    if (!GetJointIndex(joint_trajectory, name, index)) {
      continue;
    }
    if (index >= joint_trajectory.points[0].positions.size()) {
      continue;
    }
    double first_position = joint_trajectory.points[0].positions[index];

    const auto distance = angles::shortest_angular_distance(current_position, first_position);
    const auto offset = current_position + distance - first_position;
    for (auto& point : joint_trajectory.points) {
      if (index >= point.positions.size()) {
        continue;
      }
      point.positions[index] += offset;
    }
  }
}

}  // namespace

namespace tmc_manipulation_util {

JointTrajectoryPublisher::JointTrajectoryPublisher(rclcpp::Node* node,
                                                   const std::string& controller_name,
                                                   const std::vector<std::string>& joint_names,
                                                   const std::vector<std::string>& continuous_joints)
    : joint_names_(joint_names), continuous_joints_(continuous_joints) {
  pub_ = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      controller_name + "/joint_trajectory", tmc_utils::ReliableVolatileQoS());
}

JointTrajectoryPublisher::JointTrajectoryPublisher(rclcpp::Node* node,
                                                   const std::string& controller_name,
                                                   const std::vector<std::string>& joint_names)
    : JointTrajectoryPublisher(node, controller_name, joint_names, {}) {}

JointTrajectoryPublisher::JointTrajectoryPublisher(const rclcpp::Node::SharedPtr& node,
                                                   const std::string& controller_name,
                                                   const std::vector<std::string>& joint_names)
    : JointTrajectoryPublisher(node.get(), controller_name, joint_names) {}

JointTrajectoryPublisher::JointTrajectoryPublisher(rclcpp::Node* node,
                                                   const std::string& controller_name)
    : JointTrajectoryPublisher(node, controller_name, {}) {
  continuous_joints_ = tmc_utils::GetParameter<std::vector<std::string>>(
      node, controller_name + ".continuous_joints", {});

  // If you just use Joint_trajectory_controller simply,
  // you can get the joint name by obtaining a parameter from the controller.
  // It's useless if you have a unique controller or REMAP, so put the implementation from the parameters.
  joint_names_ = tmc_utils::GetParameter<std::vector<std::string>>(node, controller_name + ".joints", {});
  if (!joint_names_.empty()) {
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Joint names of " << controller_name << " is got from " << node->get_name() << "'s parameter.");
    return;
  }

  const auto wait_for_controller_milliseconds = tmc_utils::GetParameter<int>(
      node, "wait_for_controller_milliseconds", 2000);

  const auto parameter = std::make_shared<rclcpp::SyncParametersClient>(node, controller_name);
  if (!parameter->wait_for_service(std::chrono::milliseconds(wait_for_controller_milliseconds))) {
    throw std::domain_error("Communication with the trajectory controller server failed.");
  }
  const auto parameters_get_results = parameter->get_parameters({"joints"}).at(0);
  joint_names_ = parameters_get_results.as_string_array();
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "Joint names of " << controller_name << " is got from controller node.");
}

JointTrajectoryPublisher::JointTrajectoryPublisher(const rclcpp::Node::SharedPtr& node,
                                                   const std::string& controller_name)
    : JointTrajectoryPublisher(node.get(), controller_name) {}

bool JointTrajectoryPublisher::Publish(const trajectory_msgs::msg::JointTrajectory& trajectory) const {
  return Publish(trajectory, sensor_msgs::msg::JointState());
}

bool JointTrajectoryPublisher::Publish(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                       const sensor_msgs::msg::JointState& current_state) const {
  trajectory_msgs::msg::JointTrajectory partial_trajectory;
  if (!ExtractTrajectory(trajectory, joint_names_, current_state, partial_trajectory)) {
    return false;
  }
  OffsetContinuousJoint(partial_trajectory, continuous_joints_, current_state);

  pub_->publish(partial_trajectory);
  return true;
}
}  // namespace tmc_manipulation_util
