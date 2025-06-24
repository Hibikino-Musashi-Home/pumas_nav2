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
#ifndef TMC_MANIPULATION_UTIL_JOINT_TRAJECTORY_PUBLISHER_HPP_
#define TMC_MANIPULATION_UTIL_JOINT_TRAJECTORY_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace tmc_manipulation_util {

class JointTrajectoryPublisher {
 public:
  using Ptr = std::shared_ptr<JointTrajectoryPublisher>;

  JointTrajectoryPublisher(rclcpp::Node* node, const std::string& controller_name,
                           const std::vector<std::string>& joint_names,
                           const std::vector<std::string>& continuous_joints);
  JointTrajectoryPublisher(rclcpp::Node* node, const std::string& controller_name,
                           const std::vector<std::string>& joint_names);
  JointTrajectoryPublisher(const rclcpp::Node::SharedPtr& node, const std::string& controller_name,
                           const std::vector<std::string>& joint_names);

  JointTrajectoryPublisher(rclcpp::Node* node, const std::string& controller_name);
  JointTrajectoryPublisher(const rclcpp::Node::SharedPtr& node, const std::string& controller_name);

  virtual ~JointTrajectoryPublisher() = default;

  bool Publish(const trajectory_msgs::msg::JointTrajectory& trajectory) const;
  bool Publish(const trajectory_msgs::msg::JointTrajectory& trajectory,
               const sensor_msgs::msg::JointState& current_state) const;

  std::vector<std::string> joint_names() const { return joint_names_; }
  std::vector<std::string> continuous_joints() const { return continuous_joints_; }

 private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> continuous_joints_;
};

}  // namespace tmc_manipulation_util
#endif  // TMC_MANIPULATION_UTIL_JOINT_TRAJECTORY_PUBLISHER_HPP_
