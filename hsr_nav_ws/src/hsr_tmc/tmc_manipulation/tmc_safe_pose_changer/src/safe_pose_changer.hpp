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
/// Posture change node

#ifndef TMC_SAFE_POSE_CHANGER_SAFE_POSE_CHANGER_HPP_
#define TMC_SAFE_POSE_CHANGER_SAFE_POSE_CHANGER_HPP_
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tmc_manipulation_msgs/srv/filter_joint_trajectory.hpp>
#include <tmc_manipulation_util/joint_trajectory_publisher.hpp>
#include <tmc_planning_msgs/msg/joint_position.hpp>
#include <tmc_planning_msgs/srv/plan_with_joint_goals.hpp>

namespace tmc_safe_pose_changer {

class SafePoseChangerNode : public rclcpp::Node {
 public:
  SafePoseChangerNode() : SafePoseChangerNode(rclcpp::NodeOptions()) {}
  explicit SafePoseChangerNode(const rclcpp::NodeOptions& options);
  virtual ~SafePoseChangerNode();

 private:
  // Currently a joint angle subcliver
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  // Currently a joint angle MSG Shared Pointer
  sensor_msgs::msg::JointState::SharedPtr current_joint_;
  // Subscler that receives the target angle
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr request_subscription_;
  // Client of joint operation
  std::vector<tmc_manipulation_util::JointTrajectoryPublisher::Ptr> publishers_;
  // Service callback group
  rclcpp::CallbackGroup::SharedPtr planner_cb_group_;
  // Client of orbital planning service
  rclcpp::Client<tmc_planning_msgs::srv::PlanWithJointGoals>::SharedPtr planner_client_;
  // Service callback group
  rclcpp::CallbackGroup::SharedPtr filter_cb_group_;
  // Time optimization service client
  rclcpp::Client<tmc_manipulation_msgs::srv::FilterJointTrajectory>::SharedPtr filter_client_;

  // Implementation of posture transition
  void ChangePoseImpl(const sensor_msgs::msg::JointState& request_joint);

  std::vector<std::thread> threads_;
  std::vector<std::string> joint_names_;
  moveit_msgs::msg::PlanningSceneWorld floor_environment_;

  double velocity_ratio_;
  double acceleration_ratio_;
  rclcpp::CallbackGroup::SharedPtr set_param_cb_group_;
  rclcpp::AsyncParametersClient::SharedPtr param_client_;
  void UpdateFilterParameters(double velocity_ratio, double acceleration_ratio);

  void CurrentJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void RefJointStateCallBack(const sensor_msgs::msg::JointState::SharedPtr request_joint);

  bool PlanTrajectory(const sensor_msgs::msg::JointState& current_joint_state,
                      const sensor_msgs::msg::JointState& goal_state,
                      trajectory_msgs::msg::JointTrajectory& trajectory_out);

  bool FilterTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                        trajectory_msgs::msg::JointTrajectory& filtered_trajectory);
};

}  // namespace tmc_safe_pose_changer
#endif  // TMC_SAFE_POSE_CHANGER_SAFE_POSE_CHANGER_HPP_
