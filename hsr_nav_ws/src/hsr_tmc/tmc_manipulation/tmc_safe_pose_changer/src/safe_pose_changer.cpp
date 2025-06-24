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

#include "safe_pose_changer.hpp"

#include <algorithm>
#include <memory>
#include <string>

#include <tmc_utils/parameters.hpp>
#include <tmc_utils/qos.hpp>

namespace tmc_safe_pose_changer {

const std::vector<std::string> kTrajectoryControllers =
  {"/arm_trajectory_controller", "/head_trajectory_controller"};

const char* const kActionName = "follow_joint_trajectory";

SafePoseChangerNode::SafePoseChangerNode(const rclcpp::NodeOptions& options)
    : Node("safe_pose_changer", options) {
  const auto controllers = tmc_utils::GetParameter(this, "controllers", kTrajectoryControllers);
  for (const auto& controller_name : controllers) {
    try {
      publishers_.push_back(std::make_shared<tmc_manipulation_util::JointTrajectoryPublisher>(this, controller_name));
      const auto joint_names = publishers_.back()->joint_names();
      joint_names_.insert(joint_names_.end(), joint_names.begin(), joint_names.end());
    } catch(const std::domain_error& error) {
      RCLCPP_ERROR(this->get_logger(), error.what());
      return;
    }
  }

  // If it is 0.0, a collision with the wheels will appear, so a slightly smaller value will be the default value.
  auto floor_collision_height = tmc_utils::GetParameter(this, "floor_collision_height", -0.001);
  floor_environment_.collision_objects.resize(1);
  floor_environment_.collision_objects[0].id = "floor";
  floor_environment_.collision_objects[0].pose.position.z = floor_collision_height;
  floor_environment_.collision_objects[0].primitives.resize(1);
  floor_environment_.collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  floor_environment_.collision_objects[0].primitives[0].dimensions = {10.0, 10.0, 0.1};
  floor_environment_.collision_objects[0].primitive_poses.resize(1);
  floor_environment_.collision_objects[0].primitive_poses[0].position.z =
      -floor_environment_.collision_objects[0].primitives[0].dimensions[2] / 2.0;

  request_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
                            "~/joint_reference", tmc_utils::ReliableVolatileQoS(),
                            std::bind(&SafePoseChangerNode::RefJointStateCallBack, this, std::placeholders::_1));

  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
                                "joint_states", tmc_utils::BestEffortQoS(),
                                std::bind(&SafePoseChangerNode::CurrentJointStateCallback,
                                this, std::placeholders::_1));

  planner_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  planner_client_ = create_client<tmc_planning_msgs::srv::PlanWithJointGoals>(
                      "plan_with_joint_goals", rmw_qos_profile_services_default,
                       planner_cb_group_);
  if (!planner_client_->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "planner service error");
    return;
  }

  filter_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  filter_client_ = create_client<tmc_manipulation_msgs::srv::FilterJointTrajectory>(
                      "timeopt_filter_node/filter_trajectory", rmw_qos_profile_services_default,
                       filter_cb_group_);
  if (!filter_client_->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "filter service error");
    return;
  }

  velocity_ratio_ = tmc_utils::GetParameter(this, "velocity_ratio", 1.0);
  acceleration_ratio_ = tmc_utils::GetParameter(this, "acceleration_ratio", 1.0);

  set_param_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto filter_node_name = tmc_utils::GetParameter(this, "filter_node_name", "timeopt_filter_node");
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, filter_node_name, rmw_qos_profile_parameters, set_param_cb_group_);
  if (!param_client_->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "filter node set parameter service error");
    return;
  }

  RCLCPP_INFO(get_logger(), "safe_pose_changer Ready");
}

SafePoseChangerNode::~SafePoseChangerNode() {
  for (auto& thread : threads_) {
    thread.join();
  }
}

/**
 * @BRIEF Currently acquiring a joint angle 
 */
void SafePoseChangerNode::CurrentJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  current_joint_ = msg;
}

void SafePoseChangerNode::ChangePoseImpl(const sensor_msgs::msg::JointState& request_joint) {
  if (request_joint.name.empty()) {
    RCLCPP_ERROR(get_logger(), "Empty reference");
    return;
  }
  if (request_joint.name.size() != request_joint.position.size()) {
    RCLCPP_ERROR(get_logger(), "Size mismatch between joint names and positions");
    return;
  }
  for (const auto& name : request_joint.name) {
    if (std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid joint name: " << name);
      return;
    }
  }

  trajectory_msgs::msg::JointTrajectory output_trajectory;
  if (!PlanTrajectory(*current_joint_, request_joint, output_trajectory)) {
    return;
  }

  trajectory_msgs::msg::JointTrajectory filter_trajectory;
  if (!FilterTrajectory(output_trajectory, filter_trajectory)) {
    return;
  }

  for (const auto& publisher : publishers_) {
    publisher->Publish(filter_trajectory, *current_joint_);
  }
}

/**
 * @BRIEF Target angle acquisition Subscler
 */
void SafePoseChangerNode::RefJointStateCallBack(const sensor_msgs::msg::JointState::SharedPtr request_joint) {
  threads_.emplace_back(std::thread(std::bind(&SafePoseChangerNode::ChangePoseImpl, this, *request_joint)));
}

/**
 * Call the @bRIEF PLANNER service and get a trajectory plan to Goal
 * @Param current_joint_state: Current joint information
 * @Param goal_state: Target joint information
 * @Param trajectory_out: Trajectory after the orbital plan
 */
bool SafePoseChangerNode::PlanTrajectory(
    const sensor_msgs::msg::JointState& current_joint_state,
    const sensor_msgs::msg::JointState& goal_state,
    trajectory_msgs::msg::JointTrajectory& trajectory_out) {
  tmc_planning_msgs::msg::JointPosition goal_joint_position;
  goal_joint_position.position = goal_state.position;

  auto plan_srv = std::make_shared<tmc_planning_msgs::srv::PlanWithJointGoals::Request>();
  auto pose = geometry_msgs::msg::Pose();
  pose.orientation.w = 1.0;

  plan_srv->origin_to_basejoint = pose;
  plan_srv->initial_joint_state = current_joint_state;
  plan_srv->goal_joint_states.push_back(goal_joint_position);
  plan_srv->use_joints = goal_state.name;
  plan_srv->timeout.sec = 10;
  plan_srv->max_iteration = 1000;
  plan_srv->environment_before_planning = floor_environment_;

  auto future = planner_client_->async_send_request(plan_srv);
  if (future.wait_for(std::chrono::milliseconds(3000)) == std::future_status::ready) {
    const auto response = future.get();
    if (response->error_code.val > 0) {
      trajectory_out = response->solution;
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "Planner service error code : %d", response->error_code.val);
    }
  }
  return false;
}

/**
 * @Param trajectory: Trajectory
 * @Param filtered_trajectory: Return the time the best time TRAJECTORY
 * Call success or fail to optimize the time optimization service
 */
bool SafePoseChangerNode::FilterTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    trajectory_msgs::msg::JointTrajectory& filtered_trajectory) {
  auto get_param_future = param_client_->get_parameters({"velocity_ratio", "acceleration_ratio"});
  if (get_param_future.wait_for(std::chrono::milliseconds(3000)) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Failed to get velocity/acceleration ratio parameter");
    return false;
  }
  const auto ratio_params = get_param_future.get();

  UpdateFilterParameters(velocity_ratio_, acceleration_ratio_);

  auto filter_req = std::make_shared<tmc_manipulation_msgs::srv::FilterJointTrajectory::Request>();
  filter_req->trajectory = trajectory;
  filter_req->start_state.joint_state.name = trajectory.joint_names;
  filter_req->start_state.joint_state.name.push_back("base_roll_joint");
  filter_req->start_state.joint_state.position = trajectory.points[0].positions;
  filter_req->start_state.joint_state.position.push_back(0);

  auto filter_future = filter_client_->async_send_request(filter_req);
  auto is_success = false;
  if (filter_future.wait_for(std::chrono::milliseconds(3000)) == std::future_status::ready) {
    const auto response = filter_future.get();
    is_success = response->is_success;

    if (is_success) {
      filtered_trajectory = response->trajectory;
    }
  }

  UpdateFilterParameters(ratio_params[0].as_double(), ratio_params[1].as_double());
  return is_success;
}

void SafePoseChangerNode::UpdateFilterParameters(double velocity_ratio, double acceleration_ratio) {
  auto future = param_client_->set_parameters({
      rclcpp::Parameter("velocity_ratio", velocity_ratio),
      rclcpp::Parameter("acceleration_ratio", acceleration_ratio)});
  if (future.wait_for(std::chrono::milliseconds(3000)) == std::future_status::ready) {
    for (auto &result : future.get()) {
      if (!result.successful) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
      }
    }
  }
}
}  // namespace tmc_safe_pose_changer
