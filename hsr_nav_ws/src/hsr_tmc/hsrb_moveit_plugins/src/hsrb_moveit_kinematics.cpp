/*
Copyright (c) 2017 TOYOTA MOTOR CORPORATION
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

#include "hsrb_moveit_kinematics.hpp"
#include <memory>
#include <string>
#include <vector>
#include <moveit/robot_model/robot_model.h>
#include <tf2_eigen/tf2_eigen.h>
#include <urdf/model.h>
#include <hsrb_analytic_ik/hsrb_ik_solver.hpp>
#include <hsrb_analytic_ik/hsrc_ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

namespace hsrb_moveit_kinematics {

static rclcpp::Logger LOGGER = rclcpp::get_logger("hsrb_moveit_plugins.hsrb_moveit_kinematics");

using tmc_robot_kinematics_model::IKSolver;
using tmc_manipulation_types::JointState;
using tmc_robot_kinematics_model::IKRequest;
using tmc_robot_kinematics_model::IKResult;
using tmc_robot_kinematics_model::kSuccess;

HSRBKinematicsPlugin::HSRBKinematicsPlugin() : active_(false) {}

bool HSRBKinematicsPlugin::initialize(
    const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
    const std::string& group_name, const std::string& base_frame,
    const std::vector<std::string>& tip_frames, double search_discretization) {
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  std::string robot_description;
  node->get_parameter("robot_description", robot_description);
  robot_ = std::make_shared<PinocchioWrapper>(robot_description);

  dimension_ = 8;

  joint_names_.push_back("odom_x");
  joint_names_.push_back("odom_y");
  joint_names_.push_back("odom_t");
  joint_names_.push_back("arm_lift_joint");
  joint_names_.push_back("arm_flex_joint");
  joint_names_.push_back("arm_roll_joint");
  joint_names_.push_back("wrist_flex_joint");
  joint_names_.push_back("wrist_roll_joint");

  link_names_.push_back("odom_xr_link");
  link_names_.push_back("odom_yx_link");
  link_names_.push_back("base_footprint");
  link_names_.push_back("arm_lift_link");
  link_names_.push_back("arm_flex_link");
  link_names_.push_back("arm_roll_link");
  link_names_.push_back("wrist_flex_link");
  link_names_.push_back("wrist_roll_link");

  // Usage joint name
  use_joints_.push_back("arm_lift_joint");
  use_joints_.push_back("arm_flex_joint");
  use_joints_.push_back("arm_roll_joint");
  use_joints_.push_back("wrist_flex_joint");
  use_joints_.push_back("wrist_roll_joint");

  // Acquisition of robot name
  std::string solver_robot_name;
  urdf::Model urdf_model;
  urdf_model.initString(robot_description);
  solver_robot_name = urdf_model.getName();

  if (solver_robot_name == "hsrc") {
    solver_ = std::make_shared<hsrb_analytic_ik::HsrcIKSolver>(IKSolver::Ptr());
  } else {
    solver_ = std::make_shared<hsrb_analytic_ik::HsrbIKSolver>(IKSolver::Ptr());
  }

  if (node->has_parameter({group_name + ".joint_weight"})) {
    node->get_parameter(group_name + ".joint_weight", weights_);
  }
  if (weights_.size() != dimension_) {
    weights_ = {10.0, 10.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0};
  }

  if (tip_frames.size() != 1 || tip_frames[0] != "hand_palm_link") {
    RCLCPP_ERROR_STREAM(LOGGER, "tip_frame should be 'hand_palm_link'");
    return false;
  }

  if (base_frame != "odom") {
    RCLCPP_ERROR_STREAM(LOGGER, "base_frame should be 'odom'");
    return false;
  }

  active_ = true;

  return true;
}

bool HSRBKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                         const std::vector<double>& ik_seed_state,
                                         std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                                         const kinematics::KinematicsQueryOptions& options) const {
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, consistency_limits,
                          solution, solution_callback, error_code, options);
}


bool HSRBKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            std::vector<double>& solution,
                                            moveit_msgs::msg::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const {
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, consistency_limits,
                          solution, solution_callback, error_code, options);
}

bool HSRBKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            const std::vector<double>& consistency_limits,
                                            std::vector<double>& solution,
                                            moveit_msgs::msg::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const {
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, consistency_limits,
                          solution, solution_callback, error_code, options);
}

bool HSRBKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                            moveit_msgs::msg::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const {
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, consistency_limits,
                          solution, solution_callback, error_code, options);
}

bool HSRBKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            const std::vector<double>& consistency_limits,
                                            std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                            moveit_msgs::msg::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const {
  error_code.val = error_code.NO_IK_SOLUTION;
  if (!active_) {
    RCLCPP_ERROR(LOGGER, "kinematics not active");
    return false;
  }

  if (ik_seed_state.size() != dimension_) {
    RCLCPP_ERROR_STREAM(LOGGER,
                        "Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    return false;
  }

  if (!consistency_limits.empty() && consistency_limits.size() != dimension_) {
    RCLCPP_ERROR_STREAM(LOGGER, "Consistency limits be empty or must have size "
                        << dimension_ << " instead of size " << consistency_limits.size());
    return false;
  }

  solution.resize(dimension_);

  // Conversion of hand target position
  Eigen::Affine3d ref_origin_to_end;
  tf2::fromMsg(ik_pose, ref_origin_to_end);

  // Initial value of numerical IK joints
  Eigen::VectorXd init_angle;
  init_angle.resize(dimension_ - 3);
  for (int i = 0; i < dimension_ - 3; ++i) {
    init_angle[i] = ik_seed_state[i + 3];
  }

  // Planar restraint (set x, y, θ)
  IKRequest req(tmc_manipulation_types::kPlanar);
  req.frame_name = "hand_palm_link";
  req.frame_to_end = Eigen::Affine3d::Identity();
  req.initial_angle.name = use_joints_;
  req.use_joints = use_joints_;
  // The weight of the joints.However, the back three is the bogie.
  req.weight = Eigen::Map<const Eigen::VectorXd>(&weights_[0], weights_.size());
  req.ref_origin_to_end = ref_origin_to_end;
  req.initial_angle.position = init_angle;
  req.origin_to_base = Eigen::Translation3d(ik_seed_state[0], ik_seed_state[1], 0)
                     * Eigen::AngleAxisd(ik_seed_state[2], Eigen::Vector3d::UnitZ());

  JointState js_solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base_solution;
  tmc_robot_kinematics_model::IKResult result;

  result = solver_->Solve(req, js_solution, origin_to_base_solution, origin_to_hand_result);

  if (result == kSuccess) {
    Eigen::Vector3d euler_vector = origin_to_base_solution.rotation().eulerAngles(0, 1, 2);

    solution[0] = origin_to_base_solution.translation().x();
    solution[1] = origin_to_base_solution.translation().y();
    solution[2] = euler_vector(2);

    for (int i = 3; i < dimension_; ++i) {
      solution[i] = js_solution.position[i - 3];
    }

    error_code.val = error_code.SUCCESS;
    if (solution_callback) {
      solution_callback(ik_pose, solution, error_code);
    }
  }
  return error_code.val == error_code.SUCCESS;
}

bool HSRBKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                         const std::vector<double>& joint_angles,
                                         std::vector<geometry_msgs::msg::Pose>& poses) const {
  if (joint_angles.size() != dimension_) {
    RCLCPP_ERROR_STREAM(LOGGER, "joint_angles size is not " << dimension_);
    return false;
  }
  // Global angle settings
  JointState joint_state;
  joint_state.name = use_joints_;
  joint_state.position.resize(dimension_ - 3);
  for (int i = 0; i < dimension_ - 3; ++i) {
    joint_state.position[i] = joint_angles[i + 3];
  }
  robot_->SetNamedAngle(joint_state);
  // Setting of base position
  Eigen::Affine3d origin_to_base;
  origin_to_base = Eigen::Translation3d(joint_angles[0], joint_angles[1], 0)
                 * Eigen::AngleAxisd(joint_angles[2], Eigen::Vector3d::UnitZ());
  robot_->SetRobotTransform(origin_to_base);
  poses.clear();
  for (std::size_t i = 0; i < link_names.size(); ++i) {
    Eigen::Affine3d origin_to_link;
    try {
      origin_to_link = robot_->GetObjectTransform(link_names[i]);
    } catch (const tmc_robot_kinematics_model::PinocchioError& err) {
      RCLCPP_ERROR_STREAM(LOGGER, "getPositionFK failed to find '" << link_names[i] << "'");
      return false;
    }
    poses.push_back(tf2::toMsg(origin_to_link));
  }
  return true;
}

const std::vector<std::string>& HSRBKinematicsPlugin::getJointNames() const { return joint_names_; }

const std::vector<std::string>& HSRBKinematicsPlugin::getLinkNames() const { return link_names_; }

bool HSRBKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out) const {
  return true;
}

}  // namespace hsrb_moveit_kinematics

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(hsrb_moveit_kinematics::HSRBKinematicsPlugin, kinematics::KinematicsBase);
