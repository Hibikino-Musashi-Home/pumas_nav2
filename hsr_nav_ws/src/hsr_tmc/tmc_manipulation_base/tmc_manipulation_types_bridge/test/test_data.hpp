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
#ifndef TMC_MANIPULATION_TYPES_BRIDGE_TEST_TEST_DATA_HPP_
#define TMC_MANIPULATION_TYPES_BRIDGE_TEST_TEST_DATA_HPP_

#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_planning_msgs/msg/task_space_region.hpp>

namespace tmc_manipulation_types_bridge {

sensor_msgs::msg::JointState GenerateJointStateMsg() {
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"CARM/LINEAR", "CARM/SHOULDER_Y", "CARM/SHOULDER_R", "CARM/ELBOW_P", "CARM/WRIST_Y",
                      "CARM/WRIST_R", "CARM/WRIST_P", "CARM/HEAD/NECK_Y", "CARM/HEAD/NECK_P", "CARM/HAND/JOINT_11",
                      "CARM/HAND/JOINT_12", "CARM/HAND/JOINT_21", "CARM/HAND/JOINT_22", "CARM/LINEAR_PASSIVE"};
  joint_state.position = {0.0, 0.0, 0.0, 0.0, 3.0, 1.57, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0, 0.0};
  return joint_state;
}

tmc_manipulation_types::JointState GenerateJointState() {
  tmc_manipulation_types::JointState joint_state;
  joint_state.name = {"CARM/LINEAR", "CARM/SHOULDER_Y", "CARM/SHOULDER_R", "CARM/ELBOW_P", "CARM/WRIST_Y",
                      "CARM/WRIST_R", "CARM/WRIST_P", "CARM/HEAD/NECK_Y", "CARM/HEAD/NECK_P", "CARM/HAND/JOINT_11",
                      "CARM/HAND/JOINT_12", "CARM/HAND/JOINT_21", "CARM/HAND/JOINT_22", "CARM/LINEAR_PASSIVE"};

  joint_state.position.resize(joint_state.name.size());
  joint_state.position << 0.0, 0.0, 0.0, 0.0, 3.0, 1.57, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0, 0.0;

  joint_state.velocity.resize(joint_state.name.size());
  joint_state.effort.resize(joint_state.name.size());
  for (auto i = 0; i < joint_state.name.size(); ++i) {
    joint_state.velocity[i] = i * 0.1;
    joint_state.effort[i] = i * 0.1;
  }
  return joint_state;
}

tmc_planning_msgs::msg::TaskSpaceRegion GenerateTaskSpaceRegionMsg() {
  tmc_planning_msgs::msg::TaskSpaceRegion task_space_region;
  task_space_region.end_frame_id = "End";
  task_space_region.origin_to_tsr.position.x = 1.0;
  task_space_region.origin_to_tsr.position.y = 2.0;
  task_space_region.origin_to_tsr.position.z = 3.0;
  task_space_region.origin_to_tsr.orientation.x = 4.0;
  task_space_region.origin_to_tsr.orientation.y = 5.0;
  task_space_region.origin_to_tsr.orientation.z = 6.0;
  task_space_region.origin_to_tsr.orientation.w = 7.0;
  task_space_region.tsr_to_end.position.x = 8.0;
  task_space_region.tsr_to_end.position.y = 9.0;
  task_space_region.tsr_to_end.position.z = 10.0;
  task_space_region.tsr_to_end.orientation.x = 11.0;
  task_space_region.tsr_to_end.orientation.y = 12.0;
  task_space_region.tsr_to_end.orientation.z = 13.0;
  task_space_region.tsr_to_end.orientation.w = 14.0;
  task_space_region.max_bounds = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  task_space_region.min_bounds = {0.0, -0.1, -0.2, -0.3, -0.4, -0.5};
  return task_space_region;
}

tmc_manipulation_types::TaskSpaceRegion GenerateTaskSpaceRegion() {
  tmc_manipulation_types::TaskSpaceRegion task_space_region;
  task_space_region.end_frame_id = "End";
  task_space_region.origin_to_tsr = Eigen::Translation3d(1.0, 2.0, 3.0) * Eigen::Quaterniond(4.0, 5.0, 6.0, 7.0);
  task_space_region.tsr_to_end = Eigen::Translation3d(-1.0, -2.0, -3.0) * Eigen::Quaterniond(-4.0, -5.0, -6.0, -7.0);
  task_space_region.max_bounds << 0.0, 0.1, 0.2, 0.3, 0.4, 0.5;
  task_space_region.min_bounds << 0.0, -0.1, -0.2, -0.3, -0.4, -0.5;
  return task_space_region;
}

moveit_msgs::msg::AttachedCollisionObject GenerateAttachCollisionObjectMsg() {
  moveit_msgs::msg::AttachedCollisionObject attached_collision_object;
  attached_collision_object.object.id = "AttachedObject";
  attached_collision_object.link_name = "AttachedObjectFlame";
  attached_collision_object.object.pose.position.x = 0.1;
  attached_collision_object.object.pose.position.y = 0.2;
  attached_collision_object.object.pose.position.z = 0.3;
  attached_collision_object.object.pose.orientation.x = 0.4;
  attached_collision_object.object.pose.orientation.y = 0.5;
  attached_collision_object.object.pose.orientation.z = 0.6;
  attached_collision_object.object.pose.orientation.w = 0.7;
  return attached_collision_object;
}

sensor_msgs::msg::MultiDOFJointState GenerateMultiDOFJointStateMsg() {
  sensor_msgs::msg::MultiDOFJointState multi_dof_joint_state;
  multi_dof_joint_state.joint_names = {"joint_0"};
  multi_dof_joint_state.transforms.resize(1);
  multi_dof_joint_state.transforms[0].translation.x = 1.0;
  multi_dof_joint_state.transforms[0].translation.y = 2.0;
  multi_dof_joint_state.transforms[0].translation.z = 3.0;
  multi_dof_joint_state.transforms[0].rotation.x = 4.0;
  multi_dof_joint_state.transforms[0].rotation.y = 5.0;
  multi_dof_joint_state.transforms[0].rotation.z = 6.0;
  multi_dof_joint_state.transforms[0].rotation.w = 7.0;
  multi_dof_joint_state.twist.resize(1);
  multi_dof_joint_state.twist[0].linear.x = 0.1;
  multi_dof_joint_state.twist[0].linear.y = 0.2;
  multi_dof_joint_state.twist[0].linear.z = 0.3;
  multi_dof_joint_state.twist[0].angular.x = 0.4;
  multi_dof_joint_state.twist[0].angular.y = 0.5;
  multi_dof_joint_state.twist[0].angular.z = 0.6;
  multi_dof_joint_state.wrench.resize(1);
  multi_dof_joint_state.wrench[0].force.x = -0.1;
  multi_dof_joint_state.wrench[0].force.y = -0.2;
  multi_dof_joint_state.wrench[0].force.z = -0.3;
  multi_dof_joint_state.wrench[0].torque.x = -0.4;
  multi_dof_joint_state.wrench[0].torque.y = -0.5;
  multi_dof_joint_state.wrench[0].torque.z = -0.6;
  return multi_dof_joint_state;
}

tmc_manipulation_types::MultiDOFJointState GenerateMultiDOFJointState() {
  tmc_manipulation_types::MultiDOFJointState multi_dof_joint_state;
  multi_dof_joint_state.names = {"joint_0"};
  multi_dof_joint_state.poses = {
      Eigen::Translation3d(1.0, 2.0, 3.0) * Eigen::Quaterniond(4.0, 5.0, 6.0, 7.0)};
  multi_dof_joint_state.twist.resize(1);
  multi_dof_joint_state.twist[0] << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  multi_dof_joint_state.wrench.resize(1);
  multi_dof_joint_state.wrench[0] << -0.1, -0.2, -0.3, -0.4, -0.5, -0.6;
  return multi_dof_joint_state;
}

trajectory_msgs::msg::JointTrajectory GenerateJointTrajectoryMsg() {
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint0", "joint1"};

  trajectory_msgs::msg::JointTrajectoryPoint point0;
  point0.positions = {1.0, 2.0};
  point0.velocities = {1.0, -1.0};
  point0.accelerations = {-1.0, 1.0};
  point0.effort = {1.0, 1.0};
  point0.time_from_start.sec = 1;
  trajectory.points.push_back(point0);

  trajectory_msgs::msg::JointTrajectoryPoint point1;
  point1.positions = {2.0, 4.0};
  point1.velocities = {2.0, -2.0};
  point1.accelerations = {-2.0, 2.0};
  point1.effort = {2.0, 2.0};
  point1.time_from_start.sec = 2;
  trajectory.points.push_back(point1);

  return trajectory;
}

tmc_manipulation_types::JointTrajectory GenerateJointTrajectory() {
  tmc_manipulation_types::JointTrajectory trajectory;
  trajectory.names = {"joint0", "joint1"};
  trajectory.path.resize(2);
  trajectory.path[0].resize(2);
  trajectory.path[0] << 1.0, 2.0;
  trajectory.path[1].resize(2);
  trajectory.path[1] << 3.0, 4.0;

  return trajectory;
}

tmc_manipulation_types::TimedJointTrajectory GenerateTimedJointTrajectory() {
  tmc_manipulation_types::TimedJointTrajectory trajectory;
  trajectory.joint_names = {"joint0", "joint1"};

  tmc_manipulation_types::TimedJointTrajectoryPoint point0;
  point0.positions.resize(2);
  point0.positions << 1.0, 2.0;
  point0.velocities.resize(2);
  point0.velocities << -1.0, 1.0;
  point0.accelerations.resize(2);
  point0.accelerations << -1.0, 1.0;
  point0.effort.resize(2);
  point0.effort << -1.0, 1.0;
  point0.time_from_start = 1.0;
  trajectory.points.push_back(point0);

  tmc_manipulation_types::TimedJointTrajectoryPoint point1;
  point1.positions.resize(2);
  point1.positions << 3.0, 4.0;
  point1.velocities.resize(2);
  point1.velocities << -2.0, 2.0;
  point1.accelerations.resize(2);
  point1.accelerations << -2.0, 2.0;
  point1.effort.resize(2);
  point1.effort << -2.0, 2.0;
  point1.time_from_start = 2.0;
  trajectory.points.push_back(point1);

  return trajectory;
}

trajectory_msgs::msg::MultiDOFJointTrajectory GenerateMultiDOFJointTrajectoryMsg() {
  trajectory_msgs::msg::MultiDOFJointTrajectory multi_dof_trajectory;
  multi_dof_trajectory.joint_names = {"joint_0"};
  multi_dof_trajectory.points.resize(2);
  multi_dof_trajectory.points[0].transforms.resize(1);
  multi_dof_trajectory.points[0].transforms[0].translation.x = 1.0;
  multi_dof_trajectory.points[0].transforms[0].translation.y = 2.0;
  multi_dof_trajectory.points[0].transforms[0].translation.z = 3.0;
  multi_dof_trajectory.points[0].transforms[0].rotation.x = 4.0;
  multi_dof_trajectory.points[0].transforms[0].rotation.y = 5.0;
  multi_dof_trajectory.points[0].transforms[0].rotation.z = 6.0;
  multi_dof_trajectory.points[0].transforms[0].rotation.w = 7.0;
  multi_dof_trajectory.points[0].velocities.resize(1);
  multi_dof_trajectory.points[0].velocities[0].linear.x = 0.1;
  multi_dof_trajectory.points[0].velocities[0].linear.y = 0.2;
  multi_dof_trajectory.points[0].velocities[0].linear.z = 0.3;
  multi_dof_trajectory.points[0].velocities[0].angular.x = 0.4;
  multi_dof_trajectory.points[0].velocities[0].angular.y = 0.5;
  multi_dof_trajectory.points[0].velocities[0].angular.z = 0.6;
  multi_dof_trajectory.points[0].accelerations.resize(1);
  multi_dof_trajectory.points[0].accelerations[0].linear.x = -0.1;
  multi_dof_trajectory.points[0].accelerations[0].linear.y = -0.2;
  multi_dof_trajectory.points[0].accelerations[0].linear.z = -0.3;
  multi_dof_trajectory.points[0].accelerations[0].angular.x = -0.4;
  multi_dof_trajectory.points[0].accelerations[0].angular.y = -0.5;
  multi_dof_trajectory.points[0].accelerations[0].angular.z = -0.6;
  multi_dof_trajectory.points[0].time_from_start.sec = 8;

  multi_dof_trajectory.points[1].transforms.resize(1);
  multi_dof_trajectory.points[1].transforms[0].translation.x = 11.0;
  multi_dof_trajectory.points[1].transforms[0].translation.y = 12.0;
  multi_dof_trajectory.points[1].transforms[0].translation.z = 13.0;
  multi_dof_trajectory.points[1].transforms[0].rotation.x = 14.0;
  multi_dof_trajectory.points[1].transforms[0].rotation.y = 15.0;
  multi_dof_trajectory.points[1].transforms[0].rotation.z = 16.0;
  multi_dof_trajectory.points[1].transforms[0].rotation.w = 17.0;
  multi_dof_trajectory.points[1].velocities.resize(1);
  multi_dof_trajectory.points[1].velocities[0].linear.x = 1.1;
  multi_dof_trajectory.points[1].velocities[0].linear.y = 1.2;
  multi_dof_trajectory.points[1].velocities[0].linear.z = 1.3;
  multi_dof_trajectory.points[1].velocities[0].angular.x = 1.4;
  multi_dof_trajectory.points[1].velocities[0].angular.y = 1.5;
  multi_dof_trajectory.points[1].velocities[0].angular.z = 1.6;
  multi_dof_trajectory.points[1].accelerations.resize(1);
  multi_dof_trajectory.points[1].accelerations[0].linear.x = -1.1;
  multi_dof_trajectory.points[1].accelerations[0].linear.y = -1.2;
  multi_dof_trajectory.points[1].accelerations[0].linear.z = -1.3;
  multi_dof_trajectory.points[1].accelerations[0].angular.x = -1.4;
  multi_dof_trajectory.points[1].accelerations[0].angular.y = -1.5;
  multi_dof_trajectory.points[1].accelerations[0].angular.z = -1.6;
  multi_dof_trajectory.points[1].time_from_start.sec = 18;

  return multi_dof_trajectory;
}

tmc_manipulation_types::MultiDOFJointTrajectory GenerateMultiDOFJointTrajectory() {
  tmc_manipulation_types::MultiDOFJointTrajectory multi_dof_trajectory;
  multi_dof_trajectory.names = {"joint_0"};
  multi_dof_trajectory.path.resize(2);
  multi_dof_trajectory.path[0] = {
      Eigen::Translation3d(1.0, 2.0, 3.0) * Eigen::Quaterniond(4.0, 5.0, 6.0, 7.0)};
  multi_dof_trajectory.path[1] = {
      Eigen::Translation3d(11.0, 12.0, 13.0) * Eigen::Quaterniond(14.0, 15.0, 16.0, 17.0)};
  return multi_dof_trajectory;
}

tmc_manipulation_types::TimedMultiDOFJointTrajectory GenerateTimedMultiDOFJointTrajectory() {
  tmc_manipulation_types::TimedMultiDOFJointTrajectory multi_dof_trajectory;
  multi_dof_trajectory.joint_names = {"joint_0"};
  multi_dof_trajectory.points.resize(2);
  multi_dof_trajectory.points[0].transforms = {
      Eigen::Translation3d(1.0, 2.0, 3.0) * Eigen::Quaterniond(4.0, 5.0, 6.0, 7.0)};
  multi_dof_trajectory.points[0].velocities.resize(1);
  multi_dof_trajectory.points[0].velocities[0] << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  multi_dof_trajectory.points[0].accelerations.resize(1);
  multi_dof_trajectory.points[0].accelerations[0] << -0.1, -0.2, -0.3, -0.4, -0.5, -0.6;
  multi_dof_trajectory.points[0].time_from_start = 8.0;
  multi_dof_trajectory.points[1].transforms = {
      Eigen::Translation3d(11.0, 12.0, 13.0) * Eigen::Quaterniond(14.0, 15.0, 16.0, 17.0)};
  multi_dof_trajectory.points[1].velocities.resize(1);
  multi_dof_trajectory.points[1].velocities[0] << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6;
  multi_dof_trajectory.points[1].accelerations.resize(1);
  multi_dof_trajectory.points[1].accelerations[0] << -1.1, -1.2, -1.3, -1.4, -1.5, -1.6;
  multi_dof_trajectory.points[1].time_from_start = 18.0;
  return multi_dof_trajectory;
}

}  // namespace tmc_manipulation_types_bridge
#endif  // TMC_MANIPULATION_TYPES_BRIDGE_TEST_TEST_DATA_HPP_
