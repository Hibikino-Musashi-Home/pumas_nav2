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
/// \brief test for tmc_maniulation_types

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <tmc_manipulation_types/manipulation_types.hpp>

using tmc_manipulation_types::JointState;
using tmc_manipulation_types::JointTrajectory;
using tmc_manipulation_types::MultiDOFJointState;
using tmc_manipulation_types::MultiDOFJointTrajectory;
using tmc_manipulation_types::NameSeq;
using tmc_manipulation_types::RobotTrajectory;
using tmc_manipulation_types::TimedJointTrajectory;
using tmc_manipulation_types::TimedJointTrajectoryPoint;
using tmc_manipulation_types::TimedMultiDOFJointTrajectory;
using tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint;
using tmc_manipulation_types::TimedRobotTrajectory;

TEST(TestJointStateValidate, TestStructs) {
  JointState joint_state;
  joint_state.name.resize(1);
  joint_state.name[0] = "joint";
  joint_state.position.resize(1);
  joint_state.position << 0.1;
  EXPECT_TRUE(joint_state.Validate());
}

TEST(TestJointStateNoName, TestStructs) {
  JointState joint_state;
  EXPECT_FALSE(joint_state.Validate());
}

TEST(TestJointStateSizeMismatch, TestStructs) {
  JointState joint_state;
  joint_state.name.resize(1);
  joint_state.name[0] = "joint";
  joint_state.position.resize(2);
  joint_state.position << 0.1, 0.2;
  EXPECT_FALSE(joint_state.Validate());
}

TEST(TestJointTrajectoryValidate, TestStructs) {
  JointTrajectory joint_trajectory;
  joint_trajectory.names.resize(1);
  joint_trajectory.names[0] = "joint";
  joint_trajectory.path.push_back(Eigen::VectorXd::Ones(1));
  EXPECT_TRUE(joint_trajectory.Validate());
}

TEST(TestJointTrajectoryNoPath, TestStructs) {
  JointTrajectory joint_trajectory;
  joint_trajectory.names.resize(1);
  joint_trajectory.names[0] = "joint";
  EXPECT_FALSE(joint_trajectory.Validate());
}

TEST(TestJointTrajectoryNoName, TestStructs) {
  JointTrajectory joint_trajectory;
  joint_trajectory.path.push_back(Eigen::VectorXd::Ones(1));
  EXPECT_FALSE(joint_trajectory.Validate());
}

TEST(TestJointTrajectorySizeMismatch, TestStructs) {
  JointTrajectory joint_trajectory;
  joint_trajectory.names.resize(2);
  joint_trajectory.names[0] = "joint0";
  joint_trajectory.names[1] = "joint1";
  joint_trajectory.path.push_back(Eigen::VectorXd::Ones(1));
  EXPECT_FALSE(joint_trajectory.Validate());
}

TEST(TestMultiDOFJointTrajectoryValidate, TestStructs) {
  MultiDOFJointTrajectory multi_dof_joint_trajectory;
  multi_dof_joint_trajectory.names.resize(1);
  multi_dof_joint_trajectory.names[0] = "joint";
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  multi_dof_joint_trajectory.path.push_back(pose);
  EXPECT_TRUE(multi_dof_joint_trajectory.Validate());
}

TEST(TestMultiDOFJointTrajectoryNoPath, TestStructs) {
  MultiDOFJointTrajectory multi_dof_joint_trajectory;
  multi_dof_joint_trajectory.names.resize(1);
  multi_dof_joint_trajectory.names[0] = "joint";
  EXPECT_FALSE(multi_dof_joint_trajectory.Validate());
}

TEST(TestMultiDOFJointTrajectoryNoName, TestStructs) {
  MultiDOFJointTrajectory multi_dof_joint_trajectory;
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  multi_dof_joint_trajectory.path.push_back(pose);
  EXPECT_FALSE(multi_dof_joint_trajectory.Validate());
}

TEST(TestMultiDOFJointTrajectorySizeMismatch, TestStructs) {
  MultiDOFJointTrajectory multi_dof_joint_trajectory;
  multi_dof_joint_trajectory.names.resize(2);
  multi_dof_joint_trajectory.names[0] = "joint0";
  multi_dof_joint_trajectory.names[1] = "joint1";
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  multi_dof_joint_trajectory.path.push_back(pose);
  EXPECT_FALSE(multi_dof_joint_trajectory.Validate());
}

TEST(TestRobotTrajectoryValidate, TestStructs) {
  JointTrajectory joint_trajectory;
  joint_trajectory.names.resize(1);
  joint_trajectory.names[0] = "joint";
  joint_trajectory.path.push_back(Eigen::VectorXd::Ones(1));
  MultiDOFJointTrajectory multi_dof_joint_trajectory;
  multi_dof_joint_trajectory.names.resize(1);
  multi_dof_joint_trajectory.names[0] = "base";
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  multi_dof_joint_trajectory.path.push_back(pose);
  RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory = joint_trajectory;
  robot_trajectory.multi_dof_joint_trajectory = multi_dof_joint_trajectory;
  EXPECT_TRUE(robot_trajectory.Validate());
}

TEST(TestRobotTrajectoryJointTrajectoryFailure, TestStructs) {
  JointTrajectory joint_trajectory;
  joint_trajectory.path.push_back(Eigen::VectorXd::Ones(1));
  MultiDOFJointTrajectory multi_dof_joint_trajectory;
  multi_dof_joint_trajectory.names.resize(1);
  multi_dof_joint_trajectory.names[0] = "base";
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  multi_dof_joint_trajectory.path.push_back(pose);
  RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory = joint_trajectory;
  robot_trajectory.multi_dof_joint_trajectory = multi_dof_joint_trajectory;
  EXPECT_FALSE(robot_trajectory.Validate());
}

TEST(TestRobotTrajectoryValidateMultiDOFJointTrajectoryFailure, TestStructs) {
  JointTrajectory joint_trajectory;
  joint_trajectory.names.resize(1);
  joint_trajectory.names[0] = "joint";
  joint_trajectory.path.push_back(Eigen::VectorXd::Ones(1));
  MultiDOFJointTrajectory multi_dof_joint_trajectory;
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  multi_dof_joint_trajectory.path.push_back(pose);
  RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory = joint_trajectory;
  robot_trajectory.multi_dof_joint_trajectory = multi_dof_joint_trajectory;
  EXPECT_FALSE(robot_trajectory.Validate());
}

TEST(TestRobotTrajectorySizeMismatch, TestStructs) {
  JointTrajectory joint_trajectory;
  joint_trajectory.path.push_back(Eigen::VectorXd::Ones(1));
  joint_trajectory.path.push_back(Eigen::VectorXd::Ones(1));
  MultiDOFJointTrajectory multi_dof_joint_trajectory;
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  multi_dof_joint_trajectory.path.push_back(pose);
  RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory = joint_trajectory;
  robot_trajectory.multi_dof_joint_trajectory = multi_dof_joint_trajectory;
  EXPECT_FALSE(robot_trajectory.Validate());
}

TEST(TestMultiDOFJointStateValidate, TestStructs) {
  MultiDOFJointState multi_dof_joint_state;
  multi_dof_joint_state.names.resize(1);
  multi_dof_joint_state.names[0] = "joint";
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  multi_dof_joint_state.poses = pose;
  EXPECT_TRUE(multi_dof_joint_state.Validate());
}

TEST(TestMultiDOFJointStateNoName, TestStructs) {
  MultiDOFJointState multi_dof_joint_state;
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  multi_dof_joint_state.poses = pose;
  EXPECT_FALSE(multi_dof_joint_state.Validate());
}

TEST(TestMultiDOFJointStateNoPose, TestStructs) {
  MultiDOFJointState multi_dof_joint_state;
  multi_dof_joint_state.names.resize(1);
  multi_dof_joint_state.names[0] = "joint";
  EXPECT_FALSE(multi_dof_joint_state.Validate());
}

TEST(TestMultiDOFJointStateSizeMismatch, TestStructs) {
  MultiDOFJointState multi_dof_joint_state;
  multi_dof_joint_state.names.resize(2);
  multi_dof_joint_state.names[0] = "joint0";
  multi_dof_joint_state.names[1] = "joint1";
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  multi_dof_joint_state.poses = pose;
  EXPECT_FALSE(multi_dof_joint_state.Validate());
}

TEST(TestTimedJointTrajectoryValidate, TestStructs) {
  TimedJointTrajectory timed_joint_trajectory;
  timed_joint_trajectory.joint_names.resize(1);
  timed_joint_trajectory.joint_names[0] = "joint";
  TimedJointTrajectoryPoint timed_joint_trajectory_point;
  timed_joint_trajectory_point.positions.resize(1);
  timed_joint_trajectory_point.positions << 0.1;
  timed_joint_trajectory.points.push_back(timed_joint_trajectory_point);
  EXPECT_TRUE(timed_joint_trajectory.Validate());
}

TEST(TestTimedJointTrajectoryNoName, TestStructs) {
  TimedJointTrajectory timed_joint_trajectory;
  TimedJointTrajectoryPoint timed_joint_trajectory_point;
  timed_joint_trajectory_point.positions.resize(1);
  timed_joint_trajectory_point.positions << 0.1;
  timed_joint_trajectory.points.push_back(timed_joint_trajectory_point);
  EXPECT_FALSE(timed_joint_trajectory.Validate());
}

TEST(TestTimedJointTrajectoryNoPose, TestStructs) {
  TimedJointTrajectory timed_joint_trajectory;
  timed_joint_trajectory.joint_names.resize(1);
  timed_joint_trajectory.joint_names[0] = "joint";
  EXPECT_FALSE(timed_joint_trajectory.Validate());
}

TEST(TestTimedJointTrajectorySizeMismatch, TestStructs) {
  TimedJointTrajectory timed_joint_trajectory;
  timed_joint_trajectory.joint_names.resize(2);
  timed_joint_trajectory.joint_names[0] = "joint0";
  timed_joint_trajectory.joint_names[1] = "joint1";
  TimedJointTrajectoryPoint timed_joint_trajectory_point;
  timed_joint_trajectory_point.positions.resize(1);
  timed_joint_trajectory_point.positions << 0.1;
  timed_joint_trajectory.points.push_back(timed_joint_trajectory_point);
  EXPECT_FALSE(timed_joint_trajectory.Validate());
}

TEST(TestTimedMultiDOFJointTrajectoryValidate, TestStructs) {
  TimedMultiDOFJointTrajectory timed_multi_dof_joint_trajectory;
  timed_multi_dof_joint_trajectory.joint_names.resize(1);
  timed_multi_dof_joint_trajectory.joint_names[0] = "joint";
  TimedMultiDOFJointTrajectoryPoint timed_multi_dof_joint_trajectory_point;
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  timed_multi_dof_joint_trajectory_point.transforms = pose;
  timed_multi_dof_joint_trajectory.points.push_back(timed_multi_dof_joint_trajectory_point);
  EXPECT_TRUE(timed_multi_dof_joint_trajectory.Validate());
}

TEST(TestTimedMultiDOFJointTrajectoryNoName, TestStructs) {
  TimedMultiDOFJointTrajectory timed_multi_dof_joint_trajectory;
  TimedMultiDOFJointTrajectoryPoint timed_multi_dof_joint_trajectory_point;
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  timed_multi_dof_joint_trajectory_point.transforms = pose;
  timed_multi_dof_joint_trajectory.points.push_back(timed_multi_dof_joint_trajectory_point);
  EXPECT_FALSE(timed_multi_dof_joint_trajectory.Validate());
}

TEST(TestTimedMultiDOFJointTrajectoryNoPose, TestStructs) {
  TimedMultiDOFJointTrajectory timed_multi_dof_joint_trajectory;
  timed_multi_dof_joint_trajectory.joint_names.resize(1);
  timed_multi_dof_joint_trajectory.joint_names[0] = "joint";
  EXPECT_FALSE(timed_multi_dof_joint_trajectory.Validate());
}

TEST(TestTimedMultiDOFJointTrajectorySizeMismatch, TestStructs) {
  TimedMultiDOFJointTrajectory timed_multi_dof_joint_trajectory;
  timed_multi_dof_joint_trajectory.joint_names.resize(2);
  timed_multi_dof_joint_trajectory.joint_names[0] = "joint0";
  timed_multi_dof_joint_trajectory.joint_names[1] = "joint1";
  TimedMultiDOFJointTrajectoryPoint timed_multi_dof_joint_trajectory_point;
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  timed_multi_dof_joint_trajectory_point.transforms = pose;
  timed_multi_dof_joint_trajectory.points.push_back(timed_multi_dof_joint_trajectory_point);
  EXPECT_FALSE(timed_multi_dof_joint_trajectory.Validate());
}

TEST(TestTimedRobotTrajectoryValidate, TestStructs) {
  TimedJointTrajectory timed_joint_trajectory;
  timed_joint_trajectory.joint_names.resize(1);
  timed_joint_trajectory.joint_names[0] = "joint";
  TimedJointTrajectoryPoint timed_joint_trajectory_point;
  timed_joint_trajectory_point.positions.resize(1);
  timed_joint_trajectory_point.positions << 0.1;
  timed_joint_trajectory.points.push_back(timed_joint_trajectory_point);
  TimedMultiDOFJointTrajectory timed_multi_dof_joint_trajectory;
  timed_multi_dof_joint_trajectory.joint_names.resize(1);
  timed_multi_dof_joint_trajectory.joint_names[0] = "base";
  TimedMultiDOFJointTrajectoryPoint timed_multi_dof_joint_trajectory_point;
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  timed_multi_dof_joint_trajectory_point.transforms = pose;
  timed_multi_dof_joint_trajectory.points.push_back(timed_multi_dof_joint_trajectory_point);
  TimedRobotTrajectory timed_robot_trajectory;
  timed_robot_trajectory.joint_trajectory = timed_joint_trajectory;
  timed_robot_trajectory.multi_dof_joint_trajectory = timed_multi_dof_joint_trajectory;
  EXPECT_TRUE(timed_robot_trajectory.Validate());
}

TEST(TestTimedRobotTrajectoryJointTrajectoryFailure, TestStructs) {
  TimedJointTrajectory timed_joint_trajectory;
  TimedJointTrajectoryPoint timed_joint_trajectory_point;
  timed_joint_trajectory_point.positions.resize(1);
  timed_joint_trajectory_point.positions << 0.1;
  timed_joint_trajectory.points.push_back(timed_joint_trajectory_point);
  TimedMultiDOFJointTrajectory timed_multi_dof_joint_trajectory;
  timed_multi_dof_joint_trajectory.joint_names.resize(1);
  timed_multi_dof_joint_trajectory.joint_names[0] = "base";
  TimedMultiDOFJointTrajectoryPoint timed_multi_dof_joint_trajectory_point;
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  timed_multi_dof_joint_trajectory_point.transforms = pose;
  timed_multi_dof_joint_trajectory.points.push_back(timed_multi_dof_joint_trajectory_point);
  TimedRobotTrajectory timed_robot_trajectory;
  timed_robot_trajectory.joint_trajectory = timed_joint_trajectory;
  timed_robot_trajectory.multi_dof_joint_trajectory = timed_multi_dof_joint_trajectory;
  EXPECT_FALSE(timed_robot_trajectory.Validate());
}

TEST(TestTimedRobotTrajectoryMultiDOFJointTrajectoryFailure, TestStructs) {
  TimedJointTrajectory timed_joint_trajectory;
  timed_joint_trajectory.joint_names.resize(1);
  timed_joint_trajectory.joint_names[0] = "joint";
  TimedJointTrajectoryPoint timed_joint_trajectory_point;
  timed_joint_trajectory_point.positions.resize(1);
  timed_joint_trajectory_point.positions << 0.1;
  timed_joint_trajectory.points.push_back(timed_joint_trajectory_point);
  TimedMultiDOFJointTrajectory timed_multi_dof_joint_trajectory;
  TimedMultiDOFJointTrajectoryPoint timed_multi_dof_joint_trajectory_point;
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  timed_multi_dof_joint_trajectory_point.transforms = pose;
  timed_multi_dof_joint_trajectory.points.push_back(timed_multi_dof_joint_trajectory_point);
  TimedRobotTrajectory timed_robot_trajectory;
  timed_robot_trajectory.joint_trajectory = timed_joint_trajectory;
  timed_robot_trajectory.multi_dof_joint_trajectory = timed_multi_dof_joint_trajectory;
  EXPECT_FALSE(timed_robot_trajectory.Validate());
}

TEST(TestTimedRobotTrajectorySizeMismatch, TestStructs) {
  TimedJointTrajectory timed_joint_trajectory;
  TimedJointTrajectoryPoint timed_joint_trajectory_point;
  timed_joint_trajectory_point.positions.resize(1);
  timed_joint_trajectory_point.positions << 0.1;
  timed_joint_trajectory.points.push_back(timed_joint_trajectory_point);
  timed_joint_trajectory.points.push_back(timed_joint_trajectory_point);
  TimedMultiDOFJointTrajectory timed_multi_dof_joint_trajectory;
  TimedMultiDOFJointTrajectoryPoint timed_multi_dof_joint_trajectory_point;
  tmc_manipulation_types::PoseSeq pose;
  pose.push_back(Eigen::Affine3d::Identity());
  timed_multi_dof_joint_trajectory_point.transforms = pose;
  timed_multi_dof_joint_trajectory.points.push_back(timed_multi_dof_joint_trajectory_point);
  TimedRobotTrajectory timed_robot_trajectory;
  timed_robot_trajectory.joint_trajectory = timed_joint_trajectory;
  timed_robot_trajectory.multi_dof_joint_trajectory = timed_multi_dof_joint_trajectory;
  EXPECT_FALSE(timed_robot_trajectory.Validate());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
