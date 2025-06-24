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
/// @brief test for utils

#include <vector>
#include <gtest/gtest.h>
#include <tmc_manipulation_types/utils.hpp>

namespace tmc_manipulation_types {

class UtilsTest : public ::testing::Test {
 protected:
  virtual void SetUp();
  JointState joint_state_;
  TimedJointTrajectory trajectory_;
};

void UtilsTest::SetUp() {
  NameSeq joint_names{"arm", "wrist", "shoulder"};
  Eigen::VectorXd init_vector = Eigen::VectorXd::LinSpaced(joint_names.size(),
                                                            0.1,
                                                            0.1 * joint_names.size());
  trajectory_.points.resize(joint_names.size());
  for (int32_t i = 0; i < 3; ++i) {
    trajectory_.joint_names.push_back(joint_names[i]);
    trajectory_.points[i].time_from_start = (i + 1) * 1.0;
    trajectory_.points[i].positions.resize(joint_names.size());
    trajectory_.points[i].positions = init_vector * (i + 1);
    trajectory_.points[i].velocities.resize(joint_names.size());
    trajectory_.points[i].velocities = init_vector * (i + 2);
    trajectory_.points[i].accelerations.resize(joint_names.size());
    trajectory_.points[i].accelerations = init_vector * (i + 3);
    trajectory_.points[i].effort.resize(joint_names.size());
    trajectory_.points[i].effort = init_vector * (i + 4);
  }

  joint_state_.name = joint_names;
  joint_state_.position = init_vector;
  joint_state_.velocity = init_vector * 2;
  joint_state_.effort = init_vector * 3;
}

TEST_F(UtilsTest, ExtractPartialJointState) {
  NameSeq use_name = {"arm", "shoulder"};
  std::vector<double> result_list = {0.1, 0.3};
  JointState partial_joint_state = ExtractPartialJointState(joint_state_, use_name);
  ASSERT_EQ(partial_joint_state.name.size(), use_name.size());
  for (uint32_t i = 0; i < use_name.size(); ++i) {
    EXPECT_EQ(partial_joint_state.name[i], use_name[i]);
    EXPECT_DOUBLE_EQ(partial_joint_state.position[i], result_list[i]);
    EXPECT_DOUBLE_EQ(partial_joint_state.velocity[i], result_list[i] * 2);
    EXPECT_DOUBLE_EQ(partial_joint_state.effort[i], result_list[i] * 3);
  }
}

TEST_F(UtilsTest, NotExistJointNameOfJointState) {
  NameSeq use_name = {"test"};
  JointState partial_joint_state = ExtractPartialJointState(joint_state_, use_name);
  ASSERT_EQ(partial_joint_state.name.size(), 0);
  EXPECT_EQ(partial_joint_state.position, Eigen::VectorXd());
  EXPECT_EQ(partial_joint_state.velocity, Eigen::VectorXd());
  EXPECT_EQ(partial_joint_state.effort, Eigen::VectorXd());
}

TEST_F(UtilsTest, MixedNotExistJointNameOfJointState) {
  NameSeq use_name = {"arm", "test"};
  JointState partial_joint_state = ExtractPartialJointState(joint_state_, use_name);
  ASSERT_EQ(partial_joint_state.name.size(), 0);
  EXPECT_EQ(partial_joint_state.position, Eigen::VectorXd());
  EXPECT_EQ(partial_joint_state.velocity, Eigen::VectorXd());
  EXPECT_EQ(partial_joint_state.effort, Eigen::VectorXd());
}

TEST_F(UtilsTest, EmptyJointNameOfJointState) {
  NameSeq use_name = {};
  JointState partial_joint_state = ExtractPartialJointState(joint_state_, use_name);
  ASSERT_EQ(partial_joint_state.name.size(), 0);
  EXPECT_EQ(partial_joint_state.position, Eigen::VectorXd());
  EXPECT_EQ(partial_joint_state.velocity, Eigen::VectorXd());
  EXPECT_EQ(partial_joint_state.effort, Eigen::VectorXd());
}

TEST_F(UtilsTest, PartOfJointStateIsEmptyOfJointState) {
  joint_state_.effort = Eigen::VectorXd();
  NameSeq use_name = {"arm"};
  JointState partial_joint_state = ExtractPartialJointState(joint_state_, use_name);
  ASSERT_EQ(partial_joint_state.name.size(), 1);
  EXPECT_EQ(partial_joint_state.name[0], use_name[0]);
  EXPECT_DOUBLE_EQ(partial_joint_state.position[0], 0.1);
  EXPECT_DOUBLE_EQ(partial_joint_state.velocity[0], 0.2);
  EXPECT_EQ(partial_joint_state.effort, Eigen::VectorXd());
}

TEST_F(UtilsTest, ExtractPartialJointTrajectory) {
  NameSeq use_name = {"arm", "shoulder"};
  std::vector<double> result_list = {0.1, 0.3};
  TimedJointTrajectory partial_joint_trajectory = ExtractPartialJointTrajectory(
      trajectory_, use_name);
  for (uint32_t i = 0; i < use_name.size(); ++i) {
    EXPECT_EQ(partial_joint_trajectory.joint_names[i], use_name[i]);
  }
  for (uint32_t i = 0; i < partial_joint_trajectory.points.size(); ++i) {
    for (uint32_t j = 0; j < use_name.size(); ++j) {
      EXPECT_DOUBLE_EQ(partial_joint_trajectory.points[i].positions[j], result_list[j] * (i + 1));
      EXPECT_DOUBLE_EQ(partial_joint_trajectory.points[i].velocities[j], result_list[j] * (i + 2));
      EXPECT_DOUBLE_EQ(partial_joint_trajectory.points[i].accelerations[j], result_list[j] * (i + 3));
      EXPECT_DOUBLE_EQ(partial_joint_trajectory.points[i].effort[j], result_list[j] * (i + 4));
    }
    EXPECT_DOUBLE_EQ(partial_joint_trajectory.points[i].time_from_start, (i + 1) * 1.0);
  }
}

TEST_F(UtilsTest, NotExistJointNameOfJointTrajectory) {
  NameSeq use_name = {"test"};
  TimedJointTrajectory partial_joint_trajectory = ExtractPartialJointTrajectory(
      trajectory_, use_name);
  EXPECT_EQ(partial_joint_trajectory.joint_names.size(), 0);
  for (uint32_t i = 0; i < partial_joint_trajectory.points.size(); ++i) {
    for (uint32_t j = 0; j < use_name.size(); ++j) {
      EXPECT_EQ(partial_joint_trajectory.points[i].positions, Eigen::VectorXd());
      EXPECT_EQ(partial_joint_trajectory.points[i].velocities, Eigen::VectorXd());
      EXPECT_EQ(partial_joint_trajectory.points[i].accelerations, Eigen::VectorXd());
      EXPECT_EQ(partial_joint_trajectory.points[i].effort, Eigen::VectorXd());
    }
  }
}

TEST_F(UtilsTest, MixedNotExistJointNameOfJointTrajectory) {
  NameSeq use_name = {"arm", "test"};
  TimedJointTrajectory partial_joint_trajectory = ExtractPartialJointTrajectory(
      trajectory_, use_name);
  EXPECT_EQ(partial_joint_trajectory.joint_names.size(), 0);
  for (uint32_t i = 0; i < partial_joint_trajectory.points.size(); ++i) {
    EXPECT_EQ(partial_joint_trajectory.points[i].positions, Eigen::VectorXd());
    EXPECT_EQ(partial_joint_trajectory.points[i].velocities, Eigen::VectorXd());
    EXPECT_EQ(partial_joint_trajectory.points[i].accelerations, Eigen::VectorXd());
    EXPECT_EQ(partial_joint_trajectory.points[i].effort, Eigen::VectorXd());
  }
}

TEST_F(UtilsTest, EmptyJointNameOfJointTrajectory) {
  NameSeq use_name = {};
  TimedJointTrajectory partial_joint_trajectory = ExtractPartialJointTrajectory(
      trajectory_, use_name);
  EXPECT_EQ(partial_joint_trajectory.joint_names.size(), 0);
  for (uint32_t i = 0; i < partial_joint_trajectory.points.size(); ++i) {
    EXPECT_EQ(partial_joint_trajectory.points[i].positions, Eigen::VectorXd());
    EXPECT_EQ(partial_joint_trajectory.points[i].velocities, Eigen::VectorXd());
    EXPECT_EQ(partial_joint_trajectory.points[i].accelerations, Eigen::VectorXd());
    EXPECT_EQ(partial_joint_trajectory.points[i].effort, Eigen::VectorXd());
  }
}

TEST_F(UtilsTest, PartOfJointNameIsEmptyOfJointTrajectory) {
  for (uint32_t i = 0; i < trajectory_.points.size(); ++i) {
    trajectory_.points[i].effort = Eigen::VectorXd();
  }
  NameSeq use_name = {"arm"};
  TimedJointTrajectory partial_joint_trajectory = ExtractPartialJointTrajectory(
      trajectory_, use_name);

  EXPECT_EQ(partial_joint_trajectory.joint_names.size(), 1);
  EXPECT_EQ(partial_joint_trajectory.joint_names[0], use_name[0]);
  for (uint32_t i = 0; i < partial_joint_trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(partial_joint_trajectory.points[i].positions[0], 0.1 * (i + 1));
    EXPECT_DOUBLE_EQ(partial_joint_trajectory.points[i].velocities[0], 0.1 * (i + 2));
    EXPECT_DOUBLE_EQ(partial_joint_trajectory.points[i].accelerations[0], 0.1 * (i + 3));
    EXPECT_EQ(partial_joint_trajectory.points[i].effort, Eigen::VectorXd());
  }
}

TEST(TestGetJointIndex, TestFunctions) {
  NameSeq names(3);
  names[0] = "joint0";
  names[1] = "joint1";
  names[2] = "joint2";

  EXPECT_EQ(0, GetJointIndex(names, "joint0"));
  EXPECT_EQ(1, GetJointIndex(names, "joint1"));
  EXPECT_EQ(2, GetJointIndex(names, "joint2"));
  EXPECT_EQ(-1, GetJointIndex(names, "joint3"));
}

TEST(TestExtractJointPos, TestFunctions) {
  JointState state;
  NameSeq names(2);
  state.name.resize(3);
  state.name[0] = "joint0";
  state.name[1] = "joint1";
  state.name[2] = "joint2";
  state.position.resize(3);
  state.position << 0, 1, 2;
  names[0] = "joint1";
  names[1] = "joint2";
  Eigen::VectorXd pos;
  ExtractJointPos(state, names, pos);
  ASSERT_EQ(2, pos.size());
  EXPECT_DOUBLE_EQ(1.0, pos[0]);
  EXPECT_DOUBLE_EQ(2.0, pos[1]);
}

TEST(TestExtractJointPosThrow, TestFunctions) {
  JointState state;
  NameSeq names(2);
  state.name.resize(3);
  state.name[0] = "joint0";
  state.name[1] = "joint1";
  state.name[2] = "joint2";
  state.position.resize(3);
  state.position << 0, 1, 2;
  names[0] = "joint3";
  names[1] = "joint4";
  Eigen::VectorXd pos;
  EXPECT_THROW(
      ExtractJointPos(state, names, pos),
      std::invalid_argument);
}

TEST(TestExtractJointPosNoJointThrow, TestFunctions) {
  JointState state;
  NameSeq names(2);
  state.name.resize(3);
  state.name[0] = "joint0";
  state.name[1] = "joint1";
  state.name[2] = "joint2";
  state.position.resize(3);
  state.position << 0, 1, 2;
  names[0] = "joint3";
  names[1] = "joint4";
  Eigen::VectorXd pos;
  EXPECT_THROW(
      ExtractJointPos(state, names, pos),
      std::invalid_argument);
}

TEST(TestExtractJointPosNoPosThrow, TestFunctions) {
  JointState state;
  NameSeq names(1);
  state.name.resize(3);
  state.name[0] = "joint0";
  state.name[1] = "joint1";
  state.name[2] = "joint2";
  state.position.resize(2);
  state.position << 0, 1;
  names[0] = "joint2";
  Eigen::VectorXd pos;
  EXPECT_THROW(
      ExtractJointPos(state, names, pos),
      std::invalid_argument);
}

TEST(TestExtractMultiJointPos, TestFunctions) {
  MultiDOFJointState state;
  NameSeq names(2);
  state.names.resize(3);
  state.names[0] = "joint0";
  state.names[1] = "joint1";
  state.names[2] = "joint2";
  state.poses.resize(3);
  state.poses[0].translation()[0] = 0;
  state.poses[1].translation()[0] = 1;
  state.poses[2].translation()[0] = 2;
  names[0] = "joint1";
  names[1] = "joint2";
  tmc_manipulation_types::PoseSeq poses;
  ExtractMultiJointPos(state, names, poses);
  EXPECT_DOUBLE_EQ(1.0, poses[0].translation()[0]);
  EXPECT_DOUBLE_EQ(2.0, poses[1].translation()[0]);
}

TEST(TestExtractMultiJointPosNoJointThrow, TestFunctions) {
  MultiDOFJointState state;
  NameSeq names(2);
  state.names.resize(3);
  state.names[0] = "joint0";
  state.names[1] = "joint1";
  state.names[2] = "joint2";
  state.poses.resize(3);
  names[0] = "joint3";
  names[1] = "joint4";
  tmc_manipulation_types::PoseSeq poses;
  EXPECT_THROW(
      ExtractMultiJointPos(state, names, poses),
      std::invalid_argument);
}

TEST(TestExtractMultiJointNoPosThrow, TestFunctions) {
  MultiDOFJointState state;
  NameSeq names(1);
  state.names.resize(3);
  state.names[0] = "joint0";
  state.names[1] = "joint1";
  state.names[2] = "joint2";
  state.poses.resize(2);
  names[0] = "joint2";
  tmc_manipulation_types::PoseSeq poses(2);
  EXPECT_THROW(
      ExtractMultiJointPos(state, names, poses),
      std::invalid_argument);
}

}  // namespace tmc_manipulation_types

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

