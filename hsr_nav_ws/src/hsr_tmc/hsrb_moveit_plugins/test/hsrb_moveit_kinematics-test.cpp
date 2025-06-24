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

#include <gtest/gtest.h>

#include <tmc_manipulation_tests/configs.hpp>

#include "../src/hsrb_moveit_kinematics.hpp"

namespace {
void DeclareRobotDescription(const rclcpp::Node::SharedPtr& node) {
  node->declare_parameter("robot_description", tmc_manipulation_tests::hsrb::GetUrdf());
}
}  // namespace

namespace hsrb_moveit_kinematics {

class HSRBKinematicsPluginTest : public ::testing::Test {
 public:
  void SetUp() override;

 protected:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr robot_model_;
};

void HSRBKinematicsPluginTest::SetUp() {
  node_ = rclcpp::Node::make_shared("test_node");
  DeclareRobotDescription(node_);
}

// Initialize test
TEST_F(HSRBKinematicsPluginTest, initialize) {
  {
    // Successful even if you don't set anything
    HSRBKinematicsPlugin p;
    EXPECT_TRUE(p.initialize(node_, *robot_model_, "group", "odom", {"hand_palm_link"}, 0.0));
  }
}

// GetLinknames test
TEST_F(HSRBKinematicsPluginTest, getLinkNames) {
  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize(node_, *robot_model_, "group", "odom", {"hand_palm_link"}, 0.0));

  const std::vector<std::string>& link_names = p.getLinkNames();
  EXPECT_EQ(link_names.size(), 8);
  EXPECT_NE(std::find(link_names.begin(), link_names.end(), "odom_xr_link"), link_names.end());
  EXPECT_NE(std::find(link_names.begin(), link_names.end(), "odom_yx_link"), link_names.end());
  EXPECT_NE(std::find(link_names.begin(), link_names.end(), "base_footprint"), link_names.end());
  EXPECT_NE(std::find(link_names.begin(), link_names.end(), "arm_lift_link"), link_names.end());
  EXPECT_NE(std::find(link_names.begin(), link_names.end(), "arm_flex_link"), link_names.end());
  EXPECT_NE(std::find(link_names.begin(), link_names.end(), "arm_roll_link"), link_names.end());
  EXPECT_NE(std::find(link_names.begin(), link_names.end(), "wrist_flex_link"), link_names.end());
  EXPECT_NE(std::find(link_names.begin(), link_names.end(), "wrist_roll_link"), link_names.end());
}

// GetJointNames test
TEST_F(HSRBKinematicsPluginTest, getJointNames) {
  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize(node_, *robot_model_, "group", "odom", {"hand_palm_link"}, 0.0));

  const std::vector<std::string>& joint_names = p.getJointNames();
  EXPECT_EQ(joint_names.size(), 8);
  EXPECT_NE(std::find(joint_names.begin(), joint_names.end(), "odom_x"), joint_names.end());
  EXPECT_NE(std::find(joint_names.begin(), joint_names.end(), "odom_y"), joint_names.end());
  EXPECT_NE(std::find(joint_names.begin(), joint_names.end(), "odom_t"), joint_names.end());
  EXPECT_NE(std::find(joint_names.begin(), joint_names.end(), "arm_lift_joint"), joint_names.end());
  EXPECT_NE(std::find(joint_names.begin(), joint_names.end(), "arm_flex_joint"), joint_names.end());
  EXPECT_NE(std::find(joint_names.begin(), joint_names.end(), "arm_roll_joint"), joint_names.end());
  EXPECT_NE(std::find(joint_names.begin(), joint_names.end(), "wrist_flex_joint"), joint_names.end());
  EXPECT_NE(std::find(joint_names.begin(), joint_names.end(), "wrist_roll_joint"), joint_names.end());
}

// SupportSgroup test
TEST_F(HSRBKinematicsPluginTest, supportsGroup) {
  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize(node_, *robot_model_, "group", "odom", {"hand_palm_link"}, 0.0));
  EXPECT_TRUE(p.supportsGroup(NULL, NULL));
}

// Solve IK with the default weight
TEST_F(HSRBKinematicsPluginTest, searchPositionIKWithDefaultWeight) {
  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize(node_, *robot_model_, "group", "odom", {"hand_palm_link"}, 0.0));

  geometry_msgs::msg::Pose ik_pose;
  ik_pose.position.x = 1.0;
  ik_pose.position.y = 0.0;
  ik_pose.position.z = 1.0;
  ik_pose.orientation.x = 0.0;
  ik_pose.orientation.y = 0.0;
  ik_pose.orientation.z = 1.0;
  ik_pose.orientation.w = 0.0;

  std::vector<double> ik_seed_state(8, 0.0);

  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  EXPECT_TRUE(p.searchPositionIK(ik_pose, ik_seed_state, 0, solution, error_code));
  // I made the answer by displaying it
  ASSERT_EQ(8, solution.size());
  EXPECT_NEAR(0.838948,    solution[0], 1e-6);
  EXPECT_NEAR(-0.0726981,  solution[1], 1e-6);
  EXPECT_NEAR(-0.0358925,  solution[2], 1e-6);
  EXPECT_NEAR(0.174505,    solution[3], 1e-6);
  EXPECT_NEAR(-0.00100738, solution[4], 1e-6);
  EXPECT_NEAR(-0,          solution[5], 1e-6);
  EXPECT_NEAR(0.00100738,  solution[6], 1e-6);
  EXPECT_NEAR(0.0358925,   solution[7], 1e-6);
}

// Solve with a custom weight
TEST_F(HSRBKinematicsPluginTest, searchPositionIKWithDeclaredWeight) {
  node_->declare_parameter<std::vector<double>>("weight_test.joint_weight", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize(node_, *robot_model_, "weight_test", "odom", {"hand_palm_link"}, 0.0));

  geometry_msgs::msg::Pose ik_pose;
  ik_pose.position.x = 1.0;
  ik_pose.position.y = 0.0;
  ik_pose.position.z = 1.0;
  ik_pose.orientation.x = 0.0;
  ik_pose.orientation.y = 0.0;
  ik_pose.orientation.z = 1.0;
  ik_pose.orientation.w = 0.0;

  std::vector<double> ik_seed_state(8, 0.0);

  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  EXPECT_TRUE(p.searchPositionIK(ik_pose, ik_seed_state, 0, solution, error_code));
  // I made the answer by displaying it
  ASSERT_EQ(8, solution.size());
  EXPECT_NEAR(0.794193,    solution[0], 1e-6);
  EXPECT_NEAR(-0.0712208,  solution[1], 1e-6);
  EXPECT_NEAR(-0.0352144,  solution[2], 1e-6);
  EXPECT_NEAR(0.178132,    solution[3], 1e-6);
  EXPECT_NEAR(-0.131457,   solution[4], 1e-6);
  EXPECT_NEAR(-0,          solution[5], 1e-6);
  EXPECT_NEAR(0.131457,    solution[6], 1e-6);
  EXPECT_NEAR(0.0352144,   solution[7], 1e-6);
}

// If the solution is not
TEST_F(HSRBKinematicsPluginTest, searchPositionIKNoSolution) {
  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize(node_, *robot_model_, "group", "odom", {"hand_palm_link"}, 0.0));

  geometry_msgs::msg::Pose ik_pose;
  ik_pose.position.x = 0.0;
  ik_pose.position.y = 0.0;
  ik_pose.position.z = 3.0;
  ik_pose.orientation.x = 0.0;
  ik_pose.orientation.y = 0.0;
  ik_pose.orientation.z = 0.0;
  ik_pose.orientation.w = 1.0;

  std::vector<double> ik_seed_state(8, 0.0);

  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  EXPECT_FALSE(p.searchPositionIK(ik_pose, ik_seed_state, 0, solution, error_code));
}

// Not Active
TEST_F(HSRBKinematicsPluginTest, searchPositionIKNotActive) {
  HSRBKinematicsPlugin p;
  EXPECT_FALSE(p.initialize(node_, *robot_model_, "group", "baselink", {"hand_palm_link"}, 0.0));

  geometry_msgs::msg::Pose ik_pose;
  ik_pose.position.x = 1.0;
  ik_pose.position.y = 0.0;
  ik_pose.position.z = 1.0;
  ik_pose.orientation.x = 0.0;
  ik_pose.orientation.y = 0.0;
  ik_pose.orientation.z = 1.0;
  ik_pose.orientation.w = 0.0;

  std::vector<double> ik_seed_state(8, 0.0);

  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  EXPECT_FALSE(p.searchPositionIK(ik_pose, ik_seed_state, 0, solution, error_code));
}

// Dimension is invalid
TEST_F(HSRBKinematicsPluginTest, searchPositionIKInvalidDimension) {
  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize(node_, *robot_model_, "group", "odom", {"hand_palm_link"}, 0.0));

  geometry_msgs::msg::Pose ik_pose;
  ik_pose.position.x = 1.0;
  ik_pose.position.y = 0.0;
  ik_pose.position.z = 1.0;
  ik_pose.orientation.x = 0.0;
  ik_pose.orientation.y = 0.0;
  ik_pose.orientation.z = 1.0;
  ik_pose.orientation.w = 0.0;

  std::vector<double> ik_seed_state(7, 0.0);

  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  EXPECT_FALSE(p.searchPositionIK(ik_pose, ik_seed_state, 0, solution, error_code));
}

// CONSISTENCY_LIMITS size is invalidated
TEST_F(HSRBKinematicsPluginTest, searchPositionIKInvalidConsistencyLimits) {
  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize(node_, *robot_model_, "group", "odom", {"hand_palm_link"}, 0.0));

  geometry_msgs::msg::Pose ik_pose;
  ik_pose.position.x = 1.0;
  ik_pose.position.y = 0.0;
  ik_pose.position.z = 1.0;
  ik_pose.orientation.x = 0.0;
  ik_pose.orientation.y = 0.0;
  ik_pose.orientation.z = 1.0;
  ik_pose.orientation.w = 0.0;

  std::vector<double> ik_seed_state(8, 0.0);

  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  EXPECT_FALSE(p.searchPositionIK(ik_pose, ik_seed_state, 0, std::vector<double>(2, 0.0), solution, error_code));
}

// getPositionFK
TEST_F(HSRBKinematicsPluginTest, getPositionIK) {
  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize(node_, *robot_model_, "group", "odom", {"hand_palm_link"}, 0.0));

  std::vector<double> joint_angles(8, 0.0);

  // FK is successful without giving Link_names
  std::vector<geometry_msgs::msg::Pose> poses;
  EXPECT_TRUE(p.getPositionFK({}, joint_angles, poses));

  EXPECT_TRUE(p.getPositionFK({"hand_palm_link"}, joint_angles, poses));
  ASSERT_EQ(poses.size(), 1);

  // The answer was made by displaying it
  EXPECT_NEAR(poses[0].position.x, 0.158, 1e-6);
  EXPECT_NEAR(poses[0].position.y, 0.078, 1e-6);
  EXPECT_NEAR(poses[0].position.z, 0.8255, 1e-6);
  EXPECT_NEAR(poses[0].orientation.x, 0, 1e-6);
  EXPECT_NEAR(poses[0].orientation.y, 0, 1e-6);
  EXPECT_NEAR(std::abs(poses[0].orientation.z), 1, 1e-6);
  EXPECT_NEAR(poses[0].orientation.w, 0, 1e-6);

  // Base_footPrint is also FK
  joint_angles[0] = 1.0;
  EXPECT_TRUE(p.getPositionFK({"hand_palm_link", "base_footprint"}, joint_angles, poses));
  ASSERT_EQ(poses.size(), 2);

  EXPECT_NEAR(poses[0].position.x, 1.158, 1e-6);
  EXPECT_NEAR(poses[0].position.y, 0.078, 1e-6);
  EXPECT_NEAR(poses[0].position.z, 0.8255, 1e-6);
  EXPECT_NEAR(poses[0].orientation.x, 0, 1e-6);
  EXPECT_NEAR(poses[0].orientation.y, 0, 1e-6);
  EXPECT_NEAR(std::abs(poses[0].orientation.z), 1, 1e-6);
  EXPECT_NEAR(poses[0].orientation.w, 0, 1e-6);

  EXPECT_NEAR(poses[1].position.x, 1, 1e-6);
  EXPECT_NEAR(poses[1].position.y, 0, 1e-6);
  EXPECT_NEAR(poses[1].position.z, 0, 1e-6);
  EXPECT_NEAR(poses[1].orientation.x, 0, 1e-6);
  EXPECT_NEAR(poses[1].orientation.y, 0, 1e-6);
  EXPECT_NEAR(poses[1].orientation.z, 0, 1e-6);
  EXPECT_NEAR(poses[1].orientation.w, 1, 1e-6);

  // Failure if it is a link that does not exist
  EXPECT_FALSE(p.getPositionFK({"dummy"}, joint_angles, poses));
  // FK fails without eight joint angles
  EXPECT_FALSE(p.getPositionFK({"hand_palm_link"}, std::vector<double>(7, 0.0), poses));
  EXPECT_FALSE(p.getPositionFK({"hand_palm_link"}, std::vector<double>(9, 0.0), poses));
}

}  // namespace hsrb_moveit_kinematics

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
