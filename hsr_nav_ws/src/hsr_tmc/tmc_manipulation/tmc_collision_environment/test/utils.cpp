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

#include "utils.hpp"

namespace {
constexpr double kEpsilon = 1.0e-3;
}  // namespace

namespace tmc_collision_environment {

moveit_msgs::msg::CollisionObject CreateTestObject(const std::string& id, const std::string& ref_frame_id) {
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = ref_frame_id;
  object.pose.position.x = 1.0;
  object.pose.position.y = 2.0;
  object.pose.position.z = 3.0;
  object.pose.orientation.x = 0.0;
  object.pose.orientation.y = 0.0;
  object.pose.orientation.z = 1.0;
  object.pose.orientation.w = 0.0;
  object.id = id;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
  object.primitives[0].dimensions = {4.0};
  object.primitive_poses.resize(1);
  object.primitive_poses[0].position.x = 0.1;
  object.primitive_poses[0].position.y = 0.2;
  object.primitive_poses[0].position.z = 0.3;
  object.primitive_poses[0].orientation.x = 1.0;
  object.primitive_poses[0].orientation.y = 0.0;
  object.primitive_poses[0].orientation.z = 0.0;
  object.primitive_poses[0].orientation.w = 0.0;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

void IsEqual(const geometry_msgs::msg::Point& input_1, const geometry_msgs::msg::Point& input_2) {
  SCOPED_TRACE("");
  EXPECT_NEAR(input_1.x, input_2.x, kEpsilon);
  EXPECT_NEAR(input_1.y, input_2.y, kEpsilon);
  EXPECT_NEAR(input_1.z, input_2.z, kEpsilon);
}

void IsEqual(const geometry_msgs::msg::Pose& input_1, const geometry_msgs::msg::Pose& input_2) {
  SCOPED_TRACE("");
  IsEqual(input_1.position, input_2.position);
  EXPECT_NEAR(input_1.orientation.x, input_2.orientation.x, kEpsilon);
  EXPECT_NEAR(input_1.orientation.y, input_2.orientation.y, kEpsilon);
  EXPECT_NEAR(input_1.orientation.z, input_2.orientation.z, kEpsilon);
  EXPECT_NEAR(input_1.orientation.w, input_2.orientation.w, kEpsilon);
}

void IsEqual(const geometry_msgs::msg::Pose& input_1, const Eigen::Affine3d& input_2) {
  SCOPED_TRACE("");
  EXPECT_NEAR(input_1.position.x, input_2.translation().x(), kEpsilon);
  EXPECT_NEAR(input_1.position.y, input_2.translation().y(), kEpsilon);
  EXPECT_NEAR(input_1.position.z, input_2.translation().z(), kEpsilon);
  EXPECT_NEAR(input_1.orientation.x, Eigen::Quaterniond(input_2.linear()).x(), kEpsilon);
  EXPECT_NEAR(input_1.orientation.y, Eigen::Quaterniond(input_2.linear()).y(), kEpsilon);
  EXPECT_NEAR(input_1.orientation.z, Eigen::Quaterniond(input_2.linear()).z(), kEpsilon);
  EXPECT_NEAR(input_1.orientation.w, Eigen::Quaterniond(input_2.linear()).w(), kEpsilon);
}

void IsEqual(const moveit_msgs::msg::CollisionObject& input_1, const moveit_msgs::msg::CollisionObject& input_2) {
  SCOPED_TRACE("");
  EXPECT_EQ(input_1.header.frame_id, input_2.header.frame_id);
  IsEqual(input_1.pose, input_2.pose);
  EXPECT_EQ(input_1.id, input_2.id);
  ASSERT_EQ(input_1.primitives.size(), input_2.primitives.size());
  EXPECT_EQ(input_1.primitives[0].type, input_2.primitives[0].type);
  EXPECT_EQ(input_1.primitives[0].dimensions, input_2.primitives[0].dimensions);
  ASSERT_EQ(input_1.primitive_poses.size(), input_2.primitive_poses.size());
  IsEqual(input_1.primitive_poses[0], input_2.primitive_poses[0]);
}

}  // namespace tmc_collision_environment
