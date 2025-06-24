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
#ifndef TMC_COLLISION_ENVIRONMENT_TEST_UTILS_HPP_
#define TMC_COLLISION_ENVIRONMENT_TEST_UTILS_HPP_

#include <string>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

namespace tmc_collision_environment {

moveit_msgs::msg::CollisionObject CreateTestObject(const std::string& id, const std::string& ref_frame_id = "odom");

void IsEqual(const geometry_msgs::msg::Point& input_1, const geometry_msgs::msg::Point& input_2);
void IsEqual(const geometry_msgs::msg::Pose& input_1, const geometry_msgs::msg::Pose& input_2);
void IsEqual(const geometry_msgs::msg::Pose& input_1, const Eigen::Affine3d& input_2);
void IsEqual(const moveit_msgs::msg::CollisionObject& input_1, const moveit_msgs::msg::CollisionObject& input_2);

}  // namespace tmc_collision_environment
#endif  // TMC_COLLISION_ENVIRONMENT_TEST_UTILS_HPP_
