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
#ifndef TMC_MANIPULATION_TYPES_BRIDGE_TEST_EQUALITY_CHECK_HPP_
#define TMC_MANIPULATION_TYPES_BRIDGE_TEST_EQUALITY_CHECK_HPP_

#include <array>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <tmc_manipulation_types/manipulation_types.hpp>


namespace tmc_manipulation_types_bridge {

void IsEqual(const std::vector<double>& expected, const Eigen::VectorXd& actual) {
  SCOPED_TRACE("");
  ASSERT_EQ(expected.size(), actual.size());
  for (auto i = 0; i < expected.size(); ++i) {
    EXPECT_DOUBLE_EQ(expected[i], actual[i]);
  }
}

void IsEqual(const Eigen::VectorXd& expected, const std::vector<double>& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}

void IsEqual(const std::array<double, 6>& expected, const Eigen::VectorXd& actual) {
  SCOPED_TRACE("");
  const std::vector<double> expected_vec(expected.cbegin(), expected.cend());
  IsEqual(expected_vec, actual);
}

void IsEqual(const Eigen::VectorXd& expected, const std::array<double, 6>& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}

void IsEqual(const geometry_msgs::msg::Pose& expected, const Eigen::Affine3d& actual) {
  SCOPED_TRACE("");
  EXPECT_DOUBLE_EQ(expected.position.x, actual.translation().x());
  EXPECT_DOUBLE_EQ(expected.position.y, actual.translation().y());
  EXPECT_DOUBLE_EQ(expected.position.z, actual.translation().z());
  EXPECT_DOUBLE_EQ(expected.orientation.x, Eigen::Quaterniond(actual.linear()).x());
  EXPECT_DOUBLE_EQ(expected.orientation.y, Eigen::Quaterniond(actual.linear()).y());
  EXPECT_DOUBLE_EQ(expected.orientation.z, Eigen::Quaterniond(actual.linear()).z());
  EXPECT_DOUBLE_EQ(expected.orientation.w, Eigen::Quaterniond(actual.linear()).w());
}

void IsEqual(const Eigen::Affine3d& expected, const geometry_msgs::msg::Pose& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}

void IsEqual(const geometry_msgs::msg::Transform& expected, const Eigen::Affine3d& actual) {
  SCOPED_TRACE("");
  EXPECT_DOUBLE_EQ(expected.translation.x, actual.translation().x());
  EXPECT_DOUBLE_EQ(expected.translation.y, actual.translation().y());
  EXPECT_DOUBLE_EQ(expected.translation.z, actual.translation().z());
  EXPECT_DOUBLE_EQ(expected.rotation.x, Eigen::Quaterniond(actual.linear()).x());
  EXPECT_DOUBLE_EQ(expected.rotation.y, Eigen::Quaterniond(actual.linear()).y());
  EXPECT_DOUBLE_EQ(expected.rotation.z, Eigen::Quaterniond(actual.linear()).z());
  EXPECT_DOUBLE_EQ(expected.rotation.w, Eigen::Quaterniond(actual.linear()).w());
}

void IsEqual(const Eigen::Affine3d& expected, const geometry_msgs::msg::Transform& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}

void IsEqual(const geometry_msgs::msg::Twist& expected, const Eigen::Matrix<double, 6, 1>& actual) {
  SCOPED_TRACE("");
  EXPECT_DOUBLE_EQ(expected.linear.x, actual[0]);
  EXPECT_DOUBLE_EQ(expected.linear.y, actual[1]);
  EXPECT_DOUBLE_EQ(expected.linear.z, actual[2]);
  EXPECT_DOUBLE_EQ(expected.angular.x, actual[3]);
  EXPECT_DOUBLE_EQ(expected.angular.y, actual[4]);
  EXPECT_DOUBLE_EQ(expected.angular.z, actual[5]);
}

void IsEqual(const Eigen::Matrix<double, 6, 1>& expected, const geometry_msgs::msg::Twist& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}

void IsEqual(const geometry_msgs::msg::Wrench& expected, const Eigen::Matrix<double, 6, 1>& actual) {
  SCOPED_TRACE("");
  EXPECT_DOUBLE_EQ(expected.force.x, actual[0]);
  EXPECT_DOUBLE_EQ(expected.force.y, actual[1]);
  EXPECT_DOUBLE_EQ(expected.force.z, actual[2]);
  EXPECT_DOUBLE_EQ(expected.torque.x, actual[3]);
  EXPECT_DOUBLE_EQ(expected.torque.y, actual[4]);
  EXPECT_DOUBLE_EQ(expected.torque.z, actual[5]);
}

void IsEqual(const Eigen::Matrix<double, 6, 1>& expected, const geometry_msgs::msg::Wrench& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}

void IsEqual(const tmc_manipulation_types::TimedJointTrajectoryPoint& expected,
             const trajectory_msgs::msg::JointTrajectoryPoint& actual) {
  SCOPED_TRACE("");
  IsEqual(expected.positions, actual.positions);
  IsEqual(expected.velocities, actual.velocities);
  IsEqual(expected.accelerations, actual.accelerations);
  IsEqual(expected.effort, actual.effort);
  EXPECT_DOUBLE_EQ(expected.time_from_start, rclcpp::Duration(actual.time_from_start).seconds());
}

void IsEqual(const trajectory_msgs::msg::JointTrajectoryPoint& expected,
             const tmc_manipulation_types::TimedJointTrajectoryPoint& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}

void IsEqual(const tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint& expected,
             const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& actual) {
  SCOPED_TRACE("");
  ASSERT_EQ(expected.transforms.size(), actual.transforms.size());
  for (auto i = 0; i < expected.transforms.size(); ++i) {
    IsEqual(expected.transforms[i], actual.transforms[i]);
  }
  ASSERT_EQ(expected.velocities.size(), actual.velocities.size());
  for (auto i = 0; i < expected.velocities.size(); ++i) {
    IsEqual(expected.velocities[i], actual.velocities[i]);
  }
  ASSERT_EQ(expected.accelerations.size(), actual.accelerations.size());
  for (auto i = 0; i < expected.accelerations.size(); ++i) {
    IsEqual(expected.accelerations[i], actual.accelerations[i]);
  }
  EXPECT_DOUBLE_EQ(expected.time_from_start, rclcpp::Duration(actual.time_from_start).seconds());
}

void IsEqual(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& expected,
             const tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}

void IsEqual(const std::vector<geometry_msgs::msg::Transform>& expected,
             const tmc_manipulation_types::PoseSeq& actual) {
  SCOPED_TRACE("");
  ASSERT_EQ(expected.size(), actual.size());
  for (auto i = 0; i < expected.size(); ++i) {
    IsEqual(expected[i], actual[i]);
  }
}

void IsEqual(const tmc_manipulation_types::PoseSeq& expected,
             const std::vector<geometry_msgs::msg::Transform>& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}

void IsEqual(const std::vector<double>& expected,
             const rosidl_runtime_cpp::BoundedVector<double, 3>& actual) {
  SCOPED_TRACE("");
  ASSERT_EQ(expected.size(), actual.size());
  for (auto i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(expected[i], actual[i]);
  }
}

void IsEqual(const rosidl_runtime_cpp::BoundedVector<double, 3>& expected,
             const std::vector<double>& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}

void IsEqual(const std_msgs::msg::ColorRGBA& expected,
             const std_msgs::msg::ColorRGBA& actual) {
  SCOPED_TRACE("");
  EXPECT_EQ(expected.a, actual.a);
  EXPECT_EQ(expected.b, actual.b);
  EXPECT_EQ(expected.g, actual.g);
  EXPECT_EQ(expected.r, actual.r);
}

void IsEqual(const std::vector<Eigen::Vector3f>& expected, const std::vector<geometry_msgs::msg::Point>& actual) {
  SCOPED_TRACE("");
  ASSERT_EQ(expected.size(), actual.size());
  for (auto i = 0u; i < expected.size(); ++i) {
    EXPECT_FLOAT_EQ(expected[i].x(), static_cast<float>(actual[i].x));
    EXPECT_FLOAT_EQ(expected[i].y(), static_cast<float>(actual[i].y));
    EXPECT_FLOAT_EQ(expected[i].z(), static_cast<float>(actual[i].z));
  }
}

void IsEqual(const std::vector<geometry_msgs::msg::Point>& expected, const std::vector<Eigen::Vector3f>& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}

void IsEqual(const std::vector<uint32_t>& expected, const std::vector<shape_msgs::msg::MeshTriangle>& actual) {
  SCOPED_TRACE("");
  ASSERT_EQ(expected.size(), actual.size() * 3);
  for (auto i = 0u; i < actual.size(); ++i) {
    EXPECT_EQ(expected[i * 3], actual[i].vertex_indices[0]);
    EXPECT_EQ(expected[i * 3 + 1], actual[i].vertex_indices[1]);
    EXPECT_EQ(expected[i * 3 + 2], actual[i].vertex_indices[2]);
  }
}

void IsEqual(const std::vector<shape_msgs::msg::MeshTriangle>& expected, const std::vector<uint32_t>& actual) {
  SCOPED_TRACE("");
  IsEqual(actual, expected);
}
}  // namespace tmc_manipulation_types_bridge
#endif  // TMC_MANIPULATION_TYPES_BRIDGE_TEST_EQUALITY_CHECK_HPP_

