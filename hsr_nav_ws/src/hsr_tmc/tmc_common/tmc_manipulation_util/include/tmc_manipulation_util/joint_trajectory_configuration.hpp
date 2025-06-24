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
/// @brief    General -purpose function for handling joint orbits
#ifndef TMC_MANIPULATION_UTIL_JOINT_TRAJECTORY_CONFIGURATION_HPP_
#define TMC_MANIPULATION_UTIL_JOINT_TRAJECTORY_CONFIGURATION_HPP_
#include <string>
#include <vector>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace tmc_manipulation_util {
/// Extracts a joint_trajectory containing only the joints specified by joint_names.
/// If a joint does not exist in the joint_trajectory, it will be obtained from the joint_state.
/// If it is not found there either, the process will fail.
/// @param[in] joint_trajectory input orbit
/// @param[in] joint_names joint name
/// @param[in] joint_state joint state
/// @param[out] partial_joint_trajectory_out Delivery
/// @return Consciousness
bool ExtractTrajectory(const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                       const std::vector<std::string>& joint_names,
                       const sensor_msgs::msg::JointState& joint_state,
                       trajectory_msgs::msg::JointTrajectory& partial_joint_trajectory_out);

/// Merges the original_trajectory and additional_trajectory, which have the same number of points
/// but for different joints, into a single trajectory.
/// The process will fail if the points' sizes differ.
/// time_from_start will be aligned with the original trajectory.
/// @param[in] original_trajectory Original enters the rail
/// @param[in] additional_trajectory Additional input orbit
/// @param[out] merged_trajectory_out
/// @return Consciousness
bool MergeJointTrajectory(const trajectory_msgs::msg::JointTrajectory& original_trajectory,
                          const trajectory_msgs::msg::JointTrajectory& additional_trajectory,
                          trajectory_msgs::msg::JointTrajectory& merged_trajectory_out);
}  // namespace tmc_manipulation_util
#endif  // TMC_MANIPULATION_UTIL_JOINT_TRAJECTORY_CONFIGURATION_HPP_
