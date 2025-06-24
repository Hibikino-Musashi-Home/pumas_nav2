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
/// @brief    A group of functions that handle structures exchanged with TMC_MANIPULATION

#include <tmc_manipulation_types/utils.hpp>

#include <string>
#include <vector>

using Eigen::VectorXd;

namespace {

// Extract part vector from Vector
template <typename T>
T ExtractPartialVector(
    const T& vector_in,
    const std::vector<uint32_t>& indices) {
  T vector_out(indices.size());
  for (int32_t i = 0; i < indices.size(); ++i) {
    if (indices[i] >= vector_in.size()) {
      return T();
    }
    vector_out[i] = vector_in[indices[i]];
  }
  return vector_out;
}

std::vector<uint32_t> GetJointIndices(const tmc_manipulation_types::NameSeq& joint_names,
                                      const tmc_manipulation_types::NameSeq& target_names) {
  std::vector<uint32_t> indices;
  for (const auto& name : target_names) {
    uint32_t index = tmc_manipulation_types::GetJointIndex(joint_names, name);
    indices.push_back(index);
  }
  return indices;
}

}  // namespace

namespace tmc_manipulation_types {

JointState ExtractPartialJointState(
    const JointState& whole_joint_state,
    const NameSeq& joint_names) {
  JointState partial_joint_state;
  std::vector<uint32_t> indices = GetJointIndices(whole_joint_state.name, joint_names);
  partial_joint_state.name     = ExtractPartialVector(whole_joint_state.name, indices);
  partial_joint_state.position = ExtractPartialVector(whole_joint_state.position, indices);
  partial_joint_state.velocity = ExtractPartialVector(whole_joint_state.velocity, indices);
  partial_joint_state.effort   = ExtractPartialVector(whole_joint_state.effort, indices);
  return partial_joint_state;
}

TimedJointTrajectory ExtractPartialJointTrajectory(
    const TimedJointTrajectory& whole_trajectory,
    const NameSeq& joint_names) {
  TimedJointTrajectory partial_trajectory;
  std::vector<uint32_t> indices = GetJointIndices(whole_trajectory.joint_names, joint_names);
  partial_trajectory.joint_names = ExtractPartialVector(whole_trajectory.joint_names, indices);
  for (const auto& whole_point : whole_trajectory.points) {
    TimedJointTrajectoryPoint partial_point;
    partial_point.positions     = ExtractPartialVector(whole_point.positions, indices);
    partial_point.velocities    = ExtractPartialVector(whole_point.velocities, indices);
    partial_point.accelerations = ExtractPartialVector(whole_point.accelerations, indices);
    partial_point.effort        = ExtractPartialVector(whole_point.effort, indices);
    partial_point.time_from_start = whole_point.time_from_start;
    partial_trajectory.points.push_back(partial_point);
  }
  return partial_trajectory;
}

}  // namespace tmc_manipulation_types
