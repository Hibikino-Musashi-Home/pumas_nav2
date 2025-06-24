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
#ifndef TMC_MANIPULATION_TYPES_UTILS_HPP_
#define TMC_MANIPULATION_TYPES_UTILS_HPP_

#include <string>

#include <tmc_manipulation_types/manipulation_types.hpp>

namespace tmc_manipulation_types {

/// @brief Pull out partial Joint_state from Joint_state
/// @param whole_joint_state Joint_state for the entire robot
/// @param joint_names extracted joint name
/// @return Partial Joint_state extracted
JointState ExtractPartialJointState(
    const JointState& whole_joint_state,
    const NameSeq& joint_names);

/// @brief Pull out a partial Joint_trajectory from Joint_trajectory
/// @param whole_trajectory Joint_state for the entire robot
/// @param joint_names extracted joint name
/// @return Partially extracted Joint_trajectory
TimedJointTrajectory ExtractPartialJointTrajectory(
    const TimedJointTrajectory& whole_trajectory,
    const NameSeq& joint_names);


inline uint32_t GetJointIndex(const tmc_manipulation_types::NameSeq& names,
                              const std::string& name) {
  tmc_manipulation_types::NameSeq::const_iterator it = std::find(names.begin(), names.end(), name);
  if (it != names.end()) {
    return std::distance(names.begin(), it);
  } else {
    return -1;
  }
}

inline void ExtractJointPos(
    const tmc_manipulation_types::JointState& joint_state,
    const tmc_manipulation_types::NameSeq& joint_names,
    Eigen::VectorXd& joint_pos_out) {
  joint_pos_out.resize(joint_names.size());
  for (tmc_manipulation_types::NameSeq::const_iterator joint_name = joint_names.begin();
       joint_name != joint_names.end();
       ++joint_name) {
    uint32_t in_index = GetJointIndex(joint_state.name, *joint_name);
    if (in_index == -1) {
      throw std::invalid_argument("Joint " + *joint_name + " is not found");
    }
    if (in_index >= joint_state.position.size()) {
      throw std::invalid_argument("invalid size pos");
    }
    uint32_t out_index = std::distance(joint_names.begin(), joint_name);
    joint_pos_out[out_index] = joint_state.position[in_index];
  }
}

inline void ExtractMultiJointPos(
    const tmc_manipulation_types::MultiDOFJointState& multi_dof_joint_state,
    const NameSeq& joint_names,
    PoseSeq& multi_dof_joint_pos_out) {
  multi_dof_joint_pos_out.resize(joint_names.size());
  for (tmc_manipulation_types::NameSeq::const_iterator joint_name = joint_names.begin();
       joint_name != joint_names.end();
       ++joint_name) {
    uint32_t in_index = GetJointIndex(multi_dof_joint_state.names, *joint_name);
    if (in_index == -1) {
      throw std::invalid_argument("Joint " + *joint_name + " is not found");
    }
    if (in_index >= multi_dof_joint_state.poses.size()) {
      throw std::invalid_argument("invalid size pos");
    }
    uint32_t out_index = std::distance(joint_names.begin(), joint_name);
    multi_dof_joint_pos_out[out_index] = multi_dof_joint_state.poses[in_index];
  }
}


}  // namespace tmc_manipulation_types
#endif  // TMC_MANIPULATION_TYPES_UTILS_HPP_
