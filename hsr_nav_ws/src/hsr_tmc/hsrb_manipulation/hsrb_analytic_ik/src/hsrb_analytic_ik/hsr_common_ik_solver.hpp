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
/// @brief Common functions of high -speed IK for HSR
#ifndef HSRB_ANALYTIC_IK_HSR_COMMON_IK_SOLVER_HPP_
#define HSRB_ANALYTIC_IK_HSR_COMMON_IK_SOLVER_HPP_

#include <map>
#include <string>
#include <vector>

#include <tmc_robot_kinematics_model/ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

#include <hsrb_analytic_ik/ik_solver_base.hpp>
#include "robot_optimizer.hpp"

namespace hsrb_analytic_ik {

bool SuitBaseMovement(const tmc_robot_kinematics_model::IKRequest& request);
bool SuitBaseRotationZ(const tmc_robot_kinematics_model::IKRequest& request);
bool SuitUseJoint(const tmc_robot_kinematics_model::IKRequest& request);
bool MapJointAndId(const std::vector<std::string>& use_joints,
                   std::map<std::string, uint32_t>& map_out);
bool SuitFrame(const tmc_robot_kinematics_model::IKRequest& request);
bool MapJointAndId(const std::vector<std::string>& use_joints,
                     std::map<std::string, uint32_t>& map_out);

double ExtractJointPosition(
    const tmc_manipulation_types::JointState& joint_state,
    const std::string& joint_name,
    double default_pos);

tmc_robot_kinematics_model::IKResult SolveIK(
    const tmc_robot_kinematics_model::IKRequest& request,
    const RobotParameter& function_param,
    tmc_manipulation_types::JointState& solution_angle_out,
    Eigen::Affine3d& origin_to_base_out,
    Eigen::Affine3d& origin_to_end_out);

tmc_robot_kinematics_model::IKResult SolveBaseYawIK(
    const tmc_robot_kinematics_model::IKRequest& request,
    const RobotParameter& function_param,
    std::vector<tmc_robot_kinematics_model::IKResponse>& responses_out);

uint32_t SelectClosestSolution(
    const tmc_robot_kinematics_model::IKRequest& request,
    const std::vector<tmc_robot_kinematics_model::IKResponse>& responses);



}  // namespace hsrb_analytic_ik
#endif  // HSRB_ANALYTIC_IK_HSR_COMMON_IK_SOLVER_HPP_
