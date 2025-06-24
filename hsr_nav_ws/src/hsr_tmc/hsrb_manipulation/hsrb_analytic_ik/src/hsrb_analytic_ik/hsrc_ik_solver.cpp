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
/// @brief High-speed IK for HSR-C

#include "hsrb_analytic_ik/hsrc_ik_solver.hpp"

#include <map>
#include <string>
#include <vector>

#include "hsr_common_ik_solver.hpp"
#include "robot_optimizer.hpp"

namespace {

const double kBaselinkToArmLiftJointZ = 0.350;
const double kArmLiftJointToArmFlexJointX = 0.141;
const double kArmLiftJointToArmFlexJointY = 0.0785;
const double kArmFlexJointToArmRollJointX = 0.005;
const double kArmFlexJointToArmRollJointZ = 0.345;
const double kWristRollJointToHandPalmJointX = 0.0;
const double kWristRollJointToHandPalmJointZ = 0.155;
const double kArmLiftJointMin = 0.0;
const double kArmLiftJointMax = 0.69;
const double kArmFlexJointMin = -2.62;
const double kArmFlexJointMax = 0.0;
const double kArmRollJointMin = -1.92;
const double kArmRollJointMax = 3.67;
const double kWristFlexJointMin = -1.74;
const double kWristFlexJointMax = 1.22;
const double kWristRollJointMin = -1.92;
const double kWristRollJointMax = 3.67;

hsrb_analytic_ik::RobotParameter GenrateParameter() {
  hsrb_analytic_ik::RobotParameter function_param;
  function_param.L3 = kBaselinkToArmLiftJointZ;
  function_param.L41 = kArmLiftJointToArmFlexJointX;
  function_param.L42 = kArmLiftJointToArmFlexJointY;
  function_param.L51 = kArmFlexJointToArmRollJointX;
  function_param.L52 = kArmFlexJointToArmRollJointZ;
  function_param.L81 = kWristRollJointToHandPalmJointX;
  function_param.L82 = kWristRollJointToHandPalmJointZ;
  function_param.t3_min = kArmLiftJointMin;
  function_param.t3_max = kArmLiftJointMax;
  function_param.t4_min = kArmFlexJointMin;
  function_param.t4_max = kArmFlexJointMax;
  function_param.t5_min = kArmRollJointMin;
  function_param.t5_max = kArmRollJointMax;
  function_param.t6_min = kWristFlexJointMin;
  function_param.t6_max = kWristFlexJointMax;
  function_param.t7_min = kWristRollJointMin;
  function_param.t7_max = kWristRollJointMax;
  return function_param;
}

}  // namespace

namespace hsrb_analytic_ik {

BasePositionRange GetHsrcBasePositionRange(const Eigen::Affine3d& origin_to_hand) {
  return BasePositionRange(GenrateParameter(), origin_to_hand);
}


HsrcIKSolver::HsrcIKSolver() : HybridIKSolverBase(GenrateParameter()) {}

HsrcIKSolver::HsrcIKSolver(tmc_robot_kinematics_model::IKSolver::Ptr successor)
    : HybridIKSolverBase(successor, GenrateParameter()) {}


HsrcBaseYawIKSolver::HsrcBaseYawIKSolver() : BaseYawIKSolver(GenrateParameter()) {}

HsrcBaseYawIKSolver::HsrcBaseYawIKSolver(tmc_robot_kinematics_model::IKSolver::Ptr successor)
    : BaseYawIKSolver(successor, GenrateParameter()) {}

}  // namespace hsrb_analytic_ik
