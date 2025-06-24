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

#include <hsrb_analytic_ik/ik_solver_base.hpp>

#include "hsr_common_ik_solver.hpp"
#include "robot_optimizer.hpp"

namespace hsrb_analytic_ik {

RobotParameter::RobotParameter()
    : L3(0.340), L41(0.141), L42(0.078),
      L51(0.005), L52(0.345), L81(0.012), L82(0.1405),
      t3_min(0.00), t3_max(0.69), t4_min(-2.62), t4_max(0.00),
      t5_min(-1.92), t5_max(3.67), t6_min(-1.92), t6_max(1.22),
      t7_min(-1.92), t7_max(3.67) {}

BasePositionRange::BasePositionRange(const RobotParameter& robot_param, const Eigen::Affine3d& origin_to_hand) {
  opt::RobotFunction2Request request;
  request.R11 = origin_to_hand(0, 0);
  request.R21 = origin_to_hand(1, 0);
  request.R31 = origin_to_hand(2, 0);

  request.R12 = origin_to_hand(0, 1);
  request.R22 = origin_to_hand(1, 1);
  request.R32 = origin_to_hand(2, 1);

  request.R13 = origin_to_hand(0, 2);
  request.R23 = origin_to_hand(1, 2);
  request.R33 = origin_to_hand(2, 2);

  request.px = origin_to_hand(0, 3);
  request.py = origin_to_hand(1, 3);
  request.pz = origin_to_hand(2, 3);

  // Z coordinates at the intersection of arm_roll, Wrist_flex, Wrist_roll
  const double zwo = request.R31 * robot_param.L81 - request.R33 * robot_param.L82 + request.pz;
  if (zwo > robot_param.t3_max + robot_param.L3 + robot_param.L52) {
    // It's too expensive to reach, no solution
    radius_min = -1.0;
    radius_max = -1.0;
    return;
  } else if (zwo < (robot_param.t3_min + robot_param.L3 + robot_param.L52 * std::cos(robot_param.t4_min) +
                    robot_param.L51 * std::sin(robot_param.t4_min))) {
    // It's too low to reach, no solution
    radius_min = -1.0;
    radius_max = -1.0;
    return;
  }

  // X, Y coordinates at the intersection of arm_roll, Wrist_flex, Wrist_roll
  // This point, which is uniquely determined from Origin_to_hand, is a rotating center of the bogie
  center[0] = request.R11 * robot_param.L81 - request.R13 * robot_param.L82 + request.px;
  center[1] = request.R21 * robot_param.L81 - request.R23 * robot_param.L82 + request.py;

  const double l5_length = std::sqrt(robot_param.L51 * robot_param.L51 + robot_param.L52 * robot_param.L52);
  const double alpha_t4 = std::atan2(robot_param.L52, robot_param.L51);

  if (zwo < robot_param.t3_min + robot_param.L3) {
    // It will not reach unless you extend downward
    const double t4 = std::asin((zwo - robot_param.t3_min - robot_param.L3) / l5_length) - alpha_t4;
    const double S4 = std::sin(t4);
    const double C4 = std::cos(t4);
    radius_max = std::sqrt(std::pow(robot_param.L52 * S4 - robot_param.L51 * C4 - robot_param.L41, 2.0) +
                           robot_param.L42 * robot_param.L42);
  } else if (zwo > robot_param.t3_max + robot_param.L3) {
    // It will not reach unless you extend upward
    const double t4 = std::asin((zwo - robot_param.t3_max - robot_param.L3) / l5_length) - alpha_t4;
    const double S4 = std::sin(t4);
    const double C4 = std::cos(t4);
    radius_max = std::sqrt(std::pow(robot_param.L52 * S4 - robot_param.L51 * C4 - robot_param.L41, 2.0) +
                           robot_param.L42 * robot_param.L42);
  } else {
    // It will arrive if you stretch straight
    const double S4 = std::sin(-alpha_t4);
    const double C4 = std::cos(-alpha_t4);
    radius_max = std::sqrt(std::pow(robot_param.L52 * S4 - robot_param.L51 * C4 - robot_param.L41, 2.0) +
                           robot_param.L42 * robot_param.L42);
  }

  const double t4_min_rev = -robot_param.t4_min - alpha_t4;
  if (zwo >= robot_param.t3_min + robot_param.L3 + robot_param.L52) {
    // Even if your arm is folded, it is OK in height
    radius_min = std::sqrt(std::pow(-robot_param.L51 - robot_param.L41, 2.0) + robot_param.L42 * robot_param.L42);
  } else if (zwo > (robot_param.t3_min + robot_param.L3 + robot_param.L52 * std::cos(t4_min_rev) +
                    robot_param.L51 * std::sin(t4_min_rev))) {
    // The attitude of lowering the lifting axis and putting your hands forward is the closest to your body
    const double t4 = std::asin((zwo - robot_param.t3_min - robot_param.L3) / l5_length) - alpha_t4;
    const double S4 = std::sin(t4);
    const double C4 = std::cos(t4);
    radius_min = std::sqrt(std::pow(robot_param.L52 * S4 - robot_param.L51 * C4 - robot_param.L41, 2.0) +
                           robot_param.L42 * robot_param.L42);
  } else {
    // The attitude of lowering the arm to the limit is the closest
    const double S4 = std::sin(robot_param.t4_min);
    const double C4 = std::cos(robot_param.t4_min);
    radius_min = std::sqrt(std::pow(robot_param.L52 * S4 - robot_param.L51 * C4 - robot_param.L41, 2.0) +
                           robot_param.L42 * robot_param.L42);
  }
}

HybridIKSolverBase::HybridIKSolverBase(const RobotParameter& robot_param)
    : HybridIKSolverBase(nullptr, robot_param) {}

HybridIKSolverBase::HybridIKSolverBase(tmc_robot_kinematics_model::IKSolver::Ptr successor,
                                       const RobotParameter& robot_param)
    : IKSolver(successor), robot_param_(robot_param) {}

// Solve IK, but also allow BASE movement
tmc_robot_kinematics_model::IKResult HybridIKSolverBase::Solve(
    const tmc_robot_kinematics_model::IKRequest& request,
    std::function<bool()>& interrupt,
    tmc_manipulation_types::JointState& solution_angle_out,
    Eigen::Affine3d& origin_to_base_out,
    Eigen::Affine3d& origin_to_end_out) {
  // Use Hybridik only under the following conditions
  // 1. BasemovementType is KPlanar
  // 2. use_jointsが arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint(順不同)
  // 3. frame_nameがhand_palm_link or hand_palm_joint
  if (!SuitBaseMovement(request) ||
      !SuitUseJoint(request) ||
      !SuitFrame(request)) {
    return Next_(request, interrupt, solution_angle_out, origin_to_base_out, origin_to_end_out);
  }

  return SolveIK(request, robot_param_, solution_angle_out, origin_to_base_out, origin_to_end_out);
}

// Solve IK
tmc_robot_kinematics_model::IKResult HybridIKSolverBase::Solve(
    const tmc_robot_kinematics_model::IKRequest& request,
    std::function<bool()>& interrupt,
    std::vector<tmc_robot_kinematics_model::IKResponse>& responses_out) {
  tmc_robot_kinematics_model::IKResponse response;
  if (Solve(request, interrupt, response.solution_angle, response.origin_to_base, response.origin_to_end) ==
          tmc_robot_kinematics_model::kSuccess) {
    responses_out.clear();
    responses_out.push_back(response);
    return tmc_robot_kinematics_model::kSuccess;
  } else {
    return Next_(request, interrupt, responses_out);
  }
}


BaseYawIKSolver::BaseYawIKSolver(const RobotParameter& robot_param)
    : BaseYawIKSolver(nullptr, robot_param) {}

BaseYawIKSolver::BaseYawIKSolver(tmc_robot_kinematics_model::IKSolver::Ptr successor,
                                       const RobotParameter& robot_param)
    : IKSolver(successor), robot_param_(robot_param) {}

// Solve IK, but also allow BASE movement
tmc_robot_kinematics_model::IKResult BaseYawIKSolver::Solve(
    const tmc_robot_kinematics_model::IKRequest& request,
    std::function<bool()>& interrupt,
    tmc_manipulation_types::JointState& solution_angle_out,
    Eigen::Affine3d& origin_to_base_out,
    Eigen::Affine3d& origin_to_end_out) {
  std::vector<tmc_robot_kinematics_model::IKResponse> responses;
  if (Solve(request, interrupt, responses) == tmc_robot_kinematics_model::kSuccess) {
    const auto index = SelectClosestSolution(request, responses);
    solution_angle_out = responses[index].solution_angle;
    origin_to_base_out = responses[index].origin_to_base;
    origin_to_end_out = responses[index].origin_to_end;
    return tmc_robot_kinematics_model::kSuccess;
  } else {
    return Next_(request, interrupt, solution_angle_out, origin_to_base_out, origin_to_end_out);
  }
}

// Solve IK
tmc_robot_kinematics_model::IKResult BaseYawIKSolver::Solve(
    const tmc_robot_kinematics_model::IKRequest& request,
    std::function<bool()>& interrupt,
    std::vector<tmc_robot_kinematics_model::IKResponse>& responses_out) {
  // Use Baseyawik only under the following conditions
  if (!SuitBaseRotationZ(request) ||
      !SuitUseJoint(request) ||
      !SuitFrame(request)) {
    return Next_(request, interrupt, responses_out);
  }
  return SolveBaseYawIK(request, robot_param_, responses_out);
}

}  // namespace hsrb_analytic_ik
