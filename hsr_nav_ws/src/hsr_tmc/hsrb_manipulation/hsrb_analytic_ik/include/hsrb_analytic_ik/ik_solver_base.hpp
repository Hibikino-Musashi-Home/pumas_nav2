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
#ifndef HSRB_ANALYTIC_IK_IK_SOLVER_BASE_HPP_
#define HSRB_ANALYTIC_IK_IK_SOLVER_BASE_HPP_

#include <memory>
#include <vector>

#include <tmc_robot_kinematics_model/ik_solver.hpp>

namespace hsrb_analytic_ik {

struct RobotParameter {
  // Mechanism set value
  double L3;
  double L41;
  double L42;
  double L51;
  double L52;
  double L81;
  double L82;

  // Movement range of joints
  double t3_min;
  double t3_max;
  double t4_min;
  double t4_max;
  double t5_min;
  double t5_max;
  double t6_min;
  double t6_max;
  double t7_min;
  double t7_max;

  RobotParameter();
};

struct BasePositionRange {
  std::array<double, 2> center;
  // If either min/max is negative
  double radius_min;
  double radius_max;

  BasePositionRange() : radius_min(-1.0), radius_max(-1.0) {}
  BasePositionRange(const RobotParameter& robot_param, const Eigen::Affine3d& origin_to_hand);
};

class HybridIKSolverBase : public tmc_robot_kinematics_model::IKSolver {
 public:
  explicit HybridIKSolverBase(const RobotParameter& robot_param);

  HybridIKSolverBase(tmc_robot_kinematics_model::IKSolver::Ptr successor,
                     const RobotParameter& robot_param);

  virtual ~HybridIKSolverBase() = default;

  tmc_robot_kinematics_model::IKResult Solve(
      const tmc_robot_kinematics_model::IKRequest& request,
      std::function<bool()>& interrupt,
      tmc_manipulation_types::JointState& solution_angle_out,
      Eigen::Affine3d& origin_to_base_out,
      Eigen::Affine3d& origin_to_end_out) override;

  tmc_robot_kinematics_model::IKResult Solve(
      const tmc_robot_kinematics_model::IKRequest& request,
      std::function<bool()>& interrupt,
      std::vector<tmc_robot_kinematics_model::IKResponse>& responses_out) override;

 private:
  RobotParameter robot_param_;
};

class BaseYawIKSolver : public tmc_robot_kinematics_model::IKSolver {
 public:
  explicit BaseYawIKSolver(const RobotParameter& robot_param);

  BaseYawIKSolver(tmc_robot_kinematics_model::IKSolver::Ptr successor,
                  const RobotParameter& robot_param);

  virtual ~BaseYawIKSolver() = default;

  tmc_robot_kinematics_model::IKResult Solve(
      const tmc_robot_kinematics_model::IKRequest& request,
      std::function<bool()>& interrupt,
      tmc_manipulation_types::JointState& solution_angle_out,
      Eigen::Affine3d& origin_to_base_out,
      Eigen::Affine3d& origin_to_end_out) override;

  tmc_robot_kinematics_model::IKResult Solve(
      const tmc_robot_kinematics_model::IKRequest& request,
      std::function<bool()>& interrupt,
      std::vector<tmc_robot_kinematics_model::IKResponse>& responses_out) override;

 private:
  RobotParameter robot_param_;
};

}  // namespace hsrb_analytic_ik
#endif  // HSRB_ANALYTIC_IK_IK_SOLVER_BASE_HPP_
