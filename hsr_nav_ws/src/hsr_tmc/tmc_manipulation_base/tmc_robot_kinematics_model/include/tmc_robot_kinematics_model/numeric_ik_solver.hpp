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
/// @brief    Solver of IK for numerical solutions

#ifndef ROBOT_KINEMATICS_MODEL_NUMERIC_IK_SOLVER_HPP__
#define ROBOT_KINEMATICS_MODEL_NUMERIC_IK_SOLVER_HPP__

#include <string>
#include <vector>

#include <tmc_robot_kinematics_model/ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

namespace tmc_robot_kinematics_model {

class NumericIKSolver : public IKSolver {
 public:
  NumericIKSolver();

  NumericIKSolver(IKSolver::Ptr successor,
                  IRobotKinematicsModel::Ptr robot_model,
                  uint32_t max_itr,
                  double epsilon,
                  double converge_threshold)
      : IKSolver(successor), robot_model_(robot_model) , max_itr_(max_itr),
        epsilon_(epsilon), converge_threshold_(converge_threshold) {}
  virtual ~NumericIKSolver() {}

  void set_robot_description(const std::string& robot_description) override;

  IKResult Solve(
      const IKRequest& request,
      tmc_manipulation_types::JointState& solution_angle_out,
      Eigen::Affine3d& origin_to_end_out) override;


  IKResult Solve(
      const IKRequest& request,
      tmc_manipulation_types::JointState& solution_angle_out,
      Eigen::Affine3d& origin_to_base_out,
      Eigen::Affine3d& origin_to_end_out) override;

  IKResult Solve(
      const IKRequest& request,
      std::vector<IKResponse>& responses_out) override;

  IKResult Solve(
      const IKRequest& request,
      std::function<bool()>& interrupt,
      tmc_manipulation_types::JointState& solution_angle_out,
      Eigen::Affine3d& origin_to_end_out) override;

  IKResult Solve(
      const IKRequest& request,
      std::function<bool()>& interrupt,
      tmc_manipulation_types::JointState& solution_angle_out,
      Eigen::Affine3d& origin_to_base_out,
      Eigen::Affine3d& origin_to_end_out) override;

  IKResult Solve(
      const IKRequest& request,
      std::function<bool()>& interrupt,
      std::vector<IKResponse>& responses_out) override;

 private:
  IRobotKinematicsModel::Ptr robot_model_;
  uint32_t max_itr_;
  double epsilon_;
  double converge_threshold_;
};
}  // namespace tmc_robot_kinematics_model

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus
// wrapper functions for ctypes
  void* create_solver(void*, int, float, float);
  void* create_request(tmc_manipulation_types::BaseMovementType);
  void* jointstate();
  void* affine3d();
  void set_req_frame_name(void*, char*);
  void set_req_initial_angle_name(void*, char*[], int);
  void set_req_frame_to_end(void*, double*);
  void set_req_origin_to_base(void*, double*);
  void set_req_initial_angle_position(void*, float[], int);
  void set_req_weight(void*, float[], int);
  void set_req_ref_origin_to_end(void*, double*);
  void solve(void*, void*, void*, void*, void*);
  void get_joint_angle(void*, float*, int);
  void get_origin_to_base(void*, double*);
#ifdef __cplusplus
}
#endif  // __cplusplus

#endif
