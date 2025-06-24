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
#ifndef ROBOT_KINEMATICS_MODEL_IK_SOLVER_HPP__
#define ROBOT_KINEMATICS_MODEL_IK_SOLVER_HPP__

#include <memory>
#include <string>
#include <vector>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

namespace tmc_robot_kinematics_model {
enum IKResult {
  kSuccess,
  kConverge,
  kMaxItr,
  kFail,
  kInterruption
};

/// IK request
struct IKRequest {
  IKRequest() {}
  /// Specify BasemovementType when Base is operating
  explicit IKRequest(tmc_manipulation_types::BaseMovementType base_type) {
    switch (base_type) {
      case tmc_manipulation_types::kFloat:
        linear_base_movements.push_back(Eigen::Vector3d::UnitX());
        linear_base_movements.push_back(Eigen::Vector3d::UnitY());
        linear_base_movements.push_back(Eigen::Vector3d::UnitZ());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitX());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitY());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kPlanar:
        linear_base_movements.push_back(Eigen::Vector3d::UnitX());
        linear_base_movements.push_back(Eigen::Vector3d::UnitY());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kRailX:
        linear_base_movements.push_back(Eigen::Vector3d::UnitX());
        break;
      case tmc_manipulation_types::kRailY:
        linear_base_movements.push_back(Eigen::Vector3d::UnitY());
        break;
      case tmc_manipulation_types::kRailZ:
        linear_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kRotationX:
        rotational_base_movements.push_back(Eigen::Vector3d::UnitX());
        break;
      case tmc_manipulation_types::kRotationY:
        rotational_base_movements.push_back(Eigen::Vector3d::UnitY());
        break;
      case tmc_manipulation_types::kRotationZ:
        rotational_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kNone:
        break;
      default:
        break;
    }
  }

  /// The target frame name
  std::string frame_name;
  /// Offset from the target frame
  Eigen::Affine3d frame_to_end;
  /// In the early stages of the target position
  Eigen::Affine3d ref_origin_to_end;
  /// Robot position posture
  Eigen::Affine3d origin_to_base;
  /// It is good to enter all the initial posture.
  /// If not, the joint angle that is already set will be used.
  tmc_manipulation_types::JointState initial_angle;
  /// The target joint name
  std::vector<std::string> use_joints;
  /// The weight of each joint USE_Joints+Base_dof is ignored if it is not the same length
  Eigen::VectorXd weight;
  /// Base movement in parallel direction
  std::vector<Eigen::Vector3d> linear_base_movements;
  /// Base movement in the direction of rotation
  std::vector<Eigen::Vector3d> rotational_base_movements;
  /// Continuous joint name
  std::vector<std::string> continuous_joints;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct IKResponse {
  tmc_manipulation_types::JointState solution_angle;
  Eigen::Affine3d origin_to_base;
  Eigen::Affine3d origin_to_end;
};

/// IK interface
/// CHAIN ​​OF RESPONSIBILITY can customize the method of IK
/// Example: There is an analysis question using the six -arm axis, so use an analysis question when making such a request.
class IKSolver {
 public:
  using Ptr = std::shared_ptr<IKSolver>;
  IKSolver() {}
  explicit IKSolver(IKSolver::Ptr successor) : successor_(successor) {}
  virtual ~IKSolver() {}

  void set_successor(IKSolver::Ptr successor) { successor_ = successor;}
  virtual void set_robot_description(const std::string& robot_description) {}

  /// Solve IK
  /// @param [IN] request IK request
  /// @param [OUT] solution_angle_out Solution angle
  /// @param [OUT] origin_to_end_out The final target pose
  virtual IKResult Solve(const IKRequest& request,
                         tmc_manipulation_types::JointState& solution_angle_out,
                         Eigen::Affine3d& origin_to_end_out)  {
    std::function<bool()> func = []() -> bool{ return false; };
    return Solve(request, func, solution_angle_out, origin_to_end_out);
  }

  /// Solve IK
  /// @param [IN] request IK request
  /// @param [OUT] solution_angle_out Solution angle
  /// @param [OUT] origin_to_base_out Solution base pose
  /// @param [OUT] origin_to_end_out The final target pose
  virtual IKResult Solve(const IKRequest& request,
                         tmc_manipulation_types::JointState& solution_angle_out,
                         Eigen::Affine3d& origin_to_base_out,
                         Eigen::Affine3d& origin_to_end_out) {
    std::function<bool()> func = []() -> bool{ return false; };
    return Solve(request, func, solution_angle_out, origin_to_base_out, origin_to_end_out);
  }

  /// Solve IK
  /// @param [IN] request IK request
  /// @param [OUT] responses_out IK solutions
  virtual IKResult Solve(const IKRequest& request,
                         std::vector<IKResponse>& responses_out) {
    // By default, I came up with the implementation that the solution calls one Solve.
    // He said that he would chain individually to get the consistency with the existing METHOD.
    std::function<bool()> func = []() -> bool{ return false; };
    return Solve(request, func, responses_out);
  }

  /// Solve IK
  /// @param [IN] request IK request
  /// @param [IN] interrupt Interrupt function
  /// @param [OUT] solution_angle_out Solution angle
  /// @param [OUT] origin_to_end_out The final target pose
  virtual IKResult Solve(const IKRequest& request,
                         std::function<bool()>& interrupt,
                         tmc_manipulation_types::JointState& solution_angle_out,
                         Eigen::Affine3d& origin_to_end_out)  {
    return Next_(request, interrupt, solution_angle_out, origin_to_end_out);
  }

  /// Solve IK
  /// @param [IN] request IK request
  /// @param [IN] interrupt Interrupt function
  /// @param [OUT] solution_angle_out Solution angle
  /// @param [OUT] origin_to_base_out Solution base pose
  /// @param [OUT] origin_to_end_out The final target pose
  virtual IKResult Solve(const IKRequest& request,
                         std::function<bool()>& interrupt,
                         tmc_manipulation_types::JointState& solution_angle_out,
                         Eigen::Affine3d& origin_to_base_out,
                         Eigen::Affine3d& origin_to_end_out) {
    return Next_(request, interrupt, solution_angle_out, origin_to_base_out, origin_to_end_out);
  }

  /// Solve IK
  /// @param [IN] request IK request
  /// @param [IN] interrupt Interrupt function
  /// @param [OUT] responses_out IK solutions
  virtual IKResult Solve(const IKRequest& request,
                         std::function<bool()>& interrupt,
                         std::vector<IKResponse>& responses_out) {
    // By default, I came up with the implementation that the solution calls one Solve.
    // He said that he would chain individually to get the consistency with the existing METHOD.
    return Next_(request, interrupt, responses_out);
  }

 protected:
  /// Reported to Succesor
  IKResult Next_(const IKRequest& request,
                 std::function<bool()>& interrupt,
                 tmc_manipulation_types::JointState& solution_angle_out,
                 Eigen::Affine3d& origin_to_end_out) {
    if (successor_) {
      return successor_->Solve(request, interrupt, solution_angle_out, origin_to_end_out);
    } else {
      return kFail;
    }
  }

  /// Reported to Succesor
  IKResult Next_(const IKRequest& request,
                 std::function<bool()>& interrupt,
                 tmc_manipulation_types::JointState& solution_angle_out,
                 Eigen::Affine3d& origin_to_base_out,
                 Eigen::Affine3d& origin_to_end_out) {
    if (successor_) {
      return successor_->Solve(request, interrupt, solution_angle_out,
                               origin_to_base_out, origin_to_end_out);
    } else {
      return kFail;
    }
  }

  /// Reported to Succesor
  IKResult Next_(const IKRequest& request,
                 std::function<bool()>& interrupt,
                 std::vector<IKResponse>& responses_out) {
    if (successor_) {
      return successor_->Solve(request, interrupt, responses_out);
    } else {
      return kFail;
    }
  }

 private:
  Ptr successor_;
};
}  // namespace tmc_robot_kinematics_model
#endif
