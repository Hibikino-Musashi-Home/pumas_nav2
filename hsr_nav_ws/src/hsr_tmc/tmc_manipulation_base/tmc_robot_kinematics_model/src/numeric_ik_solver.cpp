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

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

using tmc_manipulation_types::JointState;

namespace {
constexpr uint32_t kDefaultMaxItr = 1000;
constexpr double kDefaultEpsilon = 1.0e-3;
constexpr double kDefaultConvergeThreshold = 1.0e-10;

const uint32_t kSE3Dim = 6;
// From the small bias Sugihara 2009 used for Sugihara Law
const double kWn = 1e-3;

std::function<bool()> ReturnFalse = []() -> bool{ return false; };

/// Place the joint angle in the limit
void SaturateAngle(const Eigen::VectorXd& min, const Eigen::VectorXd& max,
                   const std::vector<bool> is_continuous_joint,
                   Eigen::VectorXd& angle) {
  for (int32_t i = 0; i < angle.size(); ++i) {
    if (is_continuous_joint[i]) {
      // I don't think there are two laps at a stretch in the numerical IK, so this should be fine
      if (angle(i) > M_PI) {
        angle(i) -= 2.0 * M_PI;
      } else if (angle(i) < -M_PI) {
        angle(i) += 2.0 * M_PI;
      }
    } else {
      angle(i) = std::max(min(i), angle(i));
      angle(i) = std::min(max(i), angle(i));
    }
  }
}
}  // anonymous namespace

namespace tmc_robot_kinematics_model {

NumericIKSolver::NumericIKSolver()
    : max_itr_(kDefaultMaxItr), epsilon_(kDefaultEpsilon), converge_threshold_(kDefaultConvergeThreshold) {}

void NumericIKSolver::set_robot_description(const std::string& robot_description) {
  robot_model_ = std::make_shared<PinocchioWrapper>(robot_description);
}

IKResult NumericIKSolver::Solve(const IKRequest& request,
                                JointState& solution_angle_out,
                                Eigen::Affine3d& origin_to_end_out) {
  return Solve(request,
               ReturnFalse,
               solution_angle_out,
               origin_to_end_out);
}

IKResult NumericIKSolver::Solve(const IKRequest& request,
                                std::function<bool()>& interrupt,
                                JointState& solution_angle_out,
                                Eigen::Affine3d& origin_to_end_out) {
  Eigen::Affine3d origin_to_base_dummy;
  return Solve(request,
               interrupt,
               solution_angle_out,
               origin_to_base_dummy,
               origin_to_end_out);
}

IKResult NumericIKSolver::Solve(const IKRequest& request,
                                JointState& solution_angle_out,
                                Eigen::Affine3d& origin_to_base_out,
                                Eigen::Affine3d& origin_to_end_out) {
  return Solve(request,
               ReturnFalse,
               solution_angle_out,
               origin_to_base_out,
               origin_to_end_out);
}

IKResult NumericIKSolver::Solve(const IKRequest& request,
                                std::function<bool()>& interrupt,
                                JointState& solution_angle_out,
                                Eigen::Affine3d& origin_to_base_out,
                                Eigen::Affine3d& origin_to_end_out) {
  double delta = 0.0;
  double delta_old = 0.0;
  uint32_t dof = request.use_joints.size();
  robot_model_->SetRobotTransform(request.origin_to_base);
  robot_model_->SetNamedAngle(request.initial_angle);

  Eigen::Affine3d origin_to_current(Eigen::Affine3d::Identity());
  Eigen::Affine3d origin_to_base(request.origin_to_base);
  Eigen::Affine3d ref_origin_to_frame(Eigen::Affine3d::Identity());
  Eigen::Affine3d ref_to_current(Eigen::Affine3d::Identity());
  Eigen::MatrixXd jacobian(kSE3Dim, dof);
  Eigen::VectorXd angle_diff(dof);
  Eigen::Matrix<double, kSE3Dim, 1> diff;
  Eigen::AngleAxisd diff_rot;
  Eigen::Vector3d diff_pos;
  JointState current_joint;
  Eigen::VectorXd angle_max(dof);
  Eigen::VectorXd angle_min(dof);

  std::vector<Eigen::Vector3d> linear_base_movements =
      request.linear_base_movements;
  std::vector<Eigen::Vector3d> rotational_base_movements =
      request.rotational_base_movements;

  uint32_t total_dof = dof +
      linear_base_movements.size() + rotational_base_movements.size();
  Eigen::MatrixXd linear_base_jacobian(
      kSE3Dim, linear_base_movements.size());
  Eigen::MatrixXd rotational_base_jacobian(
      kSE3Dim, rotational_base_movements.size());

  // Create Jacobian in the position
  for (uint32_t i = 0; i < linear_base_movements.size(); ++i) {
    linear_base_jacobian.col(i) <<
        linear_base_movements[i], Eigen::Vector3d::Zero();
  }


  Eigen::MatrixXd jacobian_with_base(kSE3Dim, total_dof);

  // Exception sending if the number of joints+Base is 6 or less
  if (total_dof < kSE3Dim) {
    throw std::invalid_argument(
        "use_joints's + base dof has to be at least 6.");
  }

  Eigen::MatrixXd wn = Eigen::MatrixXd::Identity(total_dof, total_dof);
  // If the size of the weight is the same as the DOF, we will weight WN
  if (request.weight.size() == static_cast<int32_t>(total_dof)) {
    for (uint32_t i = 0; i < dof; ++i) {
      // If the weight is negative, exception is sent
      if (request.weight(i) < 0.0) {
        throw std::invalid_argument(
            "ik joint weights have to be positive double.");
      } else {
        wn(i, i) = request.weight(i);
      }
    }
    for (uint32_t i = dof; i < dof + linear_base_movements.size(); ++i) {
      wn(i, i) = request.weight(i);
    }
    for (uint32_t i = dof + linear_base_movements.size();
         i < total_dof; ++i) {
      wn(i, i) = request.weight(i);
    }
  }

  ref_origin_to_frame = request.ref_origin_to_end * (request.frame_to_end).inverse();

  robot_model_->GetMinMax(request.use_joints, angle_min, angle_max);

  std::vector<bool> is_continuous_joint(dof);
  for (auto i = 0; i < request.use_joints.size(); ++i) {
    if (std::find(request.continuous_joints.begin(), request.continuous_joints.end(), request.use_joints[i]) ==
            request.continuous_joints.end()) {
      is_continuous_joint[i] = false;
    } else {
      is_continuous_joint[i] = true;
    }
  }

  for (uint32_t i = 0; i < max_itr_; ++i) {
    if (interrupt()) {
      return kInterruption;
    }

    origin_to_current = robot_model_->GetObjectTransform(request.frame_name);
    current_joint = robot_model_->GetNamedAngle(request.use_joints);
    Eigen::Quaterniond diff_quat(origin_to_current.linear().transpose()
                                 * ref_origin_to_frame.linear());
    diff_rot = Eigen::AngleAxisd(diff_quat.normalized());
    diff_pos = ref_origin_to_frame.translation()
        - origin_to_current.translation();
    diff << diff_pos,
        origin_to_current.linear() * diff_rot.angle() * diff_rot.axis();

    delta_old = delta;
    delta = diff.norm();
    if (delta < epsilon_) {
      solution_angle_out = current_joint;
      origin_to_end_out = origin_to_current * request.frame_to_end;
      origin_to_base_out = origin_to_base;
      return kSuccess;
    } else if (fabs(delta - delta_old) < converge_threshold_) {
      solution_angle_out = current_joint;
      origin_to_end_out = origin_to_current * request.frame_to_end;
      origin_to_base_out = origin_to_base;
      return kConverge;
    }
    if (!request.use_joints.empty()) {
      jacobian = robot_model_->GetJacobian(request.frame_name,
                                           request.frame_to_end,
                                           request.use_joints);
    }

    // Create Jacobian for rotation
    for (uint32_t i = 0; i < rotational_base_movements.size(); ++i) {
      rotational_base_jacobian.col(i) <<
          rotational_base_movements[i].cross(origin_to_current.translation() -
                                             origin_to_base.translation()),
          rotational_base_movements[i];
    }

    // Added the base movement Jacobian
    if (request.use_joints.empty()) {
      if (linear_base_movements.empty()) {
        if (rotational_base_movements.empty()) {
        } else {
          jacobian_with_base <<
              rotational_base_jacobian;
        }
      } else {
        if (rotational_base_movements.empty()) {
          jacobian_with_base <<
              linear_base_jacobian;
        } else {
          jacobian_with_base <<
              linear_base_jacobian, rotational_base_jacobian;
        }
      }
    } else {
      if (linear_base_movements.empty()) {
        if (rotational_base_movements.empty()) {
          jacobian_with_base = jacobian;
        } else {
          jacobian_with_base <<
              jacobian, rotational_base_jacobian;
        }
      } else {
        if (rotational_base_movements.empty()) {
          jacobian_with_base <<
              jacobian, linear_base_jacobian;
        } else {
          jacobian_with_base <<
              jacobian, linear_base_jacobian, rotational_base_jacobian;
        }
      }
    }
    // Newton-Rapson
    // angle_diff = jacobian_with_base.transpose() * (jacobian_with_base *
    //                                      jacobian_with_base.transpose()).inverse() * diff;
    // LM [sugihara 2009]
    angle_diff = (jacobian_with_base.transpose() * jacobian_with_base
                  + (diff.transpose() * diff)(0, 0)
                  * wn
                  + kWn * wn).inverse()
        * jacobian_with_base.transpose() * diff;
    current_joint.position += angle_diff.head(dof);
    // Corrected to protect the joint angle limit
    SaturateAngle(angle_min, angle_max, is_continuous_joint, current_joint.position);
    // Calculate the correction of the base position
    Eigen::Vector3d mod_base_pos = Eigen::Vector3d::Zero();
    for (uint32_t i = 0; i < linear_base_movements.size(); ++i) {
      mod_base_pos += angle_diff(dof + i) * linear_base_movements[i];
    }
    // Calculate the correction of the rotation of the base
    for (uint32_t i = 0; i < rotational_base_movements.size(); ++i) {
      double angle = angle_diff(dof + linear_base_movements.size() + i);
      origin_to_base = origin_to_base *
          Eigen::AngleAxisd(
              angle,
              origin_to_base.rotation() * rotational_base_movements[i]);
    }
    origin_to_base = Eigen::Translation3d(mod_base_pos) * origin_to_base;

    robot_model_->SetRobotTransform(origin_to_base);
    robot_model_->SetNamedAngle(current_joint);
  }
  solution_angle_out = current_joint;
  origin_to_base_out = origin_to_base;
  origin_to_end_out = origin_to_current * request.frame_to_end;
  return kMaxItr;
}

IKResult NumericIKSolver::Solve(const IKRequest& request,
                                std::vector<IKResponse>& responses_out) {
  return Solve(request, ReturnFalse, responses_out);
}

IKResult NumericIKSolver::Solve(const IKRequest& request,
                                std::function<bool()>& interrupt,
                                std::vector<IKResponse>& responses_out) {
  IKResponse response;
  if (Solve(request, interrupt,
            response.solution_angle, response.origin_to_base, response.origin_to_end) == kSuccess) {
    responses_out.clear();
    responses_out.push_back(response);
    return kSuccess;
  } else {
    return kFail;
  }
}

}  // namespace tmc_robot_kinematics_model


#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(tmc_robot_kinematics_model::NumericIKSolver,
                       tmc_robot_kinematics_model::IKSolver)


/// @brief Wrapper function for CTYPES generating IKRequest
/// @param [IN] Movement -based type
/// @return Req_p IKREQUEST pointer
void* create_request(tmc_manipulation_types::BaseMovementType movement) {
  tmc_robot_kinematics_model::IKRequest* req =
    new tmc_robot_kinematics_model::IKRequest(movement);
  void* req_p = reinterpret_cast<void*>(req);
  return req_p;
}

/// @brief Wrapper function for CTYPES to set the name of the frame to solve IK
/// @param [in, out] Req_p IKRequest pointer
/// @param [IN] FRAME_NAME frame name
void set_req_frame_name(void* req_p, char* frame_name) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    (struct tmc_robot_kinematics_model::IKRequest*)req_p;
  req->frame_name = frame_name;
}

/// @brief Wrapper function for CTYPES that sets conversion from frame to solve IK to End Coordinates
/// @param [in, out] Req_p IKRequest pointer
/// @param [IN] Conversion from MAT frame to End Coordinates.4x4 matrix
void set_req_frame_to_end(void* req_p, double* mat) {
  Eigen::Matrix4d  matrix;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      matrix(i, j) = mat[i * 4 + j];
    }
  }
  struct tmc_robot_kinematics_model::IKRequest* req =
    (struct tmc_robot_kinematics_model::IKRequest*)req_p;
  req->frame_to_end.matrix() = matrix;
}

/// @brief Wrapper function for CTYPES that sets conversion from origin to bass to iKRequest
/// @param [in, out] Req_p IKRequest pointer
/// @param [IN] Conversion from the origin to the base.4x4 matrix
void set_req_origin_to_base(void* req_p, double* mat) {
  Eigen::Matrix4d  matrix;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      matrix(i, j) = mat[i * 4 + j];
    }
  }
  struct tmc_robot_kinematics_model::IKRequest* req =
    (struct tmc_robot_kinematics_model::IKRequest*)req_p;
  req->origin_to_base.matrix() = matrix;
}

/// @brief Wrapper function for CTYPES to set the name of the joint at the start of IK to IKRequest
/// @param [in, out] Req_p IKRequest pointer
/// @param [IN] Angle_names joint name
/// @param [IN] Num_elements set to set
void set_req_initial_angle_name(void* req_p, char* angle_names[], int num_elements) {
  tmc_manipulation_types::NameSeq use_name;
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  req->initial_angle.name.resize(num_elements);
  for (int index = 0; index < num_elements; ++index) {
    char* angle_name = angle_names[index];
    use_name.push_back(angle_name);
  }
  req->initial_angle.name = use_name;
  req->use_joints = use_name;
}

/// @brief Wrapper function for CTYPES that sets a joint angle column at the start of IK to IKRequest
/// @param [in, out] Req_p IKRequest pointer
/// @param [IN] POS joint angle column
/// @param [IN] Len_pos set number to set
void set_req_initial_angle_position(void* req_p, float pos[], int len_pos) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  req->initial_angle.position.resize(len_pos);

  for (int i = 0; i < len_pos; i++) {
    req->initial_angle.position[i] = pos[i];
  }
}

/// @brief Wrapper function for CTYPES that sets weight to iKRequest
/// @param [in, out] Req_p IKRequest pointer
/// @param [IN] Weight weight array
/// @param [IN] Len_weight length of weight array
void set_req_weight(void* req_p, float weight[], int len_weight) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  req->weight.resize(len_weight);
  for (int i = 0; i < len_weight; i++) {
    req->weight[i] = weight[i];
  }
}

/// @brief Wrapper function for CTYPES that sets End Coordinates conversion from the origin to iKRequest
/// @param [in, out] Req_p IKRequest pointer
/// @param [IN] conversion of End Coordinates from MAT origin.4x4 matrix
void set_req_ref_origin_to_end(void* req_p, double* mat) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  Eigen::Matrix4d  matrix;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      matrix(i, j) = mat[i * 4 + j];
    }
  }
  req->ref_origin_to_end.matrix() = matrix;
}

/// @brief Wrapper function for CTYPES that returns the pointer of the Jointstate object
/// @return RES JOINTSTATE object pointer
void* jointstate() {
  JointState* js = new JointState;

  void* res = reinterpret_cast<void*>(js);
  return res;
}

/// @brief Wrapper function for CTYPES that returns the pointer of Affine3D objects
/// @return Affine3D object pointer
void* affine3d() {
  Eigen::Affine3d* affine3d = new Eigen::Affine3d;
  return reinterpret_cast<void*>(affine3d);
}

/// @brief Wrapper function for CTYPES to create IK Solver objects
/// @param [IN] Robot_p IK robot model (TARP3_WRAPPER) pointer
/// @param [IN] Max_itr Maximum number of repeated calculations
/// @param [in] Epsilon Xu Rong error
/// @param [in] Converge_threshold, if the change is smaller than this, it will be converged.
/// @return IK Solver object pointer
void* create_solver(void* robot_p, int max_itr,
                    float epsilon, float converge_threshold) {
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr* robot =
    reinterpret_cast<
    tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr*>(robot_p);
  tmc_robot_kinematics_model::IKSolver::Ptr* numeric_solver
    = new tmc_robot_kinematics_model::IKSolver::Ptr;
  numeric_solver->reset(
    new tmc_robot_kinematics_model::NumericIKSolver(
        tmc_robot_kinematics_model::IKSolver::Ptr(),
        *robot,
        max_itr,
        epsilon,
        converge_threshold));
  return reinterpret_cast<void*>(numeric_solver);
}

/// @brief Wrapper function for CTYPES to solve IK
/// @param [IN] Solver_p Solver pointer
/// @param [OUT] Solution_p Ik solved the joint angle column pointer
/// @param [OUT] ORIGIN_TO_BASE_BASE_SOLUTION_P conversion pointer from the origin to bass as a result
/// @param [OUT] ORIGIN_TO_HAND_RESULT_P conversion pointer from the origin to hand of the solution
/// @param [OUT] Req_p request pointer
void solve(void* solver_p, void* solution_p,
           void* origin_to_base_solution_p,
           void* origin_to_hand_result_p,
           void* req_p) {
  tmc_robot_kinematics_model::IKSolver::Ptr* numeric_solver =
    reinterpret_cast<tmc_robot_kinematics_model::IKSolver::Ptr*>(solver_p);
  JointState* solution = reinterpret_cast<JointState*>(solution_p);
  Eigen::Affine3d* origin_to_hand_result =
    reinterpret_cast<Eigen::Affine3d*>(origin_to_hand_result_p);
  Eigen::Affine3d* origin_to_base_solution =
    reinterpret_cast<Eigen::Affine3d*>(origin_to_base_solution_p);
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  tmc_robot_kinematics_model::IKResult result;
  // Solve.
  result = (*numeric_solver)->Solve(*req,
                                    *solution,
                                    *origin_to_base_solution,
                                    *origin_to_hand_result);
  // Print result.
  if (result == tmc_robot_kinematics_model::kSuccess) {
  } else {
    std::cout << "ik cannot be solved" << std::endl;
  }
}

/// @brief Wrapper function for CTYPES that returns a joint angle column as a result of solving IK
/// @param [IN] Pointer to Jointstate
/// @param [OUT] Result joint angle column
/// @param [IN] LEN number to use
void get_joint_angle(void* sol_p,
                     float* result,
                     int len) {
  JointState* solution = reinterpret_cast<JointState*>(sol_p);
  for (int i=0; i < len; i++) {
    result[i] = solution->position[i];
  }
}

/// @brief Wrapper function for CTYPES that returns conversion from the origin to bass as a result of solving IK
/// @param [IN] Pointer from Origin_to_base to the base conversion (affine3d)
/// @param [OUT] Conversion from the origin to the base (4x4 matrix (Serialized))
void get_origin_to_base(void* origin_to_base,
                        double* result) {
  Eigen::Affine3d* otb = reinterpret_cast<Eigen::Affine3d*>(origin_to_base);
  Eigen::MatrixXd  mat = otb->matrix();
  for (int i=0; i < 4; i++) {
    for (int j=0; j < 4; j++) {
      result[4*i + j] = mat(i, j);
    }
  }
}
