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
#ifndef TMC_ROBOT_KINEMATICS_MODEL_PINOCCHIO_WRAPPER_HPP_
#define TMC_ROBOT_KINEMATICS_MODEL_PINOCCHIO_WRAPPER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <pinocchio/multibody/model.hpp>

#include "tmc_robot_kinematics_model/robot_kinematics_model.hpp"

namespace tmc_robot_kinematics_model {

/// Pinocchio function failed
class PinocchioError : public std::domain_error {
 public:
  explicit PinocchioError(const std::string &error) :
    std::domain_error("error: " + error + " failed") {}
};

class PinocchioWrapper : public IRobotKinematicsModel {
 public:
  PinocchioWrapper();
  explicit PinocchioWrapper(const std::string& robot_description);
  virtual ~PinocchioWrapper() = default;

  /// Initialization of robot models
  void Initialize(const std::string& robot_description) override;

  /// Enter the position posture of the robot
  void SetRobotTransform(const Eigen::Affine3d& transform) override;
  /// Acquired the position posture of the robot
  Eigen::Affine3d GetRobotTransform() const override;

  /// Specify the joint name of the robot and enter the joint angle
  void SetNamedAngle(const tmc_manipulation_types::JointState& angle) override;
  /// Obtain the joint name of the robot and its angle
  tmc_manipulation_types::JointState GetNamedAngle() const override;
  /// Obtain the joint name of the robot and its angle
  tmc_manipulation_types::JointState GetNamedAngle(const tmc_manipulation_types::NameSeq& joint_names) const override;

  /// Obtain the position posture of the object
  Eigen::Affine3d GetObjectTransform(const std::string& name) const override;
  /// Obtain the relative position posture of the object
  Eigen::Affine3d GetObjectRelativeTransform(const std::string& base_name, const std::string& name) const override;

  /// Add a frame dynamically
  void CreateFrame(const std::string& parent_frame_name,
                   const Eigen::Affine3d& transform,
                   const std::string& new_frame_name) override;
  /// Dynamically delete the frame
  void DestroyFrame(const std::string& frame_name) override;

  /// Get Jacobian
  Eigen::MatrixXd GetJacobian(const std::string& frame_name,
                              const Eigen::Affine3d& frame_to_end,
                              const std::vector<std::string>& use_joints) override;
  /// Get joint min and max
  void GetMinMax(const tmc_manipulation_types::NameSeq& use_joints,
                 Eigen::VectorXd& min,
                 Eigen::VectorXd& max) const override;

 private:
  pinocchio::Model model_;
  std::shared_ptr<pinocchio::Data> data_;

  // The GET function that calls Executefk is a consst, but it is efficient to update the data_ with EXECUTEFK.
  // Mutable inevitably
  void ExecuteFK() const;
  mutable bool do_fk_;

  Eigen::Affine3d origin_to_robot_;
  std::map<std::string, double> joint_angles_;
  int jacobian_index_;

  struct MimicJointInfo {
    std::string joint_name;
    double multiplier;
    double offset;
  };
  std::map<std::string, std::vector<MimicJointInfo>> mimic_joint_info_;
};

}  // namespace tmc_robot_kinematics_model

#endif  // TMC_ROBOT_KINEMATICS_MODEL_PINOCCHIO_WRAPPER_HPP_
