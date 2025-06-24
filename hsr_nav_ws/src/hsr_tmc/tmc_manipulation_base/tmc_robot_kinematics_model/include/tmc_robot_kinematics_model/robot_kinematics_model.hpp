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
#ifndef TMC_ROBOT_KINEMATICS_MODEL_ROBOT_KINEMATICS_MODEL_HPP_
#define TMC_ROBOT_KINEMATICS_MODEL_ROBOT_KINEMATICS_MODEL_HPP_
#include <stdint.h>

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tmc_manipulation_types/manipulation_types.hpp>

namespace tmc_robot_kinematics_model {

/// Jointstate is an unauthorized exception
class JointStateError : public std::domain_error {
 public:
  explicit JointStateError(const std::string &error) :
    std::domain_error("error: " + error + " failed") {}
};

/// Robot athletic model
/// For the time being, only the functions required around the interference check
class IRobotKinematicsModel {
 public:
  using Ptr = std::shared_ptr<IRobotKinematicsModel>;
  virtual ~IRobotKinematicsModel() {}

  /// @brief  Initialization of robot models
  /// @param  [IN] robot_description robot model
  virtual void Initialize(const std::string& robot_description) = 0;

  /// @brief  Set the pose of the robot
  /// @param  [IN] transform Robot's pose
  virtual void SetRobotTransform(const Eigen::Affine3d& transform) = 0;

  /// @brief  Acquired the position posture of the robot
  /// @return Eigen::Affine3d Robot's pose
  virtual Eigen::Affine3d GetRobotTransform(void) const = 0;

  /// @brief  Specify the joint name of the robot and enter the joint angle
  /// @param  [IN] angle robot joint configuration
  virtual void SetNamedAngle(
      const tmc_manipulation_types::JointState& angle) = 0;

  /// @brief  Get the robot joint configuration
  virtual tmc_manipulation_types::JointState GetNamedAngle(void) const = 0;

  /// @brief  Get the joint configuration of the robot by specifying the joint name
  /// @param  [IN] joint_names Joint names
  virtual tmc_manipulation_types::JointState GetNamedAngle(
      const tmc_manipulation_types::NameSeq& joint_names) const = 0;

  /// @brief  Obtain the pose of the object
  /// @param  [IN] name Object name
  virtual Eigen::Affine3d GetObjectTransform(
      const std::string& name) const = 0;
  /// Obtain the relative poseof the object
  virtual Eigen::Affine3d GetObjectRelativeTransform(
      const std::string& base_name, const std::string& name) const = 0;

  /// @brief  Add a frame dynamically
  /// @param  [IN] parent_frame_name Name of parent frame
  /// @param  [IN] transform  Pose based on parent frame
  /// @param  [IN] new_frame_name Adding frame name
  virtual void CreateFrame(
      const std::string& parent_frame_name, const Eigen::Affine3d& transform,
      const std::string& new_frame_name) = 0;

  /// @brief  Dynamically delete the frame
  /// @param  [IN] frame_name deleted frame name
  virtual void DestroyFrame(const std::string& frame_name) = 0;

  /// @brief  Get Jacobian
  /// @param  [IN] frame_name Target frame
  /// @param  [IN] frame_to_end Offset from target frame
  /// @param  [IN] use_joints Target joint list
  virtual Eigen::MatrixXd GetJacobian(const std::string& frame_name,
                                      const Eigen::Affine3d& frame_to_end,
                                      const tmc_manipulation_types::NameSeq& use_joints) = 0;

  /// @brief  Get joint min and max
  /// @param  [IN] use_joints Joint names
  /// @param  [OUT] min Lower limit
  /// @param  [OUT] max Upper limits
  virtual void GetMinMax(const tmc_manipulation_types::NameSeq& use_joints,
                         Eigen::VectorXd& min, Eigen::VectorXd& max) const = 0;
};
}  // namespace tmc_robot_kinematics_model
#endif /* ROBOT_KINEMATICS_MODEL_HPP_ */
