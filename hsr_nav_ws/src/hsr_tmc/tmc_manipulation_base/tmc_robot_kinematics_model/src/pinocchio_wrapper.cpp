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
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <urdf_parser/urdf_parser.h>

namespace {

void Convert(const pinocchio::Model& model, const std::map<std::string, double>& joint_angles,
             Eigen::VectorXd& angle_out) {
  angle_out = Eigen::VectorXd::Zero(model.nq);
  for (int i = 0; i < model.njoints; ++i) {
    if (joint_angles.find(model.names[i]) != joint_angles.end()) {
      if (model.joints[i].nq() == 2) {
        // https://github.com/stack-of-tasks/pinocchio/issues/794
        angle_out[model.idx_qs[i]] = std::cos(joint_angles.at(model.names[i]));
        angle_out[model.idx_qs[i] + 1] = std::sin(joint_angles.at(model.names[i]));
      } else {
        angle_out[model.idx_qs[i]] = joint_angles.at(model.names[i]);
      }
    }
  }
}

}  // namespace

namespace tmc_robot_kinematics_model {

PinocchioWrapper::PinocchioWrapper()
    : origin_to_robot_(Eigen::Affine3d::Identity()), do_fk_(true) {}

PinocchioWrapper::PinocchioWrapper(const std::string& robot_description) : PinocchioWrapper() {
  Initialize(robot_description);
}

void PinocchioWrapper::Initialize(const std::string& robot_description) {
  try {
    pinocchio::urdf::buildModelFromXML(robot_description, model_);
  } catch(std::invalid_argument& e) {
    // It seems useless to catch and throw it again, but it can't be helped because of the consistency with the existing test.
    throw PinocchioError(e.what());
  }

  jacobian_index_ = model_.addBodyFrame("jacobian_frame", 0);

  for (const auto& joint : model_.joints) {
    // "INDEX in joint position is negative", such as the 0th joint UNIVERSE, so play it because it is not a real joint
    // If the degree of freedom of the joints is 1 or more, the degree of freedom of the Continous joint is treated as 2.
    if (joint.idx_q() >= 0 && joint.nq() > 0) {
      joint_angles_.insert(std::make_pair(model_.names[joint.id()], 0.0));
    }
  }
  data_ = std::make_shared<pinocchio::Data>(model_);

  tmc_manipulation_types::JointState initial_state;
  auto robot = urdf::parseURDF(robot_description);
  for (const auto& joint : robot->joints_) {
    if (joint.second->mimic) {
      if (joint_angles_.find(joint.first) == joint_angles_.end()) {
        throw PinocchioError("Joint " + joint.first + " is not found.");
      }
      if (joint_angles_.find(joint.second->mimic->joint_name) == joint_angles_.end()) {
        throw PinocchioError("Joint " + joint.second->mimic->joint_name + " is not found.");
      }
      MimicJointInfo info;
      info.joint_name = joint.first;
      info.multiplier = joint.second->mimic->multiplier;
      info.offset = joint.second->mimic->offset;
      if (mimic_joint_info_.find(joint.second->mimic->joint_name) == mimic_joint_info_.end()) {
        mimic_joint_info_[joint.second->mimic->joint_name] = {info};
      } else {
        mimic_joint_info_[joint.second->mimic->joint_name].push_back(info);
      }
      initial_state.name.push_back(joint.second->mimic->joint_name);
    }
  }
  initial_state.position = Eigen::VectorXd::Zero(initial_state.name.size());
  // To update Joint_angles_ in the mimic joint
  SetNamedAngle(initial_state);
}

// Enter the position posture of the robot
void PinocchioWrapper::SetRobotTransform(const Eigen::Affine3d& transform) {
  origin_to_robot_ = transform;
  do_fk_ = true;
}

// Acquired the position posture of the robot
Eigen::Affine3d PinocchioWrapper::GetRobotTransform() const {
  return origin_to_robot_;
}

// Specify the joint name of the robot and enter the joint angle
void PinocchioWrapper::SetNamedAngle(const tmc_manipulation_types::JointState& angle) {
  if (angle.name.size() != angle.position.size()) {
    throw PinocchioError("Joint name size and position size are mismatch.");
  }
  for (int i = 0; i < angle.name.size(); ++i) {
    if (joint_angles_.find(angle.name[i]) == joint_angles_.end()) {
      throw PinocchioError("Joint " + angle.name[i] + " is not found.");
    }
    joint_angles_[angle.name[i]] = angle.position[i];
  }
  // Turn the loop again so that you can overwrite even if you have subordinate joints in Angle
  for (int i = 0; i < angle.name.size(); ++i) {
    if (mimic_joint_info_.find(angle.name[i]) != mimic_joint_info_.end()) {
      for (const auto& info : mimic_joint_info_[angle.name[i]]) {
        // If you come so far, you can not check the existence because Joint_angles_ is unlikely to have a key.
        joint_angles_[info.joint_name] = angle.position[i] * info.multiplier + info.offset;
      }
    }
  }
  do_fk_ = true;
}

// Obtain the joint name of the robot and its angle
tmc_manipulation_types::JointState PinocchioWrapper::GetNamedAngle() const {
  // TODO(Takeshita) たかだが10, 20だからmapでいいと思うけれど，別のコンテナも検討する
  tmc_manipulation_types::JointState joint_state;
  joint_state.position.resize(joint_angles_.size());
  for (std::map<std::string, double>::const_iterator it = joint_angles_.cbegin(); it != joint_angles_.cend(); ++it) {
    const auto index = std::distance(joint_angles_.cbegin(), it);
    joint_state.name.push_back(it->first);
    joint_state.position[index] = it->second;
  }
  return joint_state;
}

// Obtain the joint name of the robot and its angle
tmc_manipulation_types::JointState PinocchioWrapper::GetNamedAngle(
    const tmc_manipulation_types::NameSeq& joint_names) const {
  tmc_manipulation_types::JointState joint_state;
  joint_state.name = joint_names;
  joint_state.position.resize(joint_names.size());
  for (int i = 0; i < joint_names.size(); ++i) {
    if (joint_angles_.find(joint_names[i]) == joint_angles_.end()) {
      throw PinocchioError("Joint "+ joint_names[i] + " is not found.");
    }
    joint_state.position[i] = joint_angles_.at(joint_names[i]);
  }
  return joint_state;
}

// Obtain the position posture of the object
Eigen::Affine3d PinocchioWrapper::GetObjectTransform(const std::string& name) const {
  const auto index = model_.getFrameId(name);
  if (index == model_.nframes) {
    throw PinocchioError("object "+ name + " is not found.");
  }

  ExecuteFK();
  auto pose = pinocchio::updateFramePlacement(model_, *data_, index);
  return origin_to_robot_ * (Eigen::Translation3d(pose.translation()) * Eigen::Quaterniond(pose.rotation()));
}

// Obtain the relative position posture of the object
Eigen::Affine3d PinocchioWrapper::GetObjectRelativeTransform(
    const std::string& base_name, const std::string& name) const {
  const auto base_index = model_.getFrameId(base_name);
  if (base_index == model_.nframes) {
    throw PinocchioError("object "+ base_name + " is not found.");
  }

  const auto index = model_.getFrameId(name);
  if (index == model_.nframes) {
    throw PinocchioError("object "+ name + " is not found.");
  }

  ExecuteFK();
  auto root_to_base = pinocchio::updateFramePlacement(model_, *data_, base_index);
  auto root_to_target = pinocchio::updateFramePlacement(model_, *data_, index);
  auto base_to_target = root_to_base.inverse() * root_to_target;
  return Eigen::Translation3d(base_to_target.translation()) * Eigen::Quaterniond(base_to_target.rotation());
}

// Add a frame dynamically
void PinocchioWrapper::CreateFrame(
    const std::string& parent_frame_name,
    const Eigen::Affine3d& transform,
    const std::string& new_frame_name) {
  const auto new_frame_index = model_.addBodyFrame("jacobian_frame", 0);
  const auto parent_frame_index = model_.getFrameId(parent_frame_name);
  if (parent_frame_index == model_.nframes) {
    throw PinocchioError("object "+ parent_frame_name + " is not found.");
  }
  model_.addBodyFrame(new_frame_name, model_.frames[parent_frame_index].parent,
                      pinocchio::SE3(transform.linear(), transform.translation()), parent_frame_index);

  data_ = std::make_shared<pinocchio::Data>(model_);
}

// Dynamically delete the frame
void PinocchioWrapper::DestroyFrame(const std::string& frame_name) {
  // In the implementation of addFrame, it should be deleted with this
  // However, if you start INertia, you will need to delete it.
  const auto it = std::find_if(model_.frames.begin(), model_.frames.end(),
                               [&frame_name](decltype(model_.frames.front()) x) { return x.name == frame_name; });
  if (it != model_.frames.end()) {
    model_.frames.erase(it);
    --model_.nframes;
  } else {
    throw PinocchioError("frame "+ frame_name + " is not found.");
  }

  data_ = std::make_shared<pinocchio::Data>(model_);
}

// Get Jacobian
Eigen::MatrixXd PinocchioWrapper::GetJacobian(
    const std::string& frame_name,
    const Eigen::Affine3d& frame_to_end,
    const std::vector<std::string>& use_joints) {
  const auto index = model_.getFrameId(frame_name);
  if (index == model_.nframes) {
    throw PinocchioError("object "+ frame_name + " is not found.");
  }
  model_.frames[jacobian_index_].parent = model_.frames[index].parent;
  model_.frames[jacobian_index_].previousFrame = model_.frames[index].previousFrame;
  model_.frames[jacobian_index_].placement =
      model_.frames[index].placement * pinocchio::SE3(frame_to_end.linear(), frame_to_end.translation());

  Eigen::VectorXd joint_angles;
  Convert(model_, joint_angles_, joint_angles);
  pinocchio::Data data(model_);

  pinocchio::Data::Matrix6x jacobian(6, model_.nv);
  jacobian.setZero();
  pinocchio::computeFrameJacobian(
      model_, data, joint_angles, jacobian_index_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

  // Take out only USE_JOINTS, but it shines one due to the influence of the mysterious 0th joint UNIVERSE
  Eigen::MatrixXd result(6, use_joints.size());
  for (int joint_name_index = 0; joint_name_index < use_joints.size(); ++joint_name_index) {
    const auto joint_id = model_.getJointId(use_joints[joint_name_index]);
    if (joint_id == model_.njoints) {
      throw PinocchioError("Joint "+ use_joints[joint_name_index] + " is not found.");
    }
    result.col(joint_name_index) = jacobian.col(joint_id - 1);

    // Is this okay?Considering the meaning of Jacobian, I feel good.
    if (mimic_joint_info_.find(use_joints[joint_name_index]) != mimic_joint_info_.end()) {
      for (const auto& info : mimic_joint_info_[use_joints[joint_name_index]]) {
        const auto joint_id = model_.getJointId(info.joint_name);
        result.col(joint_name_index) += info.multiplier * jacobian.col(joint_id - 1);
      }
      result.col(joint_name_index).normalize();
    }
  }

  Eigen::MatrixXd origin_to_robot = Eigen::MatrixXd::Zero(6, 6);
  origin_to_robot.block<3, 3>(0, 0) = origin_to_robot_.linear();
  origin_to_robot.block<3, 3>(3, 3) = origin_to_robot_.linear();
  return origin_to_robot * result;
}

// Get joint min and max
void PinocchioWrapper::GetMinMax(
    const tmc_manipulation_types::NameSeq& use_joints,
    Eigen::VectorXd& min,
    Eigen::VectorXd& max) const {
  min.resize(use_joints.size());
  max.resize(use_joints.size());
  for (int i = 0; i < use_joints.size(); ++i) {
    const auto joint_id = model_.getJointId(use_joints[i]);
    if (joint_id == model_.njoints) {
      throw PinocchioError("Joint "+ use_joints[i] + " is not found.");
    }
    const auto limit_id = model_.idx_qs[joint_id];
    min[i] = model_.lowerPositionLimit[limit_id];
    max[i] = model_.upperPositionLimit[limit_id];
  }
}

void PinocchioWrapper::ExecuteFK() const {
  if (do_fk_) {
    Eigen::VectorXd joint_angles;
    Convert(model_, joint_angles_, joint_angles);
    pinocchio::forwardKinematics(model_, *data_, joint_angles);
    do_fk_ = false;
  }
}

}  // namespace tmc_robot_kinematics_model

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(tmc_robot_kinematics_model::PinocchioWrapper,
                       tmc_robot_kinematics_model::IRobotKinematicsModel)
