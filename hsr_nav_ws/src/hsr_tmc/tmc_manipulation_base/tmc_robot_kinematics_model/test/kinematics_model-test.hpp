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
#include <algorithm>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <tmc_manipulation_tests/configs.hpp>

namespace tmc_robot_kinematics_model {

using tmc_manipulation_types::JointState;
using tmc_manipulation_types::NameSeq;

const uint32_t kJointNameNum = 6;
const uint32_t kTotalJointNum = 8;
const char* const kJointName[kJointNameNum] = {
  "joint1", "joint2", "joint3",
  "joint4", "joint5", "joint6"};
const char* const kContinuousJointName = "dummy_continuous_joint";
const double kDoubleEps = 1e-10;

/// Calculate that the two postures are close
/// @param [IN] Expect expected attitude
/// @param [IN] Value comparison value
/// @param [IN] Angular_eps Angle offers [RAD]
/// @param [IN] Translation_eps Positional tolerance value [M]
void ExpectTransformsNear(Eigen::Affine3d expect,
                          Eigen::Affine3d value,
                          double angular_eps = 1.0e-6,
                          double translation_eps = 1.0e-6) {
  Eigen::Affine3d expect_to_value = expect.inverse() * value;
  double diff_angle = Eigen::AngleAxisd(expect_to_value.linear()).angle();
  EXPECT_NEAR(0.0, diff_angle, angular_eps);
  double diff_trans = expect_to_value.translation().norm();
  EXPECT_NEAR(0.0, diff_trans, translation_eps);
}

template<typename T>
class RobotKinematicsModelTest : public ::testing::Test {
 public:
  virtual ~RobotKinematicsModelTest() = default;

  void SetUp() override;

 protected:
  IRobotKinematicsModel::Ptr robot_model_;
  NameSeq joint_names_;
};

template<typename T>
void RobotKinematicsModelTest<T>::SetUp() {
  robot_model_ = std::make_shared<T>(tmc_manipulation_tests::stanford_manipulator::GetUrdf());

  for (uint32_t i = 0; i < kJointNameNum; i++) {
    joint_names_.push_back(kJointName[i]);
  }
  JointState angle;
  robot_model_->SetNamedAngle(angle);
}

TYPED_TEST_SUITE_P(RobotKinematicsModelTest);

// Urdf is fraudulent
TYPED_TEST_P(RobotKinematicsModelTest, InvalidUrdf) {
  EXPECT_THROW(
      IRobotKinematicsModel::Ptr robot_model_(new TypeParam("invalid urdf")),
      std::domain_error);
}

// It works even if generated and initialization are performed separately
TYPED_TEST_P(RobotKinematicsModelTest, SeparateGenerationAndInitialization) {
  this->robot_model_ = std::make_shared<TypeParam>();
  this->robot_model_->Initialize(tmc_manipulation_tests::stanford_manipulator::GetUrdf());

  const auto angles = this->robot_model_->GetNamedAngle();
  EXPECT_EQ(kTotalJointNum, angles.name.size());
  EXPECT_EQ(kTotalJointNum, angles.position.size());
}

// The initial position/posture of the robot coordinate system is zero
TYPED_TEST_P(RobotKinematicsModelTest, GetInitialRobotTransform) {
  const auto get_transform = this->robot_model_->GetRobotTransform();
  ExpectTransformsNear(Eigen::Affine3d::Identity(), get_transform);
}

// Check that a robot coordinate system can be set
TYPED_TEST_P(RobotKinematicsModelTest, SetAndGetRobotTransform) {
  Eigen::Affine3d transform(Eigen::Affine3d::Identity());
  transform.translation().setRandom();
  Eigen::Affine3d get_transform;
  this->robot_model_->SetRobotTransform(transform);
  get_transform = this->robot_model_->GetRobotTransform();
  ExpectTransformsNear(transform, get_transform);
}

// Check that all joint angles can be obtained from the robot
TYPED_TEST_P(RobotKinematicsModelTest, GetAllJointAngles) {
  JointState angles;
  angles = this->robot_model_->GetNamedAngle();
  EXPECT_EQ(kTotalJointNum, angles.name.size());
  EXPECT_EQ(kTotalJointNum, angles.position.size());
}

// Check that some joint angles can be obtained from the robot
TYPED_TEST_P(RobotKinematicsModelTest, GetPartialJointAngles) {
  JointState angles;
  angles = this->robot_model_->GetNamedAngle(this->joint_names_);
  EXPECT_EQ(angles.name.size(), this->joint_names_.size());
  EXPECT_EQ(angles.position.size(), this->joint_names_.size());
}

// Exception when reading Getnamedangle in a joint with a named robot for a robot
TYPED_TEST_P(RobotKinematicsModelTest, ErrorInvalidPartialJointAngles) {
  JointState angles;
  NameSeq invalid_name_seq;
  invalid_name_seq.push_back("hoge_piyo");
  EXPECT_THROW({
      angles = this->robot_model_->GetNamedAngle(invalid_name_seq);
    }, std::domain_error);
}

// Confirm that the joint angle can be set in the robot
TYPED_TEST_P(RobotKinematicsModelTest, SetPartialJointAngles) {
  JointState target_angles;
  JointState result_angles;
  target_angles.name = this->joint_names_;
  target_angles.position.resize(kJointNameNum);
  target_angles.position << 0.0, 0.1, 0.2, 0.3, 0.4, 0.5;
  this->robot_model_->SetNamedAngle(target_angles);
  result_angles = this->robot_model_->GetNamedAngle(this->joint_names_);
  ASSERT_EQ(result_angles.name.size(), target_angles.name.size());
  ASSERT_EQ(result_angles.position.size(), target_angles.position.size());

  for (uint32_t i = 0; i < this->joint_names_.size(); ++i) {
    EXPECT_EQ(result_angles.name[i], target_angles.name[i]);
    EXPECT_FLOAT_EQ(result_angles.position(i), target_angles.position(i));
  }
}

// Infinity rotation joint works correctly
TYPED_TEST_P(RobotKinematicsModelTest, SetContinuousJointAngles) {
  const auto before = this->robot_model_->GetObjectRelativeTransform("link1", "dummy_continuous_link");

  JointState target_angles;
  target_angles.name = {kContinuousJointName};
  target_angles.position.resize(1);
  target_angles.position << 10.0;
  this->robot_model_->SetNamedAngle(target_angles);

  const auto yaw = this->robot_model_->GetObjectRelativeTransform(
      "link1", "dummy_continuous_link").linear().eulerAngles(0, 1, 2)[2];
  EXPECT_DOUBLE_EQ(yaw, 10.0 - 4.0 * M_PI);

  const auto jacobian = this->robot_model_->GetJacobian(
      "dummy_continuous_link", Eigen::Affine3d::Identity(), {"dummy_continuous_joint"});
  EXPECT_NEAR(0.0, jacobian(0, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(1, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(2, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(3, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(4, 0), kDoubleEps);
  EXPECT_NEAR(1.0, jacobian(5, 0), kDoubleEps);
}

// MIMIC joint works correctly
TYPED_TEST_P(RobotKinematicsModelTest, MimicJoint) {
  auto mimic_joint_state = this->robot_model_->GetNamedAngle({"dummy_mimic_joint"});
  EXPECT_DOUBLE_EQ(mimic_joint_state.position[0], -0.5 * 0.0 + 1.57);

  JointState target_angles;
  target_angles.name = {"joint6"};
  target_angles.position.resize(1);
  target_angles.position << 0.2;
  this->robot_model_->SetNamedAngle(target_angles);

  mimic_joint_state = this->robot_model_->GetNamedAngle({"dummy_mimic_joint"});
  EXPECT_DOUBLE_EQ(mimic_joint_state.position[0], -0.5 * 0.2 + 1.57);

  const auto pitch = this->robot_model_->GetObjectRelativeTransform(
      "link7", "dummy_mimic_link").linear().eulerAngles(1, 0, 2)[0];
  EXPECT_DOUBLE_EQ(pitch, -0.5 * 0.2 + 1.57);

  // If it is tilted, the output is difficult to understand, so return it
  target_angles.position << 0.0;
  this->robot_model_->SetNamedAngle(target_angles);

  auto jacobian = this->robot_model_->GetJacobian("link7", Eigen::Affine3d::Identity(), {"joint6"});
  EXPECT_NEAR(0.0, jacobian(0, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(1, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(2, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(3, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(4, 0), kDoubleEps);
  EXPECT_NEAR(1.0, jacobian(5, 0), kDoubleEps);

  jacobian = this->robot_model_->GetJacobian("dummy_mimic_link", Eigen::Affine3d::Identity(), {"joint6"});
  EXPECT_NEAR(0.0, jacobian(0, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(1, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(2, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(3, 0), kDoubleEps);
  EXPECT_NEAR(-0.5 / std::sqrt(0.5 * 0.5 + 1.0 * 1.0), jacobian(4, 0), kDoubleEps);
  EXPECT_NEAR(1.0 / std::sqrt(0.5 * 0.5 + 1.0 * 1.0), jacobian(5, 0), kDoubleEps);
}

// Confirm that exceptions are thrown when set with an unauthorized name
TYPED_TEST_P(RobotKinematicsModelTest, InvalidNameSetPartialJointAngles) {
  JointState target_angles;
  target_angles.name = this->joint_names_;
  target_angles.position.resize(this->joint_names_.size());
  target_angles.name[0] = "hoge_piyo";
  EXPECT_THROW({
      this->robot_model_->SetNamedAngle(target_angles);
    }, std::domain_error);
}

// Confirm that exceptions are thrown when the length of the name and angle is different
TYPED_TEST_P(RobotKinematicsModelTest, UnevenJointstateSetPartialJointAngles) {
  JointState target_angles;
  target_angles.name = this->joint_names_;
  target_angles.position.resize(this->joint_names_.size() + 1);
  EXPECT_THROW({
      this->robot_model_->SetNamedAngle(target_angles);
    }, std::domain_error);
}

// Check if the position posture of the object can be obtained
TYPED_TEST_P(RobotKinematicsModelTest, GetObjectTransform) {
  std::string name = "link2";
  Eigen::Affine3d origin_to_object;
  Eigen::Affine3d origin_to_robot(Eigen::Affine3d::Identity());
  Eigen::Affine3d get_transform;
  this->robot_model_->SetRobotTransform(origin_to_robot);
  origin_to_object = this->robot_model_->GetObjectTransform(name);
  ExpectTransformsNear(Eigen::Affine3d::Identity(), origin_to_object);
}

// Check if you can get the position posture of the URDF root link
TYPED_TEST_P(RobotKinematicsModelTest, GetObjectTransformRoot) {
  std::string name = "link1";
  Eigen::Affine3d origin_to_object;
  Eigen::Affine3d origin_to_robot(Eigen::Affine3d::Identity());
  Eigen::Affine3d get_transform;
  this->robot_model_->SetRobotTransform(origin_to_robot);
  origin_to_object = this->robot_model_->GetObjectTransform(name);
  ExpectTransformsNear(origin_to_robot, origin_to_object);
}

// Check if you can get the position posture of the object via Fixed_Joint
TYPED_TEST_P(RobotKinematicsModelTest, GetObjectTransformFixed) {
  std::string name = "frame1";
  Eigen::Affine3d origin_to_object;
  Eigen::Affine3d origin_to_robot(Eigen::Affine3d::Identity());
  Eigen::Affine3d get_transform;
  this->robot_model_->SetRobotTransform(origin_to_robot);
  origin_to_object = this->robot_model_->GetObjectTransform(name);
  Eigen::Affine3d expected_origin_to_object =
      Eigen::Translation3d(1.0, 2.0, 3.0)
      * Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ());
  ExpectTransformsNear(expected_origin_to_object, origin_to_object);
}

// Domain_error appears when an object fraudulent name is given
TYPED_TEST_P(RobotKinematicsModelTest, InvalidObjectTransform) {
  std::string name = "hoge_piyo";
  Eigen::Affine3d robot_to_object;
  Eigen::Affine3d origin_to_robot(Eigen::Affine3d::Identity());
  this->robot_model_->SetRobotTransform(origin_to_robot);
  EXPECT_THROW({
      robot_to_object = this->robot_model_->GetObjectTransform(name);
    }, std::domain_error);
}

// Check if the relative position of the object can be obtained
TYPED_TEST_P(RobotKinematicsModelTest, GetObjectRelativeTransform) {
  std::string parent_name = "joint3";
  std::string child_name = "joint4";
  Eigen::Affine3d parent_to_child(Eigen::Affine3d::Identity());
  parent_to_child = this->robot_model_->GetObjectRelativeTransform(parent_name, child_name);
  Eigen::Affine3d expect_parent_to_child(Eigen::Affine3d::Identity());
  expect_parent_to_child.translation().x() = 0.0;
  expect_parent_to_child.translation().y() = 0.0;
  expect_parent_to_child.translation().z() = 0.25;
  ExpectTransformsNear(expect_parent_to_child, parent_to_child);
}

// Check if the relative position of the URDF root link and the object can be obtained
TYPED_TEST_P(RobotKinematicsModelTest, GetObjectRelativeTransformRoot) {
  std::string root_name = "link1";
  std::string target_name = "link2";
  Eigen::Affine3d root_to_target(Eigen::Affine3d::Identity());
  Eigen::Affine3d origin_to_robot(Eigen::Affine3d::Identity());
  origin_to_robot.translation().x() = 1.0;
  this->robot_model_->SetRobotTransform(origin_to_robot);
  root_to_target = this->robot_model_->GetObjectRelativeTransform(root_name, target_name);
  ExpectTransformsNear(Eigen::Affine3d::Identity(), root_to_target);
}

// Domain_error appears when an object fraudulent name is given
TYPED_TEST_P(RobotKinematicsModelTest, InvalidObjectRelativeTransform) {
  std::string parent_name = "joint3";
  std::string child_name = "joint4";
  std::string invalid_name = "hoge_piyo";
  EXPECT_THROW({
      this->robot_model_->GetObjectRelativeTransform(parent_name, invalid_name);
    }, std::domain_error);
  EXPECT_THROW({
      this->robot_model_->GetObjectRelativeTransform(invalid_name, child_name);
    }, std::domain_error);
  EXPECT_THROW({
      this->robot_model_->GetObjectRelativeTransform(invalid_name, invalid_name);
    }, std::domain_error);
}

// Dynamic frame additional test
TYPED_TEST_P(RobotKinematicsModelTest, CreateNewFrame) {
  const std::string parent_name = "joint1";
  Eigen::Affine3d parent_to_frame(Eigen::Affine3d::Identity());
  parent_to_frame.translation().setRandom();
  const std::string frame_name = "new_frame";

  this->robot_model_->CreateFrame(parent_name, parent_to_frame, frame_name);

  Eigen::Affine3d parent_to_frame_got =
      this->robot_model_->GetObjectRelativeTransform(parent_name, frame_name);
  ExpectTransformsNear(parent_to_frame, parent_to_frame_got);
}

// Additional test failure of dynamic frame
TYPED_TEST_P(RobotKinematicsModelTest, CreateToInvalidFrame) {
  const std::string parent_name = "hoge_piyo";
  Eigen::Affine3d parent_to_frame(Eigen::Affine3d::Identity());
  parent_to_frame.translation().setRandom();
  const std::string frame_name = "new_frame";
  EXPECT_THROW({
      this->robot_model_->CreateFrame(parent_name, parent_to_frame, frame_name);
    }, std::domain_error);
}

// Dynamic frame deletion
TYPED_TEST_P(RobotKinematicsModelTest, DeleteDynamicFrame) {
  const std::string parent_name = "joint1";
  Eigen::Affine3d parent_to_frame(Eigen::Affine3d::Identity());
  parent_to_frame.translation().setRandom();
  const std::string frame_name = "new_frame";

  this->robot_model_->CreateFrame(parent_name, parent_to_frame, frame_name);
  this->robot_model_->DestroyFrame(frame_name);

  EXPECT_THROW({
      Eigen::Affine3d parent_to_frame_got =
          this->robot_model_->GetObjectRelativeTransform(parent_name, frame_name);
    }, std::domain_error);
}

// Dynamic frame deletion failure
TYPED_TEST_P(RobotKinematicsModelTest, DeleteInvalidFrame) {
  const std::string parent_name = "joint1";
  Eigen::Affine3d parent_to_frame(Eigen::Affine3d::Identity());
  parent_to_frame.translation().setRandom();
  const std::string frame_name = "new_frame";

  this->robot_model_->CreateFrame(parent_name, parent_to_frame, frame_name);
  EXPECT_THROW({
      this->robot_model_->DestroyFrame("hoge_piyo");
    }, std::domain_error);
}

// Acquisition of Jacobian
TYPED_TEST_P(RobotKinematicsModelTest, GetJacobianNormal) {
  std::string frame_name("link7");
  Eigen::Affine3d frame_to_end(Eigen::Affine3d::Identity());
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("joint3");
  Eigen::MatrixXd jacobian;
  jacobian = this->robot_model_->GetJacobian(frame_name, frame_to_end, use_joints);
  ASSERT_EQ(6, jacobian.rows());
  ASSERT_EQ(use_joints.size(), jacobian.cols());
  // Only Z should be 1.0
  EXPECT_NEAR(0.0, jacobian(0, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(1, 0), kDoubleEps);
  EXPECT_NEAR(1.0, jacobian(2, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(3, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(4, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(5, 0), kDoubleEps);
}

// Jacobian changes by moving the acquired pedestal of Jacobian
TYPED_TEST_P(RobotKinematicsModelTest, GetJacobianRotateLinear) {
  std::string frame_name("link7");
  Eigen::Affine3d frame_to_end(Eigen::Affine3d::Identity());
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("joint3");
  Eigen::MatrixXd jacobian;
  Eigen::Affine3d origin_to_robot = Eigen::Affine3d::Identity() *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
  this->robot_model_->SetRobotTransform(origin_to_robot);
  jacobian = this->robot_model_->GetJacobian(frame_name, frame_to_end, use_joints);
  ASSERT_EQ(6, jacobian.rows());
  ASSERT_EQ(use_joints.size(), jacobian.cols());
  // Only y should be -1.0
  EXPECT_NEAR(0.0, jacobian(0, 0), kDoubleEps);
  EXPECT_NEAR(-1.0, jacobian(1, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(2, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(3, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(4, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(5, 0), kDoubleEps);
}

// Jacobian changes by moving the acquired pedestal of Jacobian
TYPED_TEST_P(RobotKinematicsModelTest, GetJacobianRotateRotate) {
  std::string frame_name("link7");
  Eigen::Affine3d frame_to_end(Eigen::Affine3d::Identity());
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("joint1");
  Eigen::MatrixXd jacobian;
  Eigen::Affine3d origin_to_robot = Eigen::Affine3d::Identity() *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
  this->robot_model_->SetRobotTransform(origin_to_robot);
  jacobian = this->robot_model_->GetJacobian(frame_name, frame_to_end, use_joints);
  ASSERT_EQ(6, jacobian.rows());
  ASSERT_EQ(use_joints.size(), jacobian.cols());
  // -rotate_y should be -1.0
  EXPECT_NEAR(0.0, jacobian(3, 0), kDoubleEps);
  EXPECT_NEAR(-1.0, jacobian(4, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(5, 0), kDoubleEps);
}

// A frame that does not exist in the acquisition of Jacobian has been acquired
TYPED_TEST_P(RobotKinematicsModelTest, GetJacobianInvalidFrame) {
  std::string frame_name("hoge_piyo");
  Eigen::Affine3d frame_to_end(Eigen::Affine3d::Identity());
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("joint3");
  Eigen::MatrixXd jacobian;

  EXPECT_THROW({
      jacobian = this->robot_model_->GetJacobian(frame_name, frame_to_end, use_joints);
    }, std::domain_error);
}

// A joint that does not exist in the acquisition of Jacobian was acquired
TYPED_TEST_P(RobotKinematicsModelTest, GetJacobianInvalidJoint) {
  std::string frame_name("link6");
  Eigen::Affine3d frame_to_end(Eigen::Affine3d::Identity());
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("hoge_piyo");
  Eigen::MatrixXd jacobian;

  EXPECT_THROW({
      jacobian = this->robot_model_->GetJacobian(frame_name, frame_to_end, use_joints);
    }, std::domain_error);
}

// Joint upper and lower limit tests
TYPED_TEST_P(RobotKinematicsModelTest, GetJointMinMax) {
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("joint1");
  use_joints.push_back("joint3");
  Eigen::VectorXd min;
  Eigen::VectorXd max;
  this->robot_model_->GetMinMax(use_joints, min, max);
  ASSERT_EQ(use_joints.size(), min.size());
  ASSERT_EQ(use_joints.size(), max.size());
  EXPECT_DOUBLE_EQ(-3.14, min(0));
  EXPECT_DOUBLE_EQ(3.14, max(0));
  EXPECT_DOUBLE_EQ(0.0, min(1));
  EXPECT_DOUBLE_EQ(0.5, max(1));
}

// Specify a name that does not exist when acquiring the upper and lower limit of the joints
TYPED_TEST_P(RobotKinematicsModelTest, GetInvalidJointMinMax) {
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("hoge_piyo");
  Eigen::VectorXd min;
  Eigen::VectorXd max;
  EXPECT_THROW({
      this->robot_model_->GetMinMax(use_joints, min, max);
    }, std::domain_error);
}

REGISTER_TYPED_TEST_SUITE_P(RobotKinematicsModelTest,
                            InvalidUrdf,
                            SeparateGenerationAndInitialization,
                            GetInitialRobotTransform,
                            SetAndGetRobotTransform,
                            GetAllJointAngles,
                            GetPartialJointAngles,
                            ErrorInvalidPartialJointAngles,
                            SetPartialJointAngles,
                            SetContinuousJointAngles,
                            MimicJoint,
                            InvalidNameSetPartialJointAngles,
                            UnevenJointstateSetPartialJointAngles,
                            GetObjectTransform,
                            GetObjectTransformRoot,
                            GetObjectTransformFixed,
                            InvalidObjectTransform,
                            GetObjectRelativeTransform,
                            GetObjectRelativeTransformRoot,
                            InvalidObjectRelativeTransform,
                            CreateNewFrame,
                            CreateToInvalidFrame,
                            DeleteDynamicFrame,
                            DeleteInvalidFrame,
                            GetJacobianNormal,
                            GetJacobianRotateLinear,
                            GetJacobianRotateRotate,
                            GetJacobianInvalidFrame,
                            GetJacobianInvalidJoint,
                            GetJointMinMax,
                            GetInvalidJointMinMax);

}  // namespace tmc_robot_kinematics_model
