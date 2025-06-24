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
/// @brief    Numericiksolver test in URDF

#include <string>

#include <gtest/gtest.h>
#include <sys/time.h>

#include <pluginlib/class_loader.hpp>

#include <tmc_manipulation_tests/configs.hpp>

// If you include the Pinocchio header after the Boost system, the build will not pass, so you will do it first.
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

using tmc_robot_kinematics_model::IRobotKinematicsModel;
using tmc_robot_kinematics_model::PinocchioWrapper;
using tmc_robot_kinematics_model::IKSolver;
using tmc_robot_kinematics_model::IKRequest;
using tmc_robot_kinematics_model::IKResponse;
using tmc_robot_kinematics_model::IKResult;
using tmc_robot_kinematics_model::kSuccess;
using tmc_robot_kinematics_model::kFail;
using tmc_robot_kinematics_model::kMaxItr;
using tmc_robot_kinematics_model::kConverge;
using tmc_robot_kinematics_model::kInterruption;
using tmc_robot_kinematics_model::NumericIKSolver;
using tmc_manipulation_types::JointState;
using tmc_manipulation_types::NameSeq;

namespace {
constexpr double kEpsilon = 1.0e-3;
constexpr double kConvergeThreshold = 1.0e-10;
constexpr int32_t kItr = 1000;

// Margin for the limit
constexpr double kDeltaJoint = 0.1;
// Microphilic intake given to the hand
constexpr double kHandDelta = 0.005;

double GetPosition(JointState joint_state, std::string name) {
  if (joint_state.name.size() !=
      static_cast<uint32_t>(joint_state.position.size())) {
    throw std::invalid_argument("name size and position size must be even");
  } else {
    for (uint32_t i = 0; i < joint_state.name.size(); ++i) {
      if (joint_state.name[i] == name) {
        return joint_state.position(i);
      }
    }
  }
  throw std::invalid_argument("cannot found joint");
}

/// Calculate the displacement of two posture
double CalcPoseDiff(const Eigen::Affine3d pose_src,
                    const Eigen::Affine3d pose_dst) {
  Eigen::Matrix<double, 6, 1> diff;
  Eigen::Vector3d diff_pos = pose_src.translation() - pose_dst.translation();
  Eigen::Quaterniond rotation_diff(
      pose_dst.linear().transpose() * pose_src.linear());
  rotation_diff = rotation_diff.normalized();
  Eigen::AngleAxisd diff_angle = Eigen::AngleAxisd(rotation_diff);
  diff << diff_pos, diff_angle.angle() * diff_angle.axis();
  return diff.norm();
}
}  // anonymous namespace

/// Test fiscal
class NumericIKTest : public ::testing::Test {
 protected:
  NumericIKTest() {
    Eigen::Affine3d unit(Eigen::Affine3d::Identity());

    robot_ = std::make_shared<PinocchioWrapper>(tmc_manipulation_tests::stanford_manipulator::GetUrdf());
    robot_->SetRobotTransform(unit);
    JointState angle;
    JointState numeric_angle;

    angle.position.resize(6);
    angle.position(0) = 0.1;
    angle.position(1) = 0.2;
    angle.position(2) = 0.3;
    angle.position(3) = 0.4;
    angle.position(4) = 0.5;
    angle.position(5) = 0.6;
    NameSeq use_name(6);
    use_name[0] = ("joint1");
    use_name[1] = ("joint2");
    use_name[2] = ("joint3");
    use_name[3] = ("joint4");
    use_name[4] = ("joint5");
    use_name[5] = ("joint6");

    angle.name = use_name;
    robot_->SetNamedAngle(angle);
    initial_angle_ = robot_->GetNamedAngle();
    use_name_ = use_name;
  }
  IRobotKinematicsModel::Ptr robot_;
  JointState initial_angle_;
  NameSeq use_name_;
};

// Easy case
TEST_F(NumericIKTest, SimpleCase) {
  // Only numerical solution
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, kItr, kEpsilon, kConvergeThreshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d hand = Eigen::Translation3d(-0.2, -0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req;
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d hand_result;
  ASSERT_EQ(kSuccess, solver->Solve(req, solution, hand_result));
  EXPECT_LT(CalcPoseDiff(hand_result, hand), kEpsilon);

  std::vector<IKResponse> responses;
  EXPECT_EQ(kSuccess, solver->Solve(req, responses));
  EXPECT_EQ(1, responses.size());
  EXPECT_LT(CalcPoseDiff(responses[0].origin_to_end, hand), kEpsilon);
}

// Case that contains infinite rotation joints
TEST_F(NumericIKTest, ContinuousJoint) {
  JointState state;
  state.name = {"dummy_continuous_joint"};
  state.position.resize(1);
  state.position << 2.5;
  robot_->SetNamedAngle(state);
  const auto goal_pose = robot_->GetObjectTransform("dummy_continuous_link");

  IKRequest req;
  req.frame_name = "dummy_continuous_link";
  req.frame_to_end = Eigen::Affine3d::Identity();
  req.origin_to_base = Eigen::Affine3d::Identity();
  req.ref_origin_to_end = goal_pose;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;
  req.use_joints.push_back("dummy_continuous_joint");
  req.continuous_joints.push_back("dummy_continuous_joint");

  IKSolver::Ptr solver(new NumericIKSolver(IKSolver::Ptr(), robot_, kItr, kEpsilon, kConvergeThreshold));

  JointState solution;
  Eigen::Affine3d result_pose;
  ASSERT_EQ(kSuccess, solver->Solve(req, solution, result_pose));
  EXPECT_LT(CalcPoseDiff(result_pose, goal_pose), kEpsilon);
  // Due to the shaft configuration, the rotation of 2.5 should be separated between joint6 and dummy_continuous_joint.
  EXPECT_NEAR(solution.position[5], 0.6 + 2.5 / 2.0, 0.05);
  EXPECT_NEAR(solution.position[6], 2.5 / 2.0, 0.05);
}

// Case that contains MIMIC joints
TEST_F(NumericIKTest, MimicJoint) {
  const auto goal_pose = robot_->GetObjectTransform("dummy_mimic_link");

  IKRequest req;
  req.frame_name = "dummy_mimic_link";
  req.frame_to_end = Eigen::Affine3d::Identity();
  req.origin_to_base = Eigen::Affine3d::Identity();
  req.ref_origin_to_end = goal_pose;
  req.initial_angle = initial_angle_;
  req.initial_angle.position /= 10.0;
  req.use_joints = use_name_;

  IKSolver::Ptr solver(new NumericIKSolver(IKSolver::Ptr(), robot_, kItr, kEpsilon, kConvergeThreshold));

  JointState solution;
  Eigen::Affine3d result_pose;
  ASSERT_EQ(kSuccess, solver->Solve(req, solution, result_pose));
  EXPECT_LT(CalcPoseDiff(result_pose, goal_pose), kEpsilon);
}

// Exceptions will be made by specifying the non -existent joint
TEST_F(NumericIKTest, NoExistJoint) {
  // Only numerical solution
  IKSolver::Ptr solver(new NumericIKSolver(
      IKSolver::Ptr(), robot_, kItr, kEpsilon, kConvergeThreshold));
  Eigen::Affine3d unit(Eigen::Affine3d::Identity());
  Eigen::Affine3d hand = Eigen::Translation3d(0.3, 0.0, 0.05) *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
  IKRequest req;
  NameSeq use_name(6);
  use_name[0] = ("no_exist_joint");
  use_name[1] = ("joint2");
  use_name[2] = ("joint3");
  use_name[3] = ("joint4");
  use_name[4] = ("joint5");
  use_name[5] = ("joint6");

  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name;

  JointState solution;
  Eigen::Affine3d hand_result;
  EXPECT_ANY_THROW(solver->Solve(req, solution, hand_result););
}

// Exceptions will appear by specifying the non -existent frame
TEST_F(NumericIKTest, NoExistFrame) {
  // Only numerical solution
  IKSolver::Ptr solver(new NumericIKSolver(
      IKSolver::Ptr(), robot_, kItr, kEpsilon, kConvergeThreshold));
  Eigen::Affine3d unit(Eigen::Affine3d::Identity());
  Eigen::Affine3d hand = Eigen::Translation3d(0.3, 0.0, 0.05) *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
  IKRequest req;
  NameSeq use_name(6);
  use_name[0] = ("joint1");
  use_name[1] = ("joint2");
  use_name[2] = ("joint3");
  use_name[3] = ("joint4");
  use_name[4] = ("joint5");
  use_name[5] = ("joint6");

  req.frame_name = "no_exist_frame";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name;

  JointState solution;
  Eigen::Affine3d hand_result;
  EXPECT_ANY_THROW(solver->Solve(req, solution, hand_result););
}

// 6 Exceptions will appear in settings below the degree of freedom
TEST_F(NumericIKTest, UnderSixAxis) {
  // Only numerical solution
  IKSolver::Ptr solver(new NumericIKSolver(
      IKSolver::Ptr(), robot_, kItr, kEpsilon, kConvergeThreshold));
  Eigen::Affine3d unit(Eigen::Affine3d::Identity());
  Eigen::Affine3d hand = Eigen::Translation3d(0.3, 0.0, 0.05) *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
  IKRequest req;
  NameSeq use_name(5);
  use_name[0] = ("joint1");
  use_name[1] = ("joint2");
  use_name[2] = ("joint3");
  use_name[3] = ("joint4");
  use_name[4] = ("joint5");

  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name;

  JointState solution;
  Eigen::Affine3d hand_result;
  EXPECT_ANY_THROW(solver->Solve(req, solution, hand_result););
}

// Ends with Converge or MaxitR outside reach
TEST_F(NumericIKTest, OutOfRange) {
  // Only numerical solution
  IKSolver::Ptr solver(new NumericIKSolver(
      IKSolver::Ptr(), robot_, kItr, kEpsilon, kConvergeThreshold));
  Eigen::Affine3d unit(Eigen::Affine3d::Identity());
  Eigen::Affine3d hand = Eigen::Translation3d(2.0, 0.0, 0.05) *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());

  IKRequest req;
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;
  JointState solution;
  Eigen::Affine3d hand_result;
  IKResult result = solver->Solve(req, solution, hand_result);
  EXPECT_TRUE((result == kMaxItr) || (result == kConverge));

  std::vector<IKResponse> responses;
  EXPECT_EQ(kFail, solver->Solve(req, responses));
  EXPECT_TRUE(responses.empty());
}

// Interrupt
TEST_F(NumericIKTest, Interruption) {
  // Solve by input outside reach
  IKSolver::Ptr solver(new NumericIKSolver(IKSolver::Ptr(), robot_, kItr, kEpsilon, 0.0));

  IKRequest req;
  req.frame_name = "link7";
  req.frame_to_end = Eigen::Affine3d::Identity();
  req.origin_to_base = Eigen::Affine3d::Identity();
  req.ref_origin_to_end = Eigen::Translation3d(2.0, 0.0, 0.05) * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d hand_result;

  auto start = std::chrono::system_clock::now();
  IKResult result = solver->Solve(req, solution, hand_result);
  auto end = std::chrono::system_clock::now();
  const auto no_interruption_duration = end - start;

  // It doesn't reach + Converge_threshold = 0.0, so it ends with maxitr
  EXPECT_EQ(result, kMaxItr);

  // If there is an interrupt, it should come out in an instant
  std::function<bool()> func = []() -> bool{ return true; };

  start = std::chrono::system_clock::now();
  result = solver->Solve(req, func, solution, hand_result);
  end = std::chrono::system_clock::now();

  EXPECT_EQ(result, kInterruption);
  // KITR doubles should be different, but compare them in 1/10 to stabilize the test.
  EXPECT_LT(end - start, no_interruption_duration / (kItr / 10));
}

// A solution using a plane movement will come out
TEST_F(NumericIKTest, PlanarBaseCase) {
  // Only numerical solution
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, kItr, kEpsilon, kConvergeThreshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(1.2, -0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kPlanar);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // Hand position is correct
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), kEpsilon);
  // The restraint is correct
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.z(), kEpsilon);
  EXPECT_NEAR(0.0, rotation.axis().x(), kEpsilon);
  EXPECT_NEAR(0.0, rotation.axis().y(), kEpsilon);

  std::vector<IKResponse> responses;
  EXPECT_EQ(kSuccess, solver->Solve(req, responses));
  EXPECT_EQ(1, responses.size());
  EXPECT_LT(CalcPoseDiff(responses[0].origin_to_end, origin_to_hand), kEpsilon);
}

// An solution using the floating movement will come out
TEST_F(NumericIKTest, FloatBaseCase) {
  // Only numerical solution
  IKSolver::Ptr solver(
      new NumericIKSolver(
          IKSolver::Ptr(),
          robot_, kItr, kEpsilon, kConvergeThreshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(1.2, -0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kFloat);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // Hand position is correct
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), kEpsilon);
}

// A solution using horizontal X will come out
TEST_F(NumericIKTest, RailXBaseCase) {
  // Only numerical solution
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, kItr, kEpsilon, kConvergeThreshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, -0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRailX);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // Hand position is correct
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), kEpsilon);
  // The restraint is correct
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.y(), kEpsilon);
  EXPECT_NEAR(0.0, trans.z(), kEpsilon);
  EXPECT_NEAR(0.0, rotation.angle(), kEpsilon);
}

// An solution using horizontal Y will come out
TEST_F(NumericIKTest, RailYBaseCase) {
  // Only numerical solution
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, kItr, kEpsilon, kConvergeThreshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, -1.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRailY);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // Hand position is correct
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), kEpsilon);
  // The restraint is correct
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.x(), kEpsilon);
  EXPECT_NEAR(0.0, trans.z(), kEpsilon);
  EXPECT_NEAR(0.0, rotation.angle(), kEpsilon);
}

// A solution using horizontal Z will come out
TEST_F(NumericIKTest, RailZBaseCase) {
  // Only numerical solution
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, kItr, kEpsilon, kConvergeThreshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, -0.2, 4.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRailZ);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // Hand position is correct
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), kEpsilon);
  // The restraint is correct
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.x(), kEpsilon);
  EXPECT_NEAR(0.0, trans.y(), kEpsilon);
  EXPECT_NEAR(0.0, rotation.angle(), kEpsilon);
}

// A solution using rotation X will come out
TEST_F(NumericIKTest, RotationXBaseCase) {
  // Only numerical solution
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, kItr, kEpsilon, kConvergeThreshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, 0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRotationX);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // Hand position is correct
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), kEpsilon);
  // The restraint is correct
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.x(), kEpsilon);
  EXPECT_NEAR(0.0, trans.y(), kEpsilon);
  EXPECT_NEAR(0.0, trans.z(), kEpsilon);
  EXPECT_NEAR(0.0, rotation.axis().y(), kEpsilon);
  EXPECT_NEAR(0.0, rotation.axis().z(), kEpsilon);
}

// Solution using rotation Y comes out
TEST_F(NumericIKTest, RotationYBaseCase) {
  // Only numerical solution
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, kItr, kEpsilon, kConvergeThreshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, 0.0, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRotationY);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // Hand position is correct
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), kEpsilon);
  // The restraint is correct
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.x(), kEpsilon);
  EXPECT_NEAR(0.0, trans.y(), kEpsilon);
  EXPECT_NEAR(0.0, trans.z(), kEpsilon);
  EXPECT_NEAR(0.0, rotation.axis().x(), kEpsilon);
  EXPECT_NEAR(0.0, rotation.axis().z(), kEpsilon);
}

// A solution using rotation Z will come out
TEST_F(NumericIKTest, RotationZBaseCase) {
  // Only numerical solution
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, kItr, kEpsilon, kConvergeThreshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, -0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRotationZ);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // Hand position is correct
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), kEpsilon);
  // The restraint is correct
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.x(), kEpsilon);
  EXPECT_NEAR(0.0, trans.y(), kEpsilon);
  EXPECT_NEAR(0.0, trans.z(), kEpsilon);
  EXPECT_NEAR(0.0, rotation.axis().y(), kEpsilon);
  EXPECT_NEAR(0.0, rotation.axis().x(), kEpsilon);
}

// Read as a plugin
TEST_F(NumericIKTest, Plugin) {
  pluginlib::ClassLoader<tmc_robot_kinematics_model::IKSolver> loader(
      "tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IKSolver");
  auto solver = loader.createSharedInstance("tmc_robot_kinematics_model/NumericIKSolver");
  solver->set_robot_description(tmc_manipulation_tests::stanford_manipulator::GetUrdf());

  // The numbers are the same as SimpleCase
  IKRequest req;
  req.frame_name = "link7";
  req.frame_to_end = Eigen::Affine3d::Identity();
  req.origin_to_base = Eigen::Affine3d::Identity();
  req.ref_origin_to_end = Eigen::Translation3d(-0.2, -0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d hand_result;
  ASSERT_EQ(kSuccess, solver->Solve(req, solution, hand_result));
  EXPECT_LT(CalcPoseDiff(hand_result, req.ref_origin_to_end), kEpsilon);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
