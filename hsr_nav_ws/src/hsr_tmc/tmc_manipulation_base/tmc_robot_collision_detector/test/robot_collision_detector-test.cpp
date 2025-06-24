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
/// @brief    Class test that performs interference checks using a robot model
#include <fstream>
#include <string>
#include <vector>

#include "tmc_robot_collision_detector/robot_collision_detector.hpp"

#include <gtest/gtest.h>

#include <tmc_manipulation_tests/configs.hpp>

using tmc_manipulation_types::ObjectParameterSeq;
using tmc_manipulation_types::OuterObjectParameters;
using tmc_manipulation_types::OuterObjectParametersSeq;
using tmc_manipulation_types::Cuboid;
using tmc_manipulation_types::CuboidSeq;
using tmc_manipulation_types::AABB;
using tmc_manipulation_types::JointState;

using ::testing::TestWithParam;
using ::testing::Values;

namespace {
const char* const kODE = "ODE";

const char* const kNonExistFile = "hogehoge.xml";
const char* const kIncorrectRobotModel = "gtest/incorrect_robot_model.xml";

const char* const kWall = "Wall";
const char* const kBoxes = "ThreeBoxes";
const char* const kCuboid = "Cuboid";
const char* const kWristGroup = "ARM7";
const char* const kJoint = "CARM/HEAD/NECK_Y";

const char* const kWrist1 = "CARM/SHAPE_WRIST_P_1";
const char* const kWrist2 = "CARM/SHAPE_WRIST_Y";
const char* const kLinear1 = "CARM/SHAPE_LINEAR_1";
const char* const kBase1 = "BASE/SHAPE_BASE_1";
const char* const kBase2 = "BASE/SHAPE_BASE_2";
const char* const kBase4 = "BASE/SHAPE_BASE_4";


const char* const kOutputFile = "gtest.xml";

const double kExtendLength = 0.5;
const int32_t kTopN = 5;

uint32_t GetJointIndex(const JointState& named_angle,
                       const std::string& joint_name) {
  return std::distance(named_angle.name.begin(),
                       std::find(
                           named_angle.name.begin(), named_angle.name.end(),
                           joint_name));
}

}  // anonymous namespace

namespace tmc_robot_collision_detector {

class RobotCollisionDetectorTest
    : public ::testing::TestWithParam<ModelFileType> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  RobotCollisionDetectorTest() {}
  virtual ~RobotCollisionDetectorTest() {}
 protected:
  void SetUp() override;
  RobotCollisionDetector::Ptr detector_;
  OuterObjectParameters wall_;
  OuterObjectParameters boxes_;
  CuboidSeq bounding_boxes_;
  std::string robot_description_;
  std::string collision_pair_list_;
  std::string wrist_shape_;
  std::string wrist_y_shape_;
  std::string linear_shape_;
  std::string base_shape_1_;
  std::string base_shape_2_;
};

void RobotCollisionDetectorTest::SetUp() {
  // Obtained from the parameter server
  ModelFileType type = GetParam();
  robot_description_ = tmc_manipulation_tests::hsra::GetUrdf();
  collision_pair_list_ = tmc_manipulation_tests::hsra::GetCollisionConfig();
  if (type == kUrdf) {
    wrist_shape_ = std::string(kWrist1) + "/collision/0";
    wrist_y_shape_ = std::string(kWrist2) + "/collision/0";
    linear_shape_ = std::string(kLinear1) + "/collision/0";
    base_shape_1_ = std::string(kBase1) + "/collision/0";
    base_shape_2_ = std::string(kBase2) + "/collision/0";
  }

  // Create information about the object to be created
  wall_.name.assign(kWall);
  wall_.origin_to_base.setIdentity();
  wall_.origin_to_base.translation() = Eigen::Vector3d(0.7, 0.0, 0.5);
  wall_.shape.resize(1);
  wall_.shape.at(0).type = tmc_manipulation_types::kBox;
  wall_.shape.at(0).dimensions.push_back(0.06);
  wall_.shape.at(0).dimensions.push_back(1.0);
  wall_.shape.at(0).dimensions.push_back(1.0);
  wall_.base_to_child.push_back(Eigen::Affine3d::Identity());

  boxes_.name.assign(kBoxes);
  boxes_.origin_to_base.setIdentity();
  boxes_.origin_to_base.translation() = Eigen::Vector3d(0.2, 0.5, 0.3);
  tmc_manipulation_types::Shape shape;
  shape.type = tmc_manipulation_types::kBox;
  shape.dimensions.assign(3, 0.05);
  boxes_.shape.assign(3, shape);
  boxes_.base_to_child.assign(3, Eigen::Affine3d::Identity());
  boxes_.base_to_child.at(1).translation() = Eigen::Vector3d(0.05, 0.0, 0.0);
  boxes_.base_to_child.at(2).translation() = Eigen::Vector3d(0.0, 0.0, 0.05);

  Cuboid bounding_box;
  bounding_box.box_extents = Eigen::Vector3d(0.05, 1.0, 1.0);
  bounding_box.box_transform.setIdentity();
  bounding_box.box_transform.translation() = Eigen::Vector3d(0.2, 0.0, 0.5);
  bounding_box.box_name.assign(kCuboid);
  bounding_box.box_name += '0';
  for (int32_t i = 0; i < 3; i++) {
    bounding_box.box_aabb(i, 1) =
        bounding_box.box_transform.translation().coeff(i) +
        bounding_box.box_extents.coeff(i) / 2.0;
    bounding_box.box_aabb(i, 0) =
        bounding_box.box_transform.translation().coeff(i) -
        bounding_box.box_extents.coeff(i) / 2.0;
  }
  bounding_boxes_.push_back(bounding_box);
  bounding_box.box_name.assign(kCuboid);
  bounding_box.box_name += '1';
  bounding_box.box_transform.translation() = Eigen::Vector3d(0.65, 0.0, 0.5);
  for (int32_t i = 0; i < 3; i++) {
    bounding_box.box_aabb(i, 1) =
        bounding_box.box_transform.translation().coeff(i) +
        bounding_box.box_extents.coeff(i) / 2.0;
    bounding_box.box_aabb(i, 0) =
        bounding_box.box_transform.translation().coeff(i) -
        bounding_box.box_extents.coeff(i) / 2.0;
  }
  bounding_boxes_.push_back(bounding_box);
  bounding_box.box_name.assign(kCuboid);
  bounding_box.box_name += '2';
  bounding_box.box_transform.translation() = Eigen::Vector3d(0.60, 0.0, 0.5);
  for (int32_t i = 0; i < 3; i++) {
    bounding_box.box_aabb(i, 1) =
        bounding_box.box_transform.translation().coeff(i) +
        bounding_box.box_extents.coeff(i) / 2.0;
    bounding_box.box_aabb(i, 0) =
        bounding_box.box_transform.translation().coeff(i) -
        bounding_box.box_extents.coeff(i) / 2.0;
  }
  bounding_boxes_.push_back(bounding_box);

  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, kODE);
}

TEST_P(RobotCollisionDetectorTest, Constructor) {
  // Normal system: Read a regular file (ODE)
  EXPECT_NO_THROW(
      RobotCollisionDetector(robot_description_,
                             collision_pair_list_, kODE));

  // Abnormal system: There is no robot model file
  EXPECT_ANY_THROW(
      RobotCollisionDetector(kNonExistFile,
                             collision_pair_list_, kODE));

  // Abnormal system: The grammar of the robot model file is incorrect
  EXPECT_ANY_THROW(
      RobotCollisionDetector(kIncorrectRobotModel,
                             collision_pair_list_, kODE));

  // Abnormal system: Specify a physical engine that does not exist
  EXPECT_ANY_THROW(
      RobotCollisionDetector(robot_description_,
                             collision_pair_list_, kNonExistFile));
}

TEST_P(RobotCollisionDetectorTest, ConstructorWithKinematicsModel) {
  // Robot_model should change in the same way because the athletic model enters from the outside is used.
  auto robot_model = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(robot_description_);
  auto detector_with_model = std::make_shared<RobotCollisionDetector>(
      robot_model, robot_description_, collision_pair_list_, kODE);

  detector_with_model->SetRobotTransform(Eigen::Translation3d(1.0, 2.0, 0.0) * Eigen::AngleAxisd());
  const auto pose_from_detector = detector_with_model->GetObjectTransform(kJoint);
  const auto pose_from_kinematics = robot_model->GetObjectTransform(kJoint);

  // Since we ordered a dolphin parallel, it is enough to check only the position.
  EXPECT_DOUBLE_EQ(pose_from_detector.translation().x(), pose_from_kinematics.translation().x());
  EXPECT_DOUBLE_EQ(pose_from_detector.translation().y(), pose_from_kinematics.translation().y());
  EXPECT_DOUBLE_EQ(pose_from_detector.translation().z(), pose_from_kinematics.translation().z());
}

TEST_P(RobotCollisionDetectorTest, CreateOuterObject) {
  // Normal system: Normal system
  EXPECT_NO_THROW(detector_->CreateOuterObject(boxes_));
  EXPECT_NO_THROW(detector_->CreateOuterObject(wall_));
  EXPECT_EQ(2, detector_->GetAllOuterObjectParameters().size());

  // Abnormal system: The number of Shape and Pose of the child body does not match
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, kODE);
  OuterObjectParameters incorrect_parameters = boxes_;
  incorrect_parameters.base_to_child.resize(2);
  EXPECT_ANY_THROW(detector_->CreateOuterObject(incorrect_parameters));

  // Abnormal system: No name
  incorrect_parameters = boxes_;
  incorrect_parameters.name.clear();
  EXPECT_ANY_THROW(detector_->CreateOuterObject(incorrect_parameters));

  // Abnormal system: I'm trying to use a certain name
  EXPECT_NO_THROW(detector_->CreateOuterObject(boxes_));
  EXPECT_ANY_THROW(detector_->CreateOuterObject(boxes_));
}

TEST_P(RobotCollisionDetectorTest, CreateCuboids) {
  // Normal system: Enable interference check
  EXPECT_NO_THROW(detector_->CreateCuboids(bounding_boxes_, true));
  EXPECT_EQ(3, detector_->GetAllOuterObjectParameters().size());

  // Normal system: Disable interference check
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, kODE);
  EXPECT_NO_THROW(detector_->CreateCuboids(bounding_boxes_, false));
  EXPECT_EQ(3, detector_->GetAllOuterObjectParameters().size());

  // Normal system: Create a Cuboid by specifying a group
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, kODE);
  EXPECT_NO_THROW(detector_->CreateCuboids(bounding_boxes_, false, "BODY"));
  EXPECT_EQ(3, detector_->GetAllOuterObjectParameters().size());

  // Abnormal system: Create the same Cuboid
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, kODE);
  EXPECT_NO_THROW(detector_->CreateCuboids(bounding_boxes_, false));
  EXPECT_ANY_THROW(detector_->CreateCuboids(bounding_boxes_, false));

  // Abnormal system: Create an empty Cuboid
  CuboidSeq no_name_box_;
  no_name_box_.resize(1, bounding_boxes_[0]);
  no_name_box_[0].box_name.clear();
  EXPECT_ANY_THROW(detector_->CreateCuboids(no_name_box_, false));

  // Abnormal system: Create Cuboid by specifying a group that does not exist
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, kODE);
  EXPECT_ANY_THROW(detector_->CreateCuboids(bounding_boxes_, false, "hoge"));
}

TEST_P(RobotCollisionDetectorTest, DestroyOuterObject) {
  // Normal system: Discard the existing external objects
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(detector_->DestroyOuterObject(kWall));
  EXPECT_EQ(0, detector_->GetAllOuterObjectParameters().size());

  // Normal system: Discard the robot site (do not destroy)
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(detector_->DestroyOuterObject(wrist_shape_));
  EXPECT_EQ(1, detector_->GetAllOuterObjectParameters().size());

  // Normal system: Discard (do not discard) the objects you are grabbing
  detector_->CreateOuterObject(boxes_);
  detector_->HoldObject(kBoxes, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(detector_->DestroyOuterObject(kBoxes));
  EXPECT_EQ(2, detector_->GetAllOuterObjectParameters().size());

  // Normal system: Discard all external objects
  EXPECT_NO_THROW(detector_->DestroyAllOuterObject());
  EXPECT_EQ(0, detector_->GetAllOuterObjectParameters().size());

  // Normal system: Discard all external objects without creating external objects
  EXPECT_NO_THROW(detector_->DestroyAllOuterObject());

  // Normal system: Discard CUBOID
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_NO_THROW(detector_->DestroyOuterObject(bounding_boxes_[0].box_name));
  EXPECT_EQ(2, detector_->GetAllOuterObjectParameters().size());

  // Abnormal system: Discard an unusual external object
  EXPECT_ANY_THROW(detector_->DestroyOuterObject("hoge"));

  // Abnormal system: Discard the object once destroyed
  detector_->CreateOuterObject(boxes_);
  EXPECT_NO_THROW(detector_->DestroyOuterObject(kBoxes));
  EXPECT_ANY_THROW(detector_->DestroyOuterObject(kBoxes));

  // Abnormal system: Discard child objects
  detector_->CreateOuterObject(boxes_);
  EXPECT_ANY_THROW(
      detector_->DestroyOuterObject(std::string(kBoxes) + std::string("#1")));
}

TEST_P(RobotCollisionDetectorTest, DestroyCuboids) {
  // Normal system: Discard CUBOID
  detector_->CreateOuterObject(wall_);
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_NO_THROW(detector_->DestroyCuboids());
  EXPECT_EQ(1, detector_->GetAllOuterObjectParameters().size());
}

TEST_P(RobotCollisionDetectorTest, GetObjectParameter) {
  // Normal system: Obtain parameters for created objects
  detector_->CreateOuterObject(wall_);
  OuterObjectParameters param = detector_->GetObjectParameter(kWall);
  EXPECT_EQ(std::string(kWall), param.name);

  // Normal system: Obtain parameters for robot parts
  param = detector_->GetObjectParameter(wrist_shape_);
  EXPECT_EQ(std::string(wrist_shape_), param.name);

  // Normal system: Obtain parameters for child objects
  detector_->CreateOuterObject(boxes_);
  param = detector_->GetObjectParameter(std::string(kBoxes) + "#1");
  EXPECT_EQ(std::string(kBoxes) + "#1", param.name);

  // Normal system: Obtain Cuboid parameters
  detector_->CreateCuboids(bounding_boxes_, false);
  param = detector_->GetObjectParameter(std::string(kCuboid) + "1");
  EXPECT_EQ(std::string(kCuboid) + "1", param.name);

  // Abnormal system: Obtain parameters of non -existent objects
  EXPECT_ANY_THROW(detector_->GetObjectParameter("hoge"));
}

TEST_P(RobotCollisionDetectorTest, GetAllOuterObjectParameter) {
  // Normal system: If there is no external object
  EXPECT_EQ(0, detector_->GetAllOuterObjectParameters().size());

  // Normal system: If there are two external objects
  detector_->CreateOuterObject(wall_);
  detector_->CreateOuterObject(boxes_);
  EXPECT_EQ(2, detector_->GetAllOuterObjectParameters().size());

  // Normal system: If you have Cuboid
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_EQ(5, detector_->GetAllOuterObjectParameters().size());
}

TEST_P(RobotCollisionDetectorTest, EnableDisableObject) {
  // Normal system: Check the operation of ENABLEOBJECT for external objects
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(detector_->EnableCollisionObject(kWall));

  // Normal system: Check out operation of DisableObject for external objects
  EXPECT_NO_THROW(detector_->DisableCollisionObject(kWall));

  // Normal system: Check the operation of ENABLEOBJECT for the robot site
  EXPECT_NO_THROW(detector_->EnableCollisionObject(wrist_shape_));

  // Normal system: Check the operation of DisableObject to the robot site
  EXPECT_NO_THROW(detector_->DisableCollisionObject(wrist_shape_));

  // Normal system: Check the operation of EnableObject to the child object
  detector_->CreateOuterObject(boxes_);
  std::string child_object_name(std::string(kBoxes) + std::string("#1"));
  EXPECT_NO_THROW(detector_->EnableCollisionObject(child_object_name));

  // Normal system: Check out operation of DisableObject for child objects
  EXPECT_NO_THROW(detector_->DisableCollisionObject(child_object_name));

  // Normal system: Check the operation of ENABLEOBJECT for CUBOID
  detector_->CreateCuboids(bounding_boxes_, false);
  std::string cuboid0_name(std::string(kCuboid) + "0");
  std::string cuboid1_name(std::string(kCuboid) + "1");
  EXPECT_NO_THROW(detector_->EnableCollisionObject(cuboid0_name));
  EXPECT_NE(detector_->GetObjectGroup(cuboid1_name),
            detector_->GetObjectGroup(cuboid0_name));

  // Normal system: Check out operation of DisableObject for Cuboid
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, kODE);
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_NO_THROW(detector_->DisableCollisionObject(cuboid0_name));
  EXPECT_NE(detector_->GetObjectGroup(cuboid1_name),
            detector_->GetObjectGroup(cuboid0_name));

  // Unusual system: Check the operation of enableObject to the non -existent object
  EXPECT_ANY_THROW(detector_->EnableCollisionObject("hoge"));

  // Anomaly system: Check out the operation of DisableObject to the non -existent object
  EXPECT_ANY_THROW(detector_->DisableCollisionObject("hoge"));
}

TEST_P(RobotCollisionDetectorTest, OperateGroupProperty) {
  // Normal system: Check out the operation of GetObjectGroup for external objects
  detector_->CreateOuterObject(wall_);
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kWall),
            detector_->GetObjectGroup(kWall));

  // Normal system: Check out the operation of setObjectGroup for external objects
  EXPECT_NO_THROW(detector_->SetObjectGroup(kWall, 4));
  EXPECT_EQ(4, detector_->GetObjectGroup(kWall));

  // Normal system: Check out the operation of setObjectDefaultgroup for external objects
  EXPECT_NO_THROW(detector_->SetObjectDefaultGroup(kWall));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kWall),
            detector_->GetObjectGroup(kWall));

  // Normal system: Check out the operation of GetObjectGroup for robots
  EXPECT_EQ(detector_->GetObjectDefaultGroup(wrist_shape_),
            detector_->GetObjectGroup(wrist_shape_));

  // Normal system: Check out the operation of setObjectGroup for robot sites
  EXPECT_NO_THROW(detector_->SetObjectGroup(wrist_shape_, 4));
  EXPECT_EQ(4, detector_->GetObjectGroup(wrist_shape_));

  // Normal system: Check out the operation of setObjectDefaultgroup for robot sites
  EXPECT_NO_THROW(detector_->SetObjectDefaultGroup(wrist_shape_));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(wrist_shape_),
            detector_->GetObjectGroup(wrist_shape_));

  // Normal system: Check out the operation of GetObjectGroup for the child object
  detector_->CreateOuterObject(boxes_);
  std::string child_object_name(std::string(kBoxes) + std::string("#1"));
  EXPECT_EQ(detector_->GetObjectGroup(kBoxes),
            detector_->GetObjectGroup(child_object_name));

  // Normal system: GetObjectDefaultGroup operation check for child objects
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kBoxes),
            detector_->GetObjectDefaultGroup(child_object_name));

  // Normal system: Check out the operation of setObjectGroup for child objects
  EXPECT_NO_THROW(detector_->SetObjectGroup(child_object_name, 4));
  EXPECT_EQ(4, detector_->GetObjectGroup(child_object_name));

  // Normal system: Check out the operation of setObjectDefaultgroup for child objects
  EXPECT_NO_THROW(detector_->SetObjectDefaultGroup(child_object_name));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kBoxes),
            detector_->GetObjectGroup(child_object_name));

  // Normal system: Check out the operation of GetObjectGroup for Cuboid
  detector_->DestroyAllOuterObject();
  detector_->CreateCuboids(bounding_boxes_, false);
  std::string cuboid_name(std::string(kCuboid) + "0");
  EXPECT_EQ(detector_->GetObjectDefaultGroup(cuboid_name),
            detector_->GetObjectGroup(cuboid_name));

  // Normal system: Check out the operation of SetObjectGroup for Cuboid
  EXPECT_NO_THROW(detector_->SetObjectGroup(cuboid_name, 4));
  EXPECT_EQ(4, detector_->GetObjectGroup(cuboid_name));

  // Normal system: Check out the operation of setObjectDefaultgroup for Cuboid
  EXPECT_NO_THROW(detector_->SetObjectDefaultGroup(cuboid_name));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(cuboid_name),
            detector_->GetObjectGroup(cuboid_name));

  // Anomaly system: Check out the operation of GetObjectGroup for the non -existent object
  EXPECT_ANY_THROW(detector_->GetObjectGroup("hoge"));

  // Abnormal system: Check out the operation of setObjectGroup for non -existent objects
  EXPECT_ANY_THROW(detector_->SetObjectGroup("hoge", 4));

  // Anomaly system: Check out the operation of setObjectdefaultgroup for non -existent objects
  EXPECT_ANY_THROW(detector_->SetObjectDefaultGroup("hoge"));
}

TEST_P(RobotCollisionDetectorTest, OperateFilterProperty) {
  // Normal system: GetObjectFilter operation check for external objects
  detector_->CreateOuterObject(wall_);
  EXPECT_EQ(detector_->GetObjectDefaultFilter(kWall),
            detector_->GetObjectFilter(kWall));

  // Normal system: Check out the operation of setObjectFilter for external objects
  EXPECT_NO_THROW(detector_->SetObjectFilter(kWall, 4));
  EXPECT_EQ(4, detector_->GetObjectFilter(kWall));

  // Normal system: Check out the operation of setObjectDefaultFilter for external objects
  EXPECT_NO_THROW(detector_->SetObjectDefaultFilter(kWall));
  EXPECT_EQ(detector_->GetObjectDefaultFilter(kWall),
            detector_->GetObjectFilter(kWall));

  // Normal system: GetObjectFilter operation check for robots
  EXPECT_EQ(detector_->GetObjectDefaultFilter(wrist_shape_),
            detector_->GetObjectFilter(wrist_shape_));

  // Normal system: Check out the operation of setObjectFilter for robots
  EXPECT_NO_THROW(detector_->SetObjectFilter(wrist_shape_, 4));
  EXPECT_EQ(4, detector_->GetObjectFilter(wrist_shape_));

  // Normal system: Check out the operation of setObjectDefaultFilter for robots
  EXPECT_NO_THROW(detector_->SetObjectDefaultFilter(wrist_shape_));
  EXPECT_EQ(detector_->GetObjectDefaultFilter(wrist_shape_),
            detector_->GetObjectFilter(wrist_shape_));

  // Normal system: Check out the operation of GetObjectFilter for the child object
  detector_->CreateOuterObject(boxes_);
  std::string child_object_name(std::string(kBoxes) + std::string("#1"));
  EXPECT_EQ(detector_->GetObjectFilter(kBoxes),
            detector_->GetObjectFilter(child_object_name));

  // Normal system: GetObjectDefaultFilter operation check for child objects
  EXPECT_EQ(detector_->GetObjectDefaultFilter(kBoxes),
            detector_->GetObjectDefaultFilter(child_object_name));

  // Normal system: Check out the operation of setObjectFilter for child objects
  EXPECT_NO_THROW(detector_->SetObjectFilter(child_object_name, 4));
  EXPECT_EQ(4, detector_->GetObjectFilter(child_object_name));

  // Normal system: Check out the operation of setObjectDefaultFilter for child objects
  EXPECT_NO_THROW(detector_->SetObjectDefaultFilter(child_object_name));
  EXPECT_EQ(detector_->GetObjectDefaultFilter(kBoxes),
            detector_->GetObjectFilter(child_object_name));

  // Normal system: Check out the operation of GetObjectFilter for Cuboid
  detector_->DestroyAllOuterObject();
  detector_->CreateCuboids(bounding_boxes_, false);
  std::string cuboid_name(std::string(kCuboid) + "0");
  EXPECT_EQ(detector_->GetObjectDefaultFilter(cuboid_name),
            detector_->GetObjectFilter(cuboid_name));

  // Normal system: Check out the operation of setObjectFilter for Cuboid
  EXPECT_NO_THROW(detector_->SetObjectFilter(cuboid_name, 4));
  EXPECT_EQ(4, detector_->GetObjectFilter(cuboid_name));

  // Normal system: Check out the operation of setObjectDefaultFilter for Cuboid
  EXPECT_NO_THROW(detector_->SetObjectDefaultFilter(cuboid_name));
  EXPECT_EQ(detector_->GetObjectDefaultFilter(cuboid_name),
            detector_->GetObjectFilter(cuboid_name));

  // Abnormal system: GetObjectFilter operation check for non -existent objects
  EXPECT_ANY_THROW(detector_->GetObjectFilter("hoge"));

  // Abnormal system: Check out the operation of setObjectFilter for non -existent objects
  EXPECT_ANY_THROW(detector_->SetObjectFilter("hoge", 4));

  // Abnormal system: Check out the operation of setObjectdefaultFilter for non -existent objects
  EXPECT_ANY_THROW(detector_->SetObjectDefaultFilter("hoge"));
}

TEST_P(RobotCollisionDetectorTest, DisableCollisionPairProperty) {
  // Place an object so that the child body hits on the robot site
  OuterObjectParameters boxes;
  boxes = boxes_;
  boxes.origin_to_base = detector_->GetObjectTransform("BASE/FRAME_BASE");
  detector_->CreateOuterObject(boxes);
  detector_->CreateOuterObject(wall_);
  detector_->DisableCollisionObject(kWall);

  tmc_manipulation_types::JointState named_angle;
  named_angle = detector_->GetRobotNamedAngle();
  uint32_t shoulder_p_index =
      GetJointIndex(named_angle, "CARM/SHOULDER_P");
  uint32_t elbow_p_index =
      GetJointIndex(named_angle, "CARM/ELBOW_P");

  named_angle.position(shoulder_p_index) = 1.0;
  named_angle.position(elbow_p_index) = 0.0;
  detector_->SetRobotNamedAngle(named_angle);
  detector_->DisableCollisionObject(linear_shape_);

  // Removed interference check, interference check,
  // After that, enable interference check and check again
  // Normal system: Remove from interference check (robot site/parent object)
  detector_->DisableCollisionCheckObjectToObject(base_shape_2_, kBoxes);
  std::vector<PairString> contact_list;
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(2, contact_list.size());
  detector_->EnableCollisionCheckObjectToObject(base_shape_2_, kBoxes);
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(4, contact_list.size());

  // Normal system: Remove from interference check (robot site/child object)
  detector_->DisableCollisionCheckObjectToObject(
      base_shape_2_, std::string(kBoxes) + "#0");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(3, contact_list.size());
  detector_->EnableCollisionCheckObjectToObject(
      base_shape_2_, std::string(kBoxes) + "#0");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(4, contact_list.size());

  // Normal system: Remove from interference check (robot site/external object)
  detector_->DisableCollisionCheckObjectToGroup(base_shape_2_, "OUTER");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(2, contact_list.size());
  detector_->EnableCollisionCheckObjectToGroup(base_shape_2_, "OUTER");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(4, contact_list.size());

  // Normal system: Remove from interference check (part group/parent object)
  detector_->DisableCollisionCheckObjectToGroup(kBoxes, "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(0, contact_list.size());
  detector_->EnableCollisionCheckObjectToGroup(kBoxes, "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(4, contact_list.size());

  // Normal system: Remove from interference check (part group/child object)
  detector_->DisableCollisionCheckObjectToGroup(
      std::string(kBoxes) + "#0", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(2, contact_list.size());
  detector_->EnableCollisionCheckObjectToGroup(
      std::string(kBoxes) + "#0", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(4, contact_list.size());

  // Normal system: Remove from interference check (part group/external object)
  detector_->DisableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(0, contact_list.size());
  detector_->EnableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(4, contact_list.size());

  // Normal system: Is the group setting change reflected?
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, kODE);
  detector_->DisableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CreateOuterObject(boxes);
  detector_->SetRobotNamedAngle(named_angle);
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(0, contact_list.size());
  detector_->EnableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(4, contact_list.size());

  // Abnormal system: Specify the non -existent object name (Object Object)
  EXPECT_ANY_THROW(detector_->DisableCollisionCheckObjectToObject(
      base_shape_2_, "hoge"));

  // Abnormal system: Specify the non -existent object name (Object Group)
  EXPECT_ANY_THROW(detector_->DisableCollisionCheckObjectToGroup(
      "hoge", "BODY"));

  // Abnormal system: Specify the unusual group name (Object Group)
  EXPECT_ANY_THROW(detector_->DisableCollisionCheckObjectToGroup(
      base_shape_2_, "hoge"));

  // Abnormal system: Specify a group name that does not exist (Group Group)
  EXPECT_ANY_THROW(detector_->DisableCollisionCheckGroupToGroup(
      "BODY", "hoge"));
}

TEST_P(RobotCollisionDetectorTest, EnableCollisionPairProperty) {
  // Place an object so that the child body hits on the robot site
  OuterObjectParameters boxes;
  boxes = boxes_;
  boxes.origin_to_base = detector_->GetObjectTransform("BASE/FRAME_BASE");
  detector_->CreateOuterObject(boxes);
  detector_->CreateOuterObject(wall_);
  detector_->DisableCollisionObject(kWall);

  tmc_manipulation_types::JointState named_angle;
  named_angle = detector_->GetRobotNamedAngle();
  uint32_t shoulder_p_index =
      GetJointIndex(named_angle, "CARM/SHOULDER_P");
  uint32_t elbow_p_index =
      GetJointIndex(named_angle, "CARM/ELBOW_P");

  named_angle.position(shoulder_p_index) = 1.0;
  named_angle.position(elbow_p_index) = 0.0;
  detector_->SetRobotNamedAngle(named_angle);
  detector_->DisableCollisionObject(linear_shape_);

  // Added to interference check and check interference,
  // Then disable the interference check and check the interference again
  // Normal system: After granting, added to interference check (robot site/parent object)
  detector_->HoldObject(kBoxes, "BASE/FRAME_BASE",
                        Eigen::Affine3d::Identity(), "BODY");
  detector_->EnableCollisionCheckObjectToObject(base_shape_2_, kBoxes);
  std::vector<PairString> contact_list;
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(2, contact_list.size());
  detector_->DisableCollisionCheckObjectToObject(base_shape_2_, kBoxes);
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal system: Added to interference check (robot site/child object)
  detector_->EnableCollisionCheckObjectToObject(
      base_shape_2_, std::string(kBoxes) + "#0");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(1, contact_list.size());
  detector_->DisableCollisionCheckObjectToObject(
      base_shape_2_, std::string(kBoxes) + "#0");
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal system: Added to interference check (robot site/external object)
  detector_->EnableCollisionCheckObjectToGroup(base_shape_2_, "OUTER");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(0, contact_list.size());
  detector_->DisableCollisionCheckObjectToGroup(base_shape_2_, "OUTER");
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal system: Added to interference check (part group/parent object)
  detector_->EnableCollisionCheckObjectToGroup(kBoxes, "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(8, contact_list.size());
  detector_->DisableCollisionCheckObjectToGroup(kBoxes, "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal system: Added to interference check (part group/child object)
  detector_->EnableCollisionCheckObjectToGroup(
      std::string(kBoxes) + "#0", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(2, contact_list.size());
  detector_->DisableCollisionCheckObjectToGroup(
      std::string(kBoxes) + "#0", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal system: Added to interference check (part group/external object)
  detector_->EnableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(0, contact_list.size());
  detector_->DisableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal system: Is the group setting change reflected?
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, kODE);
  detector_->DisableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->EnableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CreateOuterObject(boxes);
  detector_->SetRobotNamedAngle(named_angle);
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(4, contact_list.size());

  // Abnormal system: Specify the non -existent object name (Object Object)
  EXPECT_ANY_THROW(detector_->EnableCollisionCheckObjectToObject(
      base_shape_2_, "hoge"));

  // Abnormal system: Specify the non -existent object name (Object Group)
  EXPECT_ANY_THROW(detector_->EnableCollisionCheckObjectToGroup(
      "hoge", "BODY"));

  // Abnormal system: Specify the unusual group name (Object Group)
  EXPECT_ANY_THROW(detector_->EnableCollisionCheckObjectToGroup(
      base_shape_2_, "hoge"));

  // Abnormal system: Specify a group name that does not exist (Group Group)
  EXPECT_ANY_THROW(detector_->EnableCollisionCheckGroupToGroup(
      "BODY", "hoge"));
}

TEST_P(RobotCollisionDetectorTest, GetObjectTransform) {
  // Normal system: Acquired the posture of the robot site (Shape)
  EXPECT_NO_THROW(detector_->GetObjectTransform(wrist_shape_));

  // Normal system: Acquire the posture of the robot site (JOINT)
  EXPECT_NO_THROW(detector_->GetObjectTransform(kJoint));

  // Normal system: Get the posture of external objects
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(detector_->GetObjectTransform(kWall));

  // Normal system: Acquire the attitude of the knowledge object
  detector_->HoldObject(kWall, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(detector_->GetObjectTransform(kWall));

  // Normal system: Acquired Cuboid posture
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_NO_THROW(detector_->GetObjectTransform(std::string(kCuboid) + "0"));

  // Abnormal system: Acquire the posture of an unusable object
  EXPECT_ANY_THROW(detector_->GetObjectTransform("hoge"));
}

TEST_P(RobotCollisionDetectorTest, SetObjectTransform) {
  // Normal system: Set posture on external objects
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(
      detector_->SetObjectTransform(kWall, Eigen::Affine3d::Identity()));

  // Normal system: Set posture of robot site (Shape) (cannot be set)
  EXPECT_NO_THROW(
      detector_->SetObjectTransform(wrist_shape_, Eigen::Affine3d::Identity()));

  // Normal system: Set posture of the knowledge object (cannot be set)
  detector_->HoldObject(kWall, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(
      detector_->SetObjectTransform(kWall, Eigen::Affine3d::Identity()));

  // Normal system: Change of Cuboid posture
  detector_->CreateCuboids(bounding_boxes_, false);
  std::string cuboid_name(std::string(kCuboid) + "0");
  EXPECT_NO_THROW(
      detector_->SetObjectTransform(cuboid_name, Eigen::Affine3d::Identity()));
  EXPECT_FALSE(detector_->GetObjectParameter(cuboid_name).cuboid);

  // Abnormal system: Set the posture of the non -existent object
  EXPECT_ANY_THROW(
      detector_->SetObjectTransform("hoge", Eigen::Affine3d::Identity()));
}

TEST_P(RobotCollisionDetectorTest, HoldObject) {
  // Normal system: Child object knows one object
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(detector_->HoldObject(kWall, wrist_shape_,
                                        Eigen::Affine3d::Identity(),
                                        kWristGroup));
  EXPECT_EQ(detector_->GetObjectFilter(wrist_shape_),
            detector_->GetObjectFilter(kWall));
  Eigen::Vector3d diff(
      detector_->GetObjectTransform(kWall).translation() -
      detector_->GetObjectTransform(wrist_shape_).translation());
  EXPECT_TRUE(diff.isZero());

  // Normal system: Child object knows multiple objects
  detector_->CreateOuterObject(boxes_);
  EXPECT_NO_THROW(detector_->HoldObject(kBoxes, wrist_shape_,
                                        Eigen::Affine3d::Identity(),
                                        kWristGroup));
  EXPECT_EQ(detector_->GetObjectFilter(wrist_shape_),
            detector_->GetObjectFilter(kBoxes));
  diff = detector_->GetObjectTransform(kBoxes).translation() -
      detector_->GetObjectTransform(wrist_shape_).translation();
  EXPECT_TRUE(diff.isZero());

  // Normal system: I know the object already granted (cannot be grateful)
  EXPECT_NO_THROW(detector_->HoldObject(kBoxes, wrist_shape_,
                                        Eigen::Affine3d::Identity(),
                                        kWristGroup));

  // Normal system: I know CUBOID
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_NO_THROW(detector_->HoldObject(std::string(kCuboid) + "0", wrist_shape_,
                                        Eigen::Affine3d::Identity(),
                                        kWristGroup));

  // Normal system: I know the robot site (cannot be grateful)
  EXPECT_NO_THROW(detector_->HoldObject(wrist_shape_, wrist_shape_,
                                        Eigen::Affine3d::Identity(),
                                        kWristGroup));

  // Normal system: Pull the group from Frame_name
  detector_->ReleaseObject(kBoxes);
  EXPECT_NE(detector_->GetObjectFilter(wrist_shape_), detector_->GetObjectFilter(kBoxes));
  EXPECT_NO_THROW(detector_->HoldObject(kBoxes, "CARM/HAND/_root_", Eigen::Affine3d::Identity()));
  const auto config = std::make_shared<CollisionDetectorConfig>(collision_pair_list_);
  EXPECT_EQ(config->GetFilterBitByGroupName("HAND_GRASPED"), detector_->GetObjectFilter(kBoxes));

  // Abnormal system: Kind an object that does not exist
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, kODE);
  detector_->CreateOuterObject(boxes_);
  EXPECT_ANY_THROW(detector_->HoldObject("hoge", wrist_shape_,
                                         Eigen::Affine3d::Identity(),
                                         kWristGroup));

  // Abnormal system: I know with an unusual frame
  EXPECT_ANY_THROW(detector_->HoldObject(kBoxes, "hoge",
                                         Eigen::Affine3d::Identity(),
                                         kWristGroup));

  // Abnormal system: Knowledge in a group that does not exist
  EXPECT_ANY_THROW(detector_->HoldObject(kBoxes, wrist_shape_,
                                         Eigen::Affine3d::Identity(),
                                         "hoge"));
}

TEST_P(RobotCollisionDetectorTest, ReleaseObject) {
  // Normal system: The child object releases one object
  detector_->CreateOuterObject(wall_);
  detector_->HoldObject(kWall, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(detector_->ReleaseObject(kWall));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kWall),
            detector_->GetObjectGroup(kWall));

  // Normal system: Child objects release multiple objects
  detector_->CreateOuterObject(boxes_);
  detector_->HoldObject(kBoxes, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(detector_->ReleaseObject(kBoxes));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kBoxes),
            detector_->GetObjectGroup(kBoxes));

  // Normal system: Release all gratitude objects
  detector_->HoldObject(kWall, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  detector_->HoldObject(kBoxes, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(detector_->ReleaseAllObject());
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kWall),
            detector_->GetObjectGroup(kWall));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kBoxes),
            detector_->GetObjectGroup(kBoxes));

  // Normal system: let go of CUBOID
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, kODE);
  detector_->CreateCuboids(bounding_boxes_, false);
  detector_->HoldObject(std::string(kCuboid) + "0", wrist_shape_,
                        Eigen::Affine3d::Identity(),
                        kWristGroup);
  EXPECT_NO_THROW(detector_->ReleaseObject(std::string(kCuboid) + "0"));
  EXPECT_NE(detector_->GetObjectGroup(std::string(kCuboid) + "0"),
            detector_->GetObjectGroup(std::string(kCuboid) + "1"));

  // Abnormal system: Release non -existent objects
  EXPECT_ANY_THROW(detector_->ReleaseObject("hoge"));

  // Abnormal system: Release unknown objects
  detector_->CreateOuterObject(boxes_);
  EXPECT_ANY_THROW(detector_->ReleaseObject(kBoxes));
}

TEST_P(RobotCollisionDetectorTest, CollisionCheck) {
  // Place an object so that the child's body hits the robot
  OuterObjectParameters boxes;
  boxes = boxes_;
  boxes.origin_to_base = detector_->GetObjectTransform("BASE/FRAME_BASE");
  detector_->CreateOuterObject(boxes);

  tmc_manipulation_types::JointState named_angle;
  named_angle = detector_->GetRobotNamedAngle();
  uint32_t shoulder_p_index =
      GetJointIndex(named_angle, "CARM/SHOULDER_P");
  uint32_t elbow_p_index =
      GetJointIndex(named_angle, "CARM/ELBOW_P");

  named_angle.position(shoulder_p_index) = 1.0;
  named_angle.position(elbow_p_index) = 0.0;
  detector_->SetRobotNamedAngle(named_angle);

  // Normal system: Interference
  EXPECT_TRUE(detector_->CheckCollision());

  // Normal system: Interference check disabled → Do not interfere
  detector_->DisableCollisionObject(kBoxes);
  EXPECT_FALSE(detector_->CheckCollision());

  // Normal system: Enable after disabled → Interference
  detector_->EnableCollisionObject(kBoxes);
  EXPECT_TRUE(detector_->CheckCollision());

  // Normal system: I know → Do not interfere
  detector_->HoldObject(kBoxes, "BASE/FRAME_BASE",
                        Eigen::Affine3d::Identity(), "BODY");
  EXPECT_FALSE(detector_->CheckCollision());

  // Normal system: Release after grant → Interference
  detector_->ReleaseObject(kBoxes);
  EXPECT_TRUE(detector_->CheckCollision());

  // Normal system: Change the filter settings so that they do not interfere with BODY → Do not interfere
  detector_->SetObjectGroup(
      kBoxes, detector_->GetObjectGroup(base_shape_1_));
  detector_->SetObjectFilter(
      kBoxes, detector_->GetObjectFilter(base_shape_1_));
  EXPECT_FALSE(detector_->CheckCollision());

  // Normal system; Return the group to the original → Interference
  detector_->SetObjectDefaultGroup(kBoxes);
  detector_->SetObjectDefaultFilter(kBoxes);
  EXPECT_TRUE(detector_->CheckCollision());

  // Normal system: acquisition of interference object pair names
  std::vector<PairString> contact_list;
  EXPECT_TRUE(detector_->CheckCollision(false, contact_list));
  ASSERT_FALSE(contact_list.empty());
  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  EXPECT_TRUE(detector_->CheckCollisionPair(
      contact_list[0].first, contact_list[0].second,
      point, normal));

  // Normal system: acquisition of interference object pair name list
  EXPECT_TRUE(detector_->CheckCollision(true, contact_list));
  EXPECT_FALSE(contact_list.empty());

  // Normal system: Check if the default interference check is valid
  detector_->HoldObject(kBoxes, "BASE/FRAME_BASE",
                        Eigen::Affine3d::Identity(), "BODY");
  uint32_t wrist_p_index =
      GetJointIndex(named_angle, "CARM/WRIST_P");
  uint32_t neck_y_index =
      GetJointIndex(named_angle, "CARM/HEAD/NECK_Y");

  named_angle.position(wrist_p_index) = - M_PI / 2.0;
  named_angle.position(neck_y_index) = 69.0 / 180.0 * M_PI;
  detector_->SetRobotNamedAngle(named_angle);
  EXPECT_FALSE(detector_->CheckCollision());
}

TEST_P(RobotCollisionDetectorTest, GetAABB) {
  // Normal system: Acquire a robot AABB
  AABB default_aabb = detector_->GetRobotAABB();
  detector_->SetRobotTransform(
      Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::AngleAxisd::Identity());
  AABB moved_aabb = detector_->GetRobotAABB();
  EXPECT_DOUBLE_EQ(default_aabb(0, 0) + 1.0, moved_aabb(0, 0));

  // Normal system: Obtain AABB of parent object
  detector_->CreateCuboids(bounding_boxes_, true);
  default_aabb = detector_->GetObjectAABB(std::string(kCuboid) + "0");
  EXPECT_DOUBLE_EQ(0.175, default_aabb(0, 0));
  EXPECT_DOUBLE_EQ(0.225, default_aabb(0, 1));
  EXPECT_DOUBLE_EQ(-0.5, default_aabb(1, 0));
  EXPECT_DOUBLE_EQ(0.5, default_aabb(1, 1));
  EXPECT_DOUBLE_EQ(0.0, default_aabb(2, 0));
  EXPECT_DOUBLE_EQ(1.0, default_aabb(2, 1));

  // Normal system: Obtain AABB of child object
  default_aabb = detector_->GetObjectAABB(std::string(kCuboid) + "0#0");
  EXPECT_DOUBLE_EQ(0.175, default_aabb(0, 0));
  EXPECT_DOUBLE_EQ(0.225, default_aabb(0, 1));
  EXPECT_DOUBLE_EQ(-0.5, default_aabb(1, 0));
  EXPECT_DOUBLE_EQ(0.5, default_aabb(1, 1));
  EXPECT_DOUBLE_EQ(0.0, default_aabb(2, 0));
  EXPECT_DOUBLE_EQ(1.0, default_aabb(2, 1));

  // Abnormal system: Acquire AABB of an unusable object
  EXPECT_ANY_THROW(detector_->GetObjectAABB("hoge"));
}

TEST_P(RobotCollisionDetectorTest, RefleshOverlappedCuboids) {
  // Environmental preparation
  detector_->CreateCuboids(bounding_boxes_, false);
  tmc_manipulation_types::JointState named_angle;
  detector_->SetRobotTransform(
      Eigen::Translation3d(0.3, 0.0, 0.8) * Eigen::AngleAxisd::Identity());
  named_angle = detector_->GetRobotNamedAngle();
  uint32_t linear_index =
      GetJointIndex(named_angle, "CARM/LINEAR");
  uint32_t shoulder_p_index =
      GetJointIndex(named_angle, "CARM/SHOULDER_P");

  named_angle.position(linear_index) = 0.0;
  named_angle.position(shoulder_p_index) = 1.57;
  detector_->SetRobotNamedAngle(named_angle);

  // Normal system: legal operation (AABB, Group)
  EXPECT_NO_THROW(
      detector_->RefleshOverlappedCuboids(kOverlapAabb, kOverlapGroup));
  EXPECT_EQ(1, detector_->GetEnableCuboidsNum());

  // Normal system: legal operation (2DMAP, Group)
  EXPECT_NO_THROW(
      detector_->RefleshOverlappedCuboids(kOverlap2DMap, kOverlapGroup));
  EXPECT_EQ(3, detector_->GetEnableCuboidsNum());

  // Normal system: legal operation (AABB, Robot)
  EXPECT_NO_THROW(
      detector_->RefleshOverlappedCuboids(kOverlapAabb, kOverlapRobot));
  EXPECT_EQ(3, detector_->GetEnableCuboidsNum());

  // Normal system: legal operation (2DMAP, Robot)
  EXPECT_NO_THROW(
      detector_->RefleshOverlappedCuboids(kOverlap2DMap, kOverlapRobot));
  EXPECT_EQ(3, detector_->GetEnableCuboidsNum());
}

TEST_P(RobotCollisionDetectorTest, EnableDisableCuboids) {
  // Environmental preparation
  detector_->CreateCuboids(bounding_boxes_, false);
  detector_->SetRobotTransform(
      Eigen::Translation3d(0.2, 0.0, 0.0) * Eigen::AngleAxisd::Identity());
  tmc_manipulation_types::JointState named_angle;
  named_angle = detector_->GetRobotNamedAngle();
  uint32_t shoulder_p_index =
      GetJointIndex(named_angle, "CARM/SHOULDER_P");
  uint32_t elbow_p_index =
      GetJointIndex(named_angle, "CARM/ELBOW_P");

  named_angle.position(shoulder_p_index) = 1.0;
  named_angle.position(elbow_p_index) = 0.0;
  detector_->SetRobotNamedAngle(named_angle);

  // Normal system: Enable
  EXPECT_NO_THROW(detector_->EnableCuboids("CUBOID"));
  EXPECT_EQ(3, detector_->GetEnableCuboidsNum());
  EXPECT_TRUE(detector_->CheckCollision());

  // Normal system: Disable
  EXPECT_NO_THROW(detector_->DisableCuboids("CUBOID"));
  EXPECT_EQ(0, detector_->GetEnableCuboidsNum());
  EXPECT_FALSE(detector_->CheckCollision());

  // Normal system: CUBOID is zero, but it is enabled by specifying the existing group
  EXPECT_NO_THROW(detector_->EnableCuboids("BODY"));
  EXPECT_EQ(0, detector_->GetEnableCuboidsNum());
  EXPECT_FALSE(detector_->CheckCollision());

  // Normal system: CUBOID is 0, but it is disabled by specifying the existing group
  EXPECT_NO_THROW(detector_->EnableCuboids("CUBOID"));
  EXPECT_NO_THROW(detector_->DisableCuboids("BODY"));
  EXPECT_EQ(3, detector_->GetEnableCuboidsNum());
  EXPECT_TRUE(detector_->CheckCollision());

  // Abnormal system: Enable groups that do not exist
  EXPECT_ANY_THROW(detector_->EnableCuboids("hoge"));

  // Abnormal system: Enable groups that do not exist
  EXPECT_ANY_THROW(detector_->DisableCuboids("hoge"));
}

TEST_P(RobotCollisionDetectorTest, GetNameList) {
  // Environmental construction
  detector_->CreateOuterObject(wall_);
  detector_->CreateOuterObject(boxes_);
  detector_->HoldObject(kWall, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);

  // Normal system: Obtain a list of all object names in the space
  std::vector<std::string> name_list;
  name_list = detector_->GetObjectNameList();
  EXPECT_EQ(29, name_list.size());

  // Normal system: Obtain a list of object names included in the external object group
  name_list = detector_->GetObjectNameListByGroup("OUTER");
  EXPECT_EQ(1, name_list.size());

  // Normal system: Included in groups with knowledge objects
  // Get an object name list
  name_list = detector_->GetObjectNameListByGroup(kWristGroup);
  EXPECT_EQ(3, name_list.size());

  // Normal system: Obtain a list of object names included in the robot part group
  name_list = detector_->GetObjectNameListByGroup("BODY");
  EXPECT_EQ(9, name_list.size());

  // Normal system: Acquire an object name list included in the Cuboid group
  detector_->CreateCuboids(bounding_boxes_, false);
  name_list = detector_->GetObjectNameListByGroup("CUBOID");
  EXPECT_EQ(3, name_list.size());

  // Normal system: Obtain an object name list included in a group where Cuboid is added
  detector_->DestroyCuboids();
  detector_->CreateCuboids(bounding_boxes_, false, "BODY");
  name_list = detector_->GetObjectNameListByGroup("BODY");
  EXPECT_EQ(12, name_list.size());

  // Normal system: Objects acquire empty groups
  name_list = detector_->GetObjectNameListByGroup("HANDHELD");
  EXPECT_TRUE(name_list.empty());

  // Normal system: Acquire a list of group names
  name_list = detector_->GetGroupList();
  EXPECT_EQ(14, name_list.size());

  // Normal system: Acquired a list of amiable object names
  name_list = detector_->GetHeldObjectList();
  EXPECT_EQ(1, name_list.size());

  // Abnormal system: Specify a group name that does not exist
  EXPECT_ANY_THROW(detector_->GetObjectNameListByGroup("hoge"));
}

INSTANTIATE_TEST_CASE_P(MultiModelTypeTests,
                        RobotCollisionDetectorTest,
                        ::testing::Values(
                            tmc_robot_collision_detector::kUrdf));

}  // namespace tmc_robot_collision_detector

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
