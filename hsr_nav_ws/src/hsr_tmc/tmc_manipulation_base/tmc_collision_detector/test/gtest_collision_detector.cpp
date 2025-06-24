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
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include "tmc_collision_detector/ODE_collision_detector.hpp"

// Do you test a number of object generations?
#define CREATEOBJECT_CASE6 0

using tmc_manipulation_types::ObjectParameter;
using tmc_manipulation_types::Shape;
using tmc_manipulation_types::AABB;
using tmc_manipulation_types::CollisionObjectType;
using tmc_manipulation_types::kBox;
using tmc_manipulation_types::kMesh;
using tmc_manipulation_types::kSphere;
using tmc_manipulation_types::kCapsule;
using tmc_manipulation_types::kCylinder;
using tmc_manipulation_types::kMeshVertices;

namespace tmc_collision_detector {
const char* kNode = "gtest_collision_detector";
const char kObjectName[] = "object";
const char kNonExistName[] = "objct";

const char* kTestForlderPath = "test_forlder_path";
const char* kSTLCorrectFileName = "cylinder.stl";
const char* kSTLNothingFileName = "nothing.stl";
const char* kSTLZeroFileName = "zero.stl";
const double kPrimParams[3] = {0.10, 0.20, 0.30};
const double kPrimCoordinate[3] = {-0.10, 0.10, 0.50};
const double kMeshAABB[3] = {0.20, 0.15, 0.20};

const uint16_t kGroup[4] = {1, 2, 4, 8};
const uint16_t kFilter[4] = {14, 13, 11, 7};
const double kSpaceCoordinate[4][2] = {
    {0.0, 0.0},
    {3 * kPrimParams[0], 0.0},
    {0.0, -1.5 * kPrimParams[0]},
    {0.0, 1.5 * kPrimParams[0]}
};
const double kClosestObjectCoordinate[6][2] = {
    {0.0, 0.0},
    {3 * kPrimParams[0], 0.0},
    {-4 * kPrimParams[0], 0.0},
    {-4 * kPrimParams[0], 2 * kPrimParams[0]},
    {-4 * kPrimParams[0], 4 * kPrimParams[0]},
    {10 * kPrimParams[0], 2 * kPrimParams[0]}
};

const double kMargin = 0.02;
const double kEpsilon = 0.005;

enum UseEngine {
  kODE = 0,
  kEngineNum
};

enum TestCase {
  kNormalCase1 = 0,
  kNormalCase2,
  kNormalCase3,
  kNormalCase4,
  kNormalCase5,
  kNormalCase6,
  kNormalCase7,
  kNormalCase8,
  kNormalCase9,
  kNormalCase10,
  kAbnormalCase1,
  kAbnormalCase2,
  kAbnormalCase3,
  kAbnormalCase4,
  kAbnormalCase5,
};

class CollisionDetectorTest : public ::testing::Test {
 public:
  virtual ~CollisionDetectorTest() {}

 protected:
  // Operation test for each function
  bool CreateObject_(TestCase case_no, UseEngine engine);
  bool DestroyObject_(TestCase case_no, UseEngine engine);
  bool UseAnchor_(TestCase case_no, UseEngine engine);

  bool SetObjectTransform_(TestCase case_no, UseEngine engine);
  bool GetObjectTransform_(TestCase case_no, UseEngine engine);

  bool ChangeObjectPropertyFunctions_(TestCase case_no, UseEngine engine);

  bool CheckCollision_(enum TestCase case_no, UseEngine engine);
  bool CheckCollisionHogeHoge_(enum TestCase case_no,
                               UseEngine engine,
                               CollisionObjectType typeA,
                               CollisionObjectType typeB);
  bool CheckCollisionSpace_(enum TestCase case_no, UseEngine engine);
  bool GetContactPairList_(enum TestCase case_no, UseEngine engine);
  bool GetClosestObject_(enum TestCase case_no, UseEngine engine);
  bool RayCasting_(enum TestCase case_no, UseEngine engine);
  bool CheckCollisionPair(enum TestCase case_no, UseEngine engine);

  ObjectParameter InitObjectParameter_(const std::string& name,
                                       CollisionObjectType type);
  void SetObjectDistance_(CollisionObjectType type,
                          Eigen::Vector3d& transform,
                          uint16_t &object_number);

  ICollisionDetector::Ptr coldet_;
};

ObjectParameter CollisionDetectorTest::InitObjectParameter_(
    const std::string& name, CollisionObjectType type) {
  ObjectParameter parameter;
  parameter.name = name;
  parameter.margin = kMargin;
  parameter.shape.type = type;
  parameter.shape.dimensions.push_back(kPrimParams[0]);
  parameter.shape.dimensions.push_back(kPrimParams[1]);
  parameter.shape.dimensions.push_back(kPrimParams[2]);
  parameter.shape.filename = kSTLCorrectFileName;
  parameter.transform.setIdentity();
  parameter.group = 0x0001;
  parameter.filter = 0xFFFF;

  std::string dir(__FILE__);
  dir = dir.substr(0, dir.find_last_of("/"));
  const auto cylinder_shape = shapes::createMeshFromResource(std::string("file://") + dir + "/cylinder.stl");
  for (auto i = 0u; i < cylinder_shape->vertex_count; ++i) {
    parameter.shape.vertices.push_back(Eigen::Vector3f(cylinder_shape->vertices[3 * i],
                                                       cylinder_shape->vertices[3 * i + 1],
                                                       cylinder_shape->vertices[3 * i + 2]));
  }
  for (auto i = 0u; i < cylinder_shape->triangle_count; ++i) {
    parameter.shape.indices.push_back(cylinder_shape->triangles[3 * i]);
    parameter.shape.indices.push_back(cylinder_shape->triangles[3 * i + 1]);
    parameter.shape.indices.push_back(cylinder_shape->triangles[3 * i + 2]);
  }

  return parameter;
}

bool CollisionDetectorTest::CreateObject_(enum TestCase case_no,
                                          UseEngine engine) {
  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kBox);

  switch (case_no) {
    case kNormalCase1:
      parameter.shape.type = kSphere;
      break;
    case kNormalCase2:
      break;
    case kNormalCase3:
      parameter.shape.type = kCapsule;
      break;
    case kNormalCase4:
      parameter.shape.type = kCylinder;
      break;
    case kNormalCase5:
      parameter.shape.type = kMesh;
      break;
    case kNormalCase6:
      break;
    case kNormalCase7:
      parameter.shape.type = kMeshVertices;
      break;
    case kAbnormalCase1:
      parameter.shape.type = static_cast<CollisionObjectType>(50);
      break;
    case kAbnormalCase2:
      parameter.shape.type = kMesh;
      parameter.shape.filename.assign(kSTLNothingFileName);
      break;
    case kAbnormalCase3:
      parameter.shape.type = kMesh;
      parameter.shape.filename.assign(kSTLZeroFileName);
      break;
    case kAbnormalCase4:
      parameter.shape.type = kBox;
      parameter.shape.dimensions.clear();
      break;
    case kAbnormalCase5:
      parameter.shape.type = kSphere;
      parameter.shape.dimensions.clear();
      parameter.shape.dimensions.push_back(-kPrimParams[0]);
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }
  try {
    coldet_->CreateObject(parameter);
    coldet_->CheckCollisionSpace();
#if CREATEOBJECT_CASE6
    if (case_no == kNormalCase6) {
      for (int32_t i = 1; i < 10000; i++) {
        coldet_->CreateObject(parameter);
      }
    }
#endif
  } catch (...) {
    return false;
  }

  return true;
}
bool CollisionDetectorTest::DestroyObject_(
    enum TestCase case_no, UseEngine engine) {
  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kBox);

  switch (case_no) {
    case kNormalCase1:
      parameter.shape.type = kSphere;
      coldet_->CreateObject(parameter);
      break;
    case kNormalCase2:
      parameter.shape.type = kMesh;
      coldet_->CreateObject(parameter);
      break;
    case kAbnormalCase1:
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }
  try {
    coldet_->DestroyObject(kObjectName);
    coldet_->CheckCollisionSpace();
  } catch (...) {
    return false;
  }

  return true;
}
bool CollisionDetectorTest::UseAnchor_(TestCase case_no, UseEngine engine) {
  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kBox);

  try {
    switch (case_no) {
      case kNormalCase1:
        coldet_->CreateObject(parameter);
        coldet_->SetAnchor();
        break;
      case kNormalCase2:
        coldet_->CreateObject(parameter);
        coldet_->SetAnchor(kObjectName);
        break;
      case kNormalCase3:
        coldet_->CreateObject(parameter);
        coldet_->SetAnchor(kObjectName);
        coldet_->GetAnchor();
        break;
      case kNormalCase4:
        coldet_->CreateObject(parameter);
        coldet_->SetAnchor(kObjectName);
        coldet_->CreateObject(parameter);
        coldet_->CreateObject(parameter);
        coldet_->DestroyObject();
        break;
      case kNormalCase5:
        coldet_->CreateObject(parameter);
        coldet_->SetAnchor();
        coldet_->DestroyObject();
        break;
      case kAbnormalCase1:
        coldet_->GetAnchor();
        break;
      case kAbnormalCase2:
        coldet_->SetAnchor();
        break;
      case kAbnormalCase3:
        coldet_->CreateObject(parameter);
        coldet_->CreateObject(parameter);
        coldet_->DestroyObject();
        break;
      default:
        assert(!"Beyond expectation.");
        return false;
    }
    coldet_->CheckCollisionSpace();
  } catch (...) {
    return false;
  }

  return true;
}
bool CollisionDetectorTest::SetObjectTransform_(
    enum TestCase case_no, UseEngine engine) {
  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kBox);

  Eigen::Affine3d transform;
  transform.translation() =
      Eigen::Vector3d(kPrimCoordinate[0],
                      kPrimCoordinate[1],
                      kPrimCoordinate[2]);
  switch (case_no) {
    case kNormalCase1:
      coldet_->CreateObject(parameter);
      break;
    case kAbnormalCase1:
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }
  try {
    coldet_->SetObjectTransform(transform, kObjectName);
    coldet_->CheckCollisionSpace();
  } catch (...) {
    return false;
  }

  return true;
}
bool CollisionDetectorTest::GetObjectTransform_(
    enum TestCase case_no, UseEngine engine) {
  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kBox);

  Eigen::Affine3d transform;
  transform.translation() =
      Eigen::Vector3d(kPrimCoordinate[0],
                      kPrimCoordinate[1],
                      kPrimCoordinate[2]);
  switch (case_no) {
    case kNormalCase1:
      coldet_->CreateObject(parameter);
      coldet_->SetObjectTransform(transform, kObjectName);
      break;
    case kAbnormalCase1:
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }

  Eigen::Affine3d get_transform;
  try {
    get_transform = coldet_->GetObjectTransform(kObjectName);
    coldet_->CheckCollisionSpace();
  } catch (...) {
    return false;
  }
  if ((fabs(get_transform.translation().x() - kPrimCoordinate[0]) < kEpsilon) &&
      (fabs(get_transform.translation().y() - kPrimCoordinate[1]) < kEpsilon) &&
      (fabs(get_transform.translation().z() - kPrimCoordinate[2]) < kEpsilon)) {
    return true;
  } else {
    return false;
  }
}
bool CollisionDetectorTest::ChangeObjectPropertyFunctions_(
    TestCase case_no, UseEngine engine) {
  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }
  uint16_t parameter = 1;

  try {
    switch (case_no) {
      case kAbnormalCase1:
        coldet_->SetCollisionGroup(parameter, kObjectName);
        break;
      case kAbnormalCase2:
        coldet_->SetCollisionFilter(parameter, kObjectName);
        break;
      case kAbnormalCase3:
        coldet_->EnableObject(kObjectName);
        break;
      case kAbnormalCase4:
        coldet_->DisableObject(kObjectName);
        break;
      default:
        assert(!"Beyond expectation.");
        return false;
    }
    coldet_->CheckCollisionSpace();
  } catch (...) {
    return false;
  }

  return true;
}

bool CollisionDetectorTest::CheckCollision_(
    enum TestCase case_no, UseEngine engine) {
  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kBox);
  coldet_->CreateObject(parameter);

  try {
    switch (case_no) {
      case kAbnormalCase1:
        coldet_->CheckCollisionPair(kNonExistName, kObjectName);
        break;
      case kAbnormalCase2:
        coldet_->CheckCollisionPair(kObjectName, kNonExistName);
        break;
      default:
        assert(!"Beyond expectation.");
        return false;
    }
  } catch (...) {
    return false;
  }

  return true;
}

bool CollisionDetectorTest::CheckCollisionHogeHoge_(
    enum TestCase case_no,
    UseEngine engine,
    CollisionObjectType typeA,
    CollisionObjectType typeB) {
  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kSphere);

  // Sphere
  parameter.name = kObjectName + std::to_string(0);
  coldet_->CreateObject(parameter);
  parameter.name = kObjectName + std::to_string(6);
  coldet_->CreateObject(parameter);

  // Box
  parameter.shape.type = kBox;
  parameter.name = kObjectName + std::to_string(1);
  coldet_->CreateObject(parameter);
  parameter.name = kObjectName + std::to_string(7);
  coldet_->CreateObject(parameter);

  // Capsule
  parameter.shape.type = kCapsule;
  parameter.name = kObjectName + std::to_string(2);
  coldet_->CreateObject(parameter);
  parameter.name = kObjectName + std::to_string(8);
  coldet_->CreateObject(parameter);

  // Cylinder
  parameter.shape.type = kCylinder;
  parameter.name = kObjectName + std::to_string(3);
  coldet_->CreateObject(parameter);
  parameter.name = kObjectName + std::to_string(9);
  coldet_->CreateObject(parameter);

  // Nesh
  parameter.shape.type = kMesh;
  parameter.name = kObjectName + std::to_string(4);
  coldet_->CreateObject(parameter);
  parameter.name = kObjectName + std::to_string(10);
  coldet_->CreateObject(parameter);

  // Mesh (vertex specification)
  parameter.shape.type = kMeshVertices;
  parameter.name = kObjectName + std::to_string(5);
  coldet_->CreateObject(parameter);
  parameter.name = kObjectName + std::to_string(11);
  coldet_->CreateObject(parameter);

  Eigen::Vector3d translation(0.0, 0.0, 0.0);
  uint16_t object_number[2] = {0, 0};
  SetObjectDistance_(typeA, translation, object_number[0]);
  SetObjectDistance_(typeB, translation, object_number[1]);

  std::string hogeA_name = kObjectName + std::to_string(object_number[0]);
  std::string hogeB_name = kObjectName + std::to_string(object_number[1] + 6);

  Eigen::Affine3d object_transform;
  object_transform.setIdentity();
  bool expect_result = false;
  switch (case_no) {
    case kNormalCase1:
      object_transform.translation() =
          Eigen::Vector3d(translation.x() + 3 * kMargin, 0.0, 0.0);
      break;
    case kNormalCase2:
      object_transform.translation() =
          Eigen::Vector3d(translation.x() + kMargin, 0.0, 0.0);
      break;
    case kNormalCase3:
      object_transform.translation() =
          Eigen::Vector3d(translation.x() - kMargin, 0.0, 0.0);
      expect_result = true;
      break;
    case kNormalCase4:
      object_transform.translation() =
          Eigen::Vector3d(0.0, translation.y() + 3 * kMargin, 0.0);
      break;
    case kNormalCase5:
      object_transform.translation() =
          Eigen::Vector3d(0.0, translation.y() + kMargin, 0.0);
      break;
    case kNormalCase6:
      object_transform.translation() =
          Eigen::Vector3d(0.0, translation.y() - kMargin, 0.0);
      expect_result = true;
      break;
    case kNormalCase7:
      object_transform.translation() =
          Eigen::Vector3d(0.0, 0.0, translation.z() + 3 * kMargin);
      break;
    case kNormalCase8:
      object_transform.translation() =
          Eigen::Vector3d(0.0, 0.0, translation.z() + kMargin);
      break;
    case kNormalCase9:
      object_transform.translation() =
          Eigen::Vector3d(0.0, 0.0, translation.z() - kMargin);
      expect_result = true;
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }

  coldet_->SetObjectTransform(object_transform, hogeA_name);

  if (engine == kODE) {
    bool coldet_result = coldet_->CheckCollisionPair(hogeA_name, hogeB_name);
    if (coldet_result == expect_result) {
      return true;
    }
  } else {
    ClosestResult closest_result;
    closest_result = coldet_->GetClosestResult(hogeA_name, hogeB_name);
    if (closest_result.contact) {
      if (expect_result) {
        return true;
      }
    } else {
      if ((!expect_result) &&
          (fabs(closest_result.distance - kMargin) < kEpsilon)) {
        return true;
      }
    }
  }

  return false;
}
bool CollisionDetectorTest::CheckCollisionSpace_(
    enum TestCase case_no, UseEngine engine) {

  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kSphere);
  for (int32_t i = 0; i < 4; i++) {
    parameter.name = "sphere" + std::to_string(i);
    parameter.filter = kFilter[i];
    parameter.group = kGroup[i];
    parameter.transform.translation() =
        Eigen::Vector3d(kSpaceCoordinate[i][0], kSpaceCoordinate[i][1], 0.0);
    coldet_->CreateObject(parameter);
  }
  std::vector<PairString> disable_pairs;
  std::vector<PairString> enable_pairs;
  bool expect_result = false;
  switch (case_no) {
    case kNormalCase1:
      expect_result = true;
      break;
    case kNormalCase2:
      coldet_->DisableObject(std::string("sphere0"));
      break;
    case kNormalCase3:
      coldet_->DisableObject(std::string("sphere0"));
      coldet_->EnableObject(std::string("sphere0"));
      expect_result = true;
      break;
    case kNormalCase4:
      coldet_->SetCollisionGroup(kGroup[3], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[3], std::string("sphere0"));
      expect_result = true;
      break;
    case kNormalCase5:
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere0"));
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere3"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere3"));
      break;
    case kNormalCase6:
      coldet_->DestroyObject(std::string("sphere0"));
      coldet_->DestroyObject(std::string("sphere1"));
      coldet_->DestroyObject(std::string("sphere2"));
      coldet_->DestroyObject(std::string("sphere3"));
      break;
    case kNormalCase7:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      disable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->DisableCollisionCheck(disable_pairs);
      break;
    case kNormalCase8:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      disable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->DisableCollisionCheck(disable_pairs);
      coldet_->ResetCollisionCheckPairList();
      expect_result = true;
      break;
    case kNormalCase9:
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere0"));
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere3"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere3"));
      enable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->EnableCollisionCheck(enable_pairs);
      expect_result = true;
      break;
    case kNormalCase10:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      coldet_->DisableCollisionCheck(disable_pairs);
      enable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->EnableCollisionCheck(enable_pairs);
      coldet_->DestroyObject(std::string("sphere0"));
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }

  PairString pair_name;
  bool check_space_result = coldet_->CheckCollisionSpace(pair_name);
  if (check_space_result == expect_result) {
    return true;
  }

  return false;
}
bool CollisionDetectorTest::GetContactPairList_(
    enum TestCase case_no, UseEngine engine) {
  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kSphere);

  for (int32_t i = 0; i < 4; i++) {
    parameter.name = "sphere" + std::to_string(i);
    parameter.filter = kFilter[i];
    parameter.group = kGroup[i];
    parameter.transform.translation() =
        Eigen::Vector3d(kSpaceCoordinate[i][0], kSpaceCoordinate[i][1], 0.0);
    coldet_->CreateObject(parameter);
  }

  std::vector<PairString> disable_pairs;
  std::vector<PairString> enable_pairs;
  uint32_t expect_contact_num = 0;
  bool expect_result = false;
  switch (case_no) {
    case kNormalCase1:
      expect_contact_num = 2;
      expect_result = true;
      break;
    case kNormalCase2:
      coldet_->DisableObject(std::string("sphere0"));
      break;
    case kNormalCase3:
      coldet_->DisableObject(std::string("sphere0"));
      coldet_->EnableObject(std::string("sphere0"));
      expect_contact_num = 2;
      expect_result = true;
      break;
    case kNormalCase4:
      coldet_->SetCollisionGroup(kGroup[3], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[3], std::string("sphere0"));
      expect_contact_num = 1;
      expect_result = true;
      break;
    case kNormalCase5:
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere0"));
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere3"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere3"));
      break;
    case kNormalCase6:
      coldet_->DestroyObject(std::string("sphere0"));
      coldet_->DestroyObject(std::string("sphere1"));
      coldet_->DestroyObject(std::string("sphere2"));
      coldet_->DestroyObject(std::string("sphere3"));
      break;
    case kNormalCase7:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      disable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->DisableCollisionCheck(disable_pairs);
      break;
    case kNormalCase8:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      disable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->DisableCollisionCheck(disable_pairs);
      enable_pairs.push_back(PairString("sphere0", "sphere2"));
      coldet_->EnableCollisionCheck(enable_pairs);
      expect_contact_num = 1;
      expect_result = true;
      break;
    case kNormalCase9:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      disable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->DisableCollisionCheck(disable_pairs);
      coldet_->ResetCollisionCheckPairList();
      expect_contact_num = 2;
      expect_result = true;
      break;
    case kNormalCase10:
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere0"));
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere3"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere3"));
      enable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->EnableCollisionCheck(enable_pairs);
      expect_contact_num = 1;
      expect_result = true;
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }

  std::vector<PairString> pair_name_list;
  bool check_space_result = coldet_->GetContactPairList(pair_name_list);
  if ((check_space_result == expect_result) &&
      (expect_contact_num == pair_name_list.size())) {
    return true;
  }
  return false;
}
bool CollisionDetectorTest::GetClosestObject_(
    enum TestCase case_no, UseEngine engine) {
  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kSphere);

  for (int32_t i = 0; i < 6; i++) {
    parameter.name = "sphere" + std::to_string(i);
    parameter.filter = 3;
    parameter.group = 4;
    parameter.transform.translation() =
        Eigen::Vector3d(kClosestObjectCoordinate[i][0],
                        kClosestObjectCoordinate[i][1], 0.0);
    coldet_->CreateObject(parameter);
  }
  coldet_->SetCollisionFilter(4, std::string("sphere0"));
  coldet_->SetCollisionGroup(1, std::string("sphere0"));
  coldet_->SetCollisionFilter(4, std::string("sphere1"));
  coldet_->SetCollisionGroup(2, std::string("sphere1"));

  int32_t top_n = 3;
  double extend = 5.0 * kPrimParams[0];
  uint16_t filter = 4;
  std::string expect_answer;
  switch (case_no) {
    case kNormalCase1:
      expect_answer.assign("sphere2");
      break;
    case kNormalCase2:
      expect_answer.assign("sphere3");
      coldet_->DisableObject(std::string("sphere2"));
      break;
    case kNormalCase3:
      expect_answer.assign("");
      extend = 0.5 * kPrimParams[0];
      break;
    case kNormalCase4:
      expect_answer.assign("");
      coldet_->DisableObject(std::string("sphere2"));
      coldet_->DisableObject(std::string("sphere3"));
      coldet_->DisableObject(std::string("sphere4"));
      break;
    case kNormalCase5:
      expect_answer.assign("sphere");
      top_n = 1;
      break;
    case kNormalCase6:
      expect_answer.assign("sphere0");
      coldet_->DisableObject(std::string("sphere0"));
      break;
    case kAbnormalCase1:
      top_n = 0;
      break;
    case kAbnormalCase2:
      extend = 0.0;
      break;
    case kAbnormalCase3:
      filter = 0;
      break;
    case kAbnormalCase4:
      coldet_->DestroyObject(std::string("sphere0"));
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }
  try {
    ClosestResult result =
        coldet_->GetClosestObject(std::string("sphere0"),
                                  extend, top_n, filter);
    if (result.name.npos == result.name.find(expect_answer)) {
      return false;
    }
  } catch (...) {
    return false;
  }
  return true;
}
bool CollisionDetectorTest::RayCasting_(
    enum TestCase case_no, UseEngine engine) {
  if (engine == kODE) {
    coldet_ = ICollisionDetector::Ptr(new ODECollisionDetector());
  } else {
    throw;
  }

  Eigen::Vector3d start_point(Eigen::Vector3d::Zero());
  Eigen::Vector3d direction(Eigen::Vector3d::Random());
  direction.normalize();
  double length = 1.0;
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kSphere);
  parameter.transform.translation() = direction;

  bool expect_result = true;

  switch (case_no) {
    case kNormalCase1:
      break;
    case kNormalCase2:
      parameter.shape.type = kBox;
      break;
    case kNormalCase3:
      parameter.shape.type = kCapsule;
      break;
    case kNormalCase4:
      parameter.shape.type = kCylinder;
      break;
    case kNormalCase5:
      parameter.shape.type = kMesh;
      break;
    case kNormalCase6:
      parameter.transform.translation() = direction * (-1.0);
      expect_result = false;
      break;
    case kNormalCase7:
      parameter.transform.translation() = direction * 5.0;
      expect_result = false;
      break;
    case kNormalCase8:
      expect_result = false;
      break;
    case kAbnormalCase1:
      direction.setZero(3);
      break;
    case kAbnormalCase2:
      length = 0.0;
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }

  coldet_->CreateObject(parameter);
  try {
    Eigen::Vector3d end_point;
    std::string name;
    if (case_no == kNormalCase8) {
      coldet_->DisableObject(kObjectName);
    }
    bool result =
        coldet_->RayCasting(start_point, direction, length, end_point, name);
    if (result == expect_result) {
      return true;
    } else {
      return false;
    }
  } catch (...) {
    return false;
  }
}

void CollisionDetectorTest::SetObjectDistance_(
    CollisionObjectType type,
    Eigen::Vector3d& transform,
    uint16_t &object_number) {
  switch (type) {
    case kSphere:
      object_number = 0;
      transform += Eigen::Vector3d(kPrimParams[0],
                                   kPrimParams[0],
                                   kPrimParams[0]);
      break;
    case kBox:
      object_number = 1;
      transform += Eigen::Vector3d(kPrimParams[0] / 2.0,
                                   kPrimParams[1] / 2.0,
                                   kPrimParams[2] / 2.0);
      break;
    case kCapsule:
      object_number = 2;
      transform += Eigen::Vector3d(kPrimParams[0],
                                   kPrimParams[0],
                                   kPrimParams[0] + kPrimParams[1] / 2.0);
      break;
    case kCylinder:
      object_number = 3;
      transform += Eigen::Vector3d(kPrimParams[0],
                                   kPrimParams[0],
                                   kPrimParams[1] / 2.0);
      break;
    case kMesh:
      object_number = 4;
      transform += Eigen::Vector3d(kMeshAABB[0], kMeshAABB[1], kMeshAABB[2]);
      break;
    case kMeshVertices:
      object_number = 5;
      transform += Eigen::Vector3d(kMeshAABB[0], kMeshAABB[1], kMeshAABB[2]);
      break;
    default:
      assert(!"Beyond expectation.");
      return;
  }
}

TEST_F(CollisionDetectorTest, CreateObject) {
  // Knormalcase1: Make a ball
  // Knormalcase2: Make a box
  // Knormalcase3: Make a capsule
  // Knormalcase4: Make cylinder
  // Knormalcase5: Make a mesh
  // Knormalcase6: Make 10,000 boxes
  // Knormalcase7: Make a mesh (vertex specification)

  // Kabnormalcase1: Type is fraudulent
  // Kabnormalcase2: No STL file
  // Kabnormalcase3: STL file is empty
  // Kabnormalcase4: There is no box parameter
  // Kabnormalcase5: Sphere parameters are negative
  EXPECT_TRUE(CreateObject_(kNormalCase1, kODE));
  EXPECT_TRUE(CreateObject_(kNormalCase2, kODE));
  EXPECT_TRUE(CreateObject_(kNormalCase3, kODE));
  EXPECT_TRUE(CreateObject_(kNormalCase4, kODE));
  EXPECT_TRUE(CreateObject_(kNormalCase5, kODE));
  EXPECT_TRUE(CreateObject_(kNormalCase6, kODE));
  EXPECT_TRUE(CreateObject_(kNormalCase7, kODE));
  EXPECT_FALSE(CreateObject_(kAbnormalCase1, kODE));
  EXPECT_FALSE(CreateObject_(kAbnormalCase2, kODE));
  EXPECT_FALSE(CreateObject_(kAbnormalCase3, kODE));
  EXPECT_FALSE(CreateObject_(kAbnormalCase4, kODE));
  EXPECT_FALSE(CreateObject_(kAbnormalCase5, kODE));
}
TEST_F(CollisionDetectorTest, DestroyObject) {
  // Knormalcase1: Create and discard primitive
  // Knormalcase2: Mesh and discard
  // Kabnormalcase1: Discard an object that has not made anything
  EXPECT_TRUE(DestroyObject_(kNormalCase1, kODE));
  EXPECT_TRUE(DestroyObject_(kNormalCase2, kODE));
  EXPECT_FALSE(DestroyObject_(kAbnormalCase1, kODE));
}
TEST_F(CollisionDetectorTest, UseAnchor) {
  // Knormalcase1: Create a primitive and set the end in an anchor
  // Knormalcase2: Create a primitive and set the primitive made in an anchor
  // Knormalcase3: Set the primitive made into the anchor and then get the anchor's name
  // Knormalcase4: Set the primitive made into the anchor and then discard the object using the anchor
  // Knormalcase5: Use the anchor to discard the object using an anchor
  // Kabnormalcase1: Get the name without setting an anchor
  // Kabnormalcase2: Anchor set without primitive
  // Kabnormalcase3: Discard the object using an anchor without setting an anchor

  EXPECT_TRUE(UseAnchor_(kNormalCase1, kODE));
  EXPECT_TRUE(UseAnchor_(kNormalCase2, kODE));
  EXPECT_TRUE(UseAnchor_(kNormalCase3, kODE));
  EXPECT_TRUE(UseAnchor_(kNormalCase4, kODE));
  EXPECT_TRUE(UseAnchor_(kNormalCase5, kODE));
  EXPECT_FALSE(UseAnchor_(kAbnormalCase1, kODE));
  EXPECT_FALSE(UseAnchor_(kAbnormalCase2, kODE));
  EXPECT_FALSE(UseAnchor_(kAbnormalCase3, kODE));
}

TEST_F(CollisionDetectorTest, SetObjectTransformODE) {
  // knormalcase1: Normal system
  // Kabnormalcase1: Set coordinates on untable objects
  EXPECT_TRUE(SetObjectTransform_(kNormalCase1, kODE));
  EXPECT_FALSE(SetObjectTransform_(kAbnormalCase1, kODE));
}

TEST_F(CollisionDetectorTest, GetObjectTransform) {
  // knormalcase1: Normal system
  // Kabnormalcase1: Get the coordinates of untouched objects

  EXPECT_TRUE(GetObjectTransform_(kNormalCase1, kODE));
  EXPECT_FALSE(GetObjectTransform_(kAbnormalCase1, kODE));
}

TEST_F(CollisionDetectorTest, ChangeObjectPropertyFunctions) {
  // Category Filter set, Fail Test for enabled / invalid functions
  // KabnormalCase1: SetCollisionGroup () in untouched objects ()
  // Kabnormalcase2: SetCollisionfilter () in untouched objects ()
  // Kabnormalcase3: ENABLEOBJECT () with an untouched object ()
  // Kabnormalcase4: DisableObject () with an untouched object ()

  EXPECT_FALSE(ChangeObjectPropertyFunctions_(kAbnormalCase1, kODE));
  EXPECT_FALSE(ChangeObjectPropertyFunctions_(kAbnormalCase2, kODE));
  EXPECT_FALSE(ChangeObjectPropertyFunctions_(kAbnormalCase3, kODE));
  EXPECT_FALSE(ChangeObjectPropertyFunctions_(kAbnormalCase4, kODE));
}
TEST_F(CollisionDetectorTest, CheckCollision) {
  // Kabnormalcase1: Interference check with untouched objects
  // Kabnormalcase2: Interference check with untouched objects

  EXPECT_FALSE(CheckCollision_(kAbnormalCase1, kODE));
  EXPECT_FALSE(CheckCollision_(kAbnormalCase2, kODE));
}
// Testing whether the result of the interference check matches the expected value
// The distance between the two objects is
// 1. X -axis direction, contact distance + kmargin * 3 (not in contact
// 2. X -axis direction, contact distance + kmargin (ODE does not contact
// 3. X -axial direction, contact distance -kmargin (contact)
// 4. Y -axis direction, contact distance + kmargin * 3 (not contact
// 5. Y -axis direction, contact distance + kmargin (ODE does not contact
// 6. Y -axis direction, contact distance -kmargin (contact)
// 7. Z axial direction, contact distance + kmargin * 3 (not contact
// 8. Z axial direction, contact distance + kmargin (ODE does not contact
// 9. Z -axial direction, contact distance -kmargin (contact)
// It is.
TEST_F(CollisionDetectorTest, ColDetSphereSphere) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kSphere, kSphere));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kSphere, kSphere));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kSphere, kSphere));
}
TEST_F(CollisionDetectorTest, ColDetSphereBox) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kSphere, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kSphere, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kSphere, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase4, kODE, kSphere, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase5, kODE, kSphere, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase6, kODE, kSphere, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kSphere, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kSphere, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kSphere, kBox));
}
TEST_F(CollisionDetectorTest, ColDetSphereCapsule) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kSphere, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kSphere, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kSphere, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kSphere, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kSphere, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kSphere, kCapsule));
}
TEST_F(CollisionDetectorTest, ColDetSphereCylinder) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kSphere, kCylinder));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kSphere, kCylinder));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kSphere, kCylinder));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kSphere, kCylinder));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kSphere, kCylinder));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kSphere, kCylinder));
}
TEST_F(CollisionDetectorTest, ColDetSphereMesh) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kSphere, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kSphere, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kSphere, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase4, kODE, kSphere, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase5, kODE, kSphere, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase6, kODE, kSphere, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kSphere, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kSphere, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kSphere, kMesh));
}
TEST_F(CollisionDetectorTest, ColDetSphereMeshVertices) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kSphere, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kSphere, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kSphere, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase4, kODE, kSphere, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase5, kODE, kSphere, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase6, kODE, kSphere, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kSphere, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kSphere, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kSphere, kMeshVertices));
}
TEST_F(CollisionDetectorTest, ColDetBoxBox) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kBox, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kBox, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kBox, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase4, kODE, kBox, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase5, kODE, kBox, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase6, kODE, kBox, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kBox, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kBox, kBox));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kBox, kBox));
}
TEST_F(CollisionDetectorTest, ColDetBoxCapsule) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kBox, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kBox, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kBox, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kBox, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kBox, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kBox, kCapsule));
}
TEST_F(CollisionDetectorTest, ColDetBoxCylinder) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kBox, kCylinder));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kBox, kCylinder));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kBox, kCylinder));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kBox, kCylinder));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kBox, kCylinder));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kBox, kCylinder));
}
TEST_F(CollisionDetectorTest, ColDetBoxMesh) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kBox, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kBox, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kBox, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase4, kODE, kBox, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase5, kODE, kBox, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase6, kODE, kBox, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kBox, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kBox, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kBox, kMesh));
}
TEST_F(CollisionDetectorTest, ColDetBoxMeshVertices) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kBox, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kBox, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kBox, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase4, kODE, kBox, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase5, kODE, kBox, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase6, kODE, kBox, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kBox, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kBox, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kBox, kMeshVertices));
}
TEST_F(CollisionDetectorTest, ColDetCapsuleCapsule) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kCapsule, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kCapsule, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kCapsule, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kCapsule, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kCapsule, kCapsule));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kCapsule, kCapsule));
}
TEST_F(CollisionDetectorTest, ColDetCapsuleCylinder) {
  // In ODE, CPASULE and CYLINDER do not interfere because they do not interfere
}
TEST_F(CollisionDetectorTest, ColDetCapsuleMesh) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kCapsule, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kCapsule, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kCapsule, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kCapsule, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kCapsule, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kCapsule, kMesh));
}
TEST_F(CollisionDetectorTest, ColDetCapsuleMeshVertices) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kCapsule, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kCapsule, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kCapsule, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kCapsule, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kCapsule, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kCapsule, kMeshVertices));
}
TEST_F(CollisionDetectorTest, ColDetCylinderCylinder) {
  // In ODE, Cylinder does not test because it has a specification that does not interfere with each other.
}
TEST_F(CollisionDetectorTest, ColDetCylinderMesh) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kCylinder, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kCylinder, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kCylinder, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kCylinder, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kCylinder, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kCylinder, kMesh));
}
TEST_F(CollisionDetectorTest, ColDetCylinderMeshVertices) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kCylinder, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kCylinder, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kCylinder, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kCylinder, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kCylinder, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kCylinder, kMeshVertices));
}
TEST_F(CollisionDetectorTest, ColDetMeshMesh) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kMesh, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kMesh, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kMesh, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase4, kODE, kMesh, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase5, kODE, kMesh, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase6, kODE, kMesh, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kMesh, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kMesh, kMesh));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kMesh, kMesh));
}
TEST_F(CollisionDetectorTest, ColDetMeshMeshVertices) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kMesh, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kMesh, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kMesh, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase4, kODE, kMesh, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase5, kODE, kMesh, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase6, kODE, kMesh, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kMesh, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kMesh, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kMesh, kMeshVertices));
}
TEST_F(CollisionDetectorTest, ColDetMeshVerticesMeshVertices) {
  // ODE
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase1, kODE, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase2, kODE, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase3, kODE, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase4, kODE, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase5, kODE, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase6, kODE, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase7, kODE, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase8, kODE, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(CheckCollisionHogeHoge_(kNormalCase9, kODE, kMeshVertices, kMeshVertices));
}
TEST_F(CollisionDetectorTest, CheckCollisionSpace) {
  // Initial setting:
  // Place 4 balls with radius R and so that the center is on the XY plane
  // All set for different groups
  // Don't interfere with your group
  // The center of the ball 1 (0, 0)
  // The center of the ball 2 (3*R, 0)
  // The center of the ball 3 (0, -1.5*r)
  // The center of ball 4 (0, 1.5*R)
  // Knormalcase1: Do not mess with any settings and interfere
  // Knormalcase2: Disable ball 1 and do not interfere
  // Knormalcase3: Disable the ball 1, enable and interfere
  // Knormalcase4: Change the group and filter in the ball 1 the same as the ball 3, interfere
  // Knormalcase5: Change the group and filter with a ball 1,4 the same as the ball 3, do not interfere
  // Knormalcase6: Discard the ball 1,2,3,4, do not interfere
  // Knormalcase7: Extract and do not interfere with ball 1 and ball 4, ball 1 and ball 3 from interference checks
  // Knormalcase8: Ball 1 and ball 4, ball 1 and ball 3 are excluded from interference checks, then destroy the exclusion list and interfere
  // Knormalcase9: The group and filter of the ball 1,4 are the same as the ball 3, add to the interference check pair and interfere

  EXPECT_TRUE(CheckCollisionSpace_(kNormalCase1, kODE));
  EXPECT_TRUE(CheckCollisionSpace_(kNormalCase2, kODE));
  EXPECT_TRUE(CheckCollisionSpace_(kNormalCase3, kODE));
  EXPECT_TRUE(CheckCollisionSpace_(kNormalCase4, kODE));
  EXPECT_TRUE(CheckCollisionSpace_(kNormalCase5, kODE));
  EXPECT_TRUE(CheckCollisionSpace_(kNormalCase6, kODE));
  EXPECT_TRUE(CheckCollisionSpace_(kNormalCase7, kODE));
  EXPECT_TRUE(CheckCollisionSpace_(kNormalCase8, kODE));
  EXPECT_TRUE(CheckCollisionSpace_(kNormalCase9, kODE));
  EXPECT_TRUE(CheckCollisionSpace_(kNormalCase10, kODE));
}
TEST_F(CollisionDetectorTest, GetContactPairList) {
  // Initial setting:
  // Place 4 balls with radius R and so that the center is on the XY plane
  // All set for different groups
  // Don't interfere with your group
  // The center of the ball 1 (0, 0)
  // The center of the ball 2 (3*R, 0)
  // The center of the ball 3 (0, -1.5*r)
  // The center of ball 4 (0, 1.5*R)
  // Knormalcase1: Nothing to configure, interfere (2 places)
  // Knormalcase2: Disable ball 1 and do not interfere
  // Knormalcase3: After disabling the ball 1, enable and interfere (2 places)
  // Knormalcase4: Change the group and filter of the ball 1 the same as the ball 3, interfere (1 place)
  // Knormalcase5: Change the group and filter with a ball 1,4 the same as the ball 3, do not interfere
  // Knormalcase6: Discard the ball 1,2,3,4, do not interfere
  // Knormalcase7: Extract and do not interfere with ball 1 and ball 4, ball 1 and ball 3 from interference checks
  // Knormalcase8: A ball 1 and the ball 4, the ball 1 and the ball 3 are excluded from the interference check, and the check of the ball 1 and the ball 3 is enabled and interfered.
  // Knormalcase9: Ball 1 and ball 4, ball 1 and ball 3 are excluded from interference checks, then destroy the exclusion list and interfere
  // Knormalcase10: The group of balls 1,4 is the same as the ball 3, add to the interference check pair and interfere

  EXPECT_TRUE(GetContactPairList_(kNormalCase1, kODE));
  EXPECT_TRUE(GetContactPairList_(kNormalCase2, kODE));
  EXPECT_TRUE(GetContactPairList_(kNormalCase3, kODE));
  EXPECT_TRUE(GetContactPairList_(kNormalCase4, kODE));
  EXPECT_TRUE(GetContactPairList_(kNormalCase5, kODE));
  EXPECT_TRUE(GetContactPairList_(kNormalCase6, kODE));
  EXPECT_TRUE(GetContactPairList_(kNormalCase7, kODE));
  EXPECT_TRUE(GetContactPairList_(kNormalCase8, kODE));
  EXPECT_TRUE(GetContactPairList_(kNormalCase9, kODE));
  EXPECT_TRUE(GetContactPairList_(kNormalCase10, kODE));
}
TEST_F(CollisionDetectorTest, GetClosestObject) {
  // Initial setting:
  // Place the radius R ball as the center is on the XY plane
  // Conducted with a ball 1 and filter 100
  // Ball 1: Central (0, 0) Category 001 Filter 100
  // Ball 2: Central (3 * R, 0) Category 010 Filter 100
  // Ball 3: Central (-4 * R, 0) Category 100 filter 011
  // Ball 4: Central (-4 * R, 2 * R) Category 100 filter 011
  // Ball 5: Central (-4 * R, 4 * R) Category 100 filter 011
  // Ball 6: Central (10 * R, 2 * R) Category 100 filter 011

  // Knormalcase1: Top 3, Extended 5*R → Ball 3
  // Knormalcase2: Disable ball 3 Top 3, extended 5*R → Ball 4
  // Knormalcase3: Top 3, extension 0.5 * R → Not found
  // Knormalcase4: Disable ball 3,4,5 Top 3, extension 5*R → Not found
  // Knormalcase5: Top 1, Extended 5*R → I hope something is found, Sphere3
  // Knormalcase6: Running after disabling the ball 1 → not doing anything
  // kAbnormalCase1: top 0
  // Kabnormalcase2: Extended 0
  // Kabnormalcase3: Filter 0
  // Kabnormalcase4: Discard the ball 1 and execute it

  EXPECT_FALSE(GetClosestObject_(kNormalCase1, kODE));
}
TEST_F(CollisionDetectorTest, RayCasting) {
  // Knormalcase1: For balls
  // Knormalcase2: For boxes
  // Knormalcase3: Capsules
  // Knormalcase4: For cylinders
  // Knormalcase5: For mesh
  // Knormalcase6: There is no object in the direction of Ray
  // Knormalcase7: There is an object in the direction of Ray but it is far away
  // Knormalcase8: Disable objects

  EXPECT_TRUE(RayCasting_(kNormalCase1, kODE));
  EXPECT_TRUE(RayCasting_(kNormalCase2, kODE));
  EXPECT_TRUE(RayCasting_(kNormalCase3, kODE));
  EXPECT_TRUE(RayCasting_(kNormalCase5, kODE));
  EXPECT_TRUE(RayCasting_(kNormalCase6, kODE));
  EXPECT_TRUE(RayCasting_(kNormalCase7, kODE));
  EXPECT_TRUE(RayCasting_(kNormalCase8, kODE));
  EXPECT_FALSE(RayCasting_(kAbnormalCase1, kODE));
  EXPECT_FALSE(RayCasting_(kAbnormalCase2, kODE));
}
TEST_F(CollisionDetectorTest, GetMeshObjectParameter) {
  EXPECT_TRUE(CreateObject_(kNormalCase5, kODE));
  const auto param = coldet_->GetObjectParameter(kObjectName);
  // Cylinder.stl is the vertex 50, surface 96
  EXPECT_EQ(param.shape.vertices.size(), 50);
  EXPECT_EQ(param.shape.indices.size(), 96 * 3);
}
}  // namespace tmc_collision_detector

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
