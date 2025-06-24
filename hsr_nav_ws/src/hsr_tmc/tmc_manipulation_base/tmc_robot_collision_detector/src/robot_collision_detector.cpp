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
/// @brief Perform interference checks using a robot model
#include <tmc_robot_collision_detector/robot_collision_detector.hpp>

#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf_parser/urdf_parser.h>

#include <tmc_collision_detector/collision_detector.hpp>
#include <tmc_collision_detector/ODE_collision_detector.hpp>
#include "collision_detector_factory.hpp"

using tmc_manipulation_types::ObjectParameterSeq;
using tmc_manipulation_types::OuterObjectParameters;
using tmc_manipulation_types::OuterObjectParametersSeq;
using tmc_manipulation_types::Cuboid;
using tmc_manipulation_types::CuboidSeq;
using tmc_manipulation_types::AABB;
using urdf::parseURDF;
using urdf::ModelInterface;
using urdf::GeometrySharedPtr;
#define dpc urdf::dynamic_pointer_cast



namespace {
/// RobotConfigfile tag
const char* const kCollisionTag = "COLLISION";

/// @brief Get the child object name registered in the interference checker
std::string MakeChildObjectName(
    const std::string& parent_name, uint32_t child_num) {
  return parent_name + '#' + std::to_string(child_num);
}

/// @brief Compare and integrate AABB
AABB CombineAABB(const AABB& aabb1, const AABB& aabb2) {
  AABB return_aabb;
  for (int32_t i = 0; i < 3; i++) {
    return_aabb(i, 1) = std::max(aabb1(i, 1), aabb2(i, 1));
    return_aabb(i, 0) = std::min(aabb1(i, 0), aabb2(i, 0));
  }
  return return_aabb;
}

/// @brief Judge whether AABB overlaps
bool CheckAABBOverlap(const AABB& inputA, const AABB& inputB) {
  if ((inputA(0, 1) < inputB(0, 0)) || (inputA(1, 1) < inputB(1, 0)) ||
      (inputA(2, 1) < inputB(2, 0)) || (inputA(0, 0) > inputB(0, 1)) ||
      (inputA(1, 0) > inputB(1, 1)) || (inputA(2, 0) > inputB(2, 1))) {
    return false;
  } else {
    return true;
  }
}

/// @brief Judge whether it overlaps on the XY plane
bool CheckXYPlainOverlap(const AABB& inputA, const AABB& inputB) {
  if ((inputA(0, 1) < inputB(0, 0)) || (inputA(1, 1) < inputB(1, 0)) ||
      (inputA(0, 0) > inputB(0, 1)) || (inputA(1, 0) > inputB(1, 1))) {
    return false;
  } else {
    return true;
  }
}

/// @brief Convert Affine3D to Std :: Vector <Double>
void ConvertAffine3dToParam(const Eigen::Affine3d& transform,
                            std::vector<double>& dst_param) {
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond quaternion(transform.linear());
  dst_param.resize(7);
  dst_param[0] = position[0];
  dst_param[1] = position[1];
  dst_param[2] = position[2];
  dst_param[3] = quaternion.w();
  dst_param[4] = quaternion.x();
  dst_param[5] = quaternion.y();
  dst_param[6] = quaternion.z();
}

/// @brief Convert std::vector<double> to Affine3D
void ConvertParamToAffine3d(const std::vector<double>& param,
                            Eigen::Affine3d& dst_transform) {
  // PARAM should have seven elements of position+posture (quota tanion).
  if (param.size() != 7) {
    throw std::domain_error("ConvertParamToAffine3d error: invalid param.");
  }
  dst_transform.setIdentity();
  dst_transform.translation() = Eigen::Vector3d(param[0], param[1], param[2]);
  dst_transform.linear() = Eigen::Quaterniond(param[3], param[4], param[5],
                                              param[6]).toRotationMatrix();
}


/// @brief  Convert URDF Shape into Coldet parameters
tmc_manipulation_types::ObjectParameter ConvertUrdfShapeToCollisionObject(
    const std::shared_ptr<urdf::Collision>& collision,
    std::string name) {
  tmc_manipulation_types::ObjectParameter parameter;
  parameter.name = name;
  GeometrySharedPtr& geom = collision->geometry;
  switch (geom->type) {
    case urdf::Geometry::MESH:
      // TODO(Terada) urdfのscale属性に対応していないので，
      // Suppose tmc_collision_detector side
      parameter.shape.type = tmc_manipulation_types::kMesh;
      parameter.shape.filename =
        dpc<urdf::Mesh>(geom)->filename;
      break;
    case urdf::Geometry::SPHERE:
      parameter.shape.type = tmc_manipulation_types::kSphere;
      parameter.shape.dimensions.push_back(
        dpc<urdf::Sphere>(geom)->radius);
      break;
    case urdf::Geometry::BOX:
      parameter.shape.type = tmc_manipulation_types::kBox;
      parameter.shape.dimensions.push_back(
        dpc<urdf::Box>(geom)->dim.x);
      parameter.shape.dimensions.push_back(
        dpc<urdf::Box>(geom)->dim.y);
      parameter.shape.dimensions.push_back(
        dpc<urdf::Box>(geom)->dim.z);
      break;
    case urdf::Geometry::CYLINDER:
      parameter.shape.type = tmc_manipulation_types::kCylinder;
      parameter.shape.dimensions.push_back(
        dpc<urdf::Cylinder>(geom)->radius);
      parameter.shape.dimensions.push_back(
        dpc<urdf::Cylinder>(geom)->length);
      break;
    default:
      throw std::domain_error("error: not exist primitive type");
  }
  // Make the toDo margin readable from the configuration file
  parameter.margin = 0.0;
  return parameter;
}
}  // anonymous namespace

namespace tmc_robot_collision_detector {

void RobotCollisionDetector::Init_(const std::string& robot_model_config,
                                   const std::string& robot_collision_config,
                                   const std::string& engine,
                                   ModelFileType model_file_type) {
  robot_collision_config_ = std::make_shared<CollisionDetectorConfig>(robot_collision_config);
  // Initialization of interference check engine
  CollisionDetectorFactory::Ptr coldet_factory(
      new CollisionDetectorFactory(engine));
  coldet_ = coldet_factory->CreateCollisionDetector();
  inner_coldet_ = coldet_factory->CreateCollisionDetector();

  switch (model_file_type) {
    case kUrdf:
      if (!robot_model_) {
        robot_model_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(robot_model_config);
      }
      // Robot model generation for interference check
      CreateRobotModel_(robot_model_config, model_file_type);
      break;
    default:
      throw std::domain_error("Invalid Robot model file type.");
      break;
  }

  // Keep information to save
  environmental_data_.robot_model_config = robot_model_config;
  environmental_data_.robot_collision_config = robot_collision_config;
  environmental_data_.engine = engine;
  environmental_data_.model_file_type = model_file_type;
}

// Constructor that explicitly specifies a robot athletic model
RobotCollisionDetector::RobotCollisionDetector(
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot_model,
    const std::string& robot_model_config,
    const std::string& robot_collision_config,
    const std::string& engine) : robot_model_(robot_model) {
  Init_(robot_model_config,
        robot_collision_config,
        engine,
        kUrdf);
}

// constructor.Various initialization
RobotCollisionDetector::RobotCollisionDetector(
    const std::string& robot_model_config,
    const std::string& robot_collision_config,
    const std::string& engine) {
  // Use URDF
  Init_(robot_model_config,
        robot_collision_config,
        engine,
        kUrdf);
}


// constructor.Various initialization
RobotCollisionDetector::RobotCollisionDetector(
    const std::string& robot_model_config,
    const std::string& robot_collision_config,
    const std::string& engine,
    ModelFileType model_file_type) {
  Init_(robot_model_config,
        robot_collision_config,
        engine,
        model_file_type);
}

// Enter the angle information
void RobotCollisionDetector::SetRobotNamedAngle(
    const tmc_manipulation_types::JointState& named_angle) {
  robot_model_->SetNamedAngle(named_angle);
  UpdateCollisionModel_();
}

// Enter the position posture of the robot
void RobotCollisionDetector::SetRobotTransform(
    const Eigen::Affine3d& origin_to_robot) {
  robot_model_->SetRobotTransform(origin_to_robot);
  UpdateCollisionModel_();
}

// Creating external objects
void RobotCollisionDetector::CreateOuterObject(
    const OuterObjectParameters& parameters) {
  // Input value check
  if (parameters.base_to_child.size() != parameters.shape.size()) {
    throw std::domain_error(
        "CreateOuterObject error: mismatch child size and poses size.");
  }
  if (parameters.name.empty()) {
    throw std::domain_error("CreateOuterObject error: object name is empty");
  }
  if (outer_object_list_.find(parameters.name) != outer_object_list_.end()) {
    throw std::domain_error(
        "CreateOuterObject error: this object name already exist");
  }

  // Create an interference check model
  tmc_manipulation_types::ObjectParameter parameter;
  parameter.filter =
      robot_collision_config_->GetFilterBitByGroupName(
          std::string(tmc_robot_collision_detector::kOuterGroupName));
  parameter.group =
      robot_collision_config_->GetGroupBitByGroupName(
          std::string(tmc_robot_collision_detector::kOuterGroupName));
  parameter.margin = parameters.margin;
  for (uint32_t i = 0; i < parameters.base_to_child.size(); i++) {
    parameter.name = MakeChildObjectName(parameters.name, i);
    parameter.shape = parameters.shape.at(i);
    parameter.transform = parameters.origin_to_base * parameters.base_to_child.at(i);
    coldet_->CreateObject(parameter);
  }

  // Update object list
  outer_object_list_.insert(std::pair<std::string, OuterObjectParameters>(
      parameters.name, parameters));
  outer_object_list_[parameters.name].cuboid = false;
}

// Build an environment consisting of CUBOID
void RobotCollisionDetector::CreateCuboids(const CuboidSeq& map,
                                           bool enable_flag) {
  CreateCuboids(map, enable_flag, kCuboidGroupName);
}

// Build an environment consisting of CUBOID
void RobotCollisionDetector::CreateCuboids(
    const tmc_manipulation_types::CuboidSeq& map,
    bool enable_flag,
    const std::string& cuboid_group_name) {
  // Preparation for making Cuboid
  tmc_manipulation_types::ObjectParameter parameter;
  parameter.filter =
      robot_collision_config_->GetFilterBitByGroupName(cuboid_group_name);
  parameter.group =
      robot_collision_config_->GetGroupBitByGroupName(cuboid_group_name);
  parameter.shape.dimensions.resize(3);
  parameter.shape.type = tmc_manipulation_types::kBox;
  OuterObjectParameters outer_parameter;
  outer_parameter.base_to_child.push_back(Eigen::Affine3d::Identity());
  outer_parameter.shape.resize(1);

  // Decide the group
  std::vector<std::string>::iterator group =
      std::find(bounding_box_.group_name.begin(),
                bounding_box_.group_name.end(), cuboid_group_name);
  CuboidSeq *set;
  if (group == bounding_box_.group_name.end()) {
    bounding_box_.group_name.push_back(cuboid_group_name);
    bounding_box_.boxes.push_back(CuboidSeq());
    set = &bounding_box_.boxes.back();
  } else {
    uint32_t distance = std::distance(bounding_box_.group_name.begin(), group);
    set = &bounding_box_.boxes.at(distance);
  }

  for (CuboidSeq::const_iterator it = map.begin(); it != map.end(); ++it) {
    // Input check
    if (it->box_name.empty()) {
      throw std::domain_error("CreateCuboids error: box name is empty");
    }
    if (outer_object_list_.find(it->box_name) != outer_object_list_.end()) {
      throw std::domain_error(
          "CreateCuboids error: this object name already exist");
    }
    // Created in the interference check space
    parameter.name = MakeChildObjectName(it->box_name, 0);
    parameter.transform = it->box_transform;
    Eigen::Map<Eigen::Vector3d>(&(parameter.shape.dimensions[0])) =
        it->box_extents;
    parameter.margin = it->margin;
    coldet_->CreateObject(parameter);

    // Update object list
    outer_parameter.origin_to_base = it->box_transform;
    outer_parameter.margin = it->margin;
    outer_parameter.name = it->box_name;
    outer_parameter.shape.at(0) = parameter.shape;
    outer_parameter.cuboid = true;
    outer_parameter.parent_group = cuboid_group_name;
    outer_object_list_.insert(std::pair<std::string, OuterObjectParameters>(
        it->box_name, outer_parameter));

    // Reflect valid and invalidation
    if (!enable_flag) {
      coldet_->DisableObject(parameter.name);
      it->enable = false;
    } else {
      it->enable = true;
    }

    // Update Cuboid list
    set->push_back(*it);
  }
}

// Discard external objects
void RobotCollisionDetector::DestroyOuterObject(
    const std::string& object_name) {
  // Not eligible for internal objects
  if (IsInnerObject_(object_name)) {
    return;
  }
  std::map<std::string, OuterObjectParameters>::iterator it =
      outer_object_list_.find(object_name);
  if (it == outer_object_list_.end()) {
    throw std::domain_error(
        "DestroyOuterObject error: non exist object" + object_name);
  }
  if (it->second.cuboid) {
    EraseCuboid_(object_name);
  }
  for (uint32_t i = 0; i < it->second.base_to_child.size(); i++) {
    coldet_->DestroyObject(MakeChildObjectName(object_name, i));
  }
  outer_object_list_.erase(it);
}

// Discard all external objects
void RobotCollisionDetector::DestroyAllOuterObject(void) {
  ReleaseAllObject();
  outer_object_list_.clear();
  coldet_->DestroyObject();
  bounding_box_.boxes.clear();
  bounding_box_.group_name.clear();
  // Reset the object to be removed from the interference check
  std::vector<PairString> disable_object_pairs(
      robot_collision_config_->GetDisableObjectPairList());
  coldet_->DisableCollisionCheck(disable_object_pairs);
}

// Discard the environment consisting of CUBOID
void RobotCollisionDetector::DestroyCuboids() {
  for (std::vector<CuboidSeq>::iterator cuboids = bounding_box_.boxes.begin();
       cuboids != bounding_box_.boxes.end(); ++cuboids) {
    for (CuboidSeq::iterator it = cuboids->begin();
         it != cuboids->end(); ++it) {
      outer_object_list_.erase(outer_object_list_.find(it->box_name));
      coldet_->DestroyObject(MakeChildObjectName(it->box_name, 0));
    }
  }
  bounding_box_.boxes.clear();
  bounding_box_.group_name.clear();
}

// Obtain information on objects
OuterObjectParameters RobotCollisionDetector::GetObjectParameter(
    const std::string& object_name) const {
  // Search from external object list
  std::map<std::string, OuterObjectParameters>::const_iterator it =
      outer_object_list_.find(object_name);
  if (it != outer_object_list_.end()) {
    return it->second;
  }
  // Search from interference check space
  tmc_manipulation_types::ObjectParameter parameter =
      coldet_->GetObjectParameter(object_name);
  OuterObjectParameters return_parameter;
  return_parameter.name = object_name;
  return_parameter.origin_to_base = parameter.transform;
  return_parameter.base_to_child.push_back(Eigen::Affine3d::Identity());
  return_parameter.margin = parameter.margin;
  return_parameter.shape.push_back(parameter.shape);
  return return_parameter;
}

// Get information on all external objects
OuterObjectParametersSeq
RobotCollisionDetector::GetAllOuterObjectParameters() const {
  OuterObjectParametersSeq return_parameters;
  for (OuterObjectMap::const_iterator it = outer_object_list_.begin();
       it != outer_object_list_.end(); ++it) {
    return_parameters.push_back(it->second);
  }
  return return_parameters;
}

// Enable object interference checks
void RobotCollisionDetector::EnableCollisionObject(
    const std::string& object_name) {
  auto enable_object_function = std::bind(
      &tmc_collision_detector::ICollisionDetector::EnableObject, coldet_, std::placeholders::_1);
  SetObjectProperty_(object_name, enable_object_function);
}

// Disable object interference checks
void RobotCollisionDetector::DisableCollisionObject(
    const std::string& object_name) {
  auto disable_object_function = std::bind(
      &tmc_collision_detector::ICollisionDetector::DisableObject, coldet_, std::placeholders::_1);
  SetObjectProperty_(object_name, disable_object_function);
}

// Change the group of objects
void RobotCollisionDetector::SetObjectGroup(const std::string& object_name,
                                            uint16_t category) {
  auto set_group_function = std::bind(
      &tmc_collision_detector::ICollisionDetector::SetCollisionGroup, coldet_, category, std::placeholders::_1);
  SetObjectProperty_(object_name, set_group_function);
}

// Return the object group to default
void RobotCollisionDetector::SetObjectDefaultGroup(
    const std::string& object_name) {
  uint16_t category = GetObjectDefaultGroup(object_name);
  auto set_group_function = std::bind(
      &tmc_collision_detector::ICollisionDetector::SetCollisionGroup, coldet_, category, std::placeholders::_1);
  SetObjectProperty_(object_name, set_group_function);
}

// Get an object group
uint16_t RobotCollisionDetector::GetObjectGroup(
    const std::string& object_name) const {
  std::map<std::string, OuterObjectParameters>::const_iterator it;
  it = outer_object_list_.find(object_name);
  if (it == outer_object_list_.end()) {
    return coldet_->GetCollisionGroup(object_name);
  }
  return coldet_->GetCollisionGroup(MakeChildObjectName(object_name, 0));
}

// Get the default group of objects
uint16_t RobotCollisionDetector::GetObjectDefaultGroup(
    const std::string& object_name) const {
  for (std::vector<CuboidSeq>::const_iterator cuboid =
           bounding_box_.boxes.begin();
       cuboid != bounding_box_.boxes.end(); ++cuboid) {
    for (CuboidSeq::const_iterator it = cuboid->begin();
         it != cuboid->end(); ++it) {
      if (object_name == it->box_name) {
        uint32_t distance =
            std::distance(bounding_box_.boxes.begin(), cuboid);
        return robot_collision_config_->GetGroupBitByGroupName(
                   bounding_box_.group_name[distance]);
      }
    }
  }
  return robot_collision_config_->GetGroupBitByObjectName(object_name);
}

// Change the filter of the object
void RobotCollisionDetector::SetObjectFilter(
    const std::string& object_name, uint16_t filter) {
  auto set_filter_function = std::bind(
      &tmc_collision_detector::ICollisionDetector::SetCollisionFilter, coldet_, filter, std::placeholders::_1);
  SetObjectProperty_(object_name, set_filter_function);
}

// Return the filter of the object to the default
void RobotCollisionDetector::SetObjectDefaultFilter(
    const std::string& object_name) {
  uint16_t filter = GetObjectDefaultFilter(object_name);
  auto set_filter_function = std::bind(
      &tmc_collision_detector::ICollisionDetector::SetCollisionFilter, coldet_, filter, std::placeholders::_1);
  SetObjectProperty_(object_name, set_filter_function);
}

// Obtain an object filter
uint16_t RobotCollisionDetector::GetObjectFilter(
    const std::string& object_name) const {
  std::map<std::string, OuterObjectParameters>::const_iterator it;
  it = outer_object_list_.find(object_name);
  if (it == outer_object_list_.end()) {
    return coldet_->GetCollisionFilter(object_name);
  }
  return coldet_->GetCollisionFilter(MakeChildObjectName(object_name, 0));
}

// Get the default filter of the object
uint16_t RobotCollisionDetector::GetObjectDefaultFilter(
    const std::string& object_name) const {
  for (std::vector<CuboidSeq>::const_iterator cuboid =
           bounding_box_.boxes.begin();
       cuboid != bounding_box_.boxes.end(); ++cuboid) {
    for (CuboidSeq::const_iterator it = cuboid->begin();
         it != cuboid->end(); ++it) {
      if (object_name == it->box_name) {
        uint32_t distance =
            std::distance(bounding_box_.boxes.begin(), cuboid);
        return robot_collision_config_->GetFilterBitByGroupName(
                   bounding_box_.group_name[distance]);
      }
    }
  }
  return robot_collision_config_->GetFilterBitByObjectName(object_name);
}

// Add a pair of objects excluded from interference checks
void RobotCollisionDetector::DisableCollisionCheckObjectToObject(
    const std::string& object_name1,
    const std::string& object_name2) {
  coldet_->DisableCollisionCheck(
      GetObjectToObjectPairNameList_(object_name1, object_name2));
}

// Added an object pair to be added to the interference check
void RobotCollisionDetector::EnableCollisionCheckObjectToObject(
    const std::string& object_name1,
    const std::string& object_name2) {
  coldet_->EnableCollisionCheck(
      GetObjectToObjectPairNameList_(object_name1, object_name2));
}

// Add a pair of object groups excluded from interference checks
void RobotCollisionDetector::DisableCollisionCheckObjectToGroup(
    const std::string& object_name,
    const std::string& group_name) {
  coldet_->DisableCollisionCheck(
      GetObjectToGroupPairNameList_(object_name, group_name));
}

// Added an object group pair to add to the interference check
void RobotCollisionDetector::EnableCollisionCheckObjectToGroup(
    const std::string& object_name,
    const std::string& group_name) {
  coldet_->EnableCollisionCheck(
      GetObjectToGroupPairNameList_(object_name, group_name));
}

// Each to exclude from the interference check
// Change the filter of the object belonging to the group
void RobotCollisionDetector::DisableCollisionCheckGroupToGroup(
    const std::string& group_name1, const std::string& group_name2) {
  // Obtain an object name list from the group name
  std::vector<std::string> name_list1(GetObjectNameListByGroup(group_name1));
  std::vector<std::string> name_list2(GetObjectNameListByGroup(group_name2));

  // Change the filter of the object belonging to Group 1
  uint16_t category =
      robot_collision_config_->GetGroupBitByGroupName(group_name2);
  robot_collision_config_->SetConfig(
      group_name1,
      robot_collision_config_->GetGroupBitByGroupName(group_name1),
      robot_collision_config_->
      GetFilterBitByGroupName(group_name1) & ~category);
  for (std::vector<std::string>::iterator it = name_list1.begin();
       it != name_list1.end(); ++it) {
    uint16_t filter = GetObjectFilter(*it);
    filter = filter & ~category;
    SetObjectFilter(*it, filter);
  }

  // Change the filter of the object belonging to Group 2
  category =
      robot_collision_config_->GetGroupBitByGroupName(group_name1);
  robot_collision_config_->SetConfig(
      group_name2,
      robot_collision_config_->GetGroupBitByGroupName(group_name2),
      robot_collision_config_->
      GetFilterBitByGroupName(group_name2) & ~category);
  for (std::vector<std::string>::iterator it = name_list2.begin();
       it != name_list2.end(); ++it) {
    uint16_t filter = GetObjectFilter(*it);
    filter = filter & ~category;
    SetObjectFilter(*it, filter);
  }
}

// Each to add to the interference check
// Change the filter of the object belonging to the group
void RobotCollisionDetector::EnableCollisionCheckGroupToGroup(
    const std::string& group_name1, const std::string& group_name2) {
  // Obtain an object name list from the group name
  std::vector<std::string> name_list1(GetObjectNameListByGroup(group_name1));
  std::vector<std::string> name_list2(GetObjectNameListByGroup(group_name2));

  // Change the filter of the object belonging to Group 1
  uint16_t category =
      robot_collision_config_->GetGroupBitByGroupName(group_name2);
  robot_collision_config_->SetConfig(
      group_name1,
      robot_collision_config_->GetGroupBitByGroupName(group_name1),
      robot_collision_config_->
      GetFilterBitByGroupName(group_name1) | category);
  for (std::vector<std::string>::iterator it = name_list1.begin();
       it != name_list1.end(); ++it) {
    uint16_t filter = GetObjectFilter(*it);
    filter = filter | category;
    SetObjectFilter(*it, filter);
  }

  // Change the filter of the object belonging to Group 2
  category =
      robot_collision_config_->GetGroupBitByGroupName(group_name1);
  robot_collision_config_->SetConfig(
      group_name2,
      robot_collision_config_->GetGroupBitByGroupName(group_name2),
      robot_collision_config_->
      GetFilterBitByGroupName(group_name2) | category);
  for (std::vector<std::string>::iterator it = name_list2.begin();
       it != name_list2.end(); ++it) {
    uint16_t filter = GetObjectFilter(*it);
    filter = filter | category;
    SetObjectFilter(*it, filter);
  }
}

// Set the position posture of the object
void RobotCollisionDetector::SetObjectTransform(
    const std::string& object_name,
    const Eigen::Affine3d& origin_to_object) {
  // Not eligible for internal objects
  if (IsInnerObject_(object_name)) {
    return;
  }
  std::map<std::string, OuterObjectParameters>::iterator it =
      outer_object_list_.find(object_name);
  if (it == outer_object_list_.end()) {
    throw std::domain_error(
        "SetObjectTransform error: non exist object" + object_name);
  }
  for (uint32_t i = 0; i < it->second.base_to_child.size(); i++) {
    coldet_->SetObjectTransform(
        origin_to_object * it->second.base_to_child.at(i),
        MakeChildObjectName(object_name, i));
  }
  if (it->second.cuboid) {
    EraseCuboid_(object_name);
    it->second.cuboid = false;
  }
  it->second.origin_to_base = origin_to_object;
}

// Obtain the position of the object
Eigen::Affine3d RobotCollisionDetector::GetObjectTransform(
    const std::string& object_name) const {
  std::map<std::string, OuterObjectParameters>::const_iterator it;
  it = outer_object_list_.find(object_name);
  if (it != outer_object_list_.end()) {
    return it->second.origin_to_base;
  } else if (object_name.find('#', 0) != std::string::npos) {
    return coldet_->GetObjectTransform(object_name);
  } else {
    return GetObjectTransformFromKinematicsModel_(object_name);
  }
}

// Grab the object
void RobotCollisionDetector::HoldObject(
    const std::string& object_name,
    const std::string& frame_name,
    const Eigen::Affine3d& held_frame_to_object,
    const std::string& held_group_name) {
  // Not eligible for internal objects
  if (IsInnerObject_(object_name)) {
    return;
  }
  // Change the attribute of the object
  std::map<std::string, OuterObjectParameters>::iterator it =
      outer_object_list_.find(object_name);
  if (it == outer_object_list_.end()) {
    throw std::domain_error(
        "HoldObject error: non exist object" + object_name);
  }
  uint16_t filter =
      robot_collision_config_->GetFilterBitByGroupName(held_group_name);
  SetObjectFilter(object_name, filter);
  uint16_t group =
      robot_collision_config_->GetGroupBitByGroupName(held_group_name);
  SetObjectGroup(object_name, group);
  Eigen::Affine3d origin_to_frame(GetObjectTransformFromKinematicsModel_(frame_name));
  it->second.origin_to_base = origin_to_frame * held_frame_to_object;
  for (uint32_t i = 0; i < it->second.base_to_child.size(); i++) {
    std::string child_name(MakeChildObjectName(it->second.name, i));
    // Reflect in the current location
    coldet_->SetObjectTransform(
        it->second.origin_to_base * it->second.base_to_child.at(i), child_name);
  }
  if (it->second.cuboid) {
    EraseCuboid_(object_name);
    it->second.cuboid = false;
  }
  it->second.parent_name = frame_name;
  it->second.parent_to_base = held_frame_to_object;
  it->second.parent_group = held_group_name;
  attached_object_name_.insert(PairString(object_name, held_group_name));

  // For internal interference check
  tmc_manipulation_types::ObjectParameter parameter;
  parameter.filter = filter;
  parameter.group = group;
  parameter.margin = it->second.margin;
  for (uint32_t i = 0; i < it->second.base_to_child.size(); i++) {
    parameter.name = MakeChildObjectName(it->second.name, i);
    parameter.shape = it->second.shape.at(i);
    parameter.transform = it->second.origin_to_base * it->second.base_to_child.at(i);
    inner_coldet_->CreateObject(parameter);
  }
}

// Grab the object
void RobotCollisionDetector::HoldObject(
    const std::string& object_name,
    const std::string& frame_name,
    const Eigen::Affine3d& held_frame_to_object) {
  const auto group_name = robot_collision_config_->GetBelongedGroupName(frame_name);
  HoldObject(object_name, frame_name, held_frame_to_object, group_name);
}

// Release the object you are grabbing
void RobotCollisionDetector::ReleaseObject(const std::string& object_name) {
  if (attached_object_name_.find(object_name) != attached_object_name_.end()) {
    attached_object_name_.erase(object_name);
    // Find cannot fail
    std::map<std::string, OuterObjectParameters>::iterator it_map =
        outer_object_list_.find(object_name);
    for (uint32_t i = 0; i < it_map->second.base_to_child.size(); i++) {
      inner_coldet_->DestroyObject(MakeChildObjectName(object_name, i));
    }
    SetObjectDefaultGroup(object_name);
    SetObjectDefaultFilter(object_name);
  } else {
    throw std::domain_error(
        "ReleaseObject error: not held object " + object_name);
  }
}

// Release the objects you are grabbing
void RobotCollisionDetector::ReleaseAllObject(void) {
  // Erase Held_object_name__ in ReleaseObject, so copy it after copying
  std::vector<std::string> all_object;
  for (std::map<std::string, std::string>::iterator it =
           attached_object_name_.begin();
       it != attached_object_name_.end(); ++it) {
    all_object.push_back(it->first);
  }
  for (std::vector<std::string>::iterator it = all_object.begin();
       it != all_object.end(); ++it) {
    ReleaseObject(*it);
  }

  // Set of internal interference checker
  inner_coldet_->DestroyObject();
  std::vector<PairString> disable_object_pairs(
      robot_collision_config_->GetDisableObjectPairList());
  inner_coldet_->DisableCollisionCheck(disable_object_pairs);
}

// Check interference and check the check after detecting interference
bool RobotCollisionDetector::CheckCollision() {
  return coldet_->CheckCollisionSpace();
}

// Interference check, output the name of the interference object
bool RobotCollisionDetector::CheckCollision(
    bool end_flag,
    std::vector<PairString>& dst_contact_pair) {
  dst_contact_pair.clear();
  if (end_flag) {
    PairString object_pair;
    bool contact_result = coldet_->CheckCollisionSpace(object_pair);
    if (contact_result) {
      dst_contact_pair.push_back(object_pair);
    }
    return contact_result;
  } else {
    bool contact_result = coldet_->GetContactPairList(dst_contact_pair);
    return contact_result;
  }
}

// Explore a nearby object for each robot configuration site
bool RobotCollisionDetector::CheckClosestObject(
    double extend_length,
    int32_t top_n,
    std::vector<ClosestObject>& dst_result_list) {
  // Prepare
  dst_result_list.clear();
  bool contact = false;
  UpdateInnerModel_();

  for (std::vector<std::string>::iterator it = robot_parts_name_.begin();
       it != robot_parts_name_.end(); ++it) {
    ClosestObject result;
    result.name = *it;
    result.inner_result = inner_coldet_->GetClosestObject(
        *it, extend_length, top_n,
        robot_collision_config_->GetInnerFilterBit(*it));
    result.outer_result = coldet_->GetClosestObject(
        *it, extend_length, top_n,
        robot_collision_config_->GetGroupBitByGroupName(
            tmc_robot_collision_detector::kOuterGroupName));
    dst_result_list.push_back(result);
    if (result.outer_result.contact || result.inner_result.contact) {
      contact = true;
    }
  }
  // Objects grabbing
  for (std::map<std::string, std::string>::iterator it =
           attached_object_name_.begin();
       it != attached_object_name_.end(); ++it) {
    std::map<std::string, OuterObjectParameters>::iterator it_map;
    it_map = outer_object_list_.find(it->first);
    for (uint32_t i = 0; i < it_map->second.base_to_child.size(); i++) {
      ClosestObject result;
      std::string child_name(MakeChildObjectName(it->first, i));
      result.name = child_name;
      result.inner_result = inner_coldet_->GetClosestObject(
          child_name, extend_length, top_n, GetObjectFilter(it->first));
      result.outer_result = coldet_->GetClosestObject(
          child_name, extend_length, top_n,
          robot_collision_config_->GetGroupBitByGroupName(
              tmc_robot_collision_detector::kOuterGroupName));
      dst_result_list.push_back(result);
      if (result.outer_result.contact || result.inner_result.contact) {
        contact = true;
      }
    }
  }
  return contact;
}

// Search for Object_name nearby objects
bool RobotCollisionDetector::CheckClosestObject(
    const std::string& object_name,
    double extend_length, int32_t top_n,
    ClosestObject& dst_result) {
  // Update of robot model for INNER
  UpdateInnerModel_();

  // When Object_name is a robot site
  for (std::vector<std::string>::iterator it = robot_parts_name_.begin();
      it != robot_parts_name_.end(); ++it) {
    if (*it == object_name) {
      dst_result.name = *it;
      dst_result.inner_result = inner_coldet_->GetClosestObject(
          *it, extend_length, top_n,
          robot_collision_config_->GetInnerFilterBit(*it));
      dst_result.outer_result = coldet_->GetClosestObject(
          *it, extend_length, top_n,
          robot_collision_config_->GetGroupBitByGroupName(
              tmc_robot_collision_detector::kOuterGroupName));
      return dst_result.inner_result.contact | dst_result.outer_result.contact;
    }
  }
  // For objects that Object_name grabbed
  for (std::map<std::string, std::string>::iterator it =
           attached_object_name_.begin();
       it != attached_object_name_.end(); ++it) {
    if (it->first == object_name) {
      dst_result.name = it->first;
      dst_result.inner_result.distance = extend_length;
      dst_result.outer_result.distance = extend_length;
      std::map<std::string, OuterObjectParameters>::iterator it_map;
      it_map = outer_object_list_.find(it->first);
      for (uint32_t i = 0; i < it_map->second.base_to_child.size(); i++) {
        ClosestObject result;
        std::string child_name(MakeChildObjectName(it->first, i));
        result.inner_result = inner_coldet_->GetClosestObject(
            child_name, extend_length, top_n, GetObjectFilter(it->first));
        result.outer_result = coldet_->GetClosestObject(
            child_name, extend_length, top_n,
            robot_collision_config_->GetGroupBitByGroupName(
                tmc_robot_collision_detector::kOuterGroupName));
        if (result.inner_result.distance < dst_result.inner_result.distance) {
          dst_result.inner_result = result.inner_result;
        }
        if (result.outer_result.distance < dst_result.outer_result.distance) {
          dst_result.outer_result = result.outer_result;
        }
      }
      return dst_result.inner_result.contact | dst_result.outer_result.contact;
    }
  }
  // When Object_name is an external object
  std::map<std::string, OuterObjectParameters>::iterator it;
  it = outer_object_list_.find(object_name);
  if (it == outer_object_list_.end()) {
    throw std::domain_error(
        "CheckClosestObject error: non exist object " + object_name);
  }
  for (uint32_t i = 0; i < it->second.base_to_child.size(); i++) {
    std::string child_name(MakeChildObjectName(it->first, i));
    coldet_->DisableObject(child_name);
  }
  dst_result.name = it->first;
  dst_result.inner_result.distance = extend_length;
  dst_result.outer_result.distance = extend_length;
  for (uint32_t i = 0; i < it->second.base_to_child.size(); i++) {
    std::string child_name(MakeChildObjectName(it->first, i));
    coldet_->EnableObject(child_name);
    ClosestObject result;
    result.inner_result = coldet_->GetClosestObject(
        child_name, extend_length, top_n,
        0xFFFF - robot_collision_config_->GetGroupBitByGroupName(
            tmc_robot_collision_detector::kOuterGroupName));
    result.outer_result = coldet_->GetClosestObject(
        child_name, extend_length, top_n,
        robot_collision_config_->GetGroupBitByGroupName(
            tmc_robot_collision_detector::kOuterGroupName));
    if (result.inner_result.distance < dst_result.inner_result.distance) {
      dst_result.inner_result = result.inner_result;
    }
    if (result.outer_result.distance < dst_result.outer_result.distance) {
      dst_result.outer_result = result.outer_result;
    }
    coldet_->DisableObject(child_name);
  }
  for (uint32_t i = 0; i < it->second.base_to_child.size(); i++) {
    std::string child_name(MakeChildObjectName(it->first, i));
    coldet_->EnableObject(child_name);
  }
  return dst_result.inner_result.contact | dst_result.outer_result.contact;
}

// Obtain AABB of object
AABB RobotCollisionDetector::GetObjectAABB(
    const std::string& object_name) const {
  std::map<std::string, OuterObjectParameters>::const_iterator it =
      outer_object_list_.find(object_name);
  if (it != outer_object_list_.end()) {
    AABB return_aabb;
    return_aabb = coldet_->GetObjectAABB(MakeChildObjectName(object_name, 0));
    for (uint32_t i = 1; i < it->second.base_to_child.size(); i++) {
      return_aabb = CombineAABB(return_aabb,
                                coldet_->GetObjectAABB(
                                    MakeChildObjectName(object_name, i)));
    }
    return return_aabb;
  } else {
    return coldet_->GetObjectAABB(object_name);
  }
}

// Get AABB for the entire robot
AABB RobotCollisionDetector::GetRobotAABB() const {
  AABB return_aabb;
  return_aabb << DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX;
  for (std::vector<std::string>::const_iterator it = robot_parts_name_.begin();
       it != robot_parts_name_.end(); ++it) {
    return_aabb = CombineAABB(return_aabb, coldet_->GetObjectAABB(*it));
  }
  for (std::map<std::string, std::string>::const_iterator attached =
           attached_object_name_.begin();
       attached != attached_object_name_.end(); ++attached) {
    return_aabb = CombineAABB(return_aabb,
                              coldet_->GetObjectAABB(attached->first));
  }
  return return_aabb;
}

/// Enable only Cuboid that overlaps with robots
void RobotCollisionDetector::RefleshOverlappedCuboids(
    CuboidOverlapType overlap_type,
    CuboidOverlapGroupType group_type) {
  // Acquired AABB
  AABBSeq aabb;
  switch (group_type) {
    case kOverlapRobot: {
      aabb.push_back(GetRobotAABB());
    }
      break;
    case kOverlapGroup: {
      std::vector<std::string> group_list =
          robot_collision_config_->GetRobotPartsGroupNameList();
      for (std::vector<std::string>::iterator group = group_list.begin();
           group != group_list.end(); ++group) {
        std::vector<std::string> object_list =
            GetObjectNameListByGroup(*group);
        AABB add_aabb;
        add_aabb << DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX;
        for (std::vector<std::string>::iterator it = object_list.begin();
             it != object_list.end(); ++it) {
          add_aabb = CombineAABB(add_aabb, GetObjectAABB(*it));
        }
        aabb.push_back(add_aabb);
      }
    }
      break;
    default:
      throw std::domain_error(
          "RefleshOverlappedCuboids error: invalid overlap type");
  }
  // Switch to enable and Disable
  for (std::vector<CuboidSeq>::const_iterator cuboid =
           bounding_box_.boxes.begin();
       cuboid != bounding_box_.boxes.end(); ++cuboid) {
    for (CuboidSeq::const_iterator box = cuboid->begin();
         box != cuboid->end(); ++box) {
      bool is_overlap = false;
      for (AABBSeq::iterator it_aabb = aabb.begin();
           it_aabb != aabb.end(); ++it_aabb) {
        switch (overlap_type) {
          case kOverlap2DMap: {
            is_overlap = CheckXYPlainOverlap(*it_aabb, box->box_aabb);
          }
            break;
          case kOverlapAabb: {
            is_overlap = CheckAABBOverlap(*it_aabb, box->box_aabb);
          }
            break;
          default:
            throw std::domain_error(
                "RefleshOverlappedCuboids error: "
                "invalid overlap group type");
        }
        if (!box->enable && is_overlap) {
          box->enable = true;
          coldet_->EnableObject(MakeChildObjectName(box->box_name, 0));
          break;
        } else if (box->enable && is_overlap) {
          break;
        }
      }
      if (box->enable && !is_overlap) {
        coldet_->DisableObject(MakeChildObjectName(box->box_name, 0));
        box->enable = false;
      }
    }
  }
}

// Cuboid belonging to the group makes interference check enabled
void RobotCollisionDetector::EnableCuboids(const std::string& group_name) {
  // Check if Group_name exists
  std::vector<std::string> group_list(
      robot_collision_config_->GetGroupNameList());
  std::vector<std::string>::const_iterator group(
      std::find(group_list.begin(), group_list.end(), group_name));
  if (group == group_list.end()) {
    throw std::domain_error(
        "EnableCuboids error: invalid group name " + group_name);
  }
  // Enable cuboid
  std::vector<std::string>::iterator cuboid_group_name =
      std::find(bounding_box_.group_name.begin(),
                bounding_box_.group_name.end(), group_name);
  if (cuboid_group_name != bounding_box_.group_name.end()) {
    uint32_t distance =
        std::distance(bounding_box_.group_name.begin(), cuboid_group_name);
    for (CuboidSeq::const_iterator it = bounding_box_.boxes[distance].begin();
         it != bounding_box_.boxes[distance].end(); ++it) {
      it->enable = true;
      coldet_->EnableObject(MakeChildObjectName(it->box_name, 0));
    }
  }
}

// Interference check of Cuboid belonging to the group is invalidated
void RobotCollisionDetector::DisableCuboids(const std::string& group_name) {
  // Check if Group_name exists
  std::vector<std::string> group_list(
      robot_collision_config_->GetGroupNameList());
  std::vector<std::string>::const_iterator group(
      std::find(group_list.begin(), group_list.end(), group_name));
  if (group == group_list.end()) {
    throw std::domain_error(
        "DisableCuboids error: invalid group name " + group_name);
  }
  // Disable Cuboid
  std::vector<std::string>::iterator cuboid_group_name =
      std::find(bounding_box_.group_name.begin(),
                bounding_box_.group_name.end(), group_name);
  if (cuboid_group_name != bounding_box_.group_name.end()) {
    uint32_t distance =
        std::distance(bounding_box_.group_name.begin(), cuboid_group_name);
    for (CuboidSeq::const_iterator it = bounding_box_.boxes[distance].begin();
         it != bounding_box_.boxes[distance].end(); ++it) {
      it->enable = false;
      coldet_->DisableObject(MakeChildObjectName(it->box_name, 0));
    }
  }
}

// For test/evaluation that returns the number of cuboids that are enable
uint32_t RobotCollisionDetector::GetEnableCuboidsNum() {
  uint32_t count = 0;
  for (std::vector<CuboidSeq>::const_iterator cuboid =
           bounding_box_.boxes.begin();
       cuboid != bounding_box_.boxes.end(); ++cuboid) {
    for (CuboidSeq::const_iterator it = cuboid->begin();
         it != cuboid->end(); ++it) {
      if (it->enable) {
        ++count;
      }
    }
  }
  return count;
}

// Obtain the name of all objects in the interference check space
std::vector<std::string> RobotCollisionDetector::GetObjectNameList() const {
  std::vector<std::string> name_list(robot_parts_name_);
  name_list.reserve(name_list.size() + outer_object_list_.size());
  for (std::map<std::string, OuterObjectParameters>::const_iterator it =
       outer_object_list_.begin(); it != outer_object_list_.end(); ++it) {
    name_list.push_back(it->first);
  }
  return name_list;
}

// Get the names of all objects belonging to the group
std::vector<std::string> RobotCollisionDetector::GetObjectNameListByGroup(
    const std::string& group_name) const {
  // Check if Group_name exists
  std::vector<std::string> group_list(
      robot_collision_config_->GetGroupNameList());
  std::vector<std::string>::const_iterator group(
      std::find(group_list.begin(), group_list.end(), group_name));
  if (group == group_list.end()) {
    throw std::domain_error(
        "GetObjectNameListByGroup error: invalid group name " + group_name);
  }
  // Get the default value of the object belonging to the group
  std::vector<std::string> name_list;
  name_list.reserve(outer_object_list_.size() + robot_parts_name_.size());
  // Processing of knowledge objects
  for (std::map<std::string, std::string>::const_iterator it =
           attached_object_name_.begin();
       it != attached_object_name_.end(); ++it) {
    if (it->second == group_name) {
      name_list.push_back(it->first);
    }
  }
  // For external objects
  if (group_name == kOuterGroupName) {
    for (OuterObjectMap::const_iterator it = outer_object_list_.begin();
         it != outer_object_list_.end(); ++it) {
      // Ignore Cuboid
      if (it->second.cuboid) {
        continue;
      }
      // Ignore the knowledge object
      if (attached_object_name_.find(it->first) !=
          attached_object_name_.end()) {
        continue;
      }
      name_list.push_back(it->first);
    }
  }
  // For Cuboid
  std::vector<std::string>::const_iterator cuboid_group_name =
      std::find(bounding_box_.group_name.begin(),
                bounding_box_.group_name.end(), group_name);
  if (cuboid_group_name != bounding_box_.group_name.end()) {
    uint32_t distance =
        std::distance(bounding_box_.group_name.begin(), cuboid_group_name);
    for (CuboidSeq::const_iterator it = bounding_box_.boxes[distance].begin();
         it != bounding_box_.boxes[distance].end(); ++it) {
      name_list.push_back(it->box_name);
    }
  }
  // In the case of a robot site
  group_list = robot_collision_config_->GetRobotPartsGroupNameList();
  std::vector<std::string>::const_iterator parts_group(
      std::find(group_list.begin(), group_list.end(), group_name));
  if (parts_group != group_list.end()) {
    std::vector<std::string> parts_list =
        robot_collision_config_->GetObjectListInGroup(group_name);
    name_list.insert(name_list.end(), parts_list.begin(), parts_list.end());
  }
  return name_list;
  // Simple and correct implementation, but slow
  // Find what the group is GROUP_NAME
//  uint16_t group_bit =
//      robot_collision_config_->GetGroupBitByGroupName(group_name);
//  for (std::vector<std::string>::const_iterator it =
//           robot_parts_name_.begin();
//       it != robot_parts_name_.end(); ++it) {
//    if ((group_bit & GetObjectGroup(*it)) == group_bit) {
//      name_list.push_back(*it);
//    }
//  }
//  for (OuterObjectMap::const_iterator it = outer_object_list_.begin();
//       it != outer_object_list_.end(); ++it) {
//    if ((group_bit & GetObjectGroup(it->first)) == group_bit) {
//      name_list.push_back(it->first);
//    }
//  }
//  return name_list;
}

// Get the list of object names you know
std::vector<std::string> RobotCollisionDetector::GetHeldObjectList() const {
  std::vector<std::string> name_list;
  name_list.reserve(attached_object_name_.size());
  for (std::map<std::string, std::string>::const_iterator it =
           attached_object_name_.begin();
       it != attached_object_name_.end(); ++it) {
    name_list.push_back(it->first);
  }
  return name_list;
}

// Set the attribute of the object
void RobotCollisionDetector::SetObjectProperty_(
    const std::string& object_name,
    std::function<void(const std::string&)> set_property_function) {
  std::map<std::string, OuterObjectParameters>::iterator it =
      outer_object_list_.find(object_name);
  if (it == outer_object_list_.end()) {
    set_property_function(object_name);
  } else {
    if (it->second.cuboid) {
      EraseCuboid_(object_name);
      it->second.cuboid = false;
    }
    for (uint32_t i = 0; i < it->second.base_to_child.size(); i++) {
      set_property_function(MakeChildObjectName(object_name, i));
    }
  }
}

// Check if it is an internal object
bool RobotCollisionDetector::IsInnerObject_(const std::string& object_name) {
  // Is it a robot site?
  std::vector<std::string>::iterator robot_part =
      std::find(robot_parts_name_.begin(), robot_parts_name_.end(),
                object_name);
  if (robot_part != robot_parts_name_.end()) {
    return true;
  }
  // Is it an identified body?
  if (attached_object_name_.find(object_name) != attached_object_name_.end()) {
    return true;
  }
  return false;
}

// Check if it is a child object
bool RobotCollisionDetector::IsChildObject_(const std::string& object_name) {
  // #Take out the front part
  const auto split_pos = object_name.find_last_of("#");
  std::string parent_name = "";
  if (split_pos != std::string::npos) {
    parent_name = object_name.substr(0, split_pos);
  }
  std::map<std::string, OuterObjectParameters>::iterator it =
      outer_object_list_.find(parent_name);
  if (it == outer_object_list_.end()) {
    return false;
  } else {
    int32_t number = std::stoi(object_name.substr(split_pos + 1));
    if (number < 0) {
      return false;
    } else if (it->second.base_to_child.size() >
               static_cast<uint32_t>(number)) {
      return true;
    } else {
      return false;
    }
  }
}

// Update the interference check model for checking the vicinity of the internal object
void RobotCollisionDetector::UpdateInnerModel_(void) {
  for (std::vector<std::string>::iterator it = robot_parts_name_.begin();
       it != robot_parts_name_.end(); ++it) {
    inner_coldet_->SetObjectTransform(coldet_->GetObjectTransform(*it), *it);
  }
  for (std::map<std::string, std::string>::iterator it =
           attached_object_name_.begin();
       it != attached_object_name_.end(); ++it) {
    std::map<std::string, OuterObjectParameters>::iterator it_map;
    it_map = outer_object_list_.find(it->first);
    it_map->second.origin_to_base = GetObjectTransform(
        it_map->second.parent_name) * it_map->second.parent_to_base;
    for (uint32_t i = 0; i < it_map->second.base_to_child.size(); i++) {
      std::string child_name(MakeChildObjectName(it->first, i));
      inner_coldet_->SetObjectTransform(
          it_map->second.origin_to_base * it_map->second.base_to_child.at(i),
          child_name);
    }
  }
}

// Update the model for interference checks based on the robot athletic model
void RobotCollisionDetector::UpdateCollisionModel_(void) {
  // Update of robot site
  for (std::vector<std::string>::iterator it = robot_parts_name_.begin();
       it != robot_parts_name_.end(); ++it) {
    coldet_->SetObjectTransform(GetObjectTransformFromKinematicsModel_(*it), *it);
  }
  // Update the object you are grabbing
  for (std::map<std::string, std::string>::iterator it =
           attached_object_name_.begin();
       it != attached_object_name_.end(); ++it) {
    std::map<std::string, OuterObjectParameters>::iterator it_map;
    it_map = outer_object_list_.find(it->first);
    Eigen::Affine3d origin_to_parent_frame(
        GetObjectTransformFromKinematicsModel_(it_map->second.parent_name));
    Eigen::Affine3d origin_to_base(
        origin_to_parent_frame * it_map->second.parent_to_base);
    for (uint32_t i = 0; i < it_map->second.base_to_child.size(); i++) {
      std::string child_name(MakeChildObjectName(it->first, i));
      coldet_->SetObjectTransform(
          origin_to_base * it_map->second.base_to_child.at(i), child_name);
    }
    it_map->second.origin_to_base = origin_to_base;
  }
}

// Create an object and object object name pair list
std::vector<PairString> RobotCollisionDetector::GetObjectToObjectPairNameList_(
    const std::string& object_name1, const std::string& object_name2) {
  // Obtain an object name
  std::vector<std::string> name_list1(GetChildObjectNameList_(object_name1));
  std::vector<std::string> name_list2(GetChildObjectNameList_(object_name2));
  // Create an object name pair
  std::vector<PairString> pair_name_list;
  pair_name_list.reserve(name_list1.size() * name_list2.size());
  for (std::vector<std::string>::iterator name1 = name_list1.begin();
       name1 != name_list1.end(); ++name1) {
    for (std::vector<std::string>::iterator name2 = name_list2.begin();
         name2 != name_list2.end(); ++name2) {
      pair_name_list.push_back(PairString(*name1, *name2));
    }
  }
  return pair_name_list;
}

// Create an object and group object name pair list
std::vector<PairString> RobotCollisionDetector::GetObjectToGroupPairNameList_(
    const std::string& object_name,
    const std::string& group_name) {
  // Obtain the name of the object belonging to the group
  std::vector<std::string> name_in_group(GetObjectNameListByGroup(group_name));

  // Create an object name pair
  std::vector<PairString> pair_name_list;
  for (std::vector<std::string>::iterator it = name_in_group.begin();
       it != name_in_group.end(); ++it) {
    std::vector<PairString> tmp_name_list(
        GetObjectToObjectPairNameList_(object_name, *it));
    pair_name_list.insert(pair_name_list.end(),
                          tmp_name_list.begin(), tmp_name_list.end());
  }
  return pair_name_list;
}

// Get the child object name from the object name
std::vector<std::string> RobotCollisionDetector::GetChildObjectNameList_(
    const std::string& object_name) {
  std::vector<std::string> name_list;
  std::map<std::string, OuterObjectParameters>::iterator it =
      outer_object_list_.find(object_name);
  if (it != outer_object_list_.end()) {
    // If you are an external object, decompose into child object name
    for (uint32_t i = 0; i < it->second.base_to_child.size(); i++) {
      name_list.push_back(MakeChildObjectName(object_name, i));
    }
    if (it->second.cuboid) {
      EraseCuboid_(object_name);
      it->second.cuboid = false;
    }
    return name_list;
  } else if (IsInnerObject_(object_name) || IsChildObject_(object_name)) {
    // Robot parts and child objects do not decompose
    name_list.push_back(object_name);
    return name_list;
  }
  throw std::domain_error(
      "GetChildObjectNameList_ error: invalid object " + object_name);
}

// Erase from Bounding_box_
void RobotCollisionDetector::EraseCuboid_(const std::string& box_name) {
  for (std::vector<CuboidSeq>::iterator cuboids = bounding_box_.boxes.begin();
       cuboids != bounding_box_.boxes.end(); ++cuboids) {
    for (CuboidSeq::iterator it = cuboids->begin();
         it != cuboids->end(); ++it) {
      if (it->box_name == box_name) {
        cuboids->erase(it);
        coldet_->SetCollisionGroup(
            robot_collision_config_->GetGroupBitByGroupName(kOuterGroupName),
            MakeChildObjectName(box_name, 0));
        coldet_->SetCollisionFilter(
            robot_collision_config_->GetFilterBitByGroupName(kOuterGroupName),
            MakeChildObjectName(box_name, 0));
        return;
      }
    }
  }
  throw std::domain_error(
      "EraseCuboid_ error: non-exist bounding box " + box_name);
}

// Obtain information on robot sites from URDF files
ObjectParameterSeq RobotCollisionDetector::GetRobotPartsShape_(
    const std::string& robot_model_config,
    ModelFileType model_file_type) {
  ObjectParameterSeq parameters;

  switch (model_file_type) {
    case kUrdf: {
      std::shared_ptr<ModelInterface> robot = parseURDF(robot_model_config);
      if (!robot) {
        throw std::domain_error(
            "GetRobotPartsShape_ error: not open robot model");
      }
      // get links
      std::vector<std::shared_ptr<urdf::Link> > links;
      robot->getLinks(links);
      for (const auto& link : links) {
        uint32_t i = 0;
        for (const auto& collision : link->collision_array) {
          std::string name;
          // Link name+Collision+Shake without permission
          name = link->name + "/collision/" + std::to_string(i);
          ++i;
          parameters.push_back(
              ConvertUrdfShapeToCollisionObject(collision, name));

          CollisionFrameInfo info;
          info.parent_name = link->name;
          info.parent_to_child = Eigen::Translation3d(collision->origin.position.x,
                                                      collision->origin.position.y,
                                                      collision->origin.position.z)
                               * Eigen::Quaterniond(collision->origin.rotation.w,
                                                    collision->origin.rotation.x,
                                                    collision->origin.rotation.y,
                                                    collision->origin.rotation.z);
          collision_frame_info_map_[name] = info;
        }
      }
      break;
    }
    default:
      throw std::domain_error(
          "Model file type is invalid");
      break;
  }
  return parameters;
}

void RobotCollisionDetector::CreateRobotModel_(
    const std::string& model_config,
    ModelFileType model_file_type) {
  // Robot model generation for interference check
  ObjectParameterSeq object_parameter_list;
  object_parameter_list = GetRobotPartsShape_(model_config, model_file_type);
  for (ObjectParameterSeq::iterator it = object_parameter_list.begin();
       it != object_parameter_list.end(); ++it) {
    it->transform = GetObjectTransformFromKinematicsModel_(it->name);
    it->group = robot_collision_config_->GetGroupBitByObjectName(it->name);
    it->filter = robot_collision_config_->GetFilterBitByObjectName(it->name);
    robot_parts_name_.push_back(it->name);
    coldet_->CreateObject(*it);
    it->filter = robot_collision_config_->GetInnerFilterBit(it->name);
    inner_coldet_->CreateObject(*it);
  }

  // Anchor set for DestroyallOuterobject
  coldet_->SetAnchor();
  inner_coldet_->SetAnchor();

  // Set an object to be removed from the interference check
  std::vector<PairString> disable_object_pairs(
      robot_collision_config_->GetDisableObjectPairList());
  coldet_->DisableCollisionCheck(disable_object_pairs);
  inner_coldet_->DisableCollisionCheck(disable_object_pairs);
}

Eigen::Affine3d RobotCollisionDetector::GetObjectTransformFromKinematicsModel_(
    const std::string& object_name) const {
  const auto it = collision_frame_info_map_.find(object_name);
  if (it == collision_frame_info_map_.end()) {
    return robot_model_->GetObjectTransform(object_name);
  } else {
    return robot_model_->GetObjectTransform(it->second.parent_name) * it->second.parent_to_child;
  }
}
}  // namespace tmc_robot_collision_detector
