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
/// @file robot_collision_detector.hpp
/// @brief Perform interference checks using a robot model
#ifndef TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_HPP_
#define TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_HPP_

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

// Pinocchio needs to include before Boost
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include <Eigen/Core>  // NOLINT

#include <tmc_collision_detector/collision_detector.hpp>
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_collision_detector/robot_collision_detector_common.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>
#include "tmc_robot_collision_detector/collision_detector_config.hpp"
#include "tmc_robot_collision_detector/object_for_dump.hpp"

#if defined(__GNUC__)
#define DEPRECATED __attribute__((deprecated))
#elif defined(_WIN32)
#define DEPRECATED __declspec(deprecated)
#else
#define DEPRECATED
#endif

namespace tmc_robot_collision_detector {

/// CollisionDetector for robots
class RobotCollisionDetector {
 public:
  using Ptr = std::shared_ptr<RobotCollisionDetector>;

  /// @brief constructor.Various initialization
  RobotCollisionDetector(const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot_model,
                         const std::string& robot_model_config,
                         const std::string& robot_collision_config,
                         const std::string& engine);

  /// @brief constructor.Various initialization
  RobotCollisionDetector(const std::string& robot_model_config,
                         const std::string& robot_collision_config,
                         const std::string& engine);

  /// @brief constructor.Various initialization
  RobotCollisionDetector(const std::string& robot_model_config,
                         const std::string& robot_collision_config,
                         const std::string& engine,
                         ModelFileType model_file_type);

  virtual ~RobotCollisionDetector() {}

  /// @brief Enter the angle information
  void SetRobotNamedAngle(
      const tmc_manipulation_types::JointState& named_angle);

  /// @brief Obtained angle information
  tmc_manipulation_types::JointState GetRobotNamedAngle(void) const  {
    return robot_model_->GetNamedAngle();
  }

  /// @brief Obtained angle information
  tmc_manipulation_types::JointState GetRobotNamedAngle(
      const tmc_manipulation_types::NameSeq& name) const {
    return robot_model_->GetNamedAngle(name);
  }

  /// @brief Get joint min and MAX
  void GetRobotAngleMinMax(
      const tmc_manipulation_types::NameSeq& use_joints,
      Eigen::VectorXd& min,
      Eigen::VectorXd& max) const {
    robot_model_->GetMinMax(use_joints, min, max);
  }

  /// @brief Enter the position posture of the robot
  void SetRobotTransform(const Eigen::Affine3d& origin_to_robot);

  /// @brief Acquired the position posture of the robot
  Eigen::Affine3d GetRobotTransform(void) const {
    return robot_model_->GetRobotTransform();
  }

  /// @brief Creating external objects
  void CreateOuterObject(
      const tmc_manipulation_types::OuterObjectParameters& parameters);

  /// @brief Build an environment consisting of CUBOID
  void CreateCuboids(const tmc_manipulation_types::CuboidSeq& map,
                     bool enable_flag);

  /// @brief Build an environment consisting of CUBOID
  void CreateCuboids(const tmc_manipulation_types::CuboidSeq& map,
                     bool enable_flag,
                     const std::string& cuboid_group_name);

  /// @brief Discard external objects
  void DestroyOuterObject(const std::string& object_name);

  /// @brief Discard all external objects
  void DestroyAllOuterObject(void);

  /// @brief Discard the environment consisting of CUBOID
  void DestroyCuboids();

  /// @brief Obtain information on objects
  tmc_manipulation_types::OuterObjectParameters GetObjectParameter(
      const std::string& object_name) const;

  /// @brief Get information on all external objects
  tmc_manipulation_types::OuterObjectParametersSeq
  GetAllOuterObjectParameters(void) const;

  /// @brief Enable object interference checks
  void EnableCollisionObject(const std::string& object_name);

  /// @brief Disable object interference checks
  void DisableCollisionObject(const std::string& object_name);

  /// @brief Change the group of objects
  void SetObjectGroup(const std::string& object_name, uint16_t category);

  /// @brief Return the object group to default
  void SetObjectDefaultGroup(const std::string& object_name);

  /// @brief Get an object group
  uint16_t GetObjectGroup(const std::string& object_name) const;

  /// @brief Get the default group of objects
  uint16_t GetObjectDefaultGroup(const std::string& object_name) const;

  /// @brief Change the filter of the object
  void SetObjectFilter(const std::string& object_name, uint16_t filter);

  /// @brief Return the filter of the object to the default
  void SetObjectDefaultFilter(const std::string& object_name);

  /// @brief Obtain an object filter
  uint16_t GetObjectFilter(const std::string& object_name) const;

  /// @brief Get the default filter of the object
  uint16_t GetObjectDefaultFilter(const std::string& object_name) const;

  /// @brief Add a pair of objects excluded from interference checks
  void DisableCollisionCheckObjectToObject(const std::string& object_name1,
                                           const std::string& object_name2);

  /// @brief Added an object pair to be added to the interference check
  void EnableCollisionCheckObjectToObject(const std::string& object_name1,
                                          const std::string& object_name2);

  /// @brief Add a pair of object groups excluded from interference checks
  void DisableCollisionCheckObjectToGroup(const std::string& object_name,
                                          const std::string& group_name);

  /// @brief Added an object group pair to add to the interference check
  void EnableCollisionCheckObjectToGroup(const std::string& object_name,
                                         const std::string& group_name);

  /// @brief Belongs to each group to exclude from interference checks
  ///        Change the filter of the object
  void DisableCollisionCheckGroupToGroup(const std::string& group_name1,
                                         const std::string& group_name2);

  /// @brief Each to add to the interference check
  ///        Change the filter of the object belonging to the group
  void EnableCollisionCheckGroupToGroup(const std::string& group_name1,
                                        const std::string& group_name2);

  /// @brief Set the position posture of the object
  void SetObjectTransform(const std::string& object_name,
                          const Eigen::Affine3d& origin_to_object);

  /// @brief Obtain the position of the object
  Eigen::Affine3d GetObjectTransform(const std::string& object_name) const;

  /// @brief Grab the object
  void HoldObject(const std::string& object_name,
                  const std::string& frame_name,
                  const Eigen::Affine3d& held_frame_to_object,
                  const std::string& held_group_name);

  /// @brief The group of the object to grab and grab the object is the same as Frame_name
  void HoldObject(const std::string& object_name,
                  const std::string& frame_name,
                  const Eigen::Affine3d& held_frame_to_object);

  /// @brief Release the object you are grabbing
  void ReleaseObject(const std::string& object_name);

  /// @brief  Release the objects you are grabbing
  void ReleaseAllObject(void);

  /// @brief Check interference and check the check after detecting interference
  virtual bool CheckCollision();

  /// @brief Interference check, output the name of the interference object
  bool CheckCollision(bool end_flag,
                      std::vector<PairString>& dst_contact_pair);

  /// @brief Explore a nearby object for each robot configuration site
  bool CheckClosestObject(double extend_length, int32_t top_n,
                          std::vector<ClosestObject>& dst_result_list);

  /// @brief Search for Object_name nearby objects
  bool CheckClosestObject(const std::string& object_name, double extend_length,
                          int32_t top_n, ClosestObject& dst_result);

  /// @brief Check if the two objects are interfering
  bool CheckCollisionPair(
      const std::string& nameA, const std::string& nameB,
      Eigen::Vector3d& dst_point, Eigen::Vector3d& dst_normal) {
    return coldet_->CheckCollisionPair(nameA, nameB, dst_point, dst_normal);
  }

  /// @brief Ray casting function
  bool RayCasting(const Eigen::Vector3d& start_point,
                  const Eigen::Vector3d& direction,
                  double length,
                  Eigen::Vector3d& dst_end_point,
                  std::string& dst_name) {
    return coldet_->RayCasting(start_point, direction,
                               length, dst_end_point, dst_name);
  }

  /// @brief Obtain AABB of object
  tmc_manipulation_types::AABB GetObjectAABB(
      const std::string& object_name) const;

  /// @brief Get AABB for the entire robot
  tmc_manipulation_types::AABB GetRobotAABB() const;

  /// @brief Enable only Cuboid that overlaps with robots
  void RefleshOverlappedCuboids(CuboidOverlapType overlap_type,
                                CuboidOverlapGroupType group_type);

  /// @brief Cuboid belonging to the group makes interference check enabled
  void EnableCuboids(const std::string& group_name);

  /// @brief Interference check of Cuboid belonging to the group is invalidated
  void DisableCuboids(const std::string& group_name);

  /// @brief For test/evaluation that returns the number of cuboids that are enable
  uint32_t GetEnableCuboidsNum();

  /// @brief Obtain the name of all objects in the interference check space
  std::vector<std::string> GetObjectNameList() const;

  /// @brief Get the names of all objects belonging to the group
  std::vector<std::string> GetObjectNameListByGroup(
      const std::string& group_name) const;

  /// @brief Get a list of group names
  std::vector<std::string> GetGroupList() const {
    return robot_collision_config_->GetGroupNameList();
  }

  /// @brief Get the list of object names you know
  std::vector<std::string> GetHeldObjectList() const;

 protected:
  /// @brief Set the attribute of the object
  void SetObjectProperty_(
      const std::string& object_name,
      std::function<void(const std::string&)> set_property_function);

  /// @brief Check if it is an internal object
  bool IsInnerObject_(const std::string& object_name);

  /// @brief Check if it is a child object
  bool IsChildObject_(const std::string& object_name);

  /// @brief Update the interference check model for checking the vicinity of the internal object
  void UpdateInnerModel_(void);

  /// @brief Update the interference check model based on the robot athletic model
  void UpdateCollisionModel_(void);

  /// @brief Create an object and object object name pair list
  std::vector<PairString> GetObjectToObjectPairNameList_(
      const std::string& object_name1, const std::string& object_name2);

  /// @brief Create an object name pair of objects and groups
  std::vector<PairString> GetObjectToGroupPairNameList_(
      const std::string& object_name, const std::string& group_name);

  /// @brief Get the child object name from the object name
  std::vector<std::string> GetChildObjectNameList_(
      const std::string& object_name);

  /// @brief Erase from Bounding_box_
  void EraseCuboid_(const std::string& box_name);

  /// @brief Obtain information on robot sites from the file
  tmc_manipulation_types::ObjectParameterSeq GetRobotPartsShape_(
      const std::string& robot_model_config,
      ModelFileType model_file_type);

  /// @brief Read the robot model and create a robot
  void CreateRobotModel_(const std::string& model_config,
                         ModelFileType model_file_type);

  /// @brief Subcontracting constructor
  void Init_(const std::string& robot_model_config,
             const std::string& robot_collision_config,
             const std::string& engine,
             ModelFileType model_file_type);

  /// @brief A function to obtain a position from athletic model and absorb the difference in the handling of Collision information of URDF
  Eigen::Affine3d GetObjectTransformFromKinematicsModel_(const std::string& object_name) const;

  /// Interference check (both internal and external)
  tmc_collision_detector::ICollisionDetector::Ptr coldet_;
  /// Interference check (internal use)
  tmc_collision_detector::ICollisionDetector::Ptr inner_coldet_;
  /// Robot order athletic model
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_model_;
  /// Interference check settings
  CollisionDetectorConfig::Ptr robot_collision_config_;
  /// Name list of robot parts
  std::vector<std::string> robot_parts_name_;
  /// Name and group of knowledge objects
  std::map<std::string, std::string> attached_object_name_;
  /// List of external objects
  OuterObjectMap outer_object_list_;
  /// Environmental intelligence preservation constructor
  DumpEnvironmentalData environmental_data_;
  /// Cuboid
  BoundingBox bounding_box_;
  /// Information for Collison's position/posture
  std::map<std::string, CollisionFrameInfo> collision_frame_info_map_;
};
}  // namespace tmc_robot_collision_detector
#endif  // TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_HPP_
