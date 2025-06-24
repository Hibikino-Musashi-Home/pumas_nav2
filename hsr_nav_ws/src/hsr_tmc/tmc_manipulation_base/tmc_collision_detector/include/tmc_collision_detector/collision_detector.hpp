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
#ifndef TMC_COLLISION_DETECTOR_COLLISION_DETECTOR_HPP_
#define TMC_COLLISION_DETECTOR_COLLISION_DETECTOR_HPP_

#include  <stdint.h>
#include  <utility>

#include  <list>
#include  <map>
#include  <memory>
#include  <string>
#include  <vector>

#include  <Eigen/Core>
#include  <Eigen/Geometry>

#include  <tmc_manipulation_types/manipulation_types.hpp>
#include  "tmc_collision_detector/collision_detector_exception.hpp"

namespace tmc_collision_detector {
using PairString = std::pair<std::string, std::string>;

struct ClosestResult {
  std::string name;
  bool contact;
  double distance;
  Eigen::Vector3d closest_point_on_A;
  Eigen::Vector3d closest_point_on_B;
  Eigen::Vector3d normal_on_B;
};

/// Interference check interface class
class ICollisionDetector {
 public:
  using Ptr = std::shared_ptr<ICollisionDetector>;

  virtual ~ICollisionDetector() {}

  /// @brief Creating an object
  /// @param[in,out] parameter Object information
  virtual void CreateObject(
      const tmc_manipulation_types::ObjectParameter& parameter) = 0;

  /// @brief  Obtain information on objects
  /// @param[in] name Object name
  /// @return ObjectParameter  object information
  virtual tmc_manipulation_types::ObjectParameter GetObjectParameter(
      const std::string& name) const = 0;

  /// @brief  Obtain AABB of object
  /// @param[in] name Object name
  /// @return AABB  [xmin xmax; ymin ymax; zmin zmax] of object
  virtual tmc_manipulation_types::AABB GetObjectAABB(
      const std::string& name) const = 0;

  /// @brief  Object destruction
  /// @param[in] name Object name
  virtual void DestroyObject(const std::string& name) = 0;

  /// @brief  Discard all objects behind the anchors (do not include anchors)
  virtual void DestroyObject(void) = 0;

  /// @brief  Currently, make the last object anchor
  virtual void SetAnchor(void) = 0;

  /// @brief  Set the specified object as an anchor
  /// @param[in] name Setting object name
  virtual void SetAnchor(const std::string& name) = 0;

  /// @brief  Get the name of the anchor
  /// @return std::string  Anchor Object name
  virtual std::string GetAnchor(void) const = 0;

  /// @brief  Set the position posture of the object
  /// @param[in] transform Object pose
  /// @param[in] name Object name
  virtual void SetObjectTransform(const Eigen::Affine3d &transform,
                                  const std::string& name) = 0;

  /// @brief  Obtain the position posture of the object
  /// @param[in] name Object name
  /// @return Eigen::Affine3D  Acquired pose
  virtual Eigen::Affine3d GetObjectTransform(
      const std::string& name) const = 0;

  /// @brief  Set an object group
  /// @param[in] group Group bit
  /// @param[in] name Object name
  virtual void SetCollisionGroup(const uint16_t group,
                                 const std::string& name) = 0;

  /// @brief  Set an object filter
  /// @param[in] filter Filter bit
  /// @param[IN] name Object name
  virtual void SetCollisionFilter(const uint16_t filter,
                                  const std::string& name) = 0;

  /// @brief  Obtained an object group
  /// @param[in] name Object name
  /// @return uint16_t  Group bit
  virtual uint16_t GetCollisionGroup(const std::string& name) const = 0;

  /// @brief  Obtain an object filter
  /// @param[in] name Object name
  /// @return uint16_t  Filter bit
  virtual uint16_t GetCollisionFilter(const std::string& name) const = 0;

  /// @brief  Enable object interference checks
  /// @param[in] name Object name
  virtual void EnableObject(const std::string& name) = 0;

  /// @brief  Disable object interference checks
  /// @param[in] name Object name
  virtual void DisableObject(const std::string& name) = 0;

  /// @brief  Add an object pair to be removed from the interference check for interference checks in the space
  /// @param[in] names Pairs of object names
  virtual void DisableCollisionCheck(const std::vector<PairString>& names) = 0;

  /// @brief  Add an object pair to perform interference checks in the interference check in the space
  /// @param [in] names Pairs of object names
  virtual void EnableCollisionCheck(const std::vector<PairString>& names) = 0;

  /// @brief  Interference check exemption pairist, interference check addition pairist is cleared.
  virtual void ResetCollisionCheckPairList() = 0;

  /// @brief  Check if the two objects are interfering
  /// @param[in] nameA Object name
  /// @param[in] nameB Object name
  /// @return bool  Return the result of collision check
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB) = 0;

  /// @brief  Check if the two objects are interfering
  /// @param[in] nameA Object name
  /// @param[in] nameB Object name
  /// @param[out] point Contact point on object nameB
  /// @param[out] normal Normal vector from contact point on object nameB to object nameA
  /// @return bool  Return the result of collision check
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB,
                                  Eigen::Vector3d& point,
                                  Eigen::Vector3d& normal) = 0;


  /// @brief  Check if the object in the space is interfering
  /// @return bool  Return the result of collision check
  virtual bool CheckCollisionSpace(void) = 0;

  /// @brief  Get the name of the pair of the interference object, checking if objects in the space are interfering
  /// @param[out] dst_contact_pair Collision object name pair
  /// @return bool  Return the result of collision check
  virtual bool CheckCollisionSpace(PairString& dst_contact_pair) = 0;

  /// @brief  Create a list of pair of objects that interfere
  /// @param[out] dst_contact_pair Collision object name pairs
  virtual bool GetContactPairList(
      std::vector<PairString>& dst_contact_pair) = 0;

  /// @brief  Get the distance between the two objects
  /// @param[in] nameA Object name
  /// @param[in] nameB Object name
  /// @return ClosestResult  Closest point information
  virtual ClosestResult GetClosestResult(const std::string& nameA,
                                         const std::string& nameB) = 0;

  /// @brief  Acquire information on the objects in the space and recent objects in the space for an object
  /// @param[in] name Object name
  /// @param[in] extend_length Maximum distance to the object near the area
  /// @param[in] top_n GJK Number of objects seeking accurate distance with algorithm
  /// @param[in] filter Object filter bit
  /// @return ClosestResult  Closest point information
  virtual ClosestResult GetClosestObject(const std::string& name,
                                         double extend_length,
                                         int32_t top_n,
                                         uint16_t filter) = 0;

  /// @brief Get the physical engine you are using
  /// @return std::string  Physical engine name
  virtual std::string GetEngine() const = 0;

  /// @brief  Ray casting function
  /// @param[in] start_point Ray starting point
  /// @param[in] direction Ray direction vector
  /// @param[in] length Ray length
  /// @param[out] end_point Ray's hanging object coordinates
  /// @param[out] name Ray's hit object name
  /// @return bool  If the ray collide with an object
  virtual bool RayCasting(const Eigen::Vector3d& start_point,
                          const Eigen::Vector3d& direction,
                          double length,
                          Eigen::Vector3d& end_point,
                          std::string& name) = 0;
};
}  // end namespace tmc_collision_detector
#endif  // TMC_COLLISION_DETECTOR_COLLISION_DETECTOR_HPP_
