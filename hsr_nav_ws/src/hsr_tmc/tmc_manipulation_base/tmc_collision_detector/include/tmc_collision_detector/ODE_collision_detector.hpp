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
#ifndef TMC_COLLISION_DETECTOR_ODE_COLLISION_DETECTOR_HPP_
#define TMC_COLLISION_DETECTOR_ODE_COLLISION_DETECTOR_HPP_

#ifndef dDOUBLE
#define dDOUBLE
#endif

#include  <list>
#include  <map>
#include  <string>
#include  <utility>
#include  <vector>

#include  <ode/ode.h>

#include  <tmc_collision_detector/collision_detector.hpp>

namespace tmc_collision_detector {

/// For Vector's reserve value, speeding up to store interference objects
const int32_t kReserveContactPairNum = 1000;

/// Ode name
const char* const kODEName = "ODE";

using dGeomPair = std::pair<dGeomID, dGeomID>;
using ExclusionPair = std::pair<PairString, dGeomPair>;

/// CHECKCOLLISISPACE, use to discontinue checks after interference
struct SpaceCollideResult {
  dContact contact;
  bool result;
  dGeomPair id;
  std::vector<dGeomPair> exclusion_list;
};
/// CHECKCOLLISISPACE, use to create an interference list
struct ContactPairResult {
  dContact contact;
  std::vector<dGeomPair> contact_list;
};
/// RayCasting
struct RayCastingResult {
  dContactGeom contact;
  Eigen::Vector3d end_point;
  double distance;
  dGeomID object_id;
};
/// CollisonObject for ODE
struct ODECollisionObject {
  std::string name;
  tmc_manipulation_types::Shape shape;
  dGeomID object_id;
  dTriMeshDataID mesh_id;
  std::vector<double> vertices;
  std::vector<uint32_t> indices;
};

/// Interference check class using ODE
class ODECollisionDetector : public ICollisionDetector {
 public:
  ODECollisionDetector();
  virtual ~ODECollisionDetector();

  /// Creating an object
  virtual void CreateObject(
      const tmc_manipulation_types::ObjectParameter& parameter);
  /// Obtain an object parameter
  virtual tmc_manipulation_types::ObjectParameter GetObjectParameter(
      const std::string& name) const;
  /// Obtain AABB of object
  virtual tmc_manipulation_types::AABB GetObjectAABB(
      const std::string& name) const;

  /// Object destruction
  virtual void DestroyObject(const std::string& name);
  /// Discard all objects behind the anchors (do not include anchors)
  virtual void DestroyObject(void);

  /// Currently, make the last object anchor
  virtual void SetAnchor(void);
  /// Set an anchor
  virtual void SetAnchor(const std::string& name);
  /// Get the name of the anchor
  virtual std::string GetAnchor(void) const;

  /// Set the position posture of the object
  virtual void SetObjectTransform(const Eigen::Affine3d &transform,
                                  const std::string& name);
  /// Obtain the position posture of the object
  virtual Eigen::Affine3d GetObjectTransform(const std::string& name) const;

  /// Set an object group
  virtual void SetCollisionGroup(const uint16_t group,
                                 const std::string& name);
  /// Set an object filter
  virtual void SetCollisionFilter(const uint16_t filter,
                                  const std::string& name);
  /// Obtained an object group
  virtual uint16_t GetCollisionGroup(const std::string& name) const;
  /// Obtain an object filter
  virtual uint16_t GetCollisionFilter(const std::string& name) const;

  /// Enable object interference checks
  virtual void EnableObject(const std::string& name);
  /// Disable object interference checks
  virtual void DisableObject(const std::string& name);

  /// Add an object pair to be removed from the interference check for interference checks in the space
  virtual void DisableCollisionCheck(const std::vector<PairString>& names);
  /// Return the object pair removed from the interference check to the interference check again
  virtual void EnableCollisionCheck(const std::vector<PairString>& names);

  /// Discard the exclusion list
  virtual void ResetCollisionCheckPairList();

  /// Check if the two objects are interfering
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB);
  /// Check if the two objects are interfering (return contact information)
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB,
                                  Eigen::Vector3d& point,
                                  Eigen::Vector3d& normal);

  /// Check if the object in the space is interfering
  virtual bool CheckCollisionSpace(void);
  /// Interference check to get the name of the pair of the interference
  virtual bool CheckCollisionSpace(PairString& dst_contact_pair);
  /// Create a list of pair of objects that interfere
  virtual bool GetContactPairList(std::vector<PairString>& dst_contact_pair);
  /// Get the distance between the two objects
  virtual ClosestResult GetClosestResult(const std::string& nameA,
                                         const std::string& nameB);
  /// Get information on the nearby objects recently
  virtual ClosestResult GetClosestObject(const std::string& name,
                                         double extend_length,
                                         int32_t top_n,
                                         uint16_t filter);

  /// Get the physical engine you are using
  virtual std::string GetEngine() const {return std::string(kODEName);}

  /// Ray casting function
  virtual bool RayCasting(const Eigen::Vector3d& start_point,
                          const Eigen::Vector3d& direction,
                          double length,
                          Eigen::Vector3d& end_point,
                          std::string& name);

 private:
  /// ODE space
  dSpaceID space_;

  /// Object list
  std::list<ODECollisionObject> object_list_;

  /// anchor
  std::list<ODECollisionObject>::iterator object_anchor_;
  /// Flag to see the anchor is called
  bool anchor_called_;

  /// Get OdecollisionObject from the object name
  std::list<ODECollisionObject>::const_iterator GetODECollisionObjectConst_(
      const std::string& name) const;
  std::list<ODECollisionObject>::iterator GetODECollisionObject_(
      const std::string& name);

  /// CHECKCOLLISISPACE, use to discontinue checks after interference
  static void SpaceCollideCallback_(void* data, dGeomID o1, dGeomID o2);

  /// CHECKCOLLISISPACE, use to create an interference list
  static void MakeContactPairListCallback_(void* data, dGeomID o1, dGeomID o2);

  /// RayCasting
  static void RayCastingCallback_(void* data, dGeomID o1, dGeomID o2);

  /// Obtain an object name
  std::string GetObjectName_(dGeomID id) const;

  /// List of object pairs to be removed from interference checks
  std::vector<ExclusionPair> exclusion_list_;

  /// List of object pairs to be added to interference checks
  std::vector<PairString> add_pair_list_;

  /// Name and entity map
  std::map<std::string, ODECollisionObject*> name_map_;
};
}  // end namespace tmc_collision_detector
#endif  // TMC_COLLISION_DETECTOR_ODE_COLLISION_DETECTOR_HPP_

