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

#include  <list>
#include  <map>
#include  <string>
#include  <utility>
#include  <vector>

#include  <tmc_collision_detector/ODE_collision_detector.hpp>
#include  <tmc_stl_loader/stl_loader.hpp>

using tmc_manipulation_types::ObjectParameter;
using tmc_manipulation_types::Shape;
using tmc_manipulation_types::AABB;

namespace tmc_collision_detector {

class IsEqualName {
 public:
  explicit IsEqualName(const std::string& name) : name_(name) {}
  bool operator()(const PairString& pair) {return IsEqual_(pair);}
  bool operator()(const ExclusionPair& pair) {return IsEqual_(pair.first);}
 private:
  bool IsEqual_(const PairString& pair) {
    if ((pair.first == name_) || (pair.second == name_)) return true;
    else  return false;
  }
  std::string name_;
};

class IsEqualPairName {
 public:
  explicit IsEqualPairName(const PairString& name) : name_(name) {}
  bool operator()(const PairString& pair) {return IsEqual_(pair);}
  bool operator()(const ExclusionPair& pair) {return IsEqual_(pair.first);}
 private:
  bool IsEqual_(const PairString& pair) {
    if (((pair.first == name_.first) && (pair.second == name_.second)) ||
        ((pair.first == name_.second) && (pair.second == name_.first))) {
      return true;
    } else {
      return false;
    }
  }
  PairString name_;
};

void SerializeVertices(const std::vector<Eigen::Vector3f>& in,
                       std::vector<double>& out) {
  out.clear();
  for (uint32_t i = 0; i < static_cast<uint32_t>(in.size()); ++i) {
    out.push_back(in.at(i)[0]);
    out.push_back(in.at(i)[1]);
    out.push_back(in.at(i)[2]);
  }
}

/// @brief  constructor
ODECollisionDetector::ODECollisionDetector() {
  dInitODE();
  space_ = dSweepAndPruneSpaceCreate(0, dSAP_AXES_XYZ);
  anchor_called_ = false;
}

/// @brief  Destructor
ODECollisionDetector::~ODECollisionDetector() {
  for (std::list<ODECollisionObject>::iterator it = object_list_.begin();
       it != object_list_.end(); ++it) {
    if (it->shape.type == tmc_manipulation_types::kMesh) {
      dGeomTriMeshDataDestroy(it->mesh_id);
    }
//    dGeomDestroy(it->object_id);
  }
  object_list_.clear();

  dSpaceDestroy(space_);
  dCloseODE();
}

/// @brief  Creating an object
void ODECollisionDetector::CreateObject(const ObjectParameter& parameter) {
  for (std::vector<double>::const_iterator it =
      parameter.shape.dimensions.begin();
      it != parameter.shape.dimensions.end(); ++it) {
    if (*it < 0.0) {
      throw InvalidShapeParamError(parameter.name + "'s length is negative");
    }
  }
  ODECollisionObject object;
  object.name = parameter.name;
  object.shape = parameter.shape;
  object_list_.push_back(object);
  try {
    switch (parameter.shape.type) {
      case tmc_manipulation_types::kSphere:
        object_list_.back().object_id =
            dCreateSphere(space_, parameter.shape.dimensions.at(0));
        break;
      case tmc_manipulation_types::kBox:
        object_list_.back().object_id =
            dCreateBox(space_,
                       parameter.shape.dimensions.at(0),
                       parameter.shape.dimensions.at(1),
                       parameter.shape.dimensions.at(2));
        break;
      case tmc_manipulation_types::kCylinder:
        object_list_.back().object_id =
            dCreateCylinder(space_,
                            parameter.shape.dimensions.at(0),
                            parameter.shape.dimensions.at(1));
        break;
      case tmc_manipulation_types::kCapsule:
        object_list_.back().object_id =
            dCreateCapsule(space_,
                           parameter.shape.dimensions.at(0),
                           parameter.shape.dimensions.at(1));
        break;
      case tmc_manipulation_types::kMesh: {
        tmc_stl_loader::Mesh mesh;
        std::shared_ptr<tmc_stl_loader::STLLoader>
            stl_loader(new tmc_stl_loader::STLLoader);
        stl_loader->Load(object_list_.back().shape.filename.c_str(), mesh);
        if (mesh.vertices.size() == 0) {
          object_list_.pop_back();
          throw InvalidShapeParamError(
                     object.shape.filename + " is invalid.");
        }
        // Get the number of vertices and surfaces
        SerializeVertices(mesh.vertices, object_list_.back().vertices);
        object_list_.back().indices = mesh.indices;
        object_list_.back().mesh_id = dGeomTriMeshDataCreate();
        dGeomTriMeshDataBuildDouble(object_list_.back().mesh_id,
                                    &(object_list_.back().vertices[0]),
                                    sizeof(dReal) * 3,
                                    object_list_.back().vertices.size() / 3,
                                    &(object_list_.back().indices[0]),
                                    mesh.indices.size(),
                                    sizeof(dTriIndex) * 3);
        object_list_.back().object_id =
            dCreateTriMesh(space_, object_list_.back().mesh_id,
                           NULL, NULL, NULL);
        break;
      }
      case tmc_manipulation_types::kMeshVertices: {
        SerializeVertices(object_list_.back().shape.vertices, object_list_.back().vertices);
        object_list_.back().mesh_id = dGeomTriMeshDataCreate();
        dGeomTriMeshDataBuildDouble(object_list_.back().mesh_id,
                                    &(object_list_.back().vertices[0]),
                                    sizeof(dReal) * 3,
                                    object_list_.back().vertices.size() / 3,
                                    &(object_list_.back().shape.indices[0]),
                                    object_list_.back().shape.indices.size(),
                                    sizeof(dTriIndex) * 3);
        object_list_.back().object_id =
            dCreateTriMesh(space_, object_list_.back().mesh_id,
                           NULL, NULL, NULL);
        break;
      }
      default:
        object_list_.pop_back();
        throw NonExistTypeError(parameter.name);
    }
  } catch (const std::out_of_range& error) {
    object_list_.pop_back();
    throw InvalidShapeParamError(
              parameter.name + "'s dimensions are too short");
  }
  Eigen::Quaterniond quaternion_eigen(parameter.transform.linear());
  dQuaternion quaternion_ode;
  quaternion_ode[0] = quaternion_eigen.w();
  quaternion_ode[1] = quaternion_eigen.x();
  quaternion_ode[2] = quaternion_eigen.y();
  quaternion_ode[3] = quaternion_eigen.z();
  dGeomSetQuaternion(object_list_.back().object_id, quaternion_ode);

  dGeomSetPosition(object_list_.back().object_id,
                   parameter.transform.translation().x(),
                   parameter.transform.translation().y(),
                   parameter.transform.translation().z());
  dGeomSetCategoryBits(object_list_.back().object_id, parameter.group);
  dGeomSetCollideBits(object_list_.back().object_id, parameter.filter);
  name_map_.insert(std::pair<std::string, ODECollisionObject*>(
      object_list_.back().name, &object_list_.back()));
}

/// @brief  Obtain information on objects
ObjectParameter ODECollisionDetector::GetObjectParameter(
    const std::string& name) const {
  ObjectParameter parameter;
  parameter.name = name;
  std::list<ODECollisionObject>::const_iterator it =
      GetODECollisionObjectConst_(name);
  parameter.shape = it->shape;
  if (parameter.shape.type == tmc_manipulation_types::kMesh) {
    parameter.shape.indices = it->indices;
    parameter.shape.vertices.clear();
    for (auto i = 0u; i < it->vertices.size(); i += 3) {
      parameter.shape.vertices.push_back(Eigen::Vector3f(
          it->vertices.at(i), it->vertices.at(i + 1), it->vertices.at(i + 2)));
    }
  }
  parameter.group = static_cast<uint16_t>(dGeomGetCategoryBits(it->object_id));
  parameter.filter = static_cast<uint16_t>(dGeomGetCollideBits(it->object_id));

  const dReal *position = dGeomGetPosition(it->object_id);
  parameter.transform.translation() =
      Eigen::Vector3d(position[0], position[1], position[2]);

  dQuaternion quaternion;
  dGeomGetQuaternion(it->object_id, quaternion);
  parameter.transform.linear() =
      Eigen::Quaterniond(quaternion[0],
                         quaternion[1],
                         quaternion[2],
                         quaternion[3]).toRotationMatrix();
  return parameter;
}

/// @brief  Obtain AABB of object
AABB ODECollisionDetector::GetObjectAABB(const std::string& name) const {
  dReal aabb_ode[6];
  AABB aabb;
  std::list<ODECollisionObject>::const_iterator it =
      GetODECollisionObjectConst_(name);
  dGeomGetAABB(it->object_id, aabb_ode);
  aabb << aabb_ode[0], aabb_ode[1], aabb_ode[2],
      aabb_ode[3], aabb_ode[4], aabb_ode[5];
  return aabb;
}

/// @brief  Object destruction
void ODECollisionDetector::DestroyObject(const std::string& name) {
  for (std::list<ODECollisionObject>::iterator it =
      object_list_.begin(); it != object_list_.end(); ++it) {
    if (it->name == name) {
      // Delete from add_pair, exclusion
      std::vector<PairString>::iterator add_pair_end_itr =
          std::remove_if(add_pair_list_.begin(),
                         add_pair_list_.end(), IsEqualName(name));
      add_pair_list_.erase(add_pair_end_itr, add_pair_list_.end());
      std::vector<ExclusionPair>::iterator exclusion_end_itr =
          std::remove_if(exclusion_list_.begin(),
                         exclusion_list_.end(), IsEqualName(name));
      exclusion_list_.erase(exclusion_end_itr, exclusion_list_.end());
      // Delete objects
      if (it->shape.type == tmc_manipulation_types::kMesh) {
        dGeomTriMeshDataDestroy(it->mesh_id);
      }
      dGeomDestroy(it->object_id);
      object_list_.erase(it);
      name_map_.erase(name);
      return;
    }
  }
  throw NonCreateError(name);
}

/// @brief  Discard all objects behind the anchors (do not include anchors)
void ODECollisionDetector::DestroyObject(void) {
  if (anchor_called_ == false) {
    throw std::domain_error("error: not set anchor");
  }
  std::list<ODECollisionObject>::iterator it = object_anchor_;
  ++it;
  if (it == object_list_.end()) {
    return;
  }
  for (; it != object_list_.end(); ++it) {
    if (it->shape.type == tmc_manipulation_types::kMesh) {
      dGeomTriMeshDataDestroy(it->mesh_id);
    }
    dGeomDestroy(it->object_id);
  }
  it = object_anchor_;
  ++it;
  object_list_.erase(it, (object_list_.end()));
  ResetCollisionCheckPairList();
  name_map_.clear();
}

/// @brief  Currently, make the last object anchor
void ODECollisionDetector::SetAnchor(void) {
  if (object_list_.empty()) {
    throw std::domain_error("error: space don't have object");
  }
  object_anchor_ = --(object_list_.end());
  anchor_called_ = true;
}

/// @brief  Set the specified object as an anchor
void ODECollisionDetector::SetAnchor(const std::string& name) {
  for (std::list<ODECollisionObject>::iterator it = object_list_.begin();
      it != object_list_.end(); ++it) {
    if (it->name == name) {
      object_anchor_ = it;
      anchor_called_ = true;
      return;
    }
  }
}

/// @brief  Get the name of the anchor
std::string ODECollisionDetector::GetAnchor(void) const {
  if (!anchor_called_) {
    throw std::domain_error("error: not set anchor");
  }
  return object_anchor_->name;
}

/// @brief  Set the position posture of the object
void ODECollisionDetector::SetObjectTransform(
    const Eigen::Affine3d &transform, const std::string& name) {
  std::list<ODECollisionObject>::iterator it = GetODECollisionObject_(name);
  dGeomSetPosition(it->object_id, transform.translation().x(),
                   transform.translation().y(), transform.translation().z());

  Eigen::Quaterniond quaternion_eigen(transform.linear());
  dQuaternion quaternion_ode;
  quaternion_ode[0] = quaternion_eigen.w();
  quaternion_ode[1] = quaternion_eigen.x();
  quaternion_ode[2] = quaternion_eigen.y();
  quaternion_ode[3] = quaternion_eigen.z();
  dGeomSetQuaternion(it->object_id, quaternion_ode);
}

/// @brief  Obtain the position posture of the object
Eigen::Affine3d ODECollisionDetector::GetObjectTransform(
    const std::string& name) const {
  std::list<ODECollisionObject>::const_iterator it =
      GetODECollisionObjectConst_(name);
  const dReal *position = dGeomGetPosition(it->object_id);
  Eigen::Affine3d transform;
  transform.translation() =
      Eigen::Vector3d(position[0], position[1], position[2]);

  dQuaternion quaternion;
  dGeomGetQuaternion(it->object_id, quaternion);
  transform.linear() = Eigen::Quaterniond(quaternion[0],
                                          quaternion[1],
                                          quaternion[2],
                                          quaternion[3]).toRotationMatrix();

  return transform;
}

/// @brief  Set an object group
void ODECollisionDetector::SetCollisionGroup(
    const uint16_t group, const std::string& name) {
  dGeomSetCategoryBits(GetODECollisionObject_(name)->object_id, group);
}

/// @brief  Set an object filter
void ODECollisionDetector::SetCollisionFilter(
    const uint16_t filter, const std::string& name) {
  dGeomSetCollideBits(GetODECollisionObject_(name)->object_id, filter);
}

/// @brief  Obtained an object group
uint16_t ODECollisionDetector::GetCollisionGroup(
    const std::string& name) const {
  return static_cast<uint16_t>(
      dGeomGetCategoryBits(GetODECollisionObjectConst_(name)->object_id));
}

/// @brief  Obtain an object filter
uint16_t ODECollisionDetector::GetCollisionFilter(
    const std::string& name) const {
  return static_cast<uint16_t>(
      dGeomGetCollideBits(GetODECollisionObjectConst_(name)->object_id));
}

/// @brief  Enable object interference checks
void ODECollisionDetector::EnableObject(const std::string& name) {
  std::map<std::string, ODECollisionObject*>::iterator it =
      name_map_.find(name);
  if (it == name_map_.end()) {
    throw NonCreateError(name);
  }
  dGeomEnable(it->second->object_id);
}

/// @brief  Disable object interference checks
void ODECollisionDetector::DisableObject(const std::string& name) {
  std::map<std::string, ODECollisionObject*>::iterator it =
      name_map_.find(name);
  if (it == name_map_.end()) {
    throw NonCreateError(name);
  }
  dGeomDisable(it->second->object_id);
}

/// @brief  Add an object pair to be removed from the interference check
void ODECollisionDetector::DisableCollisionCheck(
    const std::vector<PairString>& names) {
  exclusion_list_.reserve(exclusion_list_.size() + names.size());
  std::vector<PairString>::iterator end_itr(add_pair_list_.end());
  for (std::vector<PairString>::const_iterator dis_pair = names.begin();
      dis_pair != names.end(); ++dis_pair) {
    // Delete from add_pair_list
    std::vector<PairString>::iterator new_end_itr;
    new_end_itr = std::remove_if(add_pair_list_.begin(), end_itr,
                             IsEqualPairName(*dis_pair));
    // If it is not in add_pair_list, add it to EXCLUSION_LIST
    if (new_end_itr == end_itr) {
      ExclusionPair exclusion;
      exclusion.first = *dis_pair;
      exclusion.second.first =
          GetODECollisionObject_(dis_pair->first)->object_id;
      exclusion.second.second =
          GetODECollisionObject_(dis_pair->second)->object_id;
      exclusion_list_.push_back(exclusion);
    }
    end_itr = new_end_itr;
  }
  add_pair_list_.erase(end_itr, add_pair_list_.end());
}

/// @brief  Add an object pair to check the interference check
void ODECollisionDetector::EnableCollisionCheck(
    const std::vector<PairString>& names) {
  add_pair_list_.reserve(add_pair_list_.size() + names.size());
  std::vector<ExclusionPair>::iterator end_itr(exclusion_list_.end());
  for (std::vector<PairString>::const_iterator dis_pair = names.begin();
      dis_pair != names.end(); ++dis_pair) {
    // Deleted from Exclusion_list
    std::vector<ExclusionPair>::iterator new_end_itr;
    new_end_itr = std::remove_if(exclusion_list_.begin(), end_itr,
                             IsEqualPairName(*dis_pair));
    // Added to add_pair_list
    if (new_end_itr == end_itr) {
      add_pair_list_.push_back(*dis_pair);
    }
    end_itr = new_end_itr;
  }
  exclusion_list_.erase(end_itr, exclusion_list_.end());
}

/// @brief  Interference check exemption pairist, interference check addition pairist is cleared.
void ODECollisionDetector::ResetCollisionCheckPairList() {
  exclusion_list_.clear();
  add_pair_list_.clear();
}

/// @brief  Check if the two objects are interfering
bool ODECollisionDetector::CheckCollisionPair(
    const std::string& nameA, const std::string& nameB) {
  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  return CheckCollisionPair(nameA, nameB, point, normal);
}

/// @brief  Check if the two objects are interfering (return contact information)
bool ODECollisionDetector::CheckCollisionPair(
    const std::string& nameA, const std::string& nameB,
    Eigen::Vector3d& point, Eigen::Vector3d& normal) {
  std::list<ODECollisionObject>::iterator itA = GetODECollisionObject_(nameA);
  std::list<ODECollisionObject>::iterator itB = GetODECollisionObject_(nameB);

  dContact contact;
  int32_t num_of_point = dCollide(itA->object_id, itB->object_id, 1,
                                  &(contact.geom), sizeof(contact));
  if (num_of_point == 0) {
    point.setZero(3);
    normal.setZero(3);
    return false;
  } else {
    point = Eigen::Vector3d(
        contact.geom.pos[0], contact.geom.pos[1], contact.geom.pos[2]);
    normal = Eigen::Vector3d(contact.geom.normal[0],
                             contact.geom.normal[1],
                             contact.geom.normal[2]);
    return true;
  }
}

/// @brief  Check if the object in the space is interfering
bool ODECollisionDetector::CheckCollisionSpace(void) {
  SpaceCollideResult space_collide_result;
  space_collide_result.exclusion_list.reserve(exclusion_list_.size());
  for (std::vector<ExclusionPair>::iterator it = exclusion_list_.begin();
      it != exclusion_list_.end(); ++it) {
    space_collide_result.exclusion_list.push_back(it->second);
  }
  space_collide_result.result = false;
  dSpaceCollide(space_, &space_collide_result, &SpaceCollideCallback_);
  if (space_collide_result.result) {
    return true;
  } else {
    for (std::vector<PairString>::iterator it = add_pair_list_.begin();
        it != add_pair_list_.end(); ++it) {
      if (CheckCollisionPair(it->first, it->second)) {
        return true;
      }
    }
  }
  return false;
}

/// @brief  Inside interference check that gets the name of the pair of the interference of the interference
bool ODECollisionDetector::CheckCollisionSpace(
    PairString& dst_contact_pair) {
  SpaceCollideResult space_collide_result;
  space_collide_result.exclusion_list.reserve(exclusion_list_.size());
  for (std::vector<ExclusionPair>::iterator it = exclusion_list_.begin();
      it != exclusion_list_.end(); ++it) {
    space_collide_result.exclusion_list.push_back(it->second);
  }
  space_collide_result.result = false;
  dSpaceCollide(space_, &space_collide_result, &SpaceCollideCallback_);
  if (space_collide_result.result) {
    dst_contact_pair.first = GetObjectName_(space_collide_result.id.first);
    dst_contact_pair.second = GetObjectName_(space_collide_result.id.second);
  }
  if (space_collide_result.result) {
    return true;
  } else {
    for (std::vector<PairString>::iterator it = add_pair_list_.begin();
        it != add_pair_list_.end(); ++it) {
      if (CheckCollisionPair(it->first, it->second)) {
        dst_contact_pair = *it;
        return true;
      }
    }
  }
  return false;
}

/// @brief  Create a list of pair of objects that interfere
bool ODECollisionDetector::GetContactPairList(
    std::vector<PairString>& dst_contact_pair) {
  ContactPairResult contact_pair_result_;
  contact_pair_result_.contact_list.reserve(kReserveContactPairNum);
  dSpaceCollide(space_, &contact_pair_result_, &MakeContactPairListCallback_);
  for (std::vector<dGeomPair>::iterator it =
      contact_pair_result_.contact_list.begin();
      it != contact_pair_result_.contact_list.end(); ++it) {
    std::string name0(GetObjectName_(it->first));
    std::string name1(GetObjectName_(it->second));
    dst_contact_pair.push_back(PairString(name0, name1));
  }
  std::vector<PairString>::iterator end_itr(dst_contact_pair.end());
  for (std::vector<ExclusionPair>::iterator dis_pair = exclusion_list_.begin();
      dis_pair != exclusion_list_.end(); ++dis_pair) {
    // Deleted from Exclusion_list
    end_itr = std::remove_if(dst_contact_pair.begin(), end_itr,
                             IsEqualPairName(dis_pair->first));
  }
  dst_contact_pair.erase(end_itr, dst_contact_pair.end());
  for (std::vector<PairString>::iterator it = add_pair_list_.begin();
      it != add_pair_list_.end(); ++it) {
    if (CheckCollisionPair(it->first, it->second)) {
      dst_contact_pair.push_back(*it);
    }
  }
  if (dst_contact_pair.empty()) {
    return false;
  } else {
    return true;
  }
}

/// @brief  Get the distance between the two objects
ClosestResult ODECollisionDetector::GetClosestResult(
    const std::string& nameA, const std::string& nameB) {
  throw UnsupportMethodError("ODECollisionDetector::GetClosestResult");
}

/// @brief  Get information on the nearby objects recently
ClosestResult ODECollisionDetector::GetClosestObject(
    const std::string& name, double extend_length,
    int32_t top_n, uint16_t filter) {
  throw UnsupportMethodError("ODECollisionDetector::GetClosestObject");
}

/// @brief  Ray casting function
bool ODECollisionDetector::RayCasting(const Eigen::Vector3d& start_point,
                                      const Eigen::Vector3d& direction,
                                      double length,
                                      Eigen::Vector3d& end_point,
                                      std::string& name) {
  if (direction.isZero()) {
    return false;
  }
  dGeomID ray = dCreateRay(0, length);
  dGeomRaySet(ray, start_point(0), start_point(1), start_point(2),
              direction(0), direction(1), direction(2));
  dGeomSetCategoryBits(ray, 0xFFFF);
  dGeomSetCollideBits(ray, 0xFFFF);
  dGeomRaySetClosestHit(ray, 1);
  RayCastingResult result;
  result.object_id = 0;
  result.distance = length;
  dSpaceCollide2(ray, (dGeomID)space_, &result, &RayCastingCallback_);

  dGeomDestroy(ray);
  if (result.object_id != 0) {
    end_point = result.end_point;
    name = GetObjectName_(result.object_id);
    return true;
  }
  return false;
}

/// RayCasting
void ODECollisionDetector::RayCastingCallback_(void* data, dGeomID o1,
                                               dGeomID o2) {
  if (!dGeomIsSpace(o1) && !dGeomIsSpace(o2)) {
    RayCastingResult* result = static_cast<RayCastingResult*>(data);
    int32_t num_contact = dCollide(o1, o2, 1, &(result->contact),
                                   sizeof(dContact));
    if (num_contact > 0) {
      if (result->contact.depth < result->distance) {
        result->distance = result->contact.depth;
        result->end_point = Eigen::Vector3d(result->contact.pos[0],
                                            result->contact.pos[1],
                                            result->contact.pos[2]);
        if (dGeomGetClass(o1) == dRayClass) {
          result->object_id = o2;
        } else {
          result->object_id = o1;
        }
      }
    }
  }
}

/// @brief  Callback used to discontinue checks after interference
void ODECollisionDetector::SpaceCollideCallback_(
    void *data, dGeomID o1, dGeomID o2) {
  SpaceCollideResult* result = static_cast<SpaceCollideResult*>(data);
  if (!(result->result)) {
    if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
    } else {
      int32_t num_contact = dCollide(o1, o2, 1, &(result->contact.geom),
                                     sizeof(dContact));
      if (num_contact > 0) {
        for (std::vector<dGeomPair>::iterator it =
             result->exclusion_list.begin(); it != result->exclusion_list.end();
             ++it) {
          if (((it->first == o1) && (it->second == o2)) ||
              ((it->second == o1) && (it->first == o2))) {
            return;
          }
        }
        result->result = true;
        result->id.first = o1;
        result->id.second = o2;
      }
    }
  }
}

/// @brief  Callback used to create an interference object list
void ODECollisionDetector::MakeContactPairListCallback_(
    void *data, dGeomID o1, dGeomID o2) {
  ContactPairResult* result = static_cast<ContactPairResult*>(data);
  if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
  } else {
    int32_t num_contact = dCollide(o1, o2, 1, &(result->contact.geom),
                                   sizeof(dContact));
    if (num_contact > 0) {
      result->contact_list.push_back(dGeomPair(o1, o2));
    }
  }
}

/// @brief  Get OdecollisionObject from the object name
std::list<ODECollisionObject>::const_iterator
ODECollisionDetector::GetODECollisionObjectConst_(
    const std::string& name) const {
  for (std::list<ODECollisionObject>::const_iterator it =
      object_list_.begin(); it != object_list_.end(); ++it) {
    if (it->name == name) {
      return it;
    }
  }
  throw NonCreateError(name);
}

/// @brief  Get OdecollisionObject from the object name
std::list<ODECollisionObject>::iterator
ODECollisionDetector::GetODECollisionObject_(
    const std::string& name) {
  for (std::list<ODECollisionObject>::iterator it =
      object_list_.begin(); it != object_list_.end(); ++it) {
    if (it->name == name) {
      return it;
    }
  }
  throw NonCreateError(name);
}

/// @brief  Object_list gets an object name
std::string ODECollisionDetector::GetObjectName_(dGeomID id) const {
  for (std::list<ODECollisionObject>::const_iterator it =
      object_list_.begin(); it != object_list_.end(); ++it) {
    if (id == it->object_id) {
      return it->name;
    }
  }
  throw InvalidObjectContactError("");
}
}  // end namespace tmc_collision_detector

