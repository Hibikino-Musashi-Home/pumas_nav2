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
/// @brief Objects for saving and reading robot models
#ifndef TMC_ROBOT_COLLISION_DETECTOR_OBJECT_FOR_DUMP_HPP_
#define TMC_ROBOT_COLLISION_DETECTOR_OBJECT_FOR_DUMP_HPP_

#include <string>
#include <vector>

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <tmc_robot_collision_detector/robot_collision_detector_common.hpp>

namespace boost {
namespace serialization {
template <class Archive> void serialize(Archive& ar,
                                        tmc_manipulation_types::Shape& shape,
                                        const unsigned int version) {
  ar & boost::serialization::make_nvp("Type", shape.type);
  ar & boost::serialization::make_nvp("Parameters", shape.dimensions);
  ar & boost::serialization::make_nvp("Filename", shape.filename);
}
}
}  // namespace boost
namespace tmc_robot_collision_detector {

/// DUMP constructor
struct DumpChildObjectData {
  tmc_manipulation_types::Shape shape;
  std::vector<double> base_to_child;
 private:
  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive& ar,
                                         const unsigned int version) {
    ar & boost::serialization::make_nvp("ObjectToChild", base_to_child);
    ar & boost::serialization::make_nvp("Shape", shape);
  }
};

/// DUMP constructor
struct DumpObjectData {
  std::string name;
  std::vector<double> base_transform;
  double margin;
  std::string parent_name;
  std::string parent_group;
  std::vector<double> parent_to_base;
  std::vector<DumpChildObjectData> child;
  bool cuboid;
 private:
  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive& ar,
                                         const unsigned int version) {
    ar & boost::serialization::make_nvp("ObjectName", name);
    ar & boost::serialization::make_nvp("Transform", base_transform);
    ar & boost::serialization::make_nvp("Margin", margin);
    ar & boost::serialization::make_nvp("Cuboid", cuboid);
    ar & boost::serialization::make_nvp("ParentFrameName", parent_name);
    ar & boost::serialization::make_nvp("ParentGroupName", parent_group);
    ar & boost::serialization::make_nvp("ParentToObject", parent_to_base);
    ar & boost::serialization::make_nvp("ChildObject", child);
  }
};

/// DUMP constructor
struct DumpEnvironmentalData {
  std::string robot_model_config;
  std::string robot_collision_config;
  std::string engine;
  ModelFileType model_file_type;
  tmc_manipulation_types::JointState joint_state;
  std::vector<std::string> held_object_name;
  std::vector<DumpObjectData> outer_object_data;
 private:
  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive& ar,
                                         const unsigned int version) {
    ar & boost::serialization::make_nvp(
        "RobotModelConfig", robot_model_config);
    ar & boost::serialization::make_nvp(
        "CollisionDetectorConfig", robot_collision_config);
    ar & boost::serialization::make_nvp("UseEngine", engine);
    ar & boost::serialization::make_nvp("ModelFileType", model_file_type);
    ar & boost::serialization::make_nvp("JointState", joint_state);
    ar & boost::serialization::make_nvp("HeldObject", held_object_name);
    ar & boost::serialization::make_nvp("OuterObject", outer_object_data);
  }
};
}  // namespace tmc_robot_collision_detector
#endif  // TMC_ROBOT_COLLISION_DETECTOR_OBJECT_FOR_DUMP_HPP_
