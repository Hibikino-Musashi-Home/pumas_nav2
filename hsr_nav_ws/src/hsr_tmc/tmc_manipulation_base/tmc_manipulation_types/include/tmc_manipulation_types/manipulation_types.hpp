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

#ifndef TMC_MANIPULATION_TYPES_MANIPULATION_TYPES_HPP_
#define TMC_MANIPULATION_TYPES_MANIPULATION_TYPES_HPP_

#include <stdint.h>

#include <deque>
#include <ostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace tmc_manipulation_types {

typedef Eigen::Affine3d Transform;
typedef Eigen::Affine3d Pose;
typedef std::vector<std::string> NameSeq;



// tmc_robot_kinematics_model
struct JointState {
  /// Joint name
  NameSeq name;
  /// Joint angle
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd effort;

  bool Validate() const { return !name.empty() && position.size() == name.size(); }
};  // struct JointState


// tmc_collision_detector
/// Primitive type
enum CollisionObjectType {
  /// Sphere
  kSphere = 0,
  /// Box
  kBox,
  /// Cylinder
  kCylinder,
  /// Capsule
  kCapsule,
  /// Mesh (file name specification)
  kMesh,
  /// Mesh (vertex specification)
  kMeshVertices,
};  // enum CollisionObjectType

/// Primitive shape information
struct Shape {
  /// Primitive type
  CollisionObjectType type;
  /// Primitive parameters
  std::vector<double> dimensions;
  /// Pass to STL file for mesh
  std::string filename;
  /// The top of the mesh
  std::vector<Eigen::Vector3f> vertices;
  /// Mesh surface
  std::vector<uint32_t> indices;
};  // struct Shape

typedef Eigen::Matrix<double, 6, 1> Twist;
typedef Eigen::Matrix<double, 6, 1> Wrench;
typedef Eigen::Matrix<double, 6, 1> RegionValues;


struct TaskSpaceRegion {
  Pose origin_to_tsr;
  Pose tsr_to_end;
  RegionValues min_bounds;
  RegionValues max_bounds;
  std::string origin_frame_id;
  std::string end_frame_id;

 public:
  TaskSpaceRegion(const Pose& origin_to_tsr,
                  const Pose& tsr_to_end,
                  const RegionValues& min_bounds,
                  const RegionValues& max_bounds,
                  const std::string& origin_frame_id,
                  const std::string& end_frame_id) :
      origin_to_tsr(origin_to_tsr), tsr_to_end(tsr_to_end),
      min_bounds(min_bounds), max_bounds(max_bounds),
      origin_frame_id(origin_frame_id), end_frame_id(end_frame_id) {}
  TaskSpaceRegion() :
      origin_to_tsr(Pose::Identity()), tsr_to_end(Pose::Identity()),
      origin_frame_id(), end_frame_id() {}
};  // struct TaskSpaceRegion

typedef std::vector<TaskSpaceRegion, Eigen::aligned_allocator<TaskSpaceRegion> >
  TaskSpaceRegionSeq;

struct AttachedObject {
  /// Object_id object ID
  std::string object_id;
  /// FRAME_NAME Link name attached to an object
  std::string frame_name;
  /// Simultaneous conversion from link to object
  Pose frame_to_object;
  /// Goupid after ATTACH
  std::string group_id;
  /// Exotic
  std::vector<std::string> expected_objects;
};  // struct AttachedObject

typedef std::vector<AttachedObject,
                    Eigen::aligned_allocator<AttachedObject> > AttachedObjectSeq;

/// Configuration data
typedef Eigen::VectorXd Config;
/// Pass in the configuration space
typedef std::deque<Config> Path;

/// Multidof path
typedef std::vector<Pose,
                    Eigen::aligned_allocator<Pose> > PoseSeq;
typedef std::vector<Eigen::Vector3d,
                    Eigen::aligned_allocator<Eigen::Vector3d> > Pose2dSeq;
typedef std::deque<PoseSeq> MultiDOFPath;

/// Vector with parallel speed and rotation speed
typedef std::vector<Twist,
                    Eigen::aligned_allocator<Twist> > TwistSeq;
/// [Force X, Force Y, Force Z, TORQUE X, TORQUE Y, TORQUE Z]
typedef std::vector<Wrench,
                    Eigen::aligned_allocator<Wrench> > WrenchSeq;

/// Joint path
struct JointTrajectory {
  /// Joint name
  NameSeq names;
  /// Joint value
  Path path;

  bool Validate() const {
    if (names.empty()) {
      return false;
    }

    if (path.empty()) {
      return false;
    }

    for (const Config& p : path) {
      if (p.size() != names.size()) {
        return false;
      }
    }
    return true;
  }
};

/// Complex joint path
struct MultiDOFJointTrajectory {
  /// Joint name
  NameSeq names;
  /// Joint value
  MultiDOFPath path;

  bool Validate() const {
    if (names.empty()) {
      return false;
    }

    if (path.empty()) {
      return false;
    }

    for (const PoseSeq& p : path) {
      if (p.size() != names.size()) {
        return false;
      }
    }
    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Complex joint condition
struct MultiDOFJointState {
  /// Joint name
  NameSeq names;
  /// Joint value
  PoseSeq poses;
  TwistSeq twist;
  WrenchSeq wrench;

  bool Validate() const {
    return !names.empty() && poses.size() == names.size();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Robot condition
struct RobotState {
  JointState joint_state;
  MultiDOFJointState multi_dof_joint_state;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Robot geometric trajectory
struct RobotTrajectory {
  JointTrajectory joint_trajectory;
  MultiDOFJointTrajectory multi_dof_joint_trajectory;

  bool Validate() const {
    return joint_trajectory.Validate() && multi_dof_joint_trajectory.Validate() &&
           joint_trajectory.path.size() == multi_dof_joint_trajectory.path.size();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct TimedJointTrajectoryPoint {
  Eigen::VectorXd positions;
  Eigen::VectorXd velocities;
  Eigen::VectorXd accelerations;
  Eigen::VectorXd effort;
  double time_from_start;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::deque<TimedJointTrajectoryPoint> TimedJointTrajectoryPointSeq;


/// Timed joint trajectory (here is to support ROS)
struct TimedJointTrajectory {
  NameSeq joint_names;
  TimedJointTrajectoryPointSeq points;

  bool Validate() const {
    if (joint_names.empty()) {
      return false;
    }

    if (points.empty()) {
      return false;
    }

    for (const TimedJointTrajectoryPoint& point : points) {
      if (point.positions.size() != joint_names.size()) {
        return false;
      }
    }
    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct TimedMultiDOFJointTrajectoryPoint {
  PoseSeq transforms;
  TwistSeq velocities;
  TwistSeq accelerations;
  double time_from_start;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::deque<TimedMultiDOFJointTrajectoryPoint> TimedMultiDOFJointTrajectoryPointSeq;

/// Timed multi dof joint trajectory (this is the one to support ROS)
struct TimedMultiDOFJointTrajectory {
  NameSeq joint_names;
  TimedMultiDOFJointTrajectoryPointSeq points;

  bool Validate() const {
    if (joint_names.empty()) {
      return false;
    }

    if (points.empty()) {
      return false;
    }

    for (const TimedMultiDOFJointTrajectoryPoint& point : points) {
      if (point.transforms.size() != joint_names.size()) {
        return false;
      }
    }
    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Timed robot trajectory (here is to support ROS)
struct TimedRobotTrajectory {
  TimedJointTrajectory joint_trajectory;
  TimedMultiDOFJointTrajectory multi_dof_joint_trajectory;

  bool Validate() const {
    return joint_trajectory.Validate() && multi_dof_joint_trajectory.Validate() &&
           joint_trajectory.points.size() == multi_dof_joint_trajectory.points.size();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// tmc_feasibility_checker
/// 2D interference check map metadata
struct MapMetaData {
  double resolution;
  uint32_t width;
  uint32_t height;
  Pose origin;
};

/// Grid data for 2D interference check
struct OccupancyGrid {
  MapMetaData info;
  std::vector<int8_t> data;
};

// tmc_robot_collision_detector
/// Parameters of objects for interference check
struct ObjectParameter {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  ObjectParameter() {
    name = "";
    shape.type = kSphere;
    margin = 0.0;
    transform.setIdentity();
    group = 0x0001;
    filter = 0xFFFF;
  }
  std::string name;
  Shape shape;
  double margin;
  Eigen::Affine3d transform;
  uint16_t group;
  uint16_t filter;
};  // struct ObjectParameter

typedef Eigen::Matrix<double, 3, 2> AABB;  /// [xmin xmax; ymin ymax; zmin zmax]

/// CUBOID for CollisionMap
struct Cuboid {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Cuboid() : margin(0.0), enable(false) {}

  Eigen::Affine3d box_transform;
  Eigen::Vector3d box_extents;
  AABB box_aabb;
  double margin;
  std::string box_name;
  mutable bool enable;

  bool operator< (const Cuboid& other) const {
    return box_aabb(0, 1) < other.box_aabb(0, 1);
  }
};  // struct Cuboid


struct OuterObjectParameters;
typedef std::vector<ObjectParameter,
                    Eigen::aligned_allocator<ObjectParameter> >
            ObjectParameterSeq;
typedef std::vector<OuterObjectParameters,
                    Eigen::aligned_allocator<OuterObjectParameters> >
            OuterObjectParametersSeq;
typedef std::vector<Cuboid, Eigen::aligned_allocator<Cuboid> >
            CuboidSeq;

/// @brief Parameters of external objects to be created
/// @attention
/// -When creating an object
///   Name, Base, Base_to_child, Shape must have appropriate values.
/// -Base is always updated, so if you get it with GetObjectParameter ()
///   You can also get the position posture of the object.
struct OuterObjectParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OuterObjectParameters() : margin(0.0) {}
  /// Object name
  std::string name;
  /// Object standard posture
  Eigen::Affine3d origin_to_base;
  /// Child object attitude toward standard posture
  std::vector<Eigen::Affine3d,
              Eigen::aligned_allocator<Eigen::Affine3d> > base_to_child;
  /// Child object shape
  std::vector<Shape> shape;
  /// Object margin
  double margin;
  /// Object name
  std::string parent_name;
  /// Relative attitude from parents
  Eigen::Affine3d parent_to_base;
  /// Group name that is understood
  std::string parent_group;
  /// Is it Cuboid?
  bool cuboid;
};

/// @brief Define Joint Limit
struct JointLimits {
  // joint name
  std::string joint_name;

  // true if the joint has position limits
  bool has_position_limits;

  // min position limits
  double min_position;
  // max position limits
  double max_position;

  // true if joint has velocity limits
  bool has_velocity_limits;

  // max velocity limit
  double max_velocity;
  // min_velocity is assumed to be -max_velocity

  // true if joint has acceleration limits
  bool has_acceleration_limits;
  // max acceleration limit
  double max_acceleration;
  // min_acceleration is assumed to be -max_acceleration

  // true if joint has effort limits
  bool has_effort_limits;
  // max effort limit
  double max_effort;
};

/// @brief Definition of freedom for robot reference points
/// ENUM for Base operation
enum BaseMovementType {
  kFloat,
  kPlanar,
  kRailX,
  kRailY,
  kRailZ,
  kRotationX,
  kRotationY,
  kRotationZ,
  kNone,
};

/// Get the degree of freedom of Base_type
/// @param[in] Base_type Base degree of freedom
/// @return Base freedom
inline uint32_t GetBaseDof(tmc_manipulation_types::BaseMovementType base_type) {
  uint32_t base_dof = 0;
  switch (base_type) {
    case tmc_manipulation_types::kFloat:
      base_dof = 6;
      break;
    case tmc_manipulation_types::kPlanar:
      base_dof = 3;
      break;
    case tmc_manipulation_types::kRailX:
      base_dof = 1;
      break;
    case tmc_manipulation_types::kRailY:
      base_dof = 1;
      break;
    case tmc_manipulation_types::kRailZ:
      base_dof = 1;
      break;
    case tmc_manipulation_types::kRotationX:
      base_dof = 1;
      break;
    case tmc_manipulation_types::kRotationY:
      base_dof = 1;
      break;
    case tmc_manipulation_types::kRotationZ:
      base_dof = 1;
      break;
    case tmc_manipulation_types::kNone:
      base_dof = 0;
      break;
    default:
      base_dof = 0;
      break;
  }
  return base_dof;
}

}  // namespace tmc_manipulation_types


template <typename T> std::ostream& SeqOut(std::ostream& os, const T& seq) {
  for (typename T::const_iterator it = seq.begin();
       it != seq.end();
       ++it) {
    os << *it << " ";
  }
  return os;
}

template <typename T> std::ostream& SeqOutMatrix(std::ostream& os, const T& seq) {
  for (typename T::const_iterator it = seq.begin();
       it != seq.end();
       ++it) {
    os << '\n' << it->matrix() << '\n';
  }
  return os;
}

template <typename T> std::ostream& SeqOutTrans(std::ostream& os, const T& seq) {
  for (typename T::const_iterator it = seq.begin();
       it != seq.end();
       ++it) {
    os << it->transpose() << '\n';
  }
  return os;
}


inline std::ostream& operator<<(
    std::ostream& os,
    const tmc_manipulation_types::JointState& state) {
  os << "name: ";
  SeqOut(os, state.name);
  os << '\n';
  os << "poistion: " << state.position.transpose() << '\n';
  os << "velocity: " << state.velocity.transpose() << '\n';
  os << "effort: " << state.effort.transpose() << '\n';
  return os;
}

inline std::ostream& operator<<(
    std::ostream& os,
    const tmc_manipulation_types::TaskSpaceRegion& tsr) {
  os << "origin_to_tsr: \n" << tsr.origin_to_tsr.matrix() << '\n';
  os << "tsr_to_end: \n" << tsr.tsr_to_end.matrix() << '\n';
  os << "min_bounds: " << tsr.min_bounds.transpose() << '\n';
  os << "max_bounds: " << tsr.max_bounds.transpose() << '\n';
  os << "origin_frame_id: " << tsr.origin_frame_id << '\n';
  os << "end_frame_id: " << tsr.end_frame_id << '\n';
  return os;
}

inline std::ostream& operator<<(
    std::ostream& os,
    const tmc_manipulation_types::MultiDOFJointState& state) {
  os << "name: \n";
  SeqOut(os, state.names);
  os << '\n';
  os << "poses: \n";
  SeqOutMatrix(os, state.poses);
  os << "twist: \n";
  SeqOutTrans(os, state.twist);
  os << "wrench: \n";
  SeqOutTrans(os, state.wrench);
  return os;
}

inline std::ostream& operator<<(
    std::ostream& os,
    const tmc_manipulation_types::RobotState& state) {
  os << "joint_state: \n" << state.joint_state << '\n';
  os << "multi_dof_joint_state: \n" << state.multi_dof_joint_state << '\n';
  return os;
}

inline std::ostream& operator<<(
    std::ostream& os,
    const tmc_manipulation_types::TimedJointTrajectoryPoint& point) {
  os << "positions: " << point.positions.transpose() << '\n';
  os << "velocities: " << point.velocities.transpose() << '\n';
  os << "accelerations: " << point.accelerations.transpose() << '\n';
  os << "effort: " << point.effort.transpose() << '\n';
  os << "time_from_start: " << point.time_from_start << '\n';
  os << '\n';
  return os;
}

inline std::ostream& operator<<(
    std::ostream& os,
    const tmc_manipulation_types::TimedJointTrajectory& traj) {
  os << "joint_names: ";
  SeqOut(os, traj.joint_names);
  os << '\n';
  SeqOut(os, traj.points);
  return os;
}

inline std::ostream& operator<<(
    std::ostream& os,
    const tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint& point) {
  os << "transforms: ";
  SeqOutMatrix(os, point.transforms);
  os << "velocities: ";
  SeqOutTrans(os, point.velocities);
  os << "accelerations: ";
  SeqOutTrans(os, point.accelerations);
  os << "time_from_start: " << point.time_from_start << '\n';
  return os;
}

inline std::ostream& operator<<(
    std::ostream& os,
    const tmc_manipulation_types::TimedMultiDOFJointTrajectory& traj) {
  os << "joint_names: ";
  SeqOut(os, traj.joint_names);
  os << '\n';
  SeqOut(os, traj.points);
  return os;
}

inline std::ostream& operator<<(
    std::ostream& os,
    const tmc_manipulation_types::TimedRobotTrajectory& traj) {
  os << "joint_trajectory: " << traj.joint_trajectory;
  os << "multi_dof_joint_trajectory: " << traj.multi_dof_joint_trajectory;
  return os;
}

#endif  // TMC_MANIPULATION_TYPES_MANIPULATION_TYPES_HPP_
