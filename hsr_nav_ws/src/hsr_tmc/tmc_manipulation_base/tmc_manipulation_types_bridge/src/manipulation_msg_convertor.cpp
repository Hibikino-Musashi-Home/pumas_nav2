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
/// @note     Applied for Partner-Robot Coding Rule(Ver:x.xx)

#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using std::vector;
using std::string;
// using trajectory_msgs::MultiDOFJointTrajectory;
// using trajectory_msgs::MultiDOFJointTrajectoryPoint;

namespace {
geometry_msgs::msg::Transform toMsg(const Eigen::Affine3d& in) { return tf2::eigenToTransform(in).transform; }
void fromMsg(const geometry_msgs::msg::Transform& in, Eigen::Affine3d& out) { out = tf2::transformToEigen(in); }

geometry_msgs::msg::Wrench toMsg(const tmc_manipulation_types::Wrench& in) {
  geometry_msgs::msg::Wrench out;
  (void)tf2::toMsg(in.head(3), out.force);
  (void)tf2::toMsg(in.tail(3), out.torque);
  return out;
}
void fromMsg(const geometry_msgs::msg::Wrench& in, tmc_manipulation_types::Wrench& out) {
  out << in.force.x, in.force.y, in.force.z, in.torque.x, in.torque.y, in.torque.z;
}
}  // namespace

namespace tmc_manipulation_types_bridge {

/// Divide the character string with arbitrary letters
/// @param[in] target_string Sign string to be divided
/// @param[in] delimiter Delimita (arbitrary character to be split key)
/// @return Separated string sequence (Types: Vector <string>)
std::vector<std::string> Split(const std::string& target_string,
                               const char* const delimiter) {
  vector<string> result;
  result.clear();
  size_t current = 0;
  size_t found = 0;
  while ((found = target_string.find(delimiter, current)) != string::npos) {
    result.push_back(string(target_string, current, found - current));
    current = found + string(delimiter).size();
  }
  result.push_back(string(target_string, current,
                          target_string.size() - current));
  return result;
}

/// JointState : Convert sensor_msgs to tmc_manipulation_types.
/// @param[in] sensor_msgs_joint_state (types : sensor_msgs)
/// @param[out] commmon_joint_state_out (types : tmc_manipulation_types)
void JointStateMsgToJointState(
    const sensor_msgs::msg::JointState& sensor_msgs_joint_state,
    tmc_manipulation_types::JointState& commmon_joint_state_out) {
  commmon_joint_state_out.name = sensor_msgs_joint_state.name;
  if (!sensor_msgs_joint_state.position.empty()) {
    commmon_joint_state_out.position =
        Eigen::Map<const Eigen::VectorXd>(
            &sensor_msgs_joint_state.position[0],
            sensor_msgs_joint_state.position.size());
  }
  if (!sensor_msgs_joint_state.velocity.empty()) {
    commmon_joint_state_out.velocity =
        Eigen::Map<const Eigen::VectorXd>(
            &sensor_msgs_joint_state.velocity[0],
            sensor_msgs_joint_state.velocity.size());
  }
  if (!sensor_msgs_joint_state.effort.empty()) {
    commmon_joint_state_out.effort =
        Eigen::Map<const Eigen::VectorXd>(
            &sensor_msgs_joint_state.effort[0],
            sensor_msgs_joint_state.effort.size());
  }
}

/// JointState : Convert tmc_manipulation_types to sensor_msgs.
/// @param[in] commmon_joint_state (types : tmc_manipulation_types)
/// @param[out] sensor_msgs_joint_state_out (types : sensor_msgs)
void JointStateToJointStateMsg(
    const tmc_manipulation_types::JointState& commmon_joint_state,
    sensor_msgs::msg::JointState& sensor_msgs_joint_state_out) {
  sensor_msgs_joint_state_out.name = commmon_joint_state.name;
  if (commmon_joint_state.position.size() != 0) {
    sensor_msgs_joint_state_out.position.resize(
        commmon_joint_state.position.size());
    Eigen::Map<Eigen::VectorXd> map_pos(
        &sensor_msgs_joint_state_out.position[0],
      sensor_msgs_joint_state_out.position.size());
    map_pos = commmon_joint_state.position;
  }
  if (commmon_joint_state.velocity.size() != 0) {
    sensor_msgs_joint_state_out.velocity.resize(
        commmon_joint_state.velocity.size());
    Eigen::Map<Eigen::VectorXd> map_vel(
        &sensor_msgs_joint_state_out.velocity[0],
      sensor_msgs_joint_state_out.velocity.size());
    map_vel = commmon_joint_state.velocity;
  }
  if (commmon_joint_state.effort.size() != 0) {
    sensor_msgs_joint_state_out.effort.resize(
        commmon_joint_state.effort.size());
    Eigen::Map<Eigen::VectorXd> map_effort(
        &sensor_msgs_joint_state_out.effort[0],
      sensor_msgs_joint_state_out.effort.size());
    map_effort = commmon_joint_state.effort;
  }
}

/// Convert JointPosition to Configure.
/// @param[in] Joint_position ROS message type JointPusions
/// @param[out] Common_config_out TMC_MANIPULATION_Types type type Configure
void JointPositionMsgToConfig(
    const tmc_planning_msgs::msg::JointPosition& joint_position,
    tmc_manipulation_types::Config& common_config_out) {
  common_config_out =
      Eigen::Map<const Eigen::VectorXd>(&joint_position.position[0],
                                         joint_position.position.size());
}

/// Convert Configure to JointPosition.
/// @param[in] common_config (types : tmc_manipulation_types)
/// @param[out] joint_position_out (types : tmc_planning_msgs)
void ConfigToJointPositionMsg(
    const tmc_manipulation_types::Config& common_config,
    tmc_planning_msgs::msg::JointPosition& joint_position_out) {
  joint_position_out.position.resize(common_config.size());
  Eigen::Map<Eigen::VectorXd> map(&joint_position_out.position[0],
                                  joint_position_out.position.size());
  map = common_config;
}

/// TaskSpaceRegion (Convert tmc_planning_msgs to tmc_manipulation_types.)
/// @param[in] planning_msgs_task_space_region (types : tmc_planning_msgs)
/// @param[out] common_task_space_region_out (types : tmc_manipulation_types)
void TaskSpaceRegionMsgToTaskSpaceRegion(
    const tmc_planning_msgs::msg::TaskSpaceRegion& planning_msgs_task_space_region,
    tmc_manipulation_types::TaskSpaceRegion& common_task_space_region_out) {
  Eigen::Affine3d origin_to_tsr;
  tf2::fromMsg(planning_msgs_task_space_region.origin_to_tsr, origin_to_tsr);
  common_task_space_region_out.origin_to_tsr = origin_to_tsr;
  Eigen::Affine3d tsr_to_end;
  tf2::fromMsg(planning_msgs_task_space_region.tsr_to_end, tsr_to_end);
  common_task_space_region_out.tsr_to_end = tsr_to_end;
  common_task_space_region_out.min_bounds =
      Eigen::Map<const Eigen::VectorXd>(
          &planning_msgs_task_space_region.min_bounds[0],
          planning_msgs_task_space_region.min_bounds.size());
  common_task_space_region_out.max_bounds =
      Eigen::Map<const Eigen::VectorXd>(
          &planning_msgs_task_space_region.max_bounds[0],
          planning_msgs_task_space_region.max_bounds.size());
  common_task_space_region_out.end_frame_id =
      planning_msgs_task_space_region.end_frame_id;
}

/// TaskSpaceRegion (Convert tmc_manipulation_types to tmc_planning_msgs.)
/// @param[in] common_task_space_region (types : tmc_manipulation_types)
/// @param[out] planning_msgs_task_space_region_out (types : tmc_planning_msgs)
void TaskSpaceRegionToTaskSpaceRegionMsg(
    const tmc_manipulation_types::TaskSpaceRegion& common_task_space_region,
    tmc_planning_msgs::msg::TaskSpaceRegion& planning_msgs_task_space_region_out) {
  planning_msgs_task_space_region_out.origin_to_tsr = tf2::toMsg(common_task_space_region.origin_to_tsr);
  planning_msgs_task_space_region_out.tsr_to_end = tf2::toMsg(common_task_space_region.tsr_to_end);
  for (int i = 0; i < common_task_space_region.min_bounds.size(); ++i) {
    planning_msgs_task_space_region_out.min_bounds[i] =
        Eigen::Map<const Eigen::VectorXd>(
        &common_task_space_region.min_bounds[0],
        common_task_space_region.min_bounds.size())[i];
    planning_msgs_task_space_region_out.max_bounds[i] =
        Eigen::Map<const Eigen::VectorXd>(
            &common_task_space_region.max_bounds[0],
            common_task_space_region.max_bounds.size())[i];
  }
  planning_msgs_task_space_region_out.end_frame_id =
      common_task_space_region.end_frame_id;
}

// /// Convert ObjectIdentifier to object name.
// /// @param[in] object_identifier (types : tmc_msgs)
// /// @param[out] object_name_out (types : string)
// void ObjectIdentifierMsgToObjectName(
//     const tmc_msgs::ObjectIdentifier& object_identifier,
//     std::string& object_name_out) {
//   object_name_out = object_identifier.name + "_" +
//       boost::lexical_cast<string>(object_identifier.object_id);
// }

// /// Convert object name to ObjectIdentifier.
// /// @param[in] object_name (types : string)
// /// @param[out] object_identifier_out (types : tmc_msgs)
// void ObjectNameToObjectIdentifierMsg(
//     const std::string& object_name,
//     tmc_msgs::ObjectIdentifier& object_identifier_out) {
//   vector<string> split_string = Split(object_name, "_");
//   object_identifier_out.name = split_string.at(0);
//   object_identifier_out.object_id =
//       boost::lexical_cast<uint32_t>(split_string.at(1));
// }

/// AttachedObject (Convert moveit_msgs to tmc_manipulation_types.)
/// @param[in] planning_msgs_attached_object (types : moveit_msgs)
/// @param[out] common_attached_object_out (types : tmc_manipulation_types)
void AttachedObjectMsgToAttachedObject(
    const moveit_msgs::msg::AttachedCollisionObject& planning_msgs_attached_object,
    tmc_manipulation_types::AttachedObject& common_attached_object_out) {
  common_attached_object_out.object_id = planning_msgs_attached_object.object.id;
  common_attached_object_out.frame_name = planning_msgs_attached_object.link_name;
  tf2::fromMsg(planning_msgs_attached_object.object.pose, common_attached_object_out.frame_to_object);
  common_attached_object_out.expected_objects.clear();
}

// /// AttachedObject (Convert tmc_manipulation_types to tmc_planning_msgs.)
// /// @param[in] common_attached_object (types : tmc_manipulation_types)
// /// @param[out] planning_msgs_attached_object_out (types : tmc_planning_msgs)
// void AttachedObjectToAttachedObjectMsg(
//     const tmc_manipulation_types::AttachedObject& common_attached_object,
//     tmc_planning_msgs::AttachedObject& planning_msgs_attached_object_out) {
//   planning_msgs_attached_object_out.object_id.name =
//       Split(common_attached_object.object_id, "_").at(0);
//   planning_msgs_attached_object_out.object_id.object_id =
//       std::atoi(Split(common_attached_object.object_id, "_").at(1).c_str());
//   planning_msgs_attached_object_out.frame_name =
//       common_attached_object.frame_name;
//   tf::poseEigenToMsg(
//       common_attached_object.frame_to_object,
//       planning_msgs_attached_object_out.frame_to_object);
//   planning_msgs_attached_object_out.check_outer_collision = false;
//   planning_msgs_attached_object_out.attached_group_id =
//       common_attached_object.group_id;
//   planning_msgs_attached_object_out.expected_object_ids.clear();
//   for (vector<string>::const_iterator
//       it = common_attached_object.expected_objects.begin();
//       it != common_attached_object.expected_objects.end(); ++it) {
//     vector<string> object = Split(*it, "_");
//     tmc_msgs::ObjectIdentifier out_element;
//     out_element.name = object.at(0);
//     out_element.object_id = boost::lexical_cast<uint32_t>(object.at(1));
//     planning_msgs_attached_object_out.expected_object_ids.push_back(
//         out_element);
//   }
// }

/// JointTrajectory (Convert trajectory_msgs to tmc_manipulation_types.)
/// @param[in] joint_trajectory_msg (types : trajectory_msgs)
/// @param[out] joint_trajectory_out (types : tmc_manipulation_types)
void JointTrajectoryMsgToJointTrajectory(
    const trajectory_msgs::msg::JointTrajectory& joint_trajectory_msg,
    tmc_manipulation_types::JointTrajectory& joint_trajectory_out) {
  joint_trajectory_out.names = joint_trajectory_msg.joint_names;
  joint_trajectory_out.path.resize(joint_trajectory_msg.points.size());

  for (auto i = 0; i < joint_trajectory_msg.points.size(); ++i) {
    joint_trajectory_out.path[i] =
        Eigen::Map<const Eigen::VectorXd>(
            &(joint_trajectory_msg.points[i].positions[0]),
            joint_trajectory_msg.points[i].positions.size());
  }
}

/// JointTrajectory (Convert tmc_manipulation_types to trajectory_msgs.)
/// @param[in] joint_trajectory (types : tmc_manipulation_types)
/// @param[out] joint_trajectory_msg_out (types : trajectory_msgs)
void JointTrajectoryToJointTrajectoryMsg(
    const tmc_manipulation_types::JointTrajectory& joint_trajectory,
    trajectory_msgs::msg::JointTrajectory& joint_trajectory_msg_out) {
  joint_trajectory_msg_out.joint_names = joint_trajectory.names;
  for (tmc_manipulation_types::Path::const_iterator point =
      joint_trajectory.path.begin();
      point != joint_trajectory.path.end(); ++point) {
    trajectory_msgs::msg::JointTrajectoryPoint point_msg;
    point_msg.positions.resize(point->size());
    Eigen::Map<Eigen::VectorXd> map(&point_msg.positions[0],
                                    point_msg.positions.size());
    map = *point;
    joint_trajectory_msg_out.points.push_back(point_msg);
  }
}

void SolidPrimitiveToShape(
    const shape_msgs::msg::SolidPrimitive& solid_primitive,
    tmc_manipulation_types::Shape& shape_out) {
  // Solid_primitive.Dimensions is Bounded Vector so it cannot be assigned
  shape_out.dimensions.assign(solid_primitive.dimensions.begin(), solid_primitive.dimensions.end());
  // TODO(Takeshita) 変換できないケースはどうしよう？
  switch (solid_primitive.type) {
    case shape_msgs::msg::SolidPrimitive::SPHERE:
      shape_out.type = tmc_manipulation_types::kSphere;
      break;
    case shape_msgs::msg::SolidPrimitive::BOX:
      shape_out.type = tmc_manipulation_types::kBox;
      break;
    case shape_msgs::msg::SolidPrimitive::CYLINDER:
      shape_out.type = tmc_manipulation_types::kCylinder;
      break;
    default:
      shape_out.dimensions.clear();
      break;
  }
}

void ShapeToSolidPrimitive(
    const tmc_manipulation_types::Shape& shape,
    shape_msgs::msg::SolidPrimitive& solid_primitive_out) {
  solid_primitive_out.dimensions.assign(shape.dimensions.begin(), shape.dimensions.end());
  switch (shape.type) {
    case tmc_manipulation_types::kSphere:
      solid_primitive_out.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      break;
    case tmc_manipulation_types::kBox:
      solid_primitive_out.type = shape_msgs::msg::SolidPrimitive::BOX;
      break;
    case tmc_manipulation_types::kCylinder:
      solid_primitive_out.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      break;
    default:
      solid_primitive_out.dimensions.clear();
      break;
  }
}

void MeshMsgToShape(
    const shape_msgs::msg::Mesh& mesh_msg,
    tmc_manipulation_types::Shape& shape_out) {
  shape_out.type = tmc_manipulation_types::kMeshVertices;
  for (auto i = 0u; i < mesh_msg.vertices.size(); ++i) {
    shape_out.vertices.emplace_back(Eigen::Vector3f(mesh_msg.vertices[i].x,
                                                    mesh_msg.vertices[i].y,
                                                    mesh_msg.vertices[i].z));
  }
  for (auto i = 0u; i < mesh_msg.triangles.size(); ++i) {
    shape_out.indices.push_back(mesh_msg.triangles[i].vertex_indices[0]);
    shape_out.indices.push_back(mesh_msg.triangles[i].vertex_indices[1]);
    shape_out.indices.push_back(mesh_msg.triangles[i].vertex_indices[2]);
  }
}

void ShapeToMeshMsg(
    const tmc_manipulation_types::Shape& shape,
    shape_msgs::msg::Mesh& mesh_msg_out) {
  for (auto i = 0u; i < shape.vertices.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = shape.vertices[i].x();
    point.y = shape.vertices[i].y();
    point.z = shape.vertices[i].z();
    mesh_msg_out.vertices.push_back(point);
  }
  for (auto i = 0u; i < shape.indices.size(); i += 3) {
    shape_msgs::msg::MeshTriangle triangle;
    triangle.vertex_indices[0] = shape.indices[i];
    triangle.vertex_indices[1] = shape.indices[i + 1];
    triangle.vertex_indices[2] = shape.indices[i + 2];
    mesh_msg_out.triangles.push_back(triangle);
  }
}

void OccupancyGridMsgToOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid& map_msg,
    tmc_manipulation_types::OccupancyGrid& map_out) {
  map_out.info.resolution = map_msg.info.resolution;
  map_out.info.width = map_msg.info.width;
  map_out.info.height = map_msg.info.height;
  tf2::fromMsg(map_msg.info.origin, map_out.info.origin);
  map_out.data.clear();
  std::copy(map_msg.data.begin(), map_msg.data.end(), std::back_inserter(map_out.data));
}

void CollisionObjectToOuterObjectParameters(
    const moveit_msgs::msg::CollisionObject& collision_object,
    tmc_manipulation_types::OuterObjectParameters& outer_object_out) {
  outer_object_out.name = collision_object.id;
  tf2::fromMsg(collision_object.pose, outer_object_out.origin_to_base);

  for (auto i = 0; i < collision_object.primitives.size(); ++i) {
    Eigen::Affine3d pose;
    tf2::fromMsg(collision_object.primitive_poses[i], pose);
    outer_object_out.base_to_child.emplace_back(pose);

    tmc_manipulation_types::Shape shape;
    SolidPrimitiveToShape(collision_object.primitives[i], shape);
    outer_object_out.shape.emplace_back(shape);
  }

  for (auto i = 0; i < collision_object.meshes.size(); ++i) {
    Eigen::Affine3d pose;
    tf2::fromMsg(collision_object.mesh_poses[i], pose);
    outer_object_out.base_to_child.emplace_back(pose);

    tmc_manipulation_types::Shape shape;
    MeshMsgToShape(collision_object.meshes[i], shape);
    outer_object_out.shape.emplace_back(shape);
  }
}

void OuterObjectParametersToCollisionObject(
    const tmc_manipulation_types::OuterObjectParameters& outer_object,
    moveit_msgs::msg::CollisionObject& collision_object_out) {
  collision_object_out.id = outer_object.name;
  collision_object_out.pose = tf2::toMsg(outer_object.origin_to_base);

  for (auto i = 0; i < outer_object.shape.size(); ++i) {
    if ((outer_object.shape[i].type == tmc_manipulation_types::kMesh) ||
        (outer_object.shape[i].type == tmc_manipulation_types::kMeshVertices)) {
      shape_msgs::msg::Mesh mesh_msg;
      ShapeToMeshMsg(outer_object.shape[i], mesh_msg);
      collision_object_out.meshes.push_back(mesh_msg);
      collision_object_out.mesh_poses.push_back(tf2::toMsg(outer_object.base_to_child[i]));
    } else {
      shape_msgs::msg::SolidPrimitive solid_primitive;
      ShapeToSolidPrimitive(outer_object.shape[i], solid_primitive);
      collision_object_out.primitives.push_back(solid_primitive);
      collision_object_out.primitive_poses.push_back(tf2::toMsg(outer_object.base_to_child[i]));
    }
  }
}

/// Convert tmc_collision_detector :: Shape to Visualization_msgs :: Marker.
/// @note rviz indicates
/// @param[in] collision_detector_shape: tmc_collision_detector::Shape
/// @param[in] Pose: Location and posture
/// @param[in] id: ID
/// @param[in] Frame_id: Frame ID
/// @param[in] Time: Time stamp
/// @param[in] color: Color
/// @param[out] marker_msgs_out: visualization_msgs::Marker
void PlannerShapeToMarkerMsg(
    const tmc_manipulation_types::Shape& collision_detector_shape,
    const tmc_manipulation_types::Transform& pose,
    int32_t id,
    const std::string& frame_id,
    const rclcpp::Time& time,
    const std_msgs::msg::ColorRGBA& color,
    visualization_msgs::msg::Marker& marker_msgs_out) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = time;
  marker.ns = "tmc_manipulation_types_bridge";
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color = color;
  marker.pose = tf2::toMsg(pose);

  switch (collision_detector_shape.type) {
    case(tmc_manipulation_types::kBox) : {
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.scale.x = collision_detector_shape.dimensions.at(0);
      marker.scale.y = collision_detector_shape.dimensions.at(1);
      marker.scale.z = collision_detector_shape.dimensions.at(2);
      break;
    }
    case(tmc_manipulation_types::kCylinder) : {
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      // dimensions[0] is radius.
      marker.scale.x = collision_detector_shape.dimensions.at(0) * 2.0;
      marker.scale.y = collision_detector_shape.dimensions.at(0) * 2.0;
      marker.scale.z = collision_detector_shape.dimensions.at(1);
      break;
    }
    case(tmc_manipulation_types::kSphere) : {
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.scale.x = collision_detector_shape.dimensions.at(0) * 2.0;
      marker.scale.y = collision_detector_shape.dimensions.at(0) * 2.0;
      marker.scale.z = collision_detector_shape.dimensions.at(0) * 2.0;
      break;
    }
    case(tmc_manipulation_types::kCapsule) : {
      // Since there is no capsule, substitute with cylinder
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      // dimensions[0] is radius.
      marker.scale.x = collision_detector_shape.dimensions.at(0) * 2.0;
      marker.scale.y = collision_detector_shape.dimensions.at(0) * 2.0;
      marker.scale.z = collision_detector_shape.dimensions.at(1);
      break;
    }
    case(tmc_manipulation_types::kMesh) : {
      // mesh
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      // Determine whether Filename is a URL format
      if ((collision_detector_shape.filename.find("package://") == 0) ||
          (collision_detector_shape.filename.find("file://") == 0) ||
          (collision_detector_shape.filename.find("http://") == 0)) {
        marker.mesh_resource = collision_detector_shape.filename;
      } else {
        marker.mesh_resource = "file://" + collision_detector_shape.filename;
      }
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      break;
    }
    // TODO(Takeshita) kMeshVertices
  }
  marker_msgs_out = marker;
}

/// Convert tmc_robot_collision_detector :: outerObjectParameters to Visualization_msgs::Marker.
/// @note rviz indicates
/// @param[in] outer_object_parameters: tmc_robot_collision_detector
/// @param[in] Start_id: Start ID (serial number from START_ID due to multiple markers)
/// @param[in] Frame_id: Frame ID
/// @param[in] Time: Time stamp
/// @param[in] color: Color
/// @param[out] marker_msgs_out: visualization_msgs::Marker
void OuterObjectParametersToMarkerMsg(
    const tmc_manipulation_types::OuterObjectParameters& outer_object_parameters,
    int32_t start_id,
    const std::string& frame_id,
    const rclcpp::Time& time,
    const std_msgs::msg::ColorRGBA& color,
    std::vector<visualization_msgs::msg::Marker>& marker_msgs_out) {
  marker_msgs_out.clear();
  for (size_t i = 0; i < outer_object_parameters.base_to_child.size(); ++i) {
    tmc_manipulation_types::Transform origin_to_shape =
        outer_object_parameters.origin_to_base * outer_object_parameters.base_to_child.at(i);
    visualization_msgs::msg::Marker marker;
    PlannerShapeToMarkerMsg(outer_object_parameters.shape.at(i),
                            origin_to_shape,
                            start_id + i,
                            frame_id,
                            time,
                            color,
                            marker);
    marker_msgs_out.emplace_back(marker);
  }
}

// /// TMC_MANIPULATION_MSGS :: Known_object of CollisonEnvironment
// /// Convert to vector of TMC_ROBOT_COLLISION_Detector :: OuterobjectParameters
// /// @param[in] Environmental information for Collision_environment detection
// /// @param[out] outer_object_seq_out
// ///             Converted Known_object of OuterobjectParameters
// /// @par Remarks: The object name after conversion is the object name_ object ID.
// void CollisionEnvironmentToOuterObjectSeq(
//     const tmc_manipulation_msgs::CollisionEnvironment& collision_environment,
//     tmc_manipulation_types::OuterObjectParametersSeq& outer_object_seq_out) {
//   for (uint32_t i = 0; i < collision_environment.known_objects.size(); ++i) {
//     tmc_manipulation_msgs::CollisionObject obj_msg =
//         collision_environment.known_objects[i];
//     tmc_manipulation_types::OuterObjectParameters obj;
//     obj.name = obj_msg.id.name + "_" +
//                boost::lexical_cast<string>(obj_msg.id.object_id);
//     tf::poseMsgToEigen(collision_environment.poses[i], obj.origin_to_base);
//     obj.shape.resize(obj_msg.shapes.size());
//     obj.base_to_child.resize(obj_msg.shapes.size());
//     for (uint32_t j = 0; j < obj_msg.shapes.size(); ++j) {
//       ShapeMsgToPlannerShape(obj_msg.shapes[j], obj.shape[j]);
//       tf::poseMsgToEigen(obj_msg.poses[j], obj.base_to_child[j]);
//     }
//     outer_object_seq_out.push_back(obj);
//   }
// }

// /// Collision_map of TMC_MANIPULATION_MSGS :: CollisonEnvironment
// /// Convert to vector of TMC_ROBOT_COLLIS_DETECTOR :: Cuboid
// /// @param[in] Environmental information for Collision_environment detection
// /// @param[in] Box_name Collision_map name
// /// @param[out] Vector and Collision_map of Cuboid_seq_out Cuboid
// /// @par Remarks: The number of objects after conversion is name_ number (automatically shaken).
// void CollisionEnvironmentToCuboidSeq(
//     const tmc_manipulation_msgs::CollisionEnvironment& collision_environment,
//     const std::string& box_name,
//     tmc_manipulation_types::CuboidSeq& cuboid_seq_out) {
//   tmc_manipulation_types::Cuboid cuboid;
//   int32_t counter = 0;
//   for (vector<tmc_geometry_msgs::OrientedBoundingBox>::const_iterator
//       it = collision_environment.collision_map.boxes.begin();
//       it != collision_environment.collision_map.boxes.end(); ++it) {
//     cuboid.box_name.assign(box_name);
//     cuboid.box_name += boost::lexical_cast<string>(counter);
//     cuboid.box_extents =
//         Eigen::Vector3d(it->extents.x, it->extents.y, it->extents.z);
//     cuboid.box_transform =
//         Eigen::Translation3d(it->center.x, it->center.y, it->center.z) *
//         Eigen::AngleAxisd(it->angle, Eigen::Vector3d(it->axis.x,
//                                                      it->axis.y,
//                                                      it->axis.z));
//     Eigen::Affine3d map_transform;
//     tf::poseMsgToEigen(
//         collision_environment.collision_map_pose, map_transform);
//     cuboid.box_transform = map_transform * cuboid.box_transform;

//     Eigen::Matrix3d linear(cuboid.box_transform.linear());
//     Eigen::Matrix<double, 3, 8> vertices;
//     vertices << cuboid.box_extents(0), cuboid.box_extents(0),
//                 cuboid.box_extents(0), cuboid.box_extents(0),
//                 -cuboid.box_extents(0), -cuboid.box_extents(0),
//                 -cuboid.box_extents(0), -cuboid.box_extents(0),
//                 cuboid.box_extents(1), cuboid.box_extents(1),
//                 -cuboid.box_extents(1), -cuboid.box_extents(1),
//                 cuboid.box_extents(1), cuboid.box_extents(1),
//                 -cuboid.box_extents(1), -cuboid.box_extents(1),
//                 cuboid.box_extents(2), -cuboid.box_extents(2),
//                 cuboid.box_extents(2), -cuboid.box_extents(2),
//                 cuboid.box_extents(2), -cuboid.box_extents(2),
//                 cuboid.box_extents(2), -cuboid.box_extents(2);
//     Eigen::Matrix<double, 3, 8> rotated_vertices(linear * vertices);
//     Eigen::Vector3d rotated_extents(rotated_vertices.row(0).maxCoeff() / 2.0,
//                                     rotated_vertices.row(1).maxCoeff() / 2.0,
//                                     rotated_vertices.row(2).maxCoeff() / 2.0);

//     cuboid.box_aabb <<
//         cuboid.box_transform.translation().coeff(0) - rotated_extents(0),
//         cuboid.box_transform.translation().coeff(0) + rotated_extents(0),
//         cuboid.box_transform.translation().coeff(1) - rotated_extents(1),
//         cuboid.box_transform.translation().coeff(1) + rotated_extents(1),
//         cuboid.box_transform.translation().coeff(2) - rotated_extents(2),
//         cuboid.box_transform.translation().coeff(2) + rotated_extents(2);

//     cuboid_seq_out.push_back(cuboid);
//     ++counter;
//   }
// }

// /// tmc_manipulation_msgs::CollisionMapを
// /// Convert to vector of TMC_ROBOT_COLLIS_DETECTOR :: Cuboid
// /// @param[in] Environmental information for Collision_map interference detection
// /// @param[in] Box_name Collision_map name
// /// @param[in] Origin_to_map Collision_map standard coordinates
// /// @param[out] cuboid_seq_out Cuboidのvector
// /// @par Remarks: The number of objects after conversion is name_ number (automatically shaken).
// void CollisionMapToCuboidSeq(
//     const tmc_mapping_msgs::CollisionMap& collision_map,
//     const std::string& box_name,
//     const Eigen::Affine3d& origin_to_map,
//     tmc_manipulation_types::CuboidSeq& cuboid_seq_out) {
//   tmc_manipulation_types::Cuboid cuboid;
//   int32_t counter = 0;
//   for (vector<tmc_geometry_msgs::OrientedBoundingBox>::const_iterator
//       it = collision_map.boxes.begin();
//       it != collision_map.boxes.end(); ++it) {
//     cuboid.box_name.assign(box_name);
//     cuboid.box_name += boost::lexical_cast<string>(counter);
//     cuboid.box_extents =
//         Eigen::Vector3d(it->extents.x, it->extents.y, it->extents.z);
//     cuboid.box_transform =
//         Eigen::Translation3d(it->center.x, it->center.y, it->center.z)
//           * Eigen::AngleAxisd(it->angle, Eigen::Vector3d(it->axis.x,
//                                                          it->axis.y,
//                                                          it->axis.z));

//     cuboid.box_transform = origin_to_map * cuboid.box_transform;

//     Eigen::Matrix3d linear(cuboid.box_transform.linear());
//     Eigen::Matrix<double, 3, 8> vertices;
//     vertices << cuboid.box_extents(0), cuboid.box_extents(0),
//                 cuboid.box_extents(0), cuboid.box_extents(0),
//                 -cuboid.box_extents(0), -cuboid.box_extents(0),
//                 -cuboid.box_extents(0), -cuboid.box_extents(0),
//                 cuboid.box_extents(1), cuboid.box_extents(1),
//                 -cuboid.box_extents(1), -cuboid.box_extents(1),
//                 cuboid.box_extents(1), cuboid.box_extents(1),
//                 -cuboid.box_extents(1), -cuboid.box_extents(1),
//                 cuboid.box_extents(2), -cuboid.box_extents(2),
//                 cuboid.box_extents(2), -cuboid.box_extents(2),
//                 cuboid.box_extents(2), -cuboid.box_extents(2),
//                 cuboid.box_extents(2), -cuboid.box_extents(2);
//     Eigen::Matrix<double, 3, 8> rotated_vertices(linear * vertices);
//     Eigen::Vector3d rotated_extents(rotated_vertices.row(0).maxCoeff() / 2.0,
//                                     rotated_vertices.row(1).maxCoeff() / 2.0,
//                                     rotated_vertices.row(2).maxCoeff() / 2.0);

//     cuboid.box_aabb <<
//         cuboid.box_transform.translation().coeff(0) - rotated_extents(0),
//         cuboid.box_transform.translation().coeff(0) + rotated_extents(0),
//         cuboid.box_transform.translation().coeff(1) - rotated_extents(1),
//         cuboid.box_transform.translation().coeff(1) + rotated_extents(1),
//         cuboid.box_transform.translation().coeff(2) - rotated_extents(2),
//         cuboid.box_transform.translation().coeff(2) + rotated_extents(2);

//     cuboid_seq_out.push_back(cuboid);
//     counter++;
//   }
// }

void BaseMovementTypeToBaseMovementMsg(
    const tmc_manipulation_types::BaseMovementType& base_type,
    tmc_manipulation_msgs::msg::BaseMovementType& base_type_msgs_out) {
  using MsgType = tmc_manipulation_msgs::msg::BaseMovementType;
  switch (base_type) {
    case tmc_manipulation_types::kFloat:
      base_type_msgs_out.val = MsgType::FLOAT;
      break;
    case tmc_manipulation_types::kPlanar:
      base_type_msgs_out.val = MsgType::PLANAR;
      break;
    case tmc_manipulation_types::kRailX:
      base_type_msgs_out.val = MsgType::RAIL_X;
      break;
    case tmc_manipulation_types::kRailY:
      base_type_msgs_out.val = MsgType::RAIL_Y;
      break;
    case tmc_manipulation_types::kRailZ:
      base_type_msgs_out.val = MsgType::RAIL_Z;
      break;
    case tmc_manipulation_types::kRotationX:
      base_type_msgs_out.val = MsgType::ROTATION_X;
      break;
    case tmc_manipulation_types::kRotationY:
      base_type_msgs_out.val = MsgType::ROTATION_Y;
      break;
    case tmc_manipulation_types::kRotationZ:
      base_type_msgs_out.val = MsgType::ROTATION_Z;
      break;
    case tmc_manipulation_types::kNone:
      base_type_msgs_out.val = MsgType::NONE;
      break;
    default:
      break;
  }
}

void BaseMovementTypeMsgToBaseMovement(
    const tmc_manipulation_msgs::msg::BaseMovementType& base_type_msg,
    tmc_manipulation_types::BaseMovementType& base_type_out) {
  using MsgType = tmc_manipulation_msgs::msg::BaseMovementType;
  switch (base_type_msg.val) {
    case MsgType::FLOAT:
      base_type_out = tmc_manipulation_types::kFloat;
      break;
    case MsgType::PLANAR:
      base_type_out = tmc_manipulation_types::kPlanar;
      break;
    case MsgType::RAIL_X:
      base_type_out = tmc_manipulation_types::kRailX;
      break;
    case MsgType::RAIL_Y:
      base_type_out = tmc_manipulation_types::kRailY;
      break;
    case MsgType::RAIL_Z:
      base_type_out = tmc_manipulation_types::kRailZ;
      break;
    case MsgType::ROTATION_X:
      base_type_out = tmc_manipulation_types::kRotationX;
      break;
    case MsgType::ROTATION_Y:
      base_type_out = tmc_manipulation_types::kRotationY;
      break;
    case MsgType::ROTATION_Z:
      base_type_out = tmc_manipulation_types::kRotationZ;
      break;
    case MsgType::NONE:
      base_type_out = tmc_manipulation_types::kNone;
      break;
    default:
      break;
  }
}

/// Convert MultidofJointTrajectory into a message
/// @param[in] multi_dof_jointtrajectory manipulation type
/// @param[out] multi_dof_jointtrajectory_msgs_out msg type
void MultiDOFJointTrajectoryToMultiDOFJointTrajectoryMsg(
    const tmc_manipulation_types::MultiDOFJointTrajectory& multi_dof_trajectory,
    trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory_msg_out) {
  multi_dof_trajectory_msg_out.joint_names = multi_dof_trajectory.names;
  multi_dof_trajectory_msg_out.points.clear();
  for (const auto& pose_seq : multi_dof_trajectory.path) {
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
    ConvertSequenceWithEigenIn<Eigen::Affine3d, geometry_msgs::msg::Transform>(
        pose_seq,
        point.transforms,
        std::bind<geometry_msgs::msg::Transform(const Eigen::Affine3d&)>(toMsg, std::placeholders::_1));
    multi_dof_trajectory_msg_out.points.push_back(point);
  }
}

/// Convert MultidofJointTrajectoryMSG to a common type
/// @param[in] multi_dof_jointtrajectory_msg msg type
/// @param[out] multi_dof_jointtrajectory_out manipulation type
void MultiDOFJointTrajectoryMsgToMultiDOFJointTrajectory(
    const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory_msg,
    tmc_manipulation_types::MultiDOFJointTrajectory& multi_dof_trajectory_out) {
  multi_dof_trajectory_out.path.clear();
  multi_dof_trajectory_out.names = multi_dof_trajectory_msg.joint_names;
  for (const auto& point : multi_dof_trajectory_msg.points) {
    tmc_manipulation_types::PoseSeq transforms;
    ConvertSequenceWithEigenOut<geometry_msgs::msg::Transform, Eigen::Affine3d>(
        point.transforms,
        transforms,
        std::bind<void(const geometry_msgs::msg::Transform&, Eigen::Affine3d&)>(
            fromMsg, std::placeholders::_1, std::placeholders::_2));
    multi_dof_trajectory_out.path.push_back(transforms);
  }
}

/// Convert TimeDJointTrajectory into a message
/// @param[in] trajectory Input trajectory.
/// @param[out] trajectory_msgs_out Output trajectory.
void TimedJointTrajectoryToJointTrajectoryMsg(
    const tmc_manipulation_types::TimedJointTrajectory& trajectory,
    trajectory_msgs::msg::JointTrajectory& trajectory_msg_out) {
  trajectory_msg_out.points.clear();
  trajectory_msg_out.joint_names = trajectory.joint_names;
  for (const auto& point : trajectory.points) {
    trajectory_msgs::msg::JointTrajectoryPoint point_msg;
    point_msg.positions.resize(point.positions.size());
    point_msg.velocities.resize(point.velocities.size());
    point_msg.accelerations.resize(point.accelerations.size());
    point_msg.effort.resize(point.effort.size());
    point_msg.time_from_start = rclcpp::Duration::from_seconds(point.time_from_start);
    Eigen::Map<Eigen::VectorXd> map_pos(&point_msg.positions[0],
                                        point_msg.positions.size());
    Eigen::Map<Eigen::VectorXd> map_vel(&point_msg.velocities[0],
                                        point_msg.velocities.size());
    Eigen::Map<Eigen::VectorXd> map_acc(&point_msg.accelerations[0],
                                        point_msg.accelerations.size());
    Eigen::Map<Eigen::VectorXd> map_effort(&point_msg.effort[0],
                                           point_msg.effort.size());
    map_pos = point.positions;
    map_vel = point.velocities;
    map_acc = point.accelerations;
    map_effort = point.effort;
    trajectory_msg_out.points.push_back(point_msg);
  }
}

/// TimedJointTrajectory (Convert trajectory_msgs to tmc_manipulation_types.)
/// @param[in] joint_trajectory_msg msg type
/// @param[out] joint_trajectory_out manipulation type
void JointTrajectoryMsgToTimedJointTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory_msg,
    tmc_manipulation_types::TimedJointTrajectory& trajectory_out) {
  trajectory_out.joint_names = trajectory_msg.joint_names;
  trajectory_out.points.clear();

  for (const auto& point_msg : trajectory_msg.points) {
    tmc_manipulation_types::TimedJointTrajectoryPoint point;
    point.positions =
        Eigen::Map<const Eigen::VectorXd>(
            &(point_msg.positions[0]),
            point_msg.positions.size());
    point.velocities =
        Eigen::Map<const Eigen::VectorXd>(
            &(point_msg.velocities[0]),
            point_msg.velocities.size());
    point.accelerations =
        Eigen::Map<const Eigen::VectorXd>(
            &(point_msg.accelerations[0]),
            point_msg.accelerations.size());
    point.effort =
        Eigen::Map<const Eigen::VectorXd>(
            &(point_msg.effort[0]),
            point_msg.effort.size());
    point.time_from_start = rclcpp::Duration(point_msg.time_from_start).seconds();
    trajectory_out.points.push_back(point);
  }
}

/// Convert MultidofJointState into a message
/// @param[in] multi_dof_jointstate manipulation type
/// @param[out] multi_dof_jointstate_msgs_out msg type
void MultiDOFJointStateToMultiDOFJointStateMsg(
    const tmc_manipulation_types::MultiDOFJointState& multi_dof_state,
    sensor_msgs::msg::MultiDOFJointState& multi_dof_state_msg_out) {
  multi_dof_state_msg_out.joint_names = multi_dof_state.names;
  ConvertSequenceWithEigenIn<Eigen::Affine3d, geometry_msgs::msg::Transform>(
      multi_dof_state.poses,
      multi_dof_state_msg_out.transforms,
      std::bind<geometry_msgs::msg::Transform(const Eigen::Affine3d&)>(toMsg, std::placeholders::_1));
  ConvertSequenceWithEigenIn<tmc_manipulation_types::Twist, geometry_msgs::msg::Twist>(
      multi_dof_state.twist,
      multi_dof_state_msg_out.twist,
      std::bind<geometry_msgs::msg::Twist(const tmc_manipulation_types::Twist&)>(tf2::toMsg, std::placeholders::_1));
  ConvertSequenceWithEigenIn<tmc_manipulation_types::Wrench, geometry_msgs::msg::Wrench>(
      multi_dof_state.wrench,
      multi_dof_state_msg_out.wrench,
      std::bind<geometry_msgs::msg::Wrench(const tmc_manipulation_types::Wrench&)>(toMsg, std::placeholders::_1));
}

/// Convert MultidofJointstateMSG to a common type
/// @param[in] multi_dof_jointstate_msg msg type
/// @param[out] multi_dof_jointstate_out manipulation type
void MultiDOFJointStateMsgToMultiDOFJointState(
    const sensor_msgs::msg::MultiDOFJointState& multi_dof_state_msg,
    tmc_manipulation_types::MultiDOFJointState& multi_dof_state_out) {
  multi_dof_state_out.names = multi_dof_state_msg.joint_names;
  ConvertSequenceWithEigenOut<geometry_msgs::msg::Transform, Eigen::Affine3d>(
      multi_dof_state_msg.transforms,
      multi_dof_state_out.poses,
      std::bind<void(const geometry_msgs::msg::Transform&, Eigen::Affine3d&)>(
          fromMsg, std::placeholders::_1, std::placeholders::_2));
  ConvertSequenceWithEigenOut<geometry_msgs::msg::Twist, tmc_manipulation_types::Twist>(
      multi_dof_state_msg.twist,
      multi_dof_state_out.twist,
      std::bind<void(const geometry_msgs::msg::Twist&, tmc_manipulation_types::Twist&)>(
          tf2::fromMsg, std::placeholders::_1, std::placeholders::_2));
  ConvertSequenceWithEigenOut<geometry_msgs::msg::Wrench, tmc_manipulation_types::Wrench>(
      multi_dof_state_msg.wrench,
      multi_dof_state_out.wrench,
      std::bind<void(const geometry_msgs::msg::Wrench&, tmc_manipulation_types::Wrench&)>(
          fromMsg, std::placeholders::_1, std::placeholders::_2));
}

/// Convert MultidofJointTrajectory into a message
/// @param[in] multi_dof_jointtrajectory manipulation type
/// @param[out] multi_dof_jointtrajectory_msgs_out msg type
void TimedMultiDOFJointTrajectoryToMultiDOFJointTrajectoryMsg(
    const tmc_manipulation_types::TimedMultiDOFJointTrajectory& multi_dof_trajectory,
    trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory_msg_out) {
  multi_dof_trajectory_msg_out.joint_names = multi_dof_trajectory.joint_names;
  multi_dof_trajectory_msg_out.points.clear();
  for (const auto& point : multi_dof_trajectory.points) {
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point_msg;
    point_msg.time_from_start = rclcpp::Duration::from_seconds(point.time_from_start);
    ConvertSequenceWithEigenIn<Eigen::Affine3d, geometry_msgs::msg::Transform>(
        point.transforms,
        point_msg.transforms,
        std::bind<geometry_msgs::msg::Transform(const Eigen::Affine3d&)>(toMsg, std::placeholders::_1));
    ConvertSequenceWithEigenIn<tmc_manipulation_types::Twist, geometry_msgs::msg::Twist>(
        point.velocities,
        point_msg.velocities,
        std::bind<geometry_msgs::msg::Twist(const tmc_manipulation_types::Twist&)>(tf2::toMsg, std::placeholders::_1));
    ConvertSequenceWithEigenIn<tmc_manipulation_types::Twist, geometry_msgs::msg::Twist>(
        point.accelerations,
        point_msg.accelerations,
        std::bind<geometry_msgs::msg::Twist(const tmc_manipulation_types::Twist&)>(tf2::toMsg, std::placeholders::_1));
    multi_dof_trajectory_msg_out.points.push_back(point_msg);
  }
}

/// Convert MultidofJointTrajectoryMSG to a common type
/// @param[in] multi_dof_trajectory_msg msg type
/// @param[out] multi_dof_trajectory_out manipulation type
void MultiDOFJointTrajectoryMsgToTimedMultiDOFJointTrajectory(
    const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory_msg,
    tmc_manipulation_types::TimedMultiDOFJointTrajectory& multi_dof_trajectory_out) {
  multi_dof_trajectory_out.joint_names = multi_dof_trajectory_msg.joint_names;
  for (const auto& point_msg : multi_dof_trajectory_msg.points) {
    tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration(point_msg.time_from_start).seconds();
    ConvertSequenceWithEigenOut<geometry_msgs::msg::Transform, Eigen::Affine3d>(
        point_msg.transforms,
        point.transforms,
        std::bind<void(const geometry_msgs::msg::Transform&, Eigen::Affine3d&)>(
            fromMsg, std::placeholders::_1, std::placeholders::_2));
    ConvertSequenceWithEigenOut<geometry_msgs::msg::Twist, tmc_manipulation_types::Twist>(
        point_msg.velocities,
        point.velocities,
        std::bind<void(const geometry_msgs::msg::Twist&, tmc_manipulation_types::Twist&)>(
            tf2::fromMsg, std::placeholders::_1, std::placeholders::_2));
    ConvertSequenceWithEigenOut<geometry_msgs::msg::Twist, tmc_manipulation_types::Twist>(
        point_msg.accelerations,
        point.accelerations,
        std::bind<void(const geometry_msgs::msg::Twist&, tmc_manipulation_types::Twist&)>(
            tf2::fromMsg, std::placeholders::_1, std::placeholders::_2));
    multi_dof_trajectory_out.points.push_back(point);
  }
}

/// Convert Robotstate into MSG type
/// @param[in] robot_state manipulation type
/// @param[out] robot_state_msg
void RobotStateToRobotStateMsg(
    const tmc_manipulation_types::RobotState& robot_state,
    moveit_msgs::msg::RobotState& robot_state_msg_out) {
  JointStateToJointStateMsg(robot_state.joint_state,
                            robot_state_msg_out.joint_state);
  MultiDOFJointStateToMultiDOFJointStateMsg(
      robot_state.multi_dof_joint_state,
      robot_state_msg_out.multi_dof_joint_state);
}

/// Convert Robotstate MSG to manipulation type
/// @param[in] robot_state manipulation type
/// @param[out] robot_state_msg
void RobotStateMsgToRobotState(
    const moveit_msgs::msg::RobotState& robot_state_msg,
    tmc_manipulation_types::RobotState& robot_state_out) {
  JointStateMsgToJointState(robot_state_msg.joint_state,
                            robot_state_out.joint_state);
  MultiDOFJointStateMsgToMultiDOFJointState(
      robot_state_msg.multi_dof_joint_state,
      robot_state_out.multi_dof_joint_state);
}


/// Convert Robottrajectory into MSG type
/// @param[in] robot_trajectory manipulation type
/// @param[out] robot_trajectory_msg
void TimedRobotTrajectoryToRobotTrajectoryMsg(
    const tmc_manipulation_types::TimedRobotTrajectory& robot_trajectory,
    moveit_msgs::msg::RobotTrajectory& robot_trajectory_msg_out) {
  TimedJointTrajectoryToJointTrajectoryMsg(robot_trajectory.joint_trajectory,
                                           robot_trajectory_msg_out.joint_trajectory);
  TimedMultiDOFJointTrajectoryToMultiDOFJointTrajectoryMsg(
      robot_trajectory.multi_dof_joint_trajectory,
      robot_trajectory_msg_out.multi_dof_joint_trajectory);
}

/// Convert Robottrajectory MSG to Manipulation type
/// @param[in] robot_trajectory manipulation type
/// @param[out] robot_trajectory_msg
void RobotTrajectoryMsgToTimedRobotTrajectory(
    const moveit_msgs::msg::RobotTrajectory& robot_trajectory_msg,
    tmc_manipulation_types::TimedRobotTrajectory& robot_trajectory_out) {
  JointTrajectoryMsgToTimedJointTrajectory
      (robot_trajectory_msg.joint_trajectory,
       robot_trajectory_out.joint_trajectory);
  MultiDOFJointTrajectoryMsgToTimedMultiDOFJointTrajectory(
      robot_trajectory_msg.multi_dof_joint_trajectory,
      robot_trajectory_out.multi_dof_joint_trajectory);
}

// /// Convert Jointlimits MSG to Manipulation type
// /// @param[in] joint_limit_msg msg type
// /// @param[out] joint_limit_out manipulation_type
// void JointLimitsMsgToJointLimits(
//     const tmc_manipulation_msgs::JointLimits& joint_limit_msg,
//     tmc_manipulation_types::JointLimits& joint_limit_out) {
//   joint_limit_out.joint_name = joint_limit_msg.joint_name;

//   joint_limit_out.has_position_limits = joint_limit_msg.has_position_limits;
//   joint_limit_out.min_position = joint_limit_msg.min_position;
//   joint_limit_out.max_position = joint_limit_msg.max_position;

//   joint_limit_out.has_velocity_limits = joint_limit_msg.has_velocity_limits;
//   joint_limit_out.max_velocity = joint_limit_msg.max_velocity;

//   joint_limit_out.has_acceleration_limits = joint_limit_msg.has_acceleration_limits;
//   joint_limit_out.max_acceleration = joint_limit_msg.max_acceleration;
// }

// /// Convert Jointlimits Manipulation type to MSG
// /// @param[in] joint_limit manipulation_type
// /// @param[out] joint_limit_msg_out msg_type
// void JointLimitsToJointLimitsMsg(
//     const tmc_manipulation_msgs::JointLimits& joint_limit,
//     tmc_manipulation_types::JointLimits& joint_limit_msg_out) {

//   joint_limit_msg_out.joint_name = joint_limit.joint_name;

//   joint_limit_msg_out.has_position_limits = joint_limit.has_position_limits;
//   joint_limit_msg_out.min_position = joint_limit.min_position;
//   joint_limit_msg_out.max_position = joint_limit.max_position;

//   joint_limit_msg_out.has_velocity_limits = joint_limit.has_velocity_limits;
//   joint_limit_msg_out.max_velocity = joint_limit.max_velocity;

//   joint_limit_msg_out.has_acceleration_limits = joint_limit.has_acceleration_limits;
//   joint_limit_msg_out.max_acceleration = joint_limit.max_acceleration;
// }

}  // namespace tmc_manipulation_types_bridge
