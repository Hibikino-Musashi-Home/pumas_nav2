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
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>

#include "equality_check.hpp"
#include "manipulation_msg_convertor-test.hpp"
#include "test_data.hpp"

namespace tmc_manipulation_types_bridge {

TEST_F(ManipulationTypesBridgeTest, ConvertJointStateMsgToJointState) {
  // Ready
  const auto joint_state = GenerateJointStateMsg();
  // Convert
  tmc_manipulation_types::JointState common_joint_state;
  JointStateMsgToJointState(joint_state, common_joint_state);
  // Judgment
  EXPECT_EQ(joint_state.name, common_joint_state.name);
  IsEqual(joint_state.position, common_joint_state.position);
}

TEST_F(ManipulationTypesBridgeTest,
       ConvertJointStateMsgToJointStateAtNotEqualJointNameSizeAndPosisionSize) {
  // Ready
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"CARM/LINEAR", "CARM/SHOULDER_Y"};
  joint_state.position.push_back(0.0);
  // Convert
  tmc_manipulation_types::JointState common_joint_state;
  JointStateMsgToJointState(joint_state, common_joint_state);
  // Judgment
  EXPECT_EQ(joint_state.name.size(), common_joint_state.name.size());
  EXPECT_EQ(joint_state.position.size(), common_joint_state.position.size());
}

TEST_F(ManipulationTypesBridgeTest, ConvertJointStateToJointStateMsg) {
  // Ready
  const auto common_joint_state = GenerateJointState();
  // Convert
  sensor_msgs::msg::JointState joint_state;
  JointStateToJointStateMsg(common_joint_state, joint_state);
  // Judgment
  EXPECT_EQ(common_joint_state.name, joint_state.name);
  IsEqual(common_joint_state.position, joint_state.position);
  IsEqual(common_joint_state.velocity, joint_state.velocity);
  IsEqual(common_joint_state.effort, joint_state.effort);
}

TEST_F(ManipulationTypesBridgeTest, ConvertJointPositionMsgToConfig) {
  // Ready
  tmc_planning_msgs::msg::JointPosition joint_posision;
  joint_posision.position = {0.0, 0.1};
  // Convert
  tmc_manipulation_types::Config common_config;
  JointPositionMsgToConfig(joint_posision, common_config);
  // Judgment
  IsEqual(joint_posision.position, common_config);
}

TEST_F(ManipulationTypesBridgeTest, ConvertConfigToJointPositionMsg) {
  // Ready
  tmc_manipulation_types::Config common_config(2);
  common_config << 0.0, 0.1;
  // Convert
  tmc_planning_msgs::msg::JointPosition joint_posision;
  ConfigToJointPositionMsg(common_config, joint_posision);
  // Judgment
  IsEqual(common_config, joint_posision.position);
}

TEST_F(ManipulationTypesBridgeTest, ConvertTaskSpaceRegionMsgToTaskSpaceRegion) {
  // Ready
  const auto task_space_region_in = GenerateTaskSpaceRegionMsg();
  // Convert
  tmc_manipulation_types::TaskSpaceRegion task_space_region_out;
  TaskSpaceRegionMsgToTaskSpaceRegion(task_space_region_in, task_space_region_out);
  // Judgment
  EXPECT_EQ(task_space_region_in.end_frame_id, task_space_region_out.end_frame_id);
  IsEqual(task_space_region_in.origin_to_tsr, task_space_region_out.origin_to_tsr);
  IsEqual(task_space_region_in.tsr_to_end, task_space_region_out.tsr_to_end);
  IsEqual(task_space_region_in.max_bounds, task_space_region_out.max_bounds);
  IsEqual(task_space_region_in.min_bounds, task_space_region_out.min_bounds);
}

TEST_F(ManipulationTypesBridgeTest, ConvertTaskSpaceRegionToTaskSpaceRegionMsg) {
  // Ready
  const auto task_space_region_in = GenerateTaskSpaceRegion();
  // Convert
  tmc_planning_msgs::msg::TaskSpaceRegion task_space_region_out;
  TaskSpaceRegionToTaskSpaceRegionMsg(task_space_region_in, task_space_region_out);
  // Judgment
  EXPECT_EQ(task_space_region_in.end_frame_id, task_space_region_out.end_frame_id);
  IsEqual(task_space_region_in.origin_to_tsr, task_space_region_out.origin_to_tsr);
  IsEqual(task_space_region_in.tsr_to_end, task_space_region_out.tsr_to_end);
  IsEqual(task_space_region_in.max_bounds, task_space_region_out.max_bounds);
  IsEqual(task_space_region_in.min_bounds, task_space_region_out.min_bounds);
}

// TEST_F(ManipulationTypesBridgeTest,
//        ConvertObjectIdentifierMsgToObjectName) {
//   // Ready
//   tmc_msgs::ObjectIdentifier object_identifier;
//   object_identifier.name = "ObjectName";
//   object_identifier.object_id = 1;
//   std::string object_name;
//   // Convert
//   ObjectIdentifierMsgToObjectName(object_identifier, object_name);
//   // Judgment
//   std::string expected_object_name =
//       object_identifier.name + "_" +
//       boost::lexical_cast<std::string>(object_identifier.object_id);
//   EXPECT_EQ(expected_object_name, object_name);
// }

// TEST_F(ManipulationTypesBridgeTest,
//        ConvertObjectNameToObjectIdentifierMsg) {
//   // Ready
//   std::string object_name = "ObjectName_1";
//   tmc_msgs::ObjectIdentifier object_identifier;
//   // Convert
//   ObjectNameToObjectIdentifierMsg(object_name, object_identifier);
//   // Judgment
//   std::vector<std::string> expected_list = Split(object_name, "_");
//   EXPECT_EQ(expected_list.at(0), object_identifier.name);
//   EXPECT_EQ(std::atoi(expected_list.at(1).c_str()),
//             object_identifier.object_id);
// }

TEST_F(ManipulationTypesBridgeTest, ConvertAttachedObjectMsgToAttachedObject) {
  // Ready
  const auto attached_collision_object_in = GenerateAttachCollisionObjectMsg();
  // Convert
  tmc_manipulation_types::AttachedObject attached_object_out;
  AttachedObjectMsgToAttachedObject(attached_collision_object_in, attached_object_out);
  // Judgment
  EXPECT_EQ(attached_collision_object_in.object.id, attached_object_out.object_id);
  EXPECT_EQ(attached_collision_object_in.link_name, attached_object_out.frame_name);
  IsEqual(attached_collision_object_in.object.pose, attached_object_out.frame_to_object);
  EXPECT_EQ(attached_object_out.expected_objects.size(), 0);
}

// TEST_F(ManipulationTypesBridgeTest,
//        ConvertAttachedObjectToAttachedObjectMsg) {
//   // Ready
//   tmc_manipulation_types::AttachedObject common_attached_object;
//   common_attached_object.object_id = "AttachedObject_1";
//   common_attached_object.frame_name = "AttachedObjectFlame";
//   common_attached_object.frame_to_object =
//       Eigen::Translation3d(0.1, 0.2, 0.3) *
//       Eigen::Quaternion<double>(0.4, 0.5, 0.6, 0.7);
//   common_attached_object.group_id = "3";
//   common_attached_object.expected_objects.push_back("Dummy_2");
//   tmc_planning_msgs::AttachedObject planning_attached_object;
//   // Convert
//   AttachedObjectToAttachedObjectMsg(common_attached_object,
//                                     planning_attached_object);
//   // Judgment
//   ASSERT_TRUE(!common_attached_object.object_id.empty());
//   EXPECT_EQ(Split(common_attached_object.object_id, "_").at(0),
//             planning_attached_object.object_id.name);
//   EXPECT_EQ(
//       std::atoi(Split(common_attached_object.object_id, "_").at(1).c_str()),
//       planning_attached_object.object_id.object_id);
//   EXPECT_EQ(common_attached_object.frame_name,
//             planning_attached_object.frame_name);

//   Eigen::Affine3d expected_pose;
//   tf::poseMsgToEigen(
//       planning_attached_object.frame_to_object, expected_pose);
//   EXPECT_EQ(common_attached_object.frame_to_object.translation()[0],
//             expected_pose.translation()[0]);
//   EXPECT_EQ(common_attached_object.frame_to_object.translation()[1],
//             expected_pose.translation()[1]);
//   EXPECT_EQ(common_attached_object.frame_to_object.translation()[2],
//             expected_pose.translation()[2]);
//   EXPECT_EQ(0, planning_attached_object.check_outer_collision);
//   for (uint32_t i = 0;
//        i < common_attached_object.expected_objects.size(); ++i) {
//     EXPECT_EQ(
//         common_attached_object.expected_objects[i],
//         planning_attached_object.expected_object_ids[i].name + "_" +
//             boost::lexical_cast<std::string>(
//                 planning_attached_object.expected_object_ids[i].object_id));
//   }
// }

TEST_F(ManipulationTypesBridgeTest, ConvertJointTrajectoryMsgToJointTrajectory) {
  // Ready
  const auto joint_trajectory_msg = GenerateJointTrajectoryMsg();
  // Convert
  tmc_manipulation_types::JointTrajectory joint_trajectory;
  JointTrajectoryMsgToJointTrajectory(joint_trajectory_msg, joint_trajectory);
  // Judgment
  EXPECT_EQ(joint_trajectory_msg.joint_names, joint_trajectory.names);
  ASSERT_EQ(joint_trajectory_msg.points.size(), joint_trajectory.path.size());
  for (auto i = 0; i < joint_trajectory_msg.points.size(); ++i) {
    IsEqual(joint_trajectory_msg.points[i].positions, joint_trajectory.path[i]);
  }
}

TEST_F(ManipulationTypesBridgeTest, ConvertJointTrajectoryToJointTrajectoryMsg) {
  // Ready
  const auto joint_trajectory = GenerateJointTrajectory();
  // Convert
  trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
  JointTrajectoryToJointTrajectoryMsg(joint_trajectory, joint_trajectory_msg);
  // Judgment
  EXPECT_EQ(joint_trajectory.names, joint_trajectory_msg.joint_names);
  ASSERT_EQ(joint_trajectory.path.size(), joint_trajectory_msg.points.size());
  for (auto i = 0; i < joint_trajectory.path.size(); ++i) {
    IsEqual(joint_trajectory.path[i], joint_trajectory_msg.points[i].positions);
  }
}

TEST_F(ManipulationTypesBridgeTest, ConvertSolidPrimitiveToShape) {
  {
    shape_msgs::msg::SolidPrimitive shape_msg;
    shape_msg.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    shape_msg.dimensions = {0.1};

    tmc_manipulation_types::Shape shape;
    SolidPrimitiveToShape(shape_msg, shape);

    EXPECT_EQ(shape.type, tmc_manipulation_types::kSphere);
    IsEqual(shape.dimensions, shape_msg.dimensions);
  }
  {
    shape_msgs::msg::SolidPrimitive shape_msg;
    shape_msg.type = shape_msgs::msg::SolidPrimitive::BOX;
    shape_msg.dimensions = {0.1, 0.2, 0.3};

    tmc_manipulation_types::Shape shape;
    SolidPrimitiveToShape(shape_msg, shape);

    EXPECT_EQ(shape.type, tmc_manipulation_types::kBox);
    IsEqual(shape.dimensions, shape_msg.dimensions);
  }
  {
    shape_msgs::msg::SolidPrimitive shape_msg;
    shape_msg.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    shape_msg.dimensions = {0.1, 0.2};

    tmc_manipulation_types::Shape shape;
    SolidPrimitiveToShape(shape_msg, shape);

    EXPECT_EQ(shape.type, tmc_manipulation_types::kCylinder);
    IsEqual(shape.dimensions, shape_msg.dimensions);
  }
}

TEST_F(ManipulationTypesBridgeTest, ConvertShapeToSolidPrimitive) {
  {
    tmc_manipulation_types::Shape shape;
    shape.type = tmc_manipulation_types::kSphere;
    shape.dimensions = {0.1};

    shape_msgs::msg::SolidPrimitive shape_msg;
    ShapeToSolidPrimitive(shape, shape_msg);

    EXPECT_EQ(shape_msg.type, shape_msgs::msg::SolidPrimitive::SPHERE);
    IsEqual(shape_msg.dimensions, shape.dimensions);
  }
  {
    tmc_manipulation_types::Shape shape;
    shape.type = tmc_manipulation_types::kBox;
    shape.dimensions = {0.1, 0.2, 0.3};

    shape_msgs::msg::SolidPrimitive shape_msg;
    ShapeToSolidPrimitive(shape, shape_msg);

    EXPECT_EQ(shape_msg.type, shape_msgs::msg::SolidPrimitive::BOX);
    IsEqual(shape_msg.dimensions, shape.dimensions);
  }
  {
    tmc_manipulation_types::Shape shape;
    shape.type = tmc_manipulation_types::kCylinder;
    shape.dimensions = {0.1, 0.2};

    shape_msgs::msg::SolidPrimitive shape_msg;
    ShapeToSolidPrimitive(shape, shape_msg);

    EXPECT_EQ(shape_msg.type, shape_msgs::msg::SolidPrimitive::CYLINDER);
    IsEqual(shape_msg.dimensions, shape.dimensions);
  }
}

TEST_F(ManipulationTypesBridgeTest, ConvertMeshMsgToShape) {
  shape_msgs::msg::Mesh mesh_msg;
  mesh_msg.vertices.resize(2);
  mesh_msg.vertices[0].x = 0.1;
  mesh_msg.vertices[0].y = 0.2;
  mesh_msg.vertices[0].z = 0.3;
  mesh_msg.vertices[1].x = 0.4;
  mesh_msg.vertices[1].y = 0.5;
  mesh_msg.vertices[1].z = 0.6;
  mesh_msg.triangles.resize(1);
  mesh_msg.triangles[0].vertex_indices = {0, 1, 2};

  tmc_manipulation_types::Shape shape;
  MeshMsgToShape(mesh_msg, shape);

  EXPECT_EQ(shape.type, tmc_manipulation_types::kMeshVertices);
  IsEqual(shape.vertices, mesh_msg.vertices);
  IsEqual(shape.indices, mesh_msg.triangles);
}

TEST_F(ManipulationTypesBridgeTest, ConvertShapeToMeshMsg) {
  tmc_manipulation_types::Shape shape;
  shape.type = tmc_manipulation_types::kMeshVertices;
  shape.vertices = {Eigen::Vector3f(0.1, 0.2, 0.3), Eigen::Vector3f(0.4, 0.5, 0.6)};
  shape.indices = {0, 1, 2};

  shape_msgs::msg::Mesh mesh_msg;
  ShapeToMeshMsg(shape, mesh_msg);

  IsEqual(mesh_msg.vertices, shape.vertices);
  IsEqual(mesh_msg.triangles, shape.indices);
}

TEST_F(ManipulationTypesBridgeTest, ConvertOccupancyGridMsgToOccupancyGrid) {
  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.info.resolution = 0.1;
  map_msg.info.width = 2;
  map_msg.info.height = 3;
  map_msg.info.origin.position.x = 0.4;
  map_msg.info.origin.position.y = 0.5;
  map_msg.info.origin.position.z = 0.6;
  map_msg.info.origin.orientation.x = 0.7;
  map_msg.info.origin.orientation.y = 0.8;
  map_msg.info.origin.orientation.z = 0.9;
  map_msg.info.origin.orientation.w = 1.0;
  map_msg.data = {0, 10, 20, 30, 40, 50};

  tmc_manipulation_types::OccupancyGrid map;
  OccupancyGridMsgToOccupancyGrid(map_msg, map);

  EXPECT_DOUBLE_EQ(map_msg.info.resolution, map.info.resolution);
  EXPECT_EQ(map_msg.info.width, map.info.width);
  EXPECT_EQ(map_msg.info.height, map.info.height);
  IsEqual(map_msg.info.origin, map.info.origin);
  EXPECT_EQ(map_msg.data, map.data);
}

TEST_F(ManipulationTypesBridgeTest, ConvertCollisionObjectToOuterObjectParameters) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.id = "test_object";
  collision_object.pose.position.x = 0.1;
  collision_object.pose.position.y = 0.2;
  collision_object.pose.position.z = 0.3;
  collision_object.pose.orientation.x = 1.0;
  collision_object.pose.orientation.w = 0.0;
  collision_object.primitives.resize(3);
  collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
  collision_object.primitives[0].dimensions = {0.01};
  collision_object.primitives[1].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_object.primitives[1].dimensions = {0.01, 0.02, 0.03};
  collision_object.primitives[2].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  collision_object.primitives[2].dimensions = {0.04, 0.05};
  collision_object.primitive_poses.resize(3);
  collision_object.primitive_poses[0].position.x = 0.4;
  collision_object.primitive_poses[0].orientation.y = 1.0;
  collision_object.primitive_poses[0].orientation.w = 0.0;
  collision_object.primitive_poses[1].position.x = 0.5;
  collision_object.primitive_poses[1].orientation.z = 1.0;
  collision_object.primitive_poses[1].orientation.w = 0.0;
  collision_object.primitive_poses[2].position.x = 0.6;
  collision_object.meshes.resize(1);
  collision_object.meshes[0].vertices.resize(2);
  collision_object.meshes[0].vertices[0].x = 0.1;
  collision_object.meshes[0].vertices[0].y = 0.2;
  collision_object.meshes[0].vertices[0].z = 0.3;
  collision_object.meshes[0].vertices[1].x = 0.4;
  collision_object.meshes[0].vertices[1].y = 0.5;
  collision_object.meshes[0].vertices[1].z = 0.6;
  collision_object.meshes[0].triangles.resize(1);
  collision_object.meshes[0].triangles[0].vertex_indices = {0, 1, 2};
  collision_object.mesh_poses.resize(1);
  collision_object.mesh_poses[0].position.x = 0.7;

  tmc_manipulation_types::OuterObjectParameters outer_object;
  CollisionObjectToOuterObjectParameters(collision_object, outer_object);

  EXPECT_EQ(outer_object.name, collision_object.id);
  IsEqual(outer_object.origin_to_base, collision_object.pose);
  ASSERT_EQ(outer_object.shape.size(), 4);
  EXPECT_EQ(outer_object.shape[0].type, tmc_manipulation_types::kSphere);
  IsEqual(outer_object.shape[0].dimensions, collision_object.primitives[0].dimensions);
  EXPECT_EQ(outer_object.shape[1].type, tmc_manipulation_types::kBox);
  IsEqual(outer_object.shape[1].dimensions, collision_object.primitives[1].dimensions);
  EXPECT_EQ(outer_object.shape[2].type, tmc_manipulation_types::kCylinder);
  IsEqual(outer_object.shape[2].dimensions, collision_object.primitives[2].dimensions);
  EXPECT_EQ(outer_object.shape[3].type, tmc_manipulation_types::kMeshVertices);
  IsEqual(outer_object.shape[3].vertices, collision_object.meshes[0].vertices);
  IsEqual(outer_object.shape[3].indices, collision_object.meshes[0].triangles);
  ASSERT_EQ(outer_object.base_to_child.size(), 4);
  IsEqual(outer_object.base_to_child[0], collision_object.primitive_poses[0]);
  IsEqual(outer_object.base_to_child[1], collision_object.primitive_poses[1]);
  IsEqual(outer_object.base_to_child[2], collision_object.primitive_poses[2]);
  IsEqual(outer_object.base_to_child[3], collision_object.mesh_poses[0]);
}

TEST_F(ManipulationTypesBridgeTest, ConvertOuterObjectParametersToCollisionObject) {
  tmc_manipulation_types::OuterObjectParameters outer_object;
  outer_object.name = "test_object";
  outer_object.origin_to_base =
      Eigen::Translation3d(0.1, 0.2, 0.3) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  outer_object.shape.resize(5);
  outer_object.shape[0].type = tmc_manipulation_types::kSphere;
  outer_object.shape[0].dimensions = {0.01};
  outer_object.shape[1].type = tmc_manipulation_types::kBox;
  outer_object.shape[1].dimensions = {0.01, 0.02, 0.03};
  outer_object.shape[2].type = tmc_manipulation_types::kCylinder;
  outer_object.shape[2].dimensions = {0.04, 0.05};
  outer_object.shape[3].type = tmc_manipulation_types::kMesh;
  outer_object.shape[3].vertices = {Eigen::Vector3f(0.1, 0.2, 0.3), Eigen::Vector3f(0.4, 0.5, 0.6)};
  outer_object.shape[3].indices = {0, 1, 2};
  outer_object.shape[4].type = tmc_manipulation_types::kMeshVertices;
  outer_object.shape[4].vertices = {Eigen::Vector3f(1.1, 1.2, 1.3), Eigen::Vector3f(1.4, 1.5, 1.6)};
  outer_object.shape[4].indices = {10, 11, 12};
  outer_object.base_to_child.resize(5);
  outer_object.base_to_child[0] =
      Eigen::Translation3d(0.4, 0.0, 0.0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
  outer_object.base_to_child[1] =
      Eigen::Translation3d(0.5, 0.0, 0.0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
  outer_object.base_to_child[2] =
      Eigen::Translation3d(0.6, 0.0, 0.0) * Eigen::AngleAxisd::Identity();
  outer_object.base_to_child[3] =
      Eigen::Translation3d(0.7, 0.0, 0.0) * Eigen::AngleAxisd::Identity();
  outer_object.base_to_child[4] =
      Eigen::Translation3d(0.8, 0.0, 0.0) * Eigen::AngleAxisd::Identity();

  moveit_msgs::msg::CollisionObject collision_object;
  OuterObjectParametersToCollisionObject(outer_object, collision_object);

  EXPECT_EQ(collision_object.id, outer_object.name);
  IsEqual(collision_object.pose, outer_object.origin_to_base);
  ASSERT_EQ(collision_object.primitives.size(), 3);
  EXPECT_EQ(collision_object.primitives[0].type, shape_msgs::msg::SolidPrimitive::SPHERE);
  IsEqual(collision_object.primitives[0].dimensions, outer_object.shape[0].dimensions);
  EXPECT_EQ(collision_object.primitives[1].type, shape_msgs::msg::SolidPrimitive::BOX);
  IsEqual(collision_object.primitives[1].dimensions, outer_object.shape[1].dimensions);
  EXPECT_EQ(collision_object.primitives[2].type, shape_msgs::msg::SolidPrimitive::CYLINDER);
  IsEqual(collision_object.primitives[2].dimensions, outer_object.shape[2].dimensions);
  ASSERT_EQ(collision_object.primitive_poses.size(), 3);
  IsEqual(collision_object.primitive_poses[0], outer_object.base_to_child[0]);
  IsEqual(collision_object.primitive_poses[1], outer_object.base_to_child[1]);
  IsEqual(collision_object.primitive_poses[2], outer_object.base_to_child[2]);

  ASSERT_EQ(collision_object.meshes.size(), 2);
  IsEqual(collision_object.meshes[0].vertices, outer_object.shape[3].vertices);
  IsEqual(collision_object.meshes[0].triangles, outer_object.shape[3].indices);
  IsEqual(collision_object.meshes[1].vertices, outer_object.shape[4].vertices);
  IsEqual(collision_object.meshes[1].triangles, outer_object.shape[4].indices);
  ASSERT_EQ(collision_object.mesh_poses.size(), 2);
  IsEqual(collision_object.mesh_poses[0], outer_object.base_to_child[3]);
  IsEqual(collision_object.mesh_poses[1], outer_object.base_to_child[4]);
}

// tmc_manipulation_msgs::CollisionEnvironment CreateCollisionEnvironment() {
//   tmc_manipulation_msgs::CollisionEnvironment collision_environment;
//   tmc_manipulation_msgs::CollisionObject collision_object;
//   collision_object.id.object_id = 1;
//   collision_object.id.name = "box";
//   collision_object.operation.operation =
//       tmc_manipulation_msgs::CollisionObjectOperation::ADD;
//   tmc_geometric_shapes_msgs::Shape shape;
//   shape.type = tmc_geometric_shapes_msgs::Shape::BOX;
//   shape.stl_file_name = "BOX.stl";
//   shape.dimensions.push_back(1.0);
//   shape.dimensions.push_back(1.0);
//   shape.dimensions.push_back(1.0);
//   collision_object.shapes.push_back(shape);
//   geometry_msgs::Pose pose;
//   pose.position.x = 0.0;
//   pose.position.y = 0.0;
//   pose.position.z = 0.0;
//   pose.orientation.x = 0.0;
//   pose.orientation.y = 0.0;
//   pose.orientation.z = 0.0;
//   pose.orientation.w = 1.0;
//   collision_object.poses.push_back(pose);
//   collision_environment.known_objects.push_back(collision_object);
//   collision_object.id.object_id = 2;
//   collision_object.id.name = "book";
//   collision_object.operation.operation =
//       tmc_manipulation_msgs::CollisionObjectOperation::REMOVE;
//   shape.stl_file_name = "book.stl";
//   collision_object.shapes.push_back(shape);
//   collision_object.poses.push_back(pose);
//   collision_environment.known_objects.push_back(collision_object);
//   collision_object.id.object_id = 3;
//   collision_object.id.name = "coin";
//   collision_object.operation.operation =
//       tmc_manipulation_msgs::CollisionObjectOperation
//       ::DETACH_AND_ADD_AS_OBJECT;
//   shape.stl_file_name = "coin.stl";
//   collision_object.shapes.push_back(shape);
//   collision_object.poses.push_back(pose);
//   collision_environment.known_objects.push_back(collision_object);
//   collision_object.id.object_id = 4;
//   collision_object.id.name = "card";
//   collision_object.operation.operation =
//       tmc_manipulation_msgs::CollisionObjectOperation
//       ::ATTACH_AND_REMOVE_AS_OBJECT;
//   shape.stl_file_name = "card.stl";
//   collision_object.shapes.push_back(shape);
//   collision_object.poses.push_back(pose);
//   collision_environment.known_objects.push_back(collision_object);
//   pose.position.x = 1.0;
//   pose.position.y = 0.0;
//   pose.position.z = 1.0;
//   pose.orientation.x = 0.0;
//   pose.orientation.y = 0.0;
//   pose.orientation.z = sin(270 * 0.5);
//   pose.orientation.w = cos(270 * 0.5);
//   collision_environment.poses.push_back(pose);
//   pose.position.x = 0.0;
//   pose.position.y = -1.0;
//   pose.position.z = 1.0;
//   pose.orientation.x = 0.0;
//   pose.orientation.y = 0.0;
//   pose.orientation.z = sin(0 * 0.5);
//   pose.orientation.w = cos(0 * 0.5);
//   collision_environment.poses.push_back(pose);
//   pose.position.x = -1.0;
//   pose.position.y = 0.0;
//   pose.position.z = 1.0;
//   pose.orientation.x = 0.0;
//   pose.orientation.y = 0.0;
//   pose.orientation.z = sin(90 * 0.5);
//   pose.orientation.w = cos(90 * 0.5);
//   collision_environment.poses.push_back(pose);
//   pose.position.x = 0.0;
//   pose.position.y = 1.0;
//   pose.position.z = 1.0;
//   pose.orientation.x = 0.0;
//   pose.orientation.y = 0.0;
//   pose.orientation.z = sin(180 * 0.5);
//   pose.orientation.w = cos(180 * 0.5);
//   collision_environment.poses.push_back(pose);
//   collision_environment.collision_map_pose.position.x = -0.1;
//   collision_environment.collision_map_pose.position.y = -0.2;
//   collision_environment.collision_map_pose.position.z = -0.3;
//   collision_environment.collision_map_pose.orientation.x = -0.4;
//   collision_environment.collision_map_pose.orientation.y = -0.5;
//   collision_environment.collision_map_pose.orientation.z = -0.6;
//   collision_environment.collision_map_pose.orientation.w = -0.7;
//   tmc_geometry_msgs::OrientedBoundingBox oriented_bouding_box;
//   oriented_bouding_box.center.x = 1.0;
//   oriented_bouding_box.center.y = 2.0;
//   oriented_bouding_box.center.z = 3.0;
//   oriented_bouding_box.extents.x = 4.0;
//   oriented_bouding_box.extents.y = 5.0;
//   oriented_bouding_box.extents.z = 6.0;
//   oriented_bouding_box.axis.x = 7.0;
//   oriented_bouding_box.axis.y = 8.0;
//   oriented_bouding_box.axis.z = 9.0;
//   oriented_bouding_box.angle = 10.0;
//   collision_environment.collision_map.boxes.push_back(oriented_bouding_box);
//   collision_environment.collision_map.boxes.push_back(oriented_bouding_box);
//   collision_environment.collision_map.boxes.push_back(oriented_bouding_box);
//   collision_environment.collision_map.boxes.push_back(oriented_bouding_box);
//   collision_environment.collision_map.boxes.push_back(oriented_bouding_box);
//   return collision_environment;
// }

// TEST_F(ManipulationTypesBridgeTest,
//        ConvertCollisionEnvironmentToOuterObjectSeq) {
//   // Ready
//   tmc_manipulation_msgs::CollisionEnvironment collision_environment =
//       CreateCollisionEnvironment();
//   tmc_manipulation_types::OuterObjectParametersSeq outer_object_seq;
//   // Convert
//   CollisionEnvironmentToOuterObjectSeq(collision_environment,
//                                        outer_object_seq);
//   // Judgment
//   for (uint32_t i = 0;
//        i < outer_object_seq.size(); ++i) {
//     EXPECT_EQ(
//         collision_environment.known_objects[i].id.name + "_" +
//             boost::lexical_cast<std::string>(
//                 collision_environment.known_objects[i].id.object_id),
//         outer_object_seq[i].name);
//     Eigen::Affine3d expected_pose;
//     tf::poseMsgToEigen(
//         collision_environment.poses[i],
//         expected_pose);
//     EXPECT_EQ(expected_pose.translation()[0],
//               outer_object_seq[i].origin_to_base.translation()[0]);
//     EXPECT_EQ(expected_pose.translation()[1],
//               outer_object_seq[i].origin_to_base.translation()[1]);
//     EXPECT_EQ(expected_pose.translation()[2],
//               outer_object_seq[i].origin_to_base.translation()[2]);
//     for (uint32_t k = 0; k < outer_object_seq[i].shape.size(); ++k) {
//       tf::poseMsgToEigen(
//           collision_environment.known_objects[i].poses[k],
//           expected_pose);
//       EXPECT_EQ(expected_pose.translation()[0],
//                 outer_object_seq[i].base_to_child[k].translation()[0]);
//       EXPECT_EQ(expected_pose.translation()[1],
//                 outer_object_seq[i].base_to_child[k].translation()[1]);
//       EXPECT_EQ(expected_pose.translation()[2],
//                 outer_object_seq[i].base_to_child[k].translation()[2]);
//       tmc_manipulation_types::Shape expected_shape;
//       ShapeMsgToPlannerShape(collision_environment.known_objects[i].shapes[k],
//                              expected_shape);
//       EXPECT_EQ(expected_shape.type,
//                 outer_object_seq[i].shape[k].type);
//       EXPECT_EQ(expected_shape.filename,
//                 outer_object_seq[i].shape[k].filename);
//       EXPECT_EQ(expected_shape.dimensions.size(),
//                 outer_object_seq[i].shape[k].dimensions.size());
//     }
//   }
// }

// TEST_F(ManipulationTypesBridgeTest,
//        ConvertCollisionEnvironmentToCuboidSeq) {
//   // Ready
//   tmc_manipulation_msgs::CollisionEnvironment collision_environment =
//       CreateCollisionEnvironment();
//   std::string box_name = "BOX";
//   tmc_manipulation_types::CuboidSeq cuboid_seq;
//   // Convert
//   CollisionEnvironmentToCuboidSeq(collision_environment,
//                                   box_name,
//                                   cuboid_seq);
//   // Judgment
//   EXPECT_EQ(collision_environment.collision_map.boxes.size(),
//             cuboid_seq.size());
//   for (uint32_t i = 0; i < cuboid_seq.size(); ++i) {
//     EXPECT_EQ(collision_environment.collision_map.boxes[i].extents.x,
//               cuboid_seq[i].box_extents[0]);
//     EXPECT_EQ(collision_environment.collision_map.boxes[i].extents.y,
//               cuboid_seq[i].box_extents[1]);
//     EXPECT_EQ(collision_environment.collision_map.boxes[i].extents.z,
//               cuboid_seq[i].box_extents[2]);
//     Eigen::Affine3d map_pose =
//         Eigen::Translation3d(
//             collision_environment.collision_map.boxes[i].center.x,
//             collision_environment.collision_map.boxes[i].center.y,
//             collision_environment.collision_map.boxes[i].center.z) *
//         Eigen::AngleAxisd(
//             collision_environment.collision_map.boxes[i].angle,
//             Eigen::Vector3d(
//                 collision_environment.collision_map.boxes[i].axis.x,
//                 collision_environment.collision_map.boxes[i].axis.y,
//                 collision_environment.collision_map.boxes[i].axis.z));
//     Eigen::Affine3d map_transform;
//     tf::poseMsgToEigen(
//         collision_environment.collision_map_pose, map_transform);
//     EXPECT_EQ((map_transform * map_pose).translation()[0],
//               cuboid_seq[i].box_transform.translation()[0]);
//     EXPECT_EQ((map_transform * map_pose).translation()[1],
//               cuboid_seq[i].box_transform.translation()[1]);
//     EXPECT_EQ((map_transform * map_pose).translation()[2],
//               cuboid_seq[i].box_transform.translation()[2]);
//     EXPECT_EQ(0.0, cuboid_seq[i].margin);
//     EXPECT_EQ(box_name + boost::lexical_cast<std::string>(i),
//               cuboid_seq[i].box_name);

//     Eigen::Matrix<double, 3, 8> vertices;
//     vertices << cuboid_seq[i].box_extents(0), cuboid_seq[i].box_extents(0),
//                 cuboid_seq[i].box_extents(0), cuboid_seq[i].box_extents(0),
//                 -cuboid_seq[i].box_extents(0), -cuboid_seq[i].box_extents(0),
//                 -cuboid_seq[i].box_extents(0), -cuboid_seq[i].box_extents(0),
//                 cuboid_seq[i].box_extents(1), cuboid_seq[i].box_extents(1),
//                 -cuboid_seq[i].box_extents(1), -cuboid_seq[i].box_extents(1),
//                 cuboid_seq[i].box_extents(1), cuboid_seq[i].box_extents(1),
//                 -cuboid_seq[i].box_extents(1), -cuboid_seq[i].box_extents(1),
//                 cuboid_seq[i].box_extents(2), -cuboid_seq[i].box_extents(2),
//                 cuboid_seq[i].box_extents(2), -cuboid_seq[i].box_extents(2),
//                 cuboid_seq[i].box_extents(2), -cuboid_seq[i].box_extents(2),
//                 cuboid_seq[i].box_extents(2), -cuboid_seq[i].box_extents(2);
//     Eigen::Matrix3d linear(cuboid_seq[i].box_transform.linear());
//     Eigen::Matrix<double, 3, 8> rotated_vertices(linear * vertices);
//     Eigen::Vector3d rotated_extents(rotated_vertices.row(0).maxCoeff() / 2.0,
//                                     rotated_vertices.row(1).maxCoeff() / 2.0,
//                                     rotated_vertices.row(2).maxCoeff() / 2.0);
//     tmc_manipulation_types::AABB expected_aabb;
//     expected_aabb <<
//         cuboid_seq[i].box_transform.translation().coeff(0) - rotated_extents(0),
//         cuboid_seq[i].box_transform.translation().coeff(0) + rotated_extents(0),
//         cuboid_seq[i].box_transform.translation().coeff(1) - rotated_extents(1),
//         cuboid_seq[i].box_transform.translation().coeff(1) + rotated_extents(1),
//         cuboid_seq[i].box_transform.translation().coeff(2) - rotated_extents(2),
//         cuboid_seq[i].box_transform.translation().coeff(2) + rotated_extents(2);
//     EXPECT_EQ(expected_aabb, cuboid_seq[i].box_aabb);
//   }
// }

// TEST_F(ManipulationTypesBridgeTest,
//        ConvertCollisionMapToCuboidSeq) {
//   // Ready
//   tmc_mapping_msgs::CollisionMap collision_map;
//   collision_map.header.frame_id = "CollisionMap";
//   ros::Time set_time = ros::Time::now();
//   collision_map.header.stamp = set_time;
//   tmc_geometry_msgs::OrientedBoundingBox box;
//   box.center.x = 0.1;
//   box.center.y = 0.2;
//   box.center.z = 0.3;
//   box.extents.x = 0.4;
//   box.extents.y = 0.5;
//   box.extents.z = 0.6;
//   box.axis.x = 0.7;
//   box.axis.y = 0.8;
//   box.axis.z = 0.9;
//   box.angle = 1.57;
//   collision_map.boxes.push_back(box);
//   collision_map.boxes.push_back(box);
//   collision_map.boxes.push_back(box);
//   tmc_msgs::ObjectIdentifier object_identifier;
//   object_identifier.name = "Box";
//   object_identifier.object_id = 1;
//   collision_map.subtracted_object_ids.push_back(object_identifier);
//   object_identifier.name = "Box";
//   object_identifier.object_id = 2;
//   collision_map.subtracted_object_ids.push_back(object_identifier);
//   object_identifier.name = "Box";
//   object_identifier.object_id = 3;
//   collision_map.subtracted_object_ids.push_back(object_identifier);
//   std::string box_name = "Box";
//   Eigen::Affine3d origin_to_map =
//       Eigen::Translation3d(1.0, 1.0, 0.0) *
//       Eigen::Quaternion<double>(0.0, 0.0, 0.0, 1.0);
//   tmc_manipulation_types::CuboidSeq cuboid_seq;
//   // Convert
//   CollisionMapToCuboidSeq(collision_map,
//                           box_name,
//                           origin_to_map,
//                           cuboid_seq);
//   // Judgment
//   EXPECT_EQ(collision_map.boxes.size(),
//             cuboid_seq.size());
//   for (uint32_t i = 0; i < cuboid_seq.size(); ++i) {
//     EXPECT_EQ(collision_map.boxes[i].extents.x,
//               cuboid_seq[i].box_extents[0]);
//     EXPECT_EQ(collision_map.boxes[i].extents.y,
//               cuboid_seq[i].box_extents[1]);
//     EXPECT_EQ(collision_map.boxes[i].extents.z,
//               cuboid_seq[i].box_extents[2]);
//     EXPECT_EQ(0.0, cuboid_seq[i].margin);
//     EXPECT_EQ(box_name + boost::lexical_cast<std::string>(i),
//               cuboid_seq[i].box_name);

//     Eigen::Matrix<double, 3, 8> vertices;
//     vertices << cuboid_seq[i].box_extents(0), cuboid_seq[i].box_extents(0),
//                 cuboid_seq[i].box_extents(0), cuboid_seq[i].box_extents(0),
//                 -cuboid_seq[i].box_extents(0), -cuboid_seq[i].box_extents(0),
//                 -cuboid_seq[i].box_extents(0), -cuboid_seq[i].box_extents(0),
//                 cuboid_seq[i].box_extents(1), cuboid_seq[i].box_extents(1),
//                 -cuboid_seq[i].box_extents(1), -cuboid_seq[i].box_extents(1),
//                 cuboid_seq[i].box_extents(1), cuboid_seq[i].box_extents(1),
//                 -cuboid_seq[i].box_extents(1), -cuboid_seq[i].box_extents(1),
//                 cuboid_seq[i].box_extents(2), -cuboid_seq[i].box_extents(2),
//                 cuboid_seq[i].box_extents(2), -cuboid_seq[i].box_extents(2),
//                 cuboid_seq[i].box_extents(2), -cuboid_seq[i].box_extents(2),
//                 cuboid_seq[i].box_extents(2), -cuboid_seq[i].box_extents(2);
//     Eigen::Matrix3d linear(cuboid_seq[i].box_transform.linear());
//     Eigen::Matrix<double, 3, 8> rotated_vertices(linear * vertices);
//     Eigen::Vector3d rotated_extents(rotated_vertices.row(0).maxCoeff() / 2.0,
//                                     rotated_vertices.row(1).maxCoeff() / 2.0,
//                                     rotated_vertices.row(2).maxCoeff() / 2.0);
//     tmc_manipulation_types::AABB expected_aabb;
//     expected_aabb <<
//         cuboid_seq[i].box_transform.translation().coeff(0) - rotated_extents(0),
//         cuboid_seq[i].box_transform.translation().coeff(0) + rotated_extents(0),
//         cuboid_seq[i].box_transform.translation().coeff(1) - rotated_extents(1),
//         cuboid_seq[i].box_transform.translation().coeff(1) + rotated_extents(1),
//         cuboid_seq[i].box_transform.translation().coeff(2) - rotated_extents(2),
//         cuboid_seq[i].box_transform.translation().coeff(2) + rotated_extents(2);
//     EXPECT_EQ(expected_aabb, cuboid_seq[i].box_aabb);
//   }
// }

TEST_F(ManipulationTypesBridgeTest, ConvertPlannerShapeToMarkerMsg_TypeBOX) {
  // Ready
  tmc_manipulation_types::Shape collision_detector_shape;
  collision_detector_shape.type = tmc_manipulation_types::kBox;
  collision_detector_shape.dimensions = {1.0, 2.0, 3.0};
  tmc_manipulation_types::Transform pose = Eigen::Translation3d(1.0, 2.0, 3.0) * Eigen::Quaterniond::Identity();
  int32_t id = 1;
  std::string frame_id = "shape";
  rclcpp::Time time(0, 2);
  std_msgs::msg::ColorRGBA color;
  color.a = 0.1;
  color.b = 0.2;
  color.g = 0.3;
  color.r = 0.4;
  // Convert
  visualization_msgs::msg::Marker marker_msgs;
  PlannerShapeToMarkerMsg(collision_detector_shape, pose, id, frame_id, time, color, marker_msgs);
  // Judgment
  EXPECT_EQ(static_cast<builtin_interfaces::msg::Time>(time), marker_msgs.header.stamp);
  EXPECT_EQ("tmc_manipulation_types_bridge", marker_msgs.ns);
  EXPECT_EQ(id, marker_msgs.id);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, marker_msgs.type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, marker_msgs.action);
  IsEqual(pose, marker_msgs.pose);
  EXPECT_EQ(collision_detector_shape.dimensions.at(0), marker_msgs.scale.x);
  EXPECT_EQ(collision_detector_shape.dimensions.at(1), marker_msgs.scale.y);
  EXPECT_EQ(collision_detector_shape.dimensions.at(2), marker_msgs.scale.z);
  IsEqual(color, marker_msgs.color);
}

TEST_F(ManipulationTypesBridgeTest, ConvertPlannerShapeToMarkerMsg_TypeCylinder) {
  // After that, omit tests other than shape

  // Ready
  tmc_manipulation_types::Shape collision_detector_shape;
  collision_detector_shape.type = tmc_manipulation_types::kCylinder;
  collision_detector_shape.dimensions = {1.0, 2.0};
  // Convert
  visualization_msgs::msg::Marker marker_msgs;
  PlannerShapeToMarkerMsg(collision_detector_shape, Eigen::Affine3d::Identity(), 0, "shape", rclcpp::Time(0, 2),
                          std_msgs::msg::ColorRGBA(), marker_msgs);
  // Judgment
  EXPECT_EQ(visualization_msgs::msg::Marker::CYLINDER, marker_msgs.type);
  EXPECT_EQ(collision_detector_shape.dimensions.at(0) * 2.0, marker_msgs.scale.x);
  EXPECT_EQ(collision_detector_shape.dimensions.at(0) * 2.0, marker_msgs.scale.y);
  EXPECT_EQ(collision_detector_shape.dimensions.at(1), marker_msgs.scale.z);
}

TEST_F(ManipulationTypesBridgeTest, ConvertPlannerShapeToMarkerMsg_TypeSphere) {
  // Ready
  tmc_manipulation_types::Shape collision_detector_shape;
  collision_detector_shape.type = tmc_manipulation_types::kSphere;
  collision_detector_shape.dimensions = {1.0};
  // Convert
  visualization_msgs::msg::Marker marker_msgs;
  PlannerShapeToMarkerMsg(collision_detector_shape, Eigen::Affine3d::Identity(), 0, "shape", rclcpp::Time(0, 2),
                          std_msgs::msg::ColorRGBA(), marker_msgs);
  // Judgment
  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, marker_msgs.type);
  EXPECT_EQ(collision_detector_shape.dimensions.at(0) * 2.0, marker_msgs.scale.x);
  EXPECT_EQ(collision_detector_shape.dimensions.at(0) * 2.0, marker_msgs.scale.y);
  EXPECT_EQ(collision_detector_shape.dimensions.at(0) * 2.0, marker_msgs.scale.z);
}

TEST_F(ManipulationTypesBridgeTest, ConvertPlannerShapeToMarkerMsg_TypeCapsule) {
  // Ready
  tmc_manipulation_types::Shape collision_detector_shape;
  collision_detector_shape.type = tmc_manipulation_types::kCapsule;
  collision_detector_shape.dimensions = {1.0, 2.0};
  // Convert
  visualization_msgs::msg::Marker marker_msgs;
  PlannerShapeToMarkerMsg(collision_detector_shape, Eigen::Affine3d::Identity(), 0, "shape", rclcpp::Time(0, 2),
                          std_msgs::msg::ColorRGBA(), marker_msgs);
  // Judgment
  EXPECT_EQ(visualization_msgs::msg::Marker::CYLINDER, marker_msgs.type);
  EXPECT_EQ(collision_detector_shape.dimensions.at(0) * 2.0, marker_msgs.scale.x);
  EXPECT_EQ(collision_detector_shape.dimensions.at(0) * 2.0, marker_msgs.scale.y);
  EXPECT_EQ(collision_detector_shape.dimensions.at(1), marker_msgs.scale.z);
}

TEST_F(ManipulationTypesBridgeTest, ConvertPlannerShapeToMarkerMsg_TypeMesh) {
  {
    // Ready
    tmc_manipulation_types::Shape collision_detector_shape;
    collision_detector_shape.type = tmc_manipulation_types::kMesh;
    collision_detector_shape.filename = "MeshFile";
    // Convert
    visualization_msgs::msg::Marker marker_msgs;
    PlannerShapeToMarkerMsg(collision_detector_shape, Eigen::Affine3d::Identity(), 0, "shape", rclcpp::Time(0, 2),
                            std_msgs::msg::ColorRGBA(), marker_msgs);
    // Judgment
    EXPECT_EQ(visualization_msgs::msg::Marker::MESH_RESOURCE, marker_msgs.type);
    EXPECT_EQ("file://" + collision_detector_shape.filename, marker_msgs.mesh_resource);
  }
  {
    // Ready
    tmc_manipulation_types::Shape collision_detector_shape;
    collision_detector_shape.type = tmc_manipulation_types::kMesh;
    collision_detector_shape.filename = "http://MeshFile";
    // Convert
    visualization_msgs::msg::Marker marker_msgs;
    PlannerShapeToMarkerMsg(collision_detector_shape, Eigen::Affine3d::Identity(), 0, "shape", rclcpp::Time(0, 2),
                            std_msgs::msg::ColorRGBA(), marker_msgs);
    // Judgment
    EXPECT_EQ(visualization_msgs::msg::Marker::MESH_RESOURCE, marker_msgs.type);
    EXPECT_EQ(collision_detector_shape.filename, marker_msgs.mesh_resource);
  }
}

TEST_F(ManipulationTypesBridgeTest, ConvertOuterObjectParametersToMarkerMsg) {
  // Ready
  tmc_manipulation_types::OuterObjectParameters outer_object_parameters;
  outer_object_parameters.name = "OuterObjectParameters";
  outer_object_parameters.origin_to_base = Eigen::Translation3d(0.0, 0.0, 1.0) * Eigen::Quaterniond::Identity();
  outer_object_parameters.base_to_child = {Eigen::Translation3d(1.0, 1.0, 0.0) * Eigen::Quaterniond::Identity(),
                                           Eigen::Translation3d(2.0, 2.0, 0.0) * Eigen::Quaterniond::Identity(),
                                           Eigen::Translation3d(3.0, 3.0, 0.0) * Eigen::Quaterniond::Identity(),
                                           Eigen::Translation3d(4.0, 4.0, 0.0) * Eigen::Quaterniond::Identity(),
                                           Eigen::Translation3d(5.0, 5.0, 0.0) * Eigen::Quaterniond::Identity()};
  outer_object_parameters.shape.resize(5);
  outer_object_parameters.shape[0].type = tmc_manipulation_types::kBox;
  outer_object_parameters.shape[0].dimensions = {1.0, 2.0, 3.0};
  outer_object_parameters.shape[1].type = tmc_manipulation_types::kCylinder;
  outer_object_parameters.shape[1].dimensions = {1.0, 2.0};
  outer_object_parameters.shape[2].type = tmc_manipulation_types::kSphere;
  outer_object_parameters.shape[2].dimensions = {1.0};
  outer_object_parameters.shape[3].type = tmc_manipulation_types::kCapsule;
  outer_object_parameters.shape[3].dimensions = {1.0, 2.0};
  outer_object_parameters.shape[4].type = tmc_manipulation_types::kMesh;
  outer_object_parameters.shape[4].filename = "MeshFile";

  int32_t start_id = 1;
  std::string frame_id = "Shape";
  rclcpp::Time time(0, 2);
  std_msgs::msg::ColorRGBA color;
  color.a = 0.1;
  color.b = 0.2;
  color.g = 0.3;
  color.r = 0.4;
  // Convert
  std::vector<visualization_msgs::msg::Marker> marker_msgs;
  OuterObjectParametersToMarkerMsg(outer_object_parameters, start_id, frame_id, time, color, marker_msgs);
  // Judgment
  ASSERT_EQ(outer_object_parameters.base_to_child.size(), marker_msgs.size());

  for (auto i = 0; i < outer_object_parameters.base_to_child.size(); ++i) {
    EXPECT_EQ(static_cast<builtin_interfaces::msg::Time>(time), marker_msgs[i].header.stamp);
    EXPECT_EQ("tmc_manipulation_types_bridge", marker_msgs[i].ns);
    EXPECT_EQ(start_id + i, marker_msgs[i].id);
    EXPECT_EQ(visualization_msgs::msg::Marker::ADD, marker_msgs[i].action);
    IsEqual(outer_object_parameters.origin_to_base * outer_object_parameters.base_to_child[i], marker_msgs[i].pose);
    IsEqual(color, marker_msgs[i].color);
  }
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, marker_msgs[0].type);
  EXPECT_EQ(outer_object_parameters.shape[0].dimensions.at(0), marker_msgs[0].scale.x);
  EXPECT_EQ(outer_object_parameters.shape[0].dimensions.at(1), marker_msgs[0].scale.y);
  EXPECT_EQ(outer_object_parameters.shape[0].dimensions.at(2), marker_msgs[0].scale.z);

  EXPECT_EQ(visualization_msgs::msg::Marker::CYLINDER, marker_msgs[1].type);
  EXPECT_EQ(outer_object_parameters.shape[1].dimensions.at(0) * 2.0, marker_msgs[1].scale.x);
  EXPECT_EQ(outer_object_parameters.shape[1].dimensions.at(0) * 2.0, marker_msgs[1].scale.y);
  EXPECT_EQ(outer_object_parameters.shape[1].dimensions.at(1), marker_msgs[1].scale.z);

  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, marker_msgs[2].type);
  EXPECT_EQ(outer_object_parameters.shape[2].dimensions.at(0) * 2.0, marker_msgs[2].scale.x);
  EXPECT_EQ(outer_object_parameters.shape[2].dimensions.at(0) * 2.0, marker_msgs[2].scale.y);
  EXPECT_EQ(outer_object_parameters.shape[2].dimensions.at(0) * 2.0, marker_msgs[2].scale.z);

  EXPECT_EQ(visualization_msgs::msg::Marker::CYLINDER, marker_msgs[3].type);
  EXPECT_EQ(outer_object_parameters.shape[3].dimensions.at(0) * 2.0, marker_msgs[3].scale.x);
  EXPECT_EQ(outer_object_parameters.shape[3].dimensions.at(0) * 2.0, marker_msgs[3].scale.y);
  EXPECT_EQ(outer_object_parameters.shape[3].dimensions.at(1), marker_msgs[3].scale.z);

  EXPECT_EQ(visualization_msgs::msg::Marker::MESH_RESOURCE, marker_msgs[4].type);
  EXPECT_EQ("file://" + outer_object_parameters.shape[4].filename, marker_msgs[4].mesh_resource);
}

TEST_F(ManipulationTypesBridgeTest, BaseMovementTypeToBaseMovementMsg) {
  tmc_manipulation_msgs::msg::BaseMovementType msg;

  BaseMovementTypeToBaseMovementMsg(tmc_manipulation_types::kFloat, msg);
  EXPECT_EQ(tmc_manipulation_msgs::msg::BaseMovementType::FLOAT, msg.val);

  BaseMovementTypeToBaseMovementMsg(tmc_manipulation_types::kPlanar, msg);
  EXPECT_EQ(tmc_manipulation_msgs::msg::BaseMovementType::PLANAR, msg.val);

  BaseMovementTypeToBaseMovementMsg(tmc_manipulation_types::kRailX, msg);
  EXPECT_EQ(tmc_manipulation_msgs::msg::BaseMovementType::RAIL_X, msg.val);

  BaseMovementTypeToBaseMovementMsg(tmc_manipulation_types::kRailY, msg);
  EXPECT_EQ(tmc_manipulation_msgs::msg::BaseMovementType::RAIL_Y, msg.val);

  BaseMovementTypeToBaseMovementMsg(tmc_manipulation_types::kRailZ, msg);
  EXPECT_EQ(tmc_manipulation_msgs::msg::BaseMovementType::RAIL_Z, msg.val);

  BaseMovementTypeToBaseMovementMsg(tmc_manipulation_types::kRotationX, msg);
  EXPECT_EQ(tmc_manipulation_msgs::msg::BaseMovementType::ROTATION_X, msg.val);

  BaseMovementTypeToBaseMovementMsg(tmc_manipulation_types::kRotationY, msg);
  EXPECT_EQ(tmc_manipulation_msgs::msg::BaseMovementType::ROTATION_Y, msg.val);

  BaseMovementTypeToBaseMovementMsg(tmc_manipulation_types::kRotationZ, msg);
  EXPECT_EQ(tmc_manipulation_msgs::msg::BaseMovementType::ROTATION_Z, msg.val);

  BaseMovementTypeToBaseMovementMsg(tmc_manipulation_types::kNone, msg);
  EXPECT_EQ(tmc_manipulation_msgs::msg::BaseMovementType::NONE, msg.val);
}

TEST_F(ManipulationTypesBridgeTest, BaseMovementTypeMsgToBaseMovement) {
  tmc_manipulation_msgs::msg::BaseMovementType msg;
  tmc_manipulation_types::BaseMovementType type;

  msg.val = tmc_manipulation_msgs::msg::BaseMovementType::FLOAT;
  BaseMovementTypeMsgToBaseMovement(msg, type);
  EXPECT_EQ(tmc_manipulation_types::kFloat, type);

  msg.val = tmc_manipulation_msgs::msg::BaseMovementType::PLANAR;
  BaseMovementTypeMsgToBaseMovement(msg, type);
  EXPECT_EQ(tmc_manipulation_types::kPlanar, type);

  msg.val = tmc_manipulation_msgs::msg::BaseMovementType::RAIL_X;
  BaseMovementTypeMsgToBaseMovement(msg, type);
  EXPECT_EQ(tmc_manipulation_types::kRailX, type);

  msg.val = tmc_manipulation_msgs::msg::BaseMovementType::RAIL_Y;
  BaseMovementTypeMsgToBaseMovement(msg, type);
  EXPECT_EQ(tmc_manipulation_types::kRailY, type);

  msg.val = tmc_manipulation_msgs::msg::BaseMovementType::RAIL_Z;
  BaseMovementTypeMsgToBaseMovement(msg, type);
  EXPECT_EQ(tmc_manipulation_types::kRailZ, type);

  msg.val = tmc_manipulation_msgs::msg::BaseMovementType::ROTATION_X;
  BaseMovementTypeMsgToBaseMovement(msg, type);
  EXPECT_EQ(tmc_manipulation_types::kRotationX, type);

  msg.val = tmc_manipulation_msgs::msg::BaseMovementType::ROTATION_Y;
  BaseMovementTypeMsgToBaseMovement(msg, type);
  EXPECT_EQ(tmc_manipulation_types::kRotationY, type);

  msg.val = tmc_manipulation_msgs::msg::BaseMovementType::ROTATION_Z;
  BaseMovementTypeMsgToBaseMovement(msg, type);
  EXPECT_EQ(tmc_manipulation_types::kRotationZ, type);

  msg.val = tmc_manipulation_msgs::msg::BaseMovementType::NONE;
  BaseMovementTypeMsgToBaseMovement(msg, type);
  EXPECT_EQ(tmc_manipulation_types::kNone, type);
}

TEST_F(ManipulationTypesBridgeTest, MultiDOFJointTrajectoryToMultiDOFJointTrajectoryMsg) {
  // Ready
  const auto multi_dof_trajectory = GenerateMultiDOFJointTrajectory();
  // Convert
  trajectory_msgs::msg::MultiDOFJointTrajectory multi_dof_trajectory_msg;
  MultiDOFJointTrajectoryToMultiDOFJointTrajectoryMsg(multi_dof_trajectory, multi_dof_trajectory_msg);
  // Judgment
  EXPECT_EQ(multi_dof_trajectory.names, multi_dof_trajectory_msg.joint_names);
  ASSERT_EQ(multi_dof_trajectory.path.size(), multi_dof_trajectory_msg.points.size());
  for (auto i = 0; i < multi_dof_trajectory.path.size(); ++i) {
    IsEqual(multi_dof_trajectory.path[i], multi_dof_trajectory_msg.points[i].transforms);
  }
}

TEST_F(ManipulationTypesBridgeTest, MultiDOFJointTrajectoryMsgToMultiDOFJointTrajectory) {
  // Ready
  const auto multi_dof_trajectory_msg = GenerateMultiDOFJointTrajectoryMsg();
  // Convert
  tmc_manipulation_types::MultiDOFJointTrajectory multi_dof_trajectory;
  MultiDOFJointTrajectoryMsgToMultiDOFJointTrajectory(multi_dof_trajectory_msg, multi_dof_trajectory);
  // Judgment
  EXPECT_EQ(multi_dof_trajectory_msg.joint_names, multi_dof_trajectory.names);
  ASSERT_EQ(multi_dof_trajectory_msg.points.size(), multi_dof_trajectory.path.size());
  for (auto i = 0; i < multi_dof_trajectory_msg.points.size(); ++i) {
    IsEqual(multi_dof_trajectory_msg.points[i].transforms, multi_dof_trajectory.path[i]);
  }
}

TEST_F(ManipulationTypesBridgeTest, ConvertSequence) {
  // Ready
  const auto joint_state = GenerateJointStateMsg();
  std::vector<sensor_msgs::msg::JointState> joint_states = {joint_state, joint_state};
  // Convert
  std::vector<tmc_manipulation_types::JointState> common_joint_states;
  ConvertSequence<sensor_msgs::msg::JointState, tmc_manipulation_types::JointState>(
      joint_states, common_joint_states, &JointStateMsgToJointState);
  // Judgment
  EXPECT_EQ(joint_states.size(), common_joint_states.size());
  for (auto i = 0; i < joint_states.size(); ++i) {
    EXPECT_EQ(joint_states[i].name, common_joint_states[i].name);
    IsEqual(joint_states[i].position, common_joint_states[i].position);
  }
}

TEST_F(ManipulationTypesBridgeTest, TimedJointTrajectoryToJointTrajectoryMsg) {
  // Ready
  const auto trajectory = GenerateTimedJointTrajectory();
  // Convert
  trajectory_msgs::msg::JointTrajectory msg;
  TimedJointTrajectoryToJointTrajectoryMsg(trajectory, msg);
  // Judgment
  EXPECT_EQ(trajectory.joint_names, msg.joint_names);
  ASSERT_EQ(trajectory.points.size(), msg.points.size());
  for (auto i = 0; i < trajectory.points.size(); ++i) {
    IsEqual(trajectory.points[i], msg.points[i]);
  }
}

TEST_F(ManipulationTypesBridgeTest, JointTrajectoryMsgToTimedJointTrajectory) {
  // Ready
  const auto trajectory_msg = GenerateJointTrajectoryMsg();
  // Convert
  tmc_manipulation_types::TimedJointTrajectory trajectory;
  JointTrajectoryMsgToTimedJointTrajectory(trajectory_msg, trajectory);
  // Judgment
  EXPECT_EQ(trajectory_msg.joint_names, trajectory.joint_names);
  ASSERT_EQ(trajectory_msg.points.size(), trajectory.points.size());
  for (auto i = 0; i < trajectory_msg.points.size(); ++i) {
    IsEqual(trajectory_msg.points[i], trajectory.points[i]);
  }
}

TEST_F(ManipulationTypesBridgeTest, TimedMultiDOFJointTrajectoryToMultiDOFJointTrajectoryMsg) {
  // Ready
  const auto multi_dof_trajectory = GenerateTimedMultiDOFJointTrajectory();
  // Convert
  trajectory_msgs::msg::MultiDOFJointTrajectory multi_dof_trajectory_msg;
  TimedMultiDOFJointTrajectoryToMultiDOFJointTrajectoryMsg(multi_dof_trajectory, multi_dof_trajectory_msg);
  // Judgment
  EXPECT_EQ(multi_dof_trajectory.joint_names, multi_dof_trajectory_msg.joint_names);
  ASSERT_EQ(multi_dof_trajectory.points.size(), multi_dof_trajectory_msg.points.size());
  for (auto i = 0; i < multi_dof_trajectory.points.size(); ++i) {
    IsEqual(multi_dof_trajectory.points[i], multi_dof_trajectory_msg.points[i]);
  }
}

TEST_F(ManipulationTypesBridgeTest, MultiDOFJointTrajectoryMsgToTimedMultiDOFJointTrajectory) {
  // Ready
  const auto multi_dof_trajectory_msg = GenerateMultiDOFJointTrajectoryMsg();
  // Convert
  tmc_manipulation_types::TimedMultiDOFJointTrajectory multi_dof_trajectory;
  MultiDOFJointTrajectoryMsgToTimedMultiDOFJointTrajectory(multi_dof_trajectory_msg, multi_dof_trajectory);
  // Judgment
  EXPECT_EQ(multi_dof_trajectory_msg.joint_names, multi_dof_trajectory.joint_names);
  ASSERT_EQ(multi_dof_trajectory_msg.points.size(), multi_dof_trajectory.points.size());
  for (auto i = 0; i < multi_dof_trajectory_msg.points.size(); ++i) {
    IsEqual(multi_dof_trajectory_msg.points[i], multi_dof_trajectory.points[i]);
  }
}

TEST_F(ManipulationTypesBridgeTest, MultiDOFJointStateToMultiDOFJointStateMsg) {
  // Ready
  const auto multi_dof_joint_state = GenerateMultiDOFJointState();
  // Convert
  sensor_msgs::msg::MultiDOFJointState multi_dof_joint_state_msg;
  MultiDOFJointStateToMultiDOFJointStateMsg(multi_dof_joint_state, multi_dof_joint_state_msg);
  // Judgment
  EXPECT_EQ(multi_dof_joint_state.names, multi_dof_joint_state_msg.joint_names);

  ASSERT_EQ(multi_dof_joint_state.poses.size(), multi_dof_joint_state_msg.transforms.size());
  for (auto i = 0; i < multi_dof_joint_state.poses.size(); ++i) {
    IsEqual(multi_dof_joint_state.poses[i], multi_dof_joint_state_msg.transforms[i]);
  }
  ASSERT_EQ(multi_dof_joint_state.twist.size(), multi_dof_joint_state_msg.twist.size());
  for (auto i = 0; i < multi_dof_joint_state.twist.size(); ++i) {
    IsEqual(multi_dof_joint_state.twist[i], multi_dof_joint_state_msg.twist[i]);
  }
  ASSERT_EQ(multi_dof_joint_state.wrench.size(), multi_dof_joint_state_msg.wrench.size());
  for (auto i = 0; i < multi_dof_joint_state.wrench.size(); ++i) {
    IsEqual(multi_dof_joint_state.wrench[i], multi_dof_joint_state_msg.wrench[i]);
  }
}

TEST_F(ManipulationTypesBridgeTest, MultiDOFJointStateMsgToMultiDOFJointState) {
  // Ready
  const auto multi_dof_joint_state_msg = GenerateMultiDOFJointStateMsg();
  // Convert
  tmc_manipulation_types::MultiDOFJointState multi_dof_joint_state;
  MultiDOFJointStateMsgToMultiDOFJointState(multi_dof_joint_state_msg, multi_dof_joint_state);
  // Judgment
  EXPECT_EQ(multi_dof_joint_state_msg.joint_names, multi_dof_joint_state.names);

  ASSERT_EQ(multi_dof_joint_state_msg.transforms.size(), multi_dof_joint_state.poses.size());
  for (auto i = 0; i < multi_dof_joint_state_msg.transforms.size(); ++i) {
    IsEqual(multi_dof_joint_state_msg.transforms[i], multi_dof_joint_state.poses[i]);
  }
  ASSERT_EQ(multi_dof_joint_state_msg.twist.size(), multi_dof_joint_state.twist.size());
  for (auto i = 0; i < multi_dof_joint_state_msg.twist.size(); ++i) {
    IsEqual(multi_dof_joint_state_msg.twist[i], multi_dof_joint_state.twist[i]);
  }
  ASSERT_EQ(multi_dof_joint_state_msg.wrench.size(), multi_dof_joint_state.wrench.size());
  for (auto i = 0; i < multi_dof_joint_state_msg.wrench.size(); ++i) {
    IsEqual(multi_dof_joint_state_msg.wrench[i], multi_dof_joint_state.wrench[i]);
  }
}

TEST_F(ManipulationTypesBridgeTest, RobotStateAndRobotStateMsg) {
  // Since it is a combination of other conversion, conversion => Reverse conversion makes it easier
  tmc_manipulation_types::RobotState robot_state_in;
  robot_state_in.joint_state = GenerateJointState();
  robot_state_in.multi_dof_joint_state = GenerateMultiDOFJointState();

  moveit_msgs::msg::RobotState robot_state_msg;
  RobotStateToRobotStateMsg(robot_state_in, robot_state_msg);

  tmc_manipulation_types::RobotState robot_state_out;
  RobotStateMsgToRobotState(robot_state_msg, robot_state_out);

  EXPECT_EQ(robot_state_in.joint_state.name, robot_state_out.joint_state.name);
  ASSERT_EQ(robot_state_in.joint_state.position.size(), robot_state_out.joint_state.position.size());
  ASSERT_EQ(robot_state_in.joint_state.velocity.size(), robot_state_out.joint_state.velocity.size());
  ASSERT_EQ(robot_state_in.joint_state.effort.size(), robot_state_out.joint_state.effort.size());
  for (auto i = 0; i < robot_state_in.joint_state.name.size(); ++i) {
    EXPECT_EQ(robot_state_in.joint_state.position[i], robot_state_out.joint_state.position[i]);
    EXPECT_EQ(robot_state_in.joint_state.velocity[i], robot_state_out.joint_state.velocity[i]);
    EXPECT_EQ(robot_state_in.joint_state.effort[i], robot_state_out.joint_state.effort[i]);
  }

  EXPECT_EQ(robot_state_in.multi_dof_joint_state.names, robot_state_out.multi_dof_joint_state.names);
  ASSERT_EQ(robot_state_in.multi_dof_joint_state.poses.size(), robot_state_out.multi_dof_joint_state.poses.size());
  ASSERT_EQ(robot_state_in.multi_dof_joint_state.twist.size(), robot_state_out.multi_dof_joint_state.twist.size());
  ASSERT_EQ(robot_state_in.multi_dof_joint_state.wrench.size(), robot_state_out.multi_dof_joint_state.wrench.size());
  for (auto i = 0; i < robot_state_in.multi_dof_joint_state.names.size(); ++i) {
    EXPECT_EQ(robot_state_in.multi_dof_joint_state.poses[i].translation(),
              robot_state_out.multi_dof_joint_state.poses[i].translation());
    EXPECT_EQ(robot_state_in.multi_dof_joint_state.poses[i].linear(),
              robot_state_out.multi_dof_joint_state.poses[i].linear());
    EXPECT_EQ(robot_state_in.multi_dof_joint_state.twist[i], robot_state_out.multi_dof_joint_state.twist[i]);
    EXPECT_EQ(robot_state_in.multi_dof_joint_state.wrench[i], robot_state_out.multi_dof_joint_state.wrench[i]);
  }
}

TEST_F(ManipulationTypesBridgeTest, TimedRobotTrajectoryAndRobotTrajectoryMsg) {
  // Since it is a combination of other conversion, conversion => Reverse conversion makes it easier
  tmc_manipulation_types::TimedRobotTrajectory robot_trajectory_in;
  robot_trajectory_in.joint_trajectory = GenerateTimedJointTrajectory();
  robot_trajectory_in.multi_dof_joint_trajectory = GenerateTimedMultiDOFJointTrajectory();

  moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
  TimedRobotTrajectoryToRobotTrajectoryMsg(robot_trajectory_in, robot_trajectory_msg);

  tmc_manipulation_types::TimedRobotTrajectory robot_trajectory_out;
  RobotTrajectoryMsgToTimedRobotTrajectory(robot_trajectory_msg, robot_trajectory_out);

  EXPECT_EQ(robot_trajectory_in.joint_trajectory.joint_names, robot_trajectory_out.joint_trajectory.joint_names);
  EXPECT_EQ(2, robot_trajectory_out.joint_trajectory.points.size());
  for (auto i = 0; i < 2; ++i) {
    EXPECT_EQ(robot_trajectory_in.joint_trajectory.points[i].positions,
              robot_trajectory_out.joint_trajectory.points[i].positions);
    EXPECT_EQ(robot_trajectory_in.joint_trajectory.points[i].velocities,
              robot_trajectory_out.joint_trajectory.points[i].velocities);
    EXPECT_EQ(robot_trajectory_in.joint_trajectory.points[i].accelerations,
              robot_trajectory_out.joint_trajectory.points[i].accelerations);
    EXPECT_EQ(robot_trajectory_in.joint_trajectory.points[i].effort,
              robot_trajectory_out.joint_trajectory.points[i].effort);
    EXPECT_DOUBLE_EQ(robot_trajectory_in.joint_trajectory.points[i].time_from_start,
                     robot_trajectory_out.joint_trajectory.points[i].time_from_start);
  }

  EXPECT_EQ(robot_trajectory_in.multi_dof_joint_trajectory.joint_names,
            robot_trajectory_out.multi_dof_joint_trajectory.joint_names);
  EXPECT_EQ(2, robot_trajectory_out.multi_dof_joint_trajectory.points.size());
  for (auto i = 0; i < 2; ++i) {
    ASSERT_EQ(1, robot_trajectory_out.multi_dof_joint_trajectory.points[i].transforms.size());
    EXPECT_EQ(robot_trajectory_in.multi_dof_joint_trajectory.points[i].transforms[0].translation(),
              robot_trajectory_out.multi_dof_joint_trajectory.points[i].transforms[0].translation());
    EXPECT_EQ(robot_trajectory_in.multi_dof_joint_trajectory.points[i].transforms[0].linear(),
              robot_trajectory_out.multi_dof_joint_trajectory.points[i].transforms[0].linear());
    EXPECT_EQ(robot_trajectory_in.multi_dof_joint_trajectory.points[i].velocities,
              robot_trajectory_out.multi_dof_joint_trajectory.points[i].velocities);
    EXPECT_EQ(robot_trajectory_in.multi_dof_joint_trajectory.points[i].accelerations,
              robot_trajectory_out.multi_dof_joint_trajectory.points[i].accelerations);
    EXPECT_DOUBLE_EQ(robot_trajectory_in.multi_dof_joint_trajectory.points[i].time_from_start,
                     robot_trajectory_out.multi_dof_joint_trajectory.points[i].time_from_start);
  }
}

// 割愛：ConvertSequenceWithEigenIn, ConvertSequenceWithEigenOut
}  // namespace tmc_manipulation_types_bridge


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
