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

#ifndef TMC_MANIPULATION_TYPES_BRIDGE_MANIPULATION_MSG_CONVERTOR_HPP_
#define TMC_MANIPULATION_TYPES_BRIDGE_MANIPULATION_MSG_CONVERTOR_HPP_

#include <stdint.h>

#include <string>
#include <vector>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tmc_manipulation_msgs/msg/base_movement_type.hpp>
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_planning_msgs/msg/joint_position.hpp>
#include <tmc_planning_msgs/msg/task_space_region.hpp>

// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Wrench.h>
// #include <ros/ros.h>
// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/MultiDOFJointTrajectory.h>
// #include <tmc_geometry_msgs/OrientedBoundingBox.h>
// #include <tmc_manipulation_msgs/CollisionEnvironment.h>
// #include <tmc_manipulation_msgs/CollisionObject.h>
// #include <tmc_manipulation_msgs/JointLimits.h>
// #include <tmc_mapping_msgs/CollisionMap.h>
// #include <tmc_planning_msgs/AttachedObject.h>


namespace tmc_manipulation_types_bridge {
/// Use the CoverT function to re -arrest to another array
/// @param seq_in input vector type
/// @param seq_out output vector type
/// @param convert_func conversion function
template <typename Type_in, typename Type_out>
void ConvertSequence(
    const std::vector<Type_in>& seq_in,
    std::vector<Type_out>& seq_out,
    std::function<void(const Type_in&, Type_out&)> convert_func) {
  seq_out.clear();
  for (typename std::vector<Type_in>::const_iterator it = seq_in.begin();
      it != seq_in.end(); ++it) {
    Type_out out_elem;
    convert_func(*it, out_elem);
    seq_out.push_back(out_elem);
  }
}

/// @brief Use the CoverT function to make it to another array (if the input vector is Eigen)
/// @param seq_in input vector type
/// @param seq_out output vector type
/// @param convert_func conversion function
template <typename Type_in, typename Type_out>
void ConvertSequenceWithEigenIn(
    const std::vector<Type_in, Eigen::aligned_allocator<Type_in> >& seq_in,
    std::vector<Type_out>& seq_out,
    std::function<Type_out(const Type_in&)> convert_func) {
  seq_out.clear();
  for (typename
      std::vector<Type_in, Eigen::aligned_allocator<Type_in> >::const_iterator
          it = seq_in.begin();
      it != seq_in.end(); ++it) {
    seq_out.push_back(convert_func(*it));
  }
}

/// Use the CoverT function to make it to another array (if the output vector is Eigen)
/// @param seq_in input vector type
/// @param seq_out output vector type
/// @param convert_func conversion function
template <typename Type_in, typename Type_out>
void ConvertSequenceWithEigenOut(
    const std::vector<Type_in>& seq_in,
    std::vector<Type_out, Eigen::aligned_allocator<Type_out> >& seq_out,
    std::function<void(const Type_in&, Type_out&)> convert_func) {
  seq_out.clear();
  for (typename
      std::vector<Type_in>::const_iterator it = seq_in.begin();
      it != seq_in.end(); ++it) {
    Type_out out_elem;
    convert_func(*it, out_elem);
    seq_out.push_back(out_elem);
  }
}

std::vector<std::string>
Split(const std::string& target_string, const char* const delimiter);

void JointStateMsgToJointState(
    const sensor_msgs::msg::JointState& sensor_msgs_joint_state,
    tmc_manipulation_types::JointState& commmon_joint_state_out);
void JointStateToJointStateMsg(
    const tmc_manipulation_types::JointState& commmon_joint_state,
    sensor_msgs::msg::JointState& sensor_msgs_joint_state_out);

void JointPositionMsgToConfig(
    const tmc_planning_msgs::msg::JointPosition& joint_position,
    tmc_manipulation_types::Config& common_config_out);
void ConfigToJointPositionMsg(
    const tmc_manipulation_types::Config& common_config,
    tmc_planning_msgs::msg::JointPosition& joint_position_out);

void TaskSpaceRegionMsgToTaskSpaceRegion(
    const tmc_planning_msgs::msg::TaskSpaceRegion& planning_msgs_task_space_region,
    tmc_manipulation_types::TaskSpaceRegion& common_task_space_region_out);
void TaskSpaceRegionToTaskSpaceRegionMsg(
    const tmc_manipulation_types::TaskSpaceRegion& common_task_space_region,
    tmc_planning_msgs::msg::TaskSpaceRegion& planning_msgs_task_space_region_out);

// void ObjectIdentifierMsgToObjectName(
//     const tmc_msgs::ObjectIdentifier& object_identifier,
//     std::string& object_name_out);
// void ObjectNameToObjectIdentifierMsg(
//     const std::string& object_name,
//     tmc_msgs::ObjectIdentifier& object_identifier_out);

void AttachedObjectMsgToAttachedObject(
    const moveit_msgs::msg::AttachedCollisionObject& planning_msgs_attached_object,
    tmc_manipulation_types::AttachedObject& common_attached_object_out);
// void AttachedObjectToAttachedObjectMsg(
//     const tmc_manipulation_types::AttachedObject& common_attached_object,
//     tmc_planning_msgs::AttachedObject& planning_msgs_attached_object_out);

void JointTrajectoryMsgToJointTrajectory(
    const trajectory_msgs::msg::JointTrajectory& joint_trajectory_msg,
    tmc_manipulation_types::JointTrajectory& joint_trajectory_out);
void JointTrajectoryToJointTrajectoryMsg(
    const tmc_manipulation_types::JointTrajectory& joint_trajectory,
    trajectory_msgs::msg::JointTrajectory& joint_trajectory_msg_out);

void SolidPrimitiveToShape(
    const shape_msgs::msg::SolidPrimitive& solid_primitive,
    tmc_manipulation_types::Shape& shape_out);
void ShapeToSolidPrimitive(
    const tmc_manipulation_types::Shape& shape,
    shape_msgs::msg::SolidPrimitive& solid_primitive_out);

void MeshMsgToShape(
    const shape_msgs::msg::Mesh& mesh_msg,
    tmc_manipulation_types::Shape& shape_out);
void ShapeToMeshMsg(
    const tmc_manipulation_types::Shape& shape,
    shape_msgs::msg::Mesh& mesh_msg_out);

void OccupancyGridMsgToOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid& map_msg,
    tmc_manipulation_types::OccupancyGrid& map_out);

void CollisionObjectToOuterObjectParameters(
    const moveit_msgs::msg::CollisionObject& collision_object,
    tmc_manipulation_types::OuterObjectParameters& outer_object_out);
void OuterObjectParametersToCollisionObject(
    const tmc_manipulation_types::OuterObjectParameters& outer_object,
    moveit_msgs::msg::CollisionObject& collision_object_out);

void PlannerShapeToMarkerMsg(
    const tmc_manipulation_types::Shape& collision_detector_shape,
    const tmc_manipulation_types::Transform& pose,
    int32_t id,
    const std::string& frame_id,
    const rclcpp::Time& time,
    const std_msgs::msg::ColorRGBA& color,
    visualization_msgs::msg::Marker& marker_msgs_out);

void OuterObjectParametersToMarkerMsg(
    const tmc_manipulation_types::OuterObjectParameters& outer_object_parameters,
    int32_t start_id,
    const std::string& frame_id,
    const rclcpp::Time& time,
    const std_msgs::msg::ColorRGBA& color,
    std::vector<visualization_msgs::msg::Marker>& marker_msgs_out);

// void CollisionEnvironmentToOuterObjectSeq(
//     const tmc_manipulation_msgs::CollisionEnvironment& collision_environment,
//     tmc_manipulation_types::OuterObjectParametersSeq& outer_object_seq_out);

// void CollisionEnvironmentToCuboidSeq(
//     const tmc_manipulation_msgs::CollisionEnvironment& collision_environment,
//     const std::string& box_name,
//     tmc_manipulation_types::CuboidSeq& cuboid_seq_out);

// void CollisionMapToCuboidSeq(
//     const tmc_mapping_msgs::CollisionMap& collision_map,
//     const std::string& box_name,
//     const Eigen::Affine3d& origin_to_map,
//     tmc_manipulation_types::CuboidSeq& cuboid_seq_out);

void BaseMovementTypeToBaseMovementMsg(
    const tmc_manipulation_types::BaseMovementType& base_type,
    tmc_manipulation_msgs::msg::BaseMovementType& base_type_msgs_out);

void BaseMovementTypeMsgToBaseMovement(
    const tmc_manipulation_msgs::msg::BaseMovementType& base_type_msg,
    tmc_manipulation_types::BaseMovementType& base_type_out);

void MultiDOFJointTrajectoryToMultiDOFJointTrajectoryMsg(
    const tmc_manipulation_types::MultiDOFJointTrajectory& multi_dof_trajectory,
    trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory_msg_out);

void MultiDOFJointTrajectoryMsgToMultiDOFJointTrajectory(
    const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory_msg,
    tmc_manipulation_types::MultiDOFJointTrajectory& multi_dof_trajectory_out);

void TimedJointTrajectoryToJointTrajectoryMsg(
    const tmc_manipulation_types::TimedJointTrajectory& trajectory,
    trajectory_msgs::msg::JointTrajectory& trajectory_msg_out);

void JointTrajectoryMsgToTimedJointTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory_msg,
    tmc_manipulation_types::TimedJointTrajectory& trajectory_out);

void TimedMultiDOFJointTrajectoryToMultiDOFJointTrajectoryMsg(
    const tmc_manipulation_types::TimedMultiDOFJointTrajectory& multi_dof_trajectory,
    trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory_msg_out);

void MultiDOFJointTrajectoryMsgToTimedMultiDOFJointTrajectory(
    const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory_msg,
    tmc_manipulation_types::TimedMultiDOFJointTrajectory& multi_dof_trajectory_out);

void MultiDOFJointStateToMultiDOFJointStateMsg(
    const tmc_manipulation_types::MultiDOFJointState& multi_dof_joint_state,
    sensor_msgs::msg::MultiDOFJointState& multi_dof_joint_state_msg_out);

void MultiDOFJointStateMsgToMultiDOFJointState(
    const sensor_msgs::msg::MultiDOFJointState& multi_dof_joint_state_msg,
    tmc_manipulation_types::MultiDOFJointState& multi_dof_joint_state_out);

void RobotStateToRobotStateMsg(
    const tmc_manipulation_types::RobotState& robot_state,
    moveit_msgs::msg::RobotState& robot_state_msg_out);

void RobotStateMsgToRobotState(
    const moveit_msgs::msg::RobotState& robot_state_msg,
    tmc_manipulation_types::RobotState& robot_state_out);

void TimedRobotTrajectoryToRobotTrajectoryMsg(
    const tmc_manipulation_types::TimedRobotTrajectory& robot_trajectory,
    moveit_msgs::msg::RobotTrajectory& robot_trajectory_msg_out);

void RobotTrajectoryMsgToTimedRobotTrajectory(
    const moveit_msgs::msg::RobotTrajectory& robot_trajectory_msg,
    tmc_manipulation_types::TimedRobotTrajectory& robot_trajectory_out);

}  // namespace tmc_manipulation_types_bridge

#endif  // TMC_MANIPULATION_TYPES_BRIDGE_MANIPULATION_MSG_CONVERTOR_HPP_
