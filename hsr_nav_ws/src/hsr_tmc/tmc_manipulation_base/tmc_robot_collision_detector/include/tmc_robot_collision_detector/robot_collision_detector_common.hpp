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
/// @brief Common setting of interference checks using a robot model
#ifndef TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_COMMON_HPP_
#define TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_COMMON_HPP_

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <tmc_collision_detector/collision_detector.hpp>
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

namespace tmc_robot_collision_detector {
/// AABB array
using AABBSeq = std::vector<tmc_manipulation_types::AABB, Eigen::aligned_allocator<tmc_manipulation_types::AABB>>;

/// External object map
using OuterObjectMap =
    std::map<std::string, tmc_manipulation_types::OuterObjectParameters, std::less<std::string>,
             Eigen::aligned_allocator<std::pair<const std::string, tmc_manipulation_types::OuterObjectParameters>>>;

/// Structure that holds Cuboid
struct BoundingBox {
  std::vector<std::string> group_name;
  std::vector<tmc_manipulation_types::CuboidSeq> boxes;
};

/// What to determine the overlap between Cuboid and robot
enum CuboidOverlapType {
  /// Deterts overlapping on the XY plane
  kOverlap2DMap = 0,
  /// Determination of overlapping with AABB
  kOverlapAabb
};

/// How to deal with robot AABB
enum CuboidOverlapGroupType {
  /// Measures overlapping for each group of robots
  kOverlapGroup = 0,
  /// Performs overlapping with AABB of the entire robot
  kOverlapRobot
};

/// Search results for nearby objects
struct ClosestObject {
  /// Search object name
  std::string name;
  /// Results inside the robot
  tmc_collision_detector::ClosestResult inner_result;
  /// Robot external results
  tmc_collision_detector::ClosestResult outer_result;
};

/// Robot model file type
enum ModelFileType {
  kTrml,
  kUrdf
};

/// Information for Collison's position/posture
struct CollisionFrameInfo {
  std::string parent_name;
  Eigen::Affine3d parent_to_child;
};
}  // namespace tmc_robot_collision_detector
#endif  // TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_COMMON_HPP_
