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

#include "utils.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

namespace {
constexpr uint32_t kNumColor = 100;
}  // namespace

namespace tmc_collision_environment {

std::vector<std_msgs::msg::ColorRGBA> GenerateColors(uint32_t _seed) {
  uint32_t seed(_seed);
  std::vector<std_msgs::msg::ColorRGBA> colors;
  double rand_max = static_cast<double>(RAND_MAX);
  for (auto i = 0; i < kNumColor; ++i) {
    std_msgs::msg::ColorRGBA color;
    color.r = static_cast<double>(rand_r(&seed)) / rand_max;
    color.g = static_cast<double>(rand_r(&seed)) / rand_max;
    color.b = static_cast<double>(rand_r(&seed)) / rand_max;
    color.a = 1.0;
    colors.emplace_back(color);
  }
  return colors;
}

visualization_msgs::msg::MarkerArray ConvertToMarkerArray(
    const std::vector<moveit_msgs::msg::CollisionObject>& objects,
    const std::vector<std_msgs::msg::ColorRGBA>& colors,
    const rclcpp::Duration& lifetime) {
  visualization_msgs::msg::MarkerArray markers;
  uint32_t color_index = 0;
  for (const auto& collision_object : objects) {
    Eigen::Affine3d origin_to_object_base;
    tf2::fromMsg(collision_object.pose, origin_to_object_base);
    for (auto i = 0; i < collision_object.primitives.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header = collision_object.header;
      marker.ns = collision_object.id;
      marker.id = i;
      marker.action = visualization_msgs::msg::Marker::ADD;
      Eigen::Affine3d object_base_to_object;
      tf2::fromMsg(collision_object.primitive_poses[i], object_base_to_object);
      marker.pose = tf2::toMsg(origin_to_object_base * object_base_to_object);
      marker.color = colors[color_index % colors.size()];
      marker.lifetime = lifetime;
      switch (collision_object.primitives[i].type) {
        case shape_msgs::msg::SolidPrimitive::BOX:
          marker.type = visualization_msgs::msg::Marker::CUBE;
          marker.scale.x = collision_object.primitives[i].dimensions[0];
          marker.scale.y = collision_object.primitives[i].dimensions[1];
          marker.scale.z = collision_object.primitives[i].dimensions[2];
          break;
        case shape_msgs::msg::SolidPrimitive::SPHERE:
          marker.type = visualization_msgs::msg::Marker::SPHERE;
          marker.scale.x = 2.0 * collision_object.primitives[i].dimensions[0];
          marker.scale.y = 2.0 * collision_object.primitives[i].dimensions[0];
          marker.scale.z = 2.0 * collision_object.primitives[i].dimensions[0];
          break;
        case shape_msgs::msg::SolidPrimitive::CYLINDER:
          marker.type = visualization_msgs::msg::Marker::CYLINDER;
          marker.scale.x = 2.0 * collision_object.primitives[i].dimensions[0];
          marker.scale.y = 2.0 * collision_object.primitives[i].dimensions[0];
          marker.scale.z = collision_object.primitives[i].dimensions[1];
          break;
        default:
          continue;
      }
      markers.markers.emplace_back(marker);
    }
    for (auto i = 0; i < collision_object.meshes.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header = collision_object.header;
      marker.ns = collision_object.id;
      marker.id = collision_object.primitives.size() + i;
      marker.action = visualization_msgs::msg::Marker::ADD;
      Eigen::Affine3d object_base_to_object;
      tf2::fromMsg(collision_object.mesh_poses[i], object_base_to_object);
      marker.pose = tf2::toMsg(origin_to_object_base * object_base_to_object);
      marker.lifetime = lifetime;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color = colors[color_index % colors.size()];

      marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      for (const auto triangle : collision_object.meshes[i].triangles) {
        marker.points.emplace_back(collision_object.meshes[i].vertices[triangle.vertex_indices[0]]);
        marker.points.emplace_back(collision_object.meshes[i].vertices[triangle.vertex_indices[1]]);
        marker.points.emplace_back(collision_object.meshes[i].vertices[triangle.vertex_indices[2]]);
      }
      markers.markers.emplace_back(marker);
    }

    ++color_index;
  }
  return markers;
}

}  // namespace tmc_collision_environment
