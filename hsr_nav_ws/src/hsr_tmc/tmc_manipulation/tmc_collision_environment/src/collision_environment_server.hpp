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
#ifndef TMC_COLLISION_ENVIRONMENT_COLLISION_ENVIRONMENT_SERVER_HPP_
#define TMC_COLLISION_ENVIRONMENT_COLLISION_ENVIRONMENT_SERVER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tmc_utils/parameters.hpp>

namespace tmc_collision_environment {

class CollisionEnvironmentServer : public rclcpp::Node {
 public:
  CollisionEnvironmentServer() : CollisionEnvironmentServer(rclcpp::NodeOptions()) {}
  explicit CollisionEnvironmentServer(const rclcpp::NodeOptions& options);

  virtual ~CollisionEnvironmentServer() = default;

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  void TimerCallback();

  std::map<std::string, moveit_msgs::msg::CollisionObject> objects_;

  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr collision_object_sub_;
  void CollisionObjectCallback(const moveit_msgs::msg::CollisionObject::SharedPtr msg);

  rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr environment_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr transformed_environment_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::vector<rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr> set_param_handlers_;
  tmc_utils::DynamicParameter<std::string>::Ptr origin_frame_id_;

  std::vector<std_msgs::msg::ColorRGBA> colors_;
  rclcpp::Duration marker_lifetime_;
};

}  // namespace tmc_collision_environment
#endif  // TMC_COLLISION_ENVIRONMENT_COLLISION_ENVIRONMENT_SERVER_HPP_
