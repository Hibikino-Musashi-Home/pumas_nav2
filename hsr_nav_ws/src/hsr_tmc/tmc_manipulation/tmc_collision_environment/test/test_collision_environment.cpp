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

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <tf2_ros/transform_broadcaster.h>

#include <tmc_utils/caching_subscriber.hpp>
#include <tmc_utils/qos.hpp>

#include "../src/collision_environment_server.hpp"
#include "utils.hpp"

namespace {
constexpr double kEpsilon = 1.0e-3;

moveit_msgs::msg::CollisionObject ExtractObject(
    const std::vector<moveit_msgs::msg::CollisionObject>& objects, const std::string& target_id) {
  for (const auto& object : objects) {
    if (object.id == target_id) {
      return object;
    }
  }
  return moveit_msgs::msg::CollisionObject();
}

visualization_msgs::msg::Marker ExtractMarker(
    const std::vector<visualization_msgs::msg::Marker>& markers,
    const std::string& target_name, int32_t target_type) {
  for (const auto& marker : markers) {
    if ((marker.ns == target_name) && (marker.type == target_type)) {
      return marker;
    }
  }
  return visualization_msgs::msg::Marker();
}

bool IsSameColor(const std_msgs::msg::ColorRGBA& color_1, const std_msgs::msg::ColorRGBA& color_2) {
  return (std::abs(color_1.r - color_2.r) < kEpsilon) && (std::abs(color_1.g - color_2.g) < kEpsilon) &&
         (std::abs(color_1.b - color_2.b) < kEpsilon) && (std::abs(color_1.a - color_2.a) < kEpsilon);
}

bool IsObjectNumEqual(
    const tmc_utils::CachingSubscriber<moveit_msgs::msg::PlanningSceneWorld>::Ptr& cache,
    uint32_t object_num) {
  return cache->IsSubscribed() && (cache->GetValue().collision_objects.size() == object_num);
}

bool IsRefFrameIdEqual(
    const tmc_utils::CachingSubscriber<moveit_msgs::msg::PlanningSceneWorld>::Ptr& cache,
    const std::string& ref_frame_id) {
  if (!cache->IsSubscribed()) {
    return false;
  }
  for (const auto& object : cache->GetValue().collision_objects) {
    if (object.header.frame_id == ref_frame_id) {
      return true;
    }
  }
  return false;
}

bool IsMarkerNumEqual(
    const tmc_utils::CachingSubscriber<visualization_msgs::msg::MarkerArray>::Ptr& cache,
    uint32_t object_num) {
  return cache->IsSubscribed() && (cache->GetValue().markers.size() == object_num);
}

}  // namespace

namespace tmc_collision_environment {

class CollisionEnvironmentServerTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void SpinSome() {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = client_node_->now();
    msg.child_frame_id = "odom";
    msg.transform.translation.x = -1.0;
    tf_broadcaster_->sendTransform(msg);

    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(server_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  bool WaitWithPublishingObject(const moveit_msgs::msg::CollisionObject& object,
                                std::function<bool()> waiting_func,
                                double timeout = 0.5) {
    const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(timeout);
    while (!waiting_func()) {
       object_pub_->publish(object);
       SpinSome();
       if (std::chrono::system_clock::now() > end_time) {
          return false;
       }
    }
    return true;
  }

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr object_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<CollisionEnvironmentServer> server_node_;
};

void CollisionEnvironmentServerTest::SetUp() {
  client_node_ = rclcpp::Node::make_shared("client");
  object_pub_ = client_node_->create_publisher<moveit_msgs::msg::CollisionObject>(
      "collision_environment_server/collision_object", 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*client_node_);

  server_node_ = std::make_shared<CollisionEnvironmentServer>();
}

TEST_F(CollisionEnvironmentServerTest, AddObject) {
  const auto environment_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::PlanningSceneWorld>>(
      client_node_, "collision_environment_server/environment");

  const auto input_object = CreateTestObject("test_object_1");
  ASSERT_TRUE(WaitWithPublishingObject(input_object, std::bind(IsObjectNumEqual, environment_cache, 1)));

  ASSERT_EQ(environment_cache->GetValue().collision_objects.size(), 1);
  const auto output_object = environment_cache->GetValue().collision_objects[0];

  IsEqual(input_object, output_object);
}

TEST_F(CollisionEnvironmentServerTest, AddTwoObject) {
  const auto environment_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::PlanningSceneWorld>>(
      client_node_, "collision_environment_server/environment");

  const auto input_object_1 = CreateTestObject("test_object_1");
  ASSERT_TRUE(WaitWithPublishingObject(input_object_1, std::bind(IsObjectNumEqual, environment_cache, 1)));

  const auto input_object_2 = CreateTestObject("test_object_2");
  ASSERT_TRUE(WaitWithPublishingObject(input_object_2, std::bind(IsObjectNumEqual, environment_cache, 2)));

  ASSERT_EQ(environment_cache->GetValue().collision_objects.size(), 2);

  const auto output_object_1 = ExtractObject(environment_cache->GetValue().collision_objects, "test_object_1");
  const auto output_object_2 = ExtractObject(environment_cache->GetValue().collision_objects, "test_object_2");

  IsEqual(input_object_1, output_object_1);
  IsEqual(input_object_2, output_object_2);
}

TEST_F(CollisionEnvironmentServerTest, AddSameIdObject) {
  const auto environment_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::PlanningSceneWorld>>(
      client_node_, "collision_environment_server/environment");

  const auto input_object_1 = CreateTestObject("test_object_1");
  ASSERT_TRUE(WaitWithPublishingObject(input_object_1, std::bind(IsObjectNumEqual, environment_cache, 1)));

  auto input_object_2 = CreateTestObject("test_object_1");
  input_object_2.header.frame_id = "map";
  ASSERT_TRUE(WaitWithPublishingObject(input_object_2, std::bind(IsRefFrameIdEqual, environment_cache, "map")));

  ASSERT_EQ(environment_cache->GetValue().collision_objects.size(), 1);
  const auto output_object = environment_cache->GetValue().collision_objects[0];

  IsEqual(input_object_2, output_object);
}

TEST_F(CollisionEnvironmentServerTest, RemoveObject) {
  const auto environment_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::PlanningSceneWorld>>(
      client_node_, "collision_environment_server/environment");

  const auto input_object_1 = CreateTestObject("test_object_1");
  ASSERT_TRUE(WaitWithPublishingObject(input_object_1, std::bind(IsObjectNumEqual, environment_cache, 1)));

  const auto input_object_2 = CreateTestObject("test_object_2");
  ASSERT_TRUE(WaitWithPublishingObject(input_object_2, std::bind(IsObjectNumEqual, environment_cache, 2)));

  moveit_msgs::msg::CollisionObject object;
  object.id = "test_object_1";
  object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  ASSERT_TRUE(WaitWithPublishingObject(object, std::bind(IsObjectNumEqual, environment_cache, 1)));

  ASSERT_EQ(environment_cache->GetValue().collision_objects.size(), 1);
  const auto output_object = environment_cache->GetValue().collision_objects[0];

  IsEqual(input_object_2, output_object);
}

TEST_F(CollisionEnvironmentServerTest, TransformedEnvironment) {
  const auto environment_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::PlanningSceneWorld>>(
      client_node_, "collision_environment_server/transformed_environment");
  server_node_->set_parameter(rclcpp::Parameter("origin_frame_id", "odom"));

  auto input_object_1 = CreateTestObject("test_object_1");
  ASSERT_TRUE(WaitWithPublishingObject(input_object_1, std::bind(IsObjectNumEqual, environment_cache, 1)));

  auto input_object_2 = CreateTestObject("test_object_2");
  input_object_2.header.frame_id = "map";
  ASSERT_TRUE(WaitWithPublishingObject(input_object_2, std::bind(IsObjectNumEqual, environment_cache, 2)));

  // When Origin_frame_id is empty, the topic is not issued
  server_node_->set_parameter(rclcpp::Parameter("origin_frame_id", ""));
  EXPECT_FALSE(WaitWithPublishingObject(moveit_msgs::msg::CollisionObject(),
                                        std::bind(IsObjectNumEqual, environment_cache, 0)));

  // Not included Object not included
  server_node_->set_parameter(rclcpp::Parameter("origin_frame_id", "odom"));
  input_object_1.header.frame_id = "invalid";
  ASSERT_TRUE(WaitWithPublishingObject(input_object_1, std::bind(IsObjectNumEqual, environment_cache, 1), 2.0));

  ASSERT_EQ(environment_cache->GetValue().collision_objects.size(), 1);
  const auto output_object = environment_cache->GetValue().collision_objects[0];

  input_object_2.header.frame_id = "odom";
  input_object_2.pose.position.x = 2.0;
  IsEqual(input_object_2, output_object);
}

TEST_F(CollisionEnvironmentServerTest, MarkerArray) {
  const auto markaer_cache = std::make_shared<tmc_utils::CachingSubscriber<visualization_msgs::msg::MarkerArray>>(
      client_node_, "collision_environment_server/marker_array", tmc_utils::BestEffortQoS());

  auto input_object_1 = CreateTestObject("test_object_1");
  ASSERT_TRUE(WaitWithPublishingObject(input_object_1, std::bind(IsMarkerNumEqual, markaer_cache, 1)));

  moveit_msgs::msg::CollisionObject input_object_2;
  input_object_2.header.frame_id = "map";
  input_object_2.pose.position.x = 0.1;
  input_object_2.id = "test_object_2";
  input_object_2.primitives.resize(3);
  input_object_2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  input_object_2.primitives[0].dimensions = {0.2, 0.3, 0.4};
  input_object_2.primitives[1].type = shape_msgs::msg::SolidPrimitive::SPHERE;
  input_object_2.primitives[1].dimensions = {0.6};
  input_object_2.primitives[2].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  input_object_2.primitives[2].dimensions = {0.7, 0.8};
  input_object_2.primitive_poses.resize(3);
  input_object_2.primitive_poses[0].position.y = 0.1;
  input_object_2.primitive_poses[1].position.y = 0.2;
  input_object_2.primitive_poses[2].position.y = 0.3;
  input_object_2.meshes.resize(1);
  input_object_2.meshes[0].vertices.resize(4);
  input_object_2.meshes[0].vertices[0].x = 0.0;
  input_object_2.meshes[0].vertices[0].y = 0.0;
  input_object_2.meshes[0].vertices[1].x = 1.0;
  input_object_2.meshes[0].vertices[1].y = 0.0;
  input_object_2.meshes[0].vertices[2].x = 0.0;
  input_object_2.meshes[0].vertices[2].y = 1.0;
  input_object_2.meshes[0].vertices[3].x = 1.0;
  input_object_2.meshes[0].vertices[3].y = 1.0;
  input_object_2.meshes[0].triangles.resize(2);
  input_object_2.meshes[0].triangles[0].vertex_indices = {0, 1, 2};
  input_object_2.meshes[0].triangles[1].vertex_indices = {1, 2, 3};
  input_object_2.mesh_poses.resize(1);
  input_object_2.mesh_poses[0].position.y = 0.4;
  input_object_2.operation = moveit_msgs::msg::CollisionObject::ADD;

  ASSERT_TRUE(WaitWithPublishingObject(input_object_2, std::bind(IsMarkerNumEqual, markaer_cache, 5)));

  const auto marker_1 = ExtractMarker(markaer_cache->GetValue().markers,
                                      "test_object_1", visualization_msgs::msg::Marker::SPHERE);
  EXPECT_EQ(marker_1.header.frame_id, "odom");
  EXPECT_EQ(marker_1.action, visualization_msgs::msg::Marker::ADD);
  IsEqual(marker_1.pose, (Eigen::Translation3d(1.0, 2.0, 3.0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())) *
                         (Eigen::Translation3d(0.1, 0.2, 0.3) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())));
  EXPECT_NEAR(marker_1.scale.x, 8.0, kEpsilon);
  EXPECT_NEAR(marker_1.scale.y, 8.0, kEpsilon);
  EXPECT_NEAR(marker_1.scale.z, 8.0, kEpsilon);

  const auto marker_2_box = ExtractMarker(markaer_cache->GetValue().markers,
                                          "test_object_2", visualization_msgs::msg::Marker::CUBE);
  EXPECT_EQ(marker_2_box.header.frame_id, "map");
  EXPECT_EQ(marker_2_box.action, visualization_msgs::msg::Marker::ADD);
  IsEqual(marker_2_box.pose, Eigen::Translation3d(0.1, 0.1, 0.0) * Eigen::AngleAxisd::Identity());
  EXPECT_NEAR(marker_2_box.scale.x, 0.2, kEpsilon);
  EXPECT_NEAR(marker_2_box.scale.y, 0.3, kEpsilon);
  EXPECT_NEAR(marker_2_box.scale.z, 0.4, kEpsilon);

  const auto marker_2_sphere = ExtractMarker(markaer_cache->GetValue().markers,
                                             "test_object_2", visualization_msgs::msg::Marker::SPHERE);
  EXPECT_EQ(marker_2_sphere.header.frame_id, "map");
  EXPECT_EQ(marker_2_sphere.action, visualization_msgs::msg::Marker::ADD);
  IsEqual(marker_2_sphere.pose, Eigen::Translation3d(0.1, 0.2, 0.0) * Eigen::AngleAxisd::Identity());
  EXPECT_NEAR(marker_2_sphere.scale.x, 1.2, kEpsilon);
  EXPECT_NEAR(marker_2_sphere.scale.y, 1.2, kEpsilon);
  EXPECT_NEAR(marker_2_sphere.scale.z, 1.2, kEpsilon);

  const auto marker_2_cylinder = ExtractMarker(markaer_cache->GetValue().markers,
                                               "test_object_2", visualization_msgs::msg::Marker::CYLINDER);
  EXPECT_EQ(marker_2_cylinder.header.frame_id, "map");
  EXPECT_EQ(marker_2_cylinder.action, visualization_msgs::msg::Marker::ADD);
  IsEqual(marker_2_cylinder.pose, Eigen::Translation3d(0.1, 0.3, 0.0) * Eigen::AngleAxisd::Identity());
  EXPECT_NEAR(marker_2_cylinder.scale.x, 1.4, kEpsilon);
  EXPECT_NEAR(marker_2_cylinder.scale.y, 1.4, kEpsilon);
  EXPECT_NEAR(marker_2_cylinder.scale.z, 0.8, kEpsilon);

  const auto marker_2_mesh = ExtractMarker(markaer_cache->GetValue().markers,
                                           "test_object_2", visualization_msgs::msg::Marker::TRIANGLE_LIST);
  EXPECT_EQ(marker_2_mesh.header.frame_id, "map");
  EXPECT_EQ(marker_2_mesh.action, visualization_msgs::msg::Marker::ADD);
  IsEqual(marker_2_mesh.pose, Eigen::Translation3d(0.1, 0.4, 0.0) * Eigen::AngleAxisd::Identity());
  EXPECT_NEAR(marker_2_mesh.scale.x, 1.0, kEpsilon);
  EXPECT_NEAR(marker_2_mesh.scale.y, 1.0, kEpsilon);
  EXPECT_NEAR(marker_2_mesh.scale.z, 1.0, kEpsilon);
  ASSERT_EQ(marker_2_mesh.points.size(), 6);
  IsEqual(marker_2_mesh.points[0], input_object_2.meshes[0].vertices[0]);
  IsEqual(marker_2_mesh.points[1], input_object_2.meshes[0].vertices[1]);
  IsEqual(marker_2_mesh.points[2], input_object_2.meshes[0].vertices[2]);
  IsEqual(marker_2_mesh.points[3], input_object_2.meshes[0].vertices[1]);
  IsEqual(marker_2_mesh.points[4], input_object_2.meshes[0].vertices[2]);
  IsEqual(marker_2_mesh.points[5], input_object_2.meshes[0].vertices[3]);

  EXPECT_FALSE(IsSameColor(marker_1.color, marker_2_box.color));
  EXPECT_TRUE(IsSameColor(marker_2_box.color, marker_2_sphere.color));
  EXPECT_TRUE(IsSameColor(marker_2_box.color, marker_2_cylinder.color));
  EXPECT_TRUE(IsSameColor(marker_2_box.color, marker_2_mesh.color));

  EXPECT_EQ(marker_1.id, 0);
  EXPECT_EQ(marker_2_box.id, 0);
  EXPECT_EQ(marker_2_sphere.id, 1);
  EXPECT_EQ(marker_2_cylinder.id, 2);
}

}  // namespace tmc_collision_environment

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
