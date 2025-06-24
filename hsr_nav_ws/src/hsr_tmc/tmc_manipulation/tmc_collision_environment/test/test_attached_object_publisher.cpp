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

#include <tf2_ros/transform_broadcaster.h>

#include <tmc_utils/caching_subscriber.hpp>
#include <tmc_utils/qos.hpp>

#include "../src/attached_object_publisher.hpp"
#include "utils.hpp"

namespace {
const char* const kGraspingFrame = "hand_link";
const char* const kReleasingFrame = "map";
constexpr double kEpsilon = 1.0e-3;

bool IsObjectNumEqual(
    const tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>::Ptr& cache,
    uint32_t object_num) {
  return cache->IsSubscribed() && (cache->GetValue().attached_collision_objects.size() == object_num);
}

bool IsObjectNameEqual(
    const tmc_utils::CachingSubscriber<moveit_msgs::msg::CollisionObject>::Ptr& cache,
    const std::string& object_name) {
  return cache->IsSubscribed() && (cache->GetValue().id == object_name);
}

bool IsPositionZMoved(
    const tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>::Ptr& cache,
    double current_position_z) {
  return (cache->IsSubscribed() && !cache->GetValue().attached_collision_objects.empty() &&
          cache->GetValue().attached_collision_objects[0].object.pose.position.z != current_position_z);
}

bool IsMarkerNumEqual(
    const tmc_utils::CachingSubscriber<visualization_msgs::msg::MarkerArray>::Ptr& cache,
    uint32_t object_num) {
  return cache->IsSubscribed() && (cache->GetValue().markers.size() == object_num);
}
}  // namespace

namespace tmc_collision_environment {

class AttachedObjectPublisherTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void SpinSome() {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = client_node_->now();
    msg.child_frame_id = kGraspingFrame;
    msg.transform.translation.x = -1.0;
    tf_broadcaster_->sendTransform(msg);

    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(server_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  bool WaitFor(std::function<bool()> waiting_func, double timeout = 0.5) {
    const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(timeout);
    while (!waiting_func()) {
       SpinSome();
       if (std::chrono::system_clock::now() > end_time) {
          return false;
       }
    }
    return true;
  }

  void AddObjectToEnvironment(const moveit_msgs::msg::CollisionObject& object) {
    moveit_msgs::msg::PlanningSceneWorld env;
    env.collision_objects.push_back(object);
    environment_pub_->publish(env);
    SpinSome();
  }

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grasping_name_pub_;
  rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr grasping_info_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr releasing_name_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr environment_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<AttachedObjectServer> server_node_;
};

void AttachedObjectPublisherTest::SetUp() {
  client_node_ = rclcpp::Node::make_shared("client");

  grasping_name_pub_ = client_node_->create_publisher<std_msgs::msg::String>(
      "attached_object_publisher/attaching_object_name", 1);
  grasping_info_pub_ = client_node_->create_publisher<moveit_msgs::msg::AttachedCollisionObject>(
      "attached_object_publisher/attaching_object_info", 1);
  releasing_name_pub_ = client_node_->create_publisher<std_msgs::msg::String>(
      "attached_object_publisher/releasing_object_name", 1);
  environment_pub_ = client_node_->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
      "collision_environment_server/environment", 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*client_node_);

  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("grasping_frame_id", kGraspingFrame),
                                   rclcpp::Parameter("releasing_frame_id", kReleasingFrame)};
  server_node_ = std::make_shared<AttachedObjectServer>(options);
}

// Objects that exist in the environment are deleted from the environment by specifying the name and are added to the identified body.
TEST_F(AttachedObjectPublisherTest, AttachExistingObjectByName) {
  const auto collision_object = CreateTestObject("test_object_1");
  AddObjectToEnvironment(collision_object);

  const auto removing_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::CollisionObject>>(
      client_node_, "collision_environment_server/collision_object");

  std_msgs::msg::String msg;
  msg.data = "test_object_1";
  grasping_name_pub_->publish(msg);

  ASSERT_TRUE(WaitFor(std::bind(IsObjectNameEqual, removing_object_cache, "test_object_1")));
  EXPECT_EQ(removing_object_cache->GetValue().operation, moveit_msgs::msg::CollisionObject::REMOVE);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  ASSERT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));

  // ODOM_TO_HAND shifts only
  auto expected_attached_object = collision_object;
  expected_attached_object.header.frame_id = kGraspingFrame;
  expected_attached_object.pose.position.x += 1.0;

  const auto attached_object = atacched_object_cache->GetValue().attached_collision_objects[0];
  EXPECT_EQ(attached_object.link_name, kGraspingFrame);
  IsEqual(attached_object.object, expected_attached_object);
}

// Objects that do not exist in the environment by specifying the name-> Nothing happens
TEST_F(AttachedObjectPublisherTest, AttachNotExistingObjectByName) {
  std_msgs::msg::String msg;
  msg.data = "test_object_1";
  grasping_name_pub_->publish(msg);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  EXPECT_FALSE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));
}

// Objects registered as an amiable material by name specification-> Nothing happens
TEST_F(AttachedObjectPublisherTest, AttachRegisteredObjectByName) {
  const auto collision_object = CreateTestObject("test_object_1");
  AddObjectToEnvironment(collision_object);

  std_msgs::msg::String msg;
  msg.data = "test_object_1";
  grasping_name_pub_->publish(msg);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  ASSERT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));

  grasping_name_pub_->publish(msg);
  EXPECT_FALSE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 2)));
}

// Hand_to_object cannot be obtained objects by specifying the name-> Nothing happens
TEST_F(AttachedObjectPublisherTest, AttachUnknownTransformationObjectByName) {
  auto collision_object = CreateTestObject("test_object_1");
  collision_object.header.frame_id = "map";
  AddObjectToEnvironment(collision_object);

  std_msgs::msg::String msg;
  msg.data = "test_object_1";
  grasping_name_pub_->publish(msg);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  EXPECT_FALSE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));
}

// Objects that exist in the environment are deleted from the environment in the grontment information-> Added to the identified body.
TEST_F(AttachedObjectPublisherTest, AttachExistingObjectByInfo) {
  const auto collision_object = CreateTestObject("test_object_1");
  AddObjectToEnvironment(collision_object);

  const auto removing_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::CollisionObject>>(
      client_node_, "collision_environment_server/collision_object");

  moveit_msgs::msg::AttachedCollisionObject object_info;
  object_info.link_name = kGraspingFrame;
  object_info.object = collision_object;
  // Change the values ​​to the objects that exist in the environment
  object_info.object.pose.position.x += 1.0;
  grasping_info_pub_->publish(object_info);

  ASSERT_TRUE(WaitFor(std::bind(IsObjectNameEqual, removing_object_cache, "test_object_1")));
  EXPECT_EQ(removing_object_cache->GetValue().operation, moveit_msgs::msg::CollisionObject::REMOVE);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  ASSERT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));

  // ODOM_TO_HAND shifts only
  auto expected_attached_object = object_info;
  expected_attached_object.object.header.frame_id = kGraspingFrame;
  expected_attached_object.object.pose.position.x += 1.0;

  const auto attached_object = atacched_object_cache->GetValue().attached_collision_objects[0];
  EXPECT_EQ(attached_object.link_name, kGraspingFrame);
  IsEqual(attached_object.object, expected_attached_object.object);
}

// Objects that do not exist in the environment are granted with gratified information-> Added to the identified body
TEST_F(AttachedObjectPublisherTest, AttachNotExistingObjectByInfo) {
  moveit_msgs::msg::AttachedCollisionObject object_info;
  object_info.link_name = kGraspingFrame;
  object_info.object = CreateTestObject("test_object_1");
  grasping_info_pub_->publish(object_info);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  ASSERT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));

  // ODOM_TO_HAND shifts only
  auto expected_attached_object = object_info;
  expected_attached_object.object.header.frame_id = kGraspingFrame;
  expected_attached_object.object.pose.position.x += 1.0;

  const auto attached_object = atacched_object_cache->GetValue().attached_collision_objects[0];
  EXPECT_EQ(attached_object.link_name, kGraspingFrame);
  IsEqual(attached_object.object, expected_attached_object.object);
}

// Objects registered as a personal knowledge are updated by grant information-> Informative information information.
TEST_F(AttachedObjectPublisherTest, AttachRegisteredObjectByInfo) {
  moveit_msgs::msg::AttachedCollisionObject object_info;
  object_info.link_name = kGraspingFrame;
  object_info.object = CreateTestObject("test_object_1");
  grasping_info_pub_->publish(object_info);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  ASSERT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));

  const auto prev_position_z = atacched_object_cache->GetValue().attached_collision_objects[0].object.pose.position.z;

  object_info.object.pose.position.z += 1.0;
  grasping_info_pub_->publish(object_info);
  ASSERT_TRUE(WaitFor(std::bind(IsPositionZMoved, atacched_object_cache, prev_position_z)));
}

// Hand_to_object cannot be obtained by grant information with the knowledge information-> Nothing happens
TEST_F(AttachedObjectPublisherTest, AttachUnknownTransformationObjectByInfo) {
  moveit_msgs::msg::AttachedCollisionObject object_info;
  object_info.link_name = "invalid_frame";
  object_info.object = CreateTestObject("test_object_1");
  object_info.object.header.frame_id = "map";
  grasping_info_pub_->publish(object_info);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  EXPECT_FALSE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));
}

// Link_name is incorrect with illegal ingredients-> Nothing happens
TEST_F(AttachedObjectPublisherTest, AttachInvalidLinkNameObject) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "odom";
  tf.header.stamp = client_node_->now();
  tf.child_frame_id = "invalid_frame";
  tf_broadcaster_->sendTransform(tf);

  moveit_msgs::msg::AttachedCollisionObject object_info;
  object_info.link_name = "invalid_frame";
  object_info.object = CreateTestObject("test_object_1");
  grasping_info_pub_->publish(object_info);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  EXPECT_FALSE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));
}

// Clear the control
TEST_F(AttachedObjectPublisherTest, DetachAll) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = kGraspingFrame;
  tf.header.stamp = client_node_->now();
  tf.child_frame_id = kReleasingFrame;
  tf.transform.translation.y = 1.0;
  tf_broadcaster_->sendTransform(tf);

  moveit_msgs::msg::AttachedCollisionObject object_info;
  object_info.link_name = kGraspingFrame;
  object_info.object = CreateTestObject("test_object_1");
  grasping_info_pub_->publish(object_info);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  ASSERT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));

  const auto adding_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::CollisionObject>>(
      client_node_, "collision_environment_server/collision_object");

  object_info.object.id.clear();
  object_info.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  grasping_info_pub_->publish(object_info);
  ASSERT_TRUE(WaitFor(std::bind(IsObjectNameEqual, adding_object_cache, "test_object_1")));

  auto expected_collision_object = CreateTestObject("test_object_1");
  // ODOM_TO_HAND and hand_to_map are out of course
  expected_collision_object.header.frame_id = kReleasingFrame;
  expected_collision_object.pose.position.x += 1.0;
  expected_collision_object.pose.position.y -= 1.0;
  IsEqual(adding_object_cache->GetValue(), expected_collision_object);

  EXPECT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 0)));
}

// The identified object is deleted from the identification body and added to the environment.
TEST_F(AttachedObjectPublisherTest, DetachRegisteredObject) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = kGraspingFrame;
  tf.header.stamp = client_node_->now();
  tf.child_frame_id = kReleasingFrame;
  tf.transform.translation.y = 1.0;
  tf_broadcaster_->sendTransform(tf);

  const auto collision_object = CreateTestObject("test_object_1");
  AddObjectToEnvironment(collision_object);

  std_msgs::msg::String msg;
  msg.data = "test_object_1";
  grasping_name_pub_->publish(msg);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  ASSERT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));

  const auto adding_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::CollisionObject>>(
      client_node_, "collision_environment_server/collision_object");

  releasing_name_pub_->publish(msg);
  ASSERT_TRUE(WaitFor(std::bind(IsObjectNameEqual, adding_object_cache, "test_object_1")));

  auto expected_collision_object = collision_object;
  // ODOM_TO_HAND and hand_to_map are out of course
  expected_collision_object.header.frame_id = kReleasingFrame;
  expected_collision_object.pose.position.x += 1.0;
  expected_collision_object.pose.position.y -= 1.0;
  IsEqual(adding_object_cache->GetValue(), expected_collision_object);

  EXPECT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 0)));
}

// Unknown objects, unlocking-> Nothing happens
TEST_F(AttachedObjectPublisherTest, DetachNotRegisteredObject) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = kGraspingFrame;
  tf.header.stamp = client_node_->now();
  tf.child_frame_id = kReleasingFrame;
  tf.transform.translation.y = 1.0;
  tf_broadcaster_->sendTransform(tf);

  const auto collision_object = CreateTestObject("test_object_1");
  AddObjectToEnvironment(collision_object);

  std_msgs::msg::String msg;
  msg.data = "test_object_1";
  grasping_name_pub_->publish(msg);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  ASSERT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));

  const auto adding_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::CollisionObject>>(
      client_node_, "collision_environment_server/collision_object");

  msg.data = "test_object_2";
  releasing_name_pub_->publish(msg);
  EXPECT_FALSE(WaitFor(std::bind(IsObjectNameEqual, adding_object_cache, "test_object_1")));
}

// The objects that have been understood are canceled, but the position cannot be resolved-> It is deleted from the gratient body, but it is not added to the environment.
TEST_F(AttachedObjectPublisherTest, DetachWithTfError) {
  const auto collision_object = CreateTestObject("test_object_1");
  AddObjectToEnvironment(collision_object);

  std_msgs::msg::String msg;
  msg.data = "test_object_1";
  grasping_name_pub_->publish(msg);

  const auto atacched_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::RobotState>>(
      client_node_, "attached_object_publisher/attached_object");
  ASSERT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 1)));

  const auto adding_object_cache = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::CollisionObject>>(
      client_node_, "collision_environment_server/collision_object");

  releasing_name_pub_->publish(msg);
  ASSERT_FALSE(WaitFor(std::bind(IsObjectNameEqual, adding_object_cache, "test_object_1")));

  EXPECT_TRUE(WaitFor(std::bind(IsObjectNumEqual, atacched_object_cache, 0)));
}

// A poster is issued by MarkerArray
TEST_F(AttachedObjectPublisherTest, MarkerArray) {
  const auto collision_object = CreateTestObject("test_object_1");
  AddObjectToEnvironment(collision_object);

  std_msgs::msg::String msg;
  msg.data = "test_object_1";
  grasping_name_pub_->publish(msg);

  const auto markaer_cache = std::make_shared<tmc_utils::CachingSubscriber<visualization_msgs::msg::MarkerArray>>(
      client_node_, "attached_object_publisher/marker_array", tmc_utils::BestEffortQoS());
  ASSERT_TRUE(WaitFor(std::bind(IsMarkerNumEqual, markaer_cache, 1)));

  const auto marker = markaer_cache->GetValue().markers[0];
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::SPHERE);
  EXPECT_EQ(marker.header.frame_id, kGraspingFrame);
  EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
  IsEqual(marker.pose, (Eigen::Translation3d(2.0, 2.0, 3.0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())) *
                       (Eigen::Translation3d(0.1, 0.2, 0.3) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())));
  EXPECT_NEAR(marker.scale.x, 8.0, kEpsilon);
  EXPECT_NEAR(marker.scale.y, 8.0, kEpsilon);
  EXPECT_NEAR(marker.scale.z, 8.0, kEpsilon);
}

}  // namespace tmc_collision_environment

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
