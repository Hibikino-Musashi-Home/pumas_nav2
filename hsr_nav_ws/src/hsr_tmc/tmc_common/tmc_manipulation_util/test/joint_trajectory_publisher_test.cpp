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

#include <rclcpp/rclcpp.hpp>
#include <tmc_manipulation_util/joint_trajectory_publisher.hpp>
#include <tmc_utils/caching_subscriber.hpp>

namespace {
const char* const kControllerName = "joint_controller";

trajectory_msgs::msg::JointTrajectory MakeTrajectory() {
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nanosec = 2;
  msg.joint_names = {"shoulder", "wrist"};
  msg.points.resize(1);
  msg.points[0].positions = {0.1, 0.2};
  msg.points[0].velocities = {0.3, 0.4};
  msg.points[0].accelerations = {0.5, 0.6};
  msg.points[0].time_from_start.sec = 3;
  msg.points[0].time_from_start.nanosec = 4;
  return msg;
}

sensor_msgs::msg::JointState MakeJointState() {
  sensor_msgs::msg::JointState msg;
  msg.name = {"elbow"};
  msg.position = {42.0};
  return msg;
}
}  // namespace

namespace tmc_manipulation_util {

class JointTrajectoryPublisherTest : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Node::SharedPtr subscriber_node_;

  tmc_utils::CachingSubscriber<trajectory_msgs::msg::JointTrajectory>::Ptr trajectory_cache_;

  void WaitForSubscribe(const JointTrajectoryPublisher::Ptr& publisher,
                        const trajectory_msgs::msg::JointTrajectory& trajectory,
                        const sensor_msgs::msg::JointState& current_state);
  void WaitForSubscribe(const JointTrajectoryPublisher::Ptr& publisher);
};

void JointTrajectoryPublisherTest::SetUp() {
  publisher_node_ = rclcpp::Node::make_shared("publisher");

  subscriber_node_ = rclcpp::Node::make_shared(kControllerName);
  trajectory_cache_ = std::make_shared<tmc_utils::CachingSubscriber<trajectory_msgs::msg::JointTrajectory>>(
      subscriber_node_, "~/joint_trajectory");
}

void JointTrajectoryPublisherTest::WaitForSubscribe(const JointTrajectoryPublisher::Ptr& publisher,
                                                    const trajectory_msgs::msg::JointTrajectory& trajectory,
                                                    const sensor_msgs::msg::JointState& current_state) {
  const auto timeout = publisher_node_->now() + rclcpp::Duration(1, 0);
  trajectory_cache_ = std::make_shared<tmc_utils::CachingSubscriber<trajectory_msgs::msg::JointTrajectory>>(
      subscriber_node_, "~/joint_trajectory");
  while (!trajectory_cache_->IsSubscribed()) {
    ASSERT_TRUE(publisher->Publish(trajectory, current_state));
    rclcpp::spin_some(publisher_node_);
    rclcpp::spin_some(subscriber_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (publisher_node_->now() > timeout) {
      FAIL();
      return;
    }
  }
}

void JointTrajectoryPublisherTest::WaitForSubscribe(const JointTrajectoryPublisher::Ptr& publisher) {
  return WaitForSubscribe(publisher, MakeTrajectory(), MakeJointState());
}

TEST_F(JointTrajectoryPublisherTest, PublishSimple) {
  const std::vector<std::string> joint_names = {"shoulder", "wrist"};
  publisher_node_->declare_parameter(std::string(kControllerName) + ".joints", joint_names);
  const auto publisher = std::make_shared<JointTrajectoryPublisher>(publisher_node_, kControllerName);
  EXPECT_EQ(publisher->joint_names(), joint_names);

  WaitForSubscribe(publisher);

  auto trajectory = trajectory_cache_->GetValue();
  EXPECT_EQ(trajectory.header.stamp.sec, 1);
  EXPECT_EQ(trajectory.header.stamp.nanosec, 2u);
  EXPECT_EQ(trajectory.joint_names, joint_names);
  ASSERT_EQ(trajectory.points.size(), 1u);
  EXPECT_EQ(trajectory.points[0].positions, std::vector<double>({0.1, 0.2}));
  EXPECT_EQ(trajectory.points[0].velocities, std::vector<double>({0.3, 0.4}));
  EXPECT_EQ(trajectory.points[0].accelerations, std::vector<double>({0.5, 0.6}));
  EXPECT_EQ(trajectory.points[0].time_from_start.sec, 3);
  EXPECT_EQ(trajectory.points[0].time_from_start.nanosec, 4u);
}

TEST_F(JointTrajectoryPublisherTest, PublishExtracted) {
  const std::vector<std::string> joint_names = {"wrist"};
  publisher_node_->declare_parameter(std::string(kControllerName) + ".joints", joint_names);
  const auto publisher = std::make_shared<JointTrajectoryPublisher>(publisher_node_, kControllerName);
  EXPECT_EQ(publisher->joint_names(), joint_names);

  WaitForSubscribe(publisher);

  auto trajectory = trajectory_cache_->GetValue();
  // After that, skip the header and time_from_start.
  EXPECT_EQ(trajectory.joint_names, joint_names);
  ASSERT_EQ(trajectory.points.size(), 1u);
  EXPECT_EQ(trajectory.points[0].positions, std::vector<double>({0.2}));
  EXPECT_EQ(trajectory.points[0].velocities, std::vector<double>({0.4}));
  EXPECT_EQ(trajectory.points[0].accelerations, std::vector<double>({0.6}));
}

TEST_F(JointTrajectoryPublisherTest, PublishFilled) {
  const std::vector<std::string> joint_names = {"shoulder", "elbow", "wrist"};
  publisher_node_->declare_parameter(std::string(kControllerName) + ".joints", joint_names);
  const auto publisher = std::make_shared<JointTrajectoryPublisher>(publisher_node_, kControllerName);
  EXPECT_EQ(publisher->joint_names(), joint_names);

  WaitForSubscribe(publisher);

  auto trajectory = trajectory_cache_->GetValue();
  EXPECT_EQ(trajectory.joint_names, joint_names);
  ASSERT_EQ(trajectory.points.size(), 1u);
  EXPECT_EQ(trajectory.points[0].positions, std::vector<double>({0.1, 42.0, 0.2}));
  EXPECT_EQ(trajectory.points[0].velocities, std::vector<double>({0.3, 0.0, 0.4}));
  EXPECT_EQ(trajectory.points[0].accelerations, std::vector<double>({0.5, 0.0, 0.6}));
}

TEST_F(JointTrajectoryPublisherTest, PublishFailure) {
  const std::vector<std::string> joint_names = {"invalid"};
  publisher_node_->declare_parameter(std::string(kControllerName) + ".joints", joint_names);
  const auto publisher = std::make_shared<JointTrajectoryPublisher>(publisher_node_, kControllerName);

  EXPECT_FALSE(publisher->Publish(MakeTrajectory(), MakeJointState()));
}

TEST_F(JointTrajectoryPublisherTest, ExplicitJoints) {
  const std::vector<std::string> joint_names = {"shoulder", "wrist"};
  const std::vector<std::string> continuous_joints = {"shoulder"};
  const auto publisher = std::make_shared<JointTrajectoryPublisher>(
      publisher_node_.get(), kControllerName, joint_names, continuous_joints);
  EXPECT_EQ(publisher->joint_names(), joint_names);
  EXPECT_EQ(publisher->continuous_joints(), continuous_joints);
}

TEST_F(JointTrajectoryPublisherTest, UseControllerNodeJoints) {
  const std::vector<std::string> joint_names = {"shoulder", "wrist"};
  subscriber_node_->declare_parameter("joints", joint_names);

  auto spin_func = [this]() {
    // There is no particular basis for the number of loops
    for (auto i = 0; i < 20; ++i) {
      rclcpp::spin_some(subscriber_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  };
  auto spin_thread = std::thread(spin_func);
  const auto publisher = std::make_shared<JointTrajectoryPublisher>(publisher_node_, kControllerName);
  spin_thread.join();

  EXPECT_EQ(publisher->joint_names(), joint_names);
}

TEST_F(JointTrajectoryPublisherTest, UseParameterJoints) {
  const std::vector<std::string> publisher_joint_names = {"shoulder"};
  publisher_node_->declare_parameter(std::string(kControllerName) + ".joints", publisher_joint_names);

  const std::vector<std::string> subscriber_joint_names = {"shoulder", "wrist"};
  subscriber_node_->declare_parameter("joints", subscriber_joint_names);

  auto spin_func = [this]() {
    // There is no particular basis for the number of loops
    for (auto i = 0; i < 20; ++i) {
      rclcpp::spin_some(subscriber_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  };
  auto spin_thread = std::thread(spin_func);
  const auto publisher = std::make_shared<JointTrajectoryPublisher>(publisher_node_, kControllerName);
  spin_thread.join();

  EXPECT_EQ(publisher->joint_names(), publisher_joint_names);
}

TEST_F(JointTrajectoryPublisherTest, ContinuousJoints) {
  const std::vector<std::string> publisher_joint_names = {"continuous_joint"};
  publisher_node_->declare_parameter(std::string(kControllerName) + ".joints", publisher_joint_names);
  publisher_node_->declare_parameter(std::string(kControllerName) + ".continuous_joints", publisher_joint_names);
  const auto publisher = std::make_shared<JointTrajectoryPublisher>(publisher_node_, kControllerName);

  EXPECT_EQ(publisher->continuous_joints(), publisher_joint_names);

  // It is troublesome to make a parameterized test only here, so turn it with a for loop.
  // Orbit position, current location, expected value
  std::vector<std::array<double, 3>> test_inputs = {{0.0, 0.0, 0.0},
                                                    {3.0, 0.0, 3.0},
                                                    {4.0, 0.0, 4.0 - 2.0 * M_PI},
                                                    {12.0, 0.0, 12.0 - 4.0 * M_PI},
                                                    {-3.0, 0.0, -3.0},
                                                    {-4.0, 0.0, -4.0 + 2.0 * M_PI},
                                                    {-12.0, 0.0, -12.0 + 4.0 * M_PI},
                                                    {0.0, 3.0, 0.0},
                                                    {0.0, 4.0, 2.0 * M_PI},
                                                    {0.0, 12.0, 4.0 * M_PI},
                                                    {0.0, -3.0, 0.0},
                                                    {0.0, -4.0, -2.0 * M_PI},
                                                    {0.0, -12.0, -4.0 * M_PI}};
  for (const auto& test_input : test_inputs) {
    trajectory_msgs::msg::JointTrajectory trajectory_in;
    trajectory_in.joint_names = {"continuous_joint"};
    trajectory_in.points.resize(2);
    trajectory_in.points[0].positions = {test_input[0]};
    trajectory_in.points[1].positions = {test_input[0] + 1.0};

    sensor_msgs::msg::JointState state;
    state.name = {"continuous_joint"};
    state.position = {test_input[1]};

    WaitForSubscribe(publisher, trajectory_in, state);

    const auto trajectory_out = trajectory_cache_->GetValue();
    EXPECT_EQ(trajectory_out.joint_names, std::vector<std::string>({"continuous_joint"}));
    ASSERT_EQ(trajectory_out.points.size(), 2u);
    ASSERT_EQ(trajectory_out.points[0].positions.size(), 1u);
    EXPECT_DOUBLE_EQ(trajectory_out.points[0].positions[0], test_input[2]);
    ASSERT_EQ(trajectory_out.points[1].positions.size(), 1u);
    EXPECT_DOUBLE_EQ(trajectory_out.points[1].positions[0], test_input[2] + 1.0);
  }
}

}  // namespace tmc_manipulation_util

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
