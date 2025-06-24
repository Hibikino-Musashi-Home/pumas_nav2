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

#include <std_msgs/msg/string.hpp>

#include <tmc_utils/caching_subscriber.hpp>
#include <tmc_utils/qos.hpp>

namespace tmc_utils {

TEST(CachingSubscriberTest, Normal) {
  auto sub_node = rclcpp::Node::make_shared("sub");
  auto cache = CachingSubscriber<std_msgs::msg::String>(sub_node, "test_topic");
  EXPECT_FALSE(cache.IsSubscribed());

  auto pub_node = rclcpp::Node::make_shared("pub");
  auto test_pub = pub_node->create_publisher<std_msgs::msg::String>("test_topic", 1);

  std_msgs::msg::String test_msg;
  test_msg.data = "test";

  const auto timeout = pub_node->now() + rclcpp::Duration::from_seconds(1.0);
  while (!cache.IsSubscribed() && pub_node->now() < timeout) {
    test_pub->publish(test_msg);

    rclcpp::spin_some(sub_node);
    rclcpp::spin_some(pub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  ASSERT_TRUE(cache.IsSubscribed());
  EXPECT_EQ(cache.GetValue().data, "test");
}

TEST(CachingSubscriberTest, QoS) {
  auto pub_node = rclcpp::Node::make_shared("pub");
  auto pub_transient_local = pub_node->create_publisher<std_msgs::msg::String>(
      "test_topic", ReliableTransientLocalQoS());

  std_msgs::msg::String test_msg;
  test_msg.data = "first";
  pub_transient_local->publish(test_msg);

  auto sub_node = rclcpp::Node::make_shared("sub");

  auto cache_reliable = CachingSubscriber<std_msgs::msg::String>(sub_node, "test_topic", ReliableVolatileQoS());
  EXPECT_FALSE(cache_reliable.IsSubscribed());

  auto cache_best_effort = CachingSubscriber<std_msgs::msg::String>(sub_node, "test_topic", BestEffortQoS());
  EXPECT_FALSE(cache_best_effort.IsSubscribed());

  auto cache_transient_local = CachingSubscriber<std_msgs::msg::String>(
      sub_node, "test_topic", ReliableTransientLocalQoS());
  EXPECT_FALSE(cache_transient_local.IsSubscribed());

  // Because it is TransientLocal, it should be sent without Publishing.
  const auto timeout = pub_node->now() + rclcpp::Duration::from_seconds(1.0);
  while (!cache_transient_local.IsSubscribed() && pub_node->now() < timeout) {
    rclcpp::spin_some(sub_node);
    rclcpp::spin_some(pub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(cache_transient_local.IsSubscribed());
  EXPECT_EQ(cache_transient_local.GetValue().data, "first");

  auto pub_best_effort = pub_node->create_publisher<std_msgs::msg::String>("test_topic", BestEffortQoS());
  test_msg.data = "second";

  while (!cache_best_effort.IsSubscribed() && pub_node->now() < timeout) {
    pub_best_effort->publish(test_msg);

    rclcpp::spin_some(sub_node);
    rclcpp::spin_some(pub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  EXPECT_FALSE(cache_reliable.IsSubscribed());

  ASSERT_TRUE(cache_best_effort.IsSubscribed());
  EXPECT_EQ(cache_best_effort.GetValue().data, "second");

  ASSERT_TRUE(cache_transient_local.IsSubscribed());
  EXPECT_EQ(cache_transient_local.GetValue().data, "first");
}

}  // namespace tmc_utils

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
