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
#ifndef TMC_IMU_DIAG_UPDATER_TEST_UTILS_HPP_
#define TMC_IMU_DIAG_UPDATER_TEST_UTILS_HPP_
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "generator.hpp"
#include "publisher.hpp"
#include "randomizer.hpp"
#include "subscriber.hpp"
#include "util_function.hpp"


using test_utils::FixedGenerator;
using test_utils::ForwardStampGenerator;
using test_utils::IGenerator;
using test_utils::WaitUntil;
using ImuMsgGen = test_utils::StampAndMessageGenerator<sensor_msgs::msg::Imu>;
using ImuPublisher = test_utils::Publisher<ImuMsgGen>;
using DiagSubscriber = test_utils::CacheSubscriber<diagnostic_msgs::msg::DiagnosticArray>;

namespace test_utils {

class Vector3Randomizer : public IGenerator<geometry_msgs::msg::Vector3> {
 public:
  explicit Vector3Randomizer(const double min, const double max, const std::string& target = "xyz")
      : target_(target), randomizer_(new test_utils::random::ScalarUniformDistribution(min, max)) {}

  geometry_msgs::msg::Vector3 Generate() override {
    geometry_msgs::msg::Vector3 msg;
    std::array<double*, 3> iteratable{ &(msg.x), &(msg.y), &(msg.z) };
    for (int i = 0; i < 3; ++i) {
      if (target_.find(kPropertyNames[i]) != std::string::npos) *(iteratable[i]) = randomizer_->Generate();
    }
    return msg;
  }

 private:
  const std::string target_;
  static const std::array<std::string, 3> kPropertyNames;
  IGenerator<double>::SharedPtr randomizer_;
};
const std::array<std::string, 3> Vector3Randomizer::kPropertyNames{ "x", "y", "z" };

class RandomImuGenerator : public IGenerator<sensor_msgs::msg::Imu> {
 public:
  using SharedPtr = std::shared_ptr<RandomImuGenerator>;

  RandomImuGenerator(const sensor_msgs::msg::Imu& base, const Vector3Randomizer& rand_vel,
                     const Vector3Randomizer& rand_acc)
      : base_(base), rand_vel_(rand_vel), rand_acc_(rand_acc) {}
  virtual ~RandomImuGenerator() = default;

  sensor_msgs::msg::Imu Generate() override {
    base_.angular_velocity = rand_vel_.Generate();
    base_.linear_acceleration = rand_acc_.Generate();
    return base_;
  }

 private:
  sensor_msgs::msg::Imu base_;
  Vector3Randomizer rand_vel_;
  Vector3Randomizer rand_acc_;
};

ImuPublisher::SharedPtr GenerateFixedPublisher(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::Imu& imu_msg) {
  auto msg_gen = ImuMsgGen::SharedPtr(new ImuMsgGen(
      FixedGenerator<sensor_msgs::msg::Imu>::SharedPtr(new FixedGenerator<sensor_msgs::msg::Imu>(imu_msg)),
      ForwardStampGenerator::SharedPtr(new ForwardStampGenerator())));

  return ImuPublisher::SharedPtr(new ImuPublisher(node, "imu", 1, msg_gen));
}

ImuPublisher::SharedPtr GenerateRandomPublisher(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::Imu& base_msg,
                                                const Vector3Randomizer& rand_vel, const Vector3Randomizer& rand_acc) {
  auto msg_gen = ImuMsgGen::SharedPtr(
      new ImuMsgGen(IGenerator<sensor_msgs::msg::Imu>::SharedPtr(new RandomImuGenerator(base_msg, rand_vel, rand_acc)),
                    ForwardStampGenerator::SharedPtr(new ForwardStampGenerator())));

  return ImuPublisher::SharedPtr(new ImuPublisher(node, "imu", 1, msg_gen));
}

class DiagUpdaterNodeTest : public testing::Test {
 public:
  explicit DiagUpdaterNodeTest(rclcpp::Node::SharedPtr node) : node_(node) {}

  virtual ~DiagUpdaterNodeTest() = default;

  // Test start every process
  void SetUp() override {
    diag_sub_.reset(new DiagSubscriber(node_, "diagnostics", 1));
    // It's troublesome to bind every time, so I'll do it here
    imu_is_subscribed_ = [this]() { return imu_pub_->IsSubscribed(3.0); };
    diag_is_advertised_ = [this]() { return diag_sub_->IsPublished(); };
    cache_length_greater_than_3_ = [this]() { return diag_sub_->GetCacheLength() > 3; };
  }
  rclcpp::Node::SharedPtr getNode() { return node_; }

  // Testing every process
  // -The end of Publish
  // -Subscription ends
  void TearDown() override {
    imu_pub_.reset();
    diag_sub_.reset();
  }

 protected:
  // Access point to ROS function
  rclcpp::Node::SharedPtr node_;

  // Sending and receiving confirmation Utilities
  std::function<bool()> imu_is_subscribed_;
  std::function<bool()> diag_is_advertised_;
  std::function<bool()> cache_length_greater_than_3_;

  // I/O
  ImuPublisher::SharedPtr imu_pub_;
  DiagSubscriber::SharedPtr diag_sub_;
};

struct ResultPack {
  uint32_t size;
  uint32_t level;
  std::string name;
  std::string message;
  std::string hardware_id;
};

}  // namespace test_utils
#endif  // TMC_IMU_DIAG_UPDATER_TEST_UTILS_HPP_
