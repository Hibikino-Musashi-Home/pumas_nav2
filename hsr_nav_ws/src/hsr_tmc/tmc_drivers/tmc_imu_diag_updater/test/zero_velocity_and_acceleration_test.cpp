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
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include <sensor_msgs/msg/imu.hpp>
#include "test_utils.hpp"

using test_utils::DiagUpdaterNodeTest;
using test_utils::ResultPack;
using test_utils::WaitUntil;


namespace {
const char* const kName = "imu_diag_updater: imu topic status";
const char* const kHardwareID = "imu";
constexpr double kThreshold = 0.00001;
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

enum Level {
  kOK = diagnostic_msgs::msg::DiagnosticStatus::OK,
  kError = diagnostic_msgs::msg::DiagnosticStatus::ERROR,
};
}  // anonymous namespace


namespace tmc_imu_diag_updater {

struct TestParam {
  double value;
  ResultPack expected;
};

class GetZeroVecocityAndAccelerationErrorTest : public DiagUpdaterNodeTest,
                                                public ::testing::WithParamInterface<TestParam> {
 public:
  GetZeroVecocityAndAccelerationErrorTest()
      : DiagUpdaterNodeTest(rclcpp::Node::make_shared("zero_velocity_and_acceleration_test")) {
    node_ = getNode();
  }
  rclcpp::Node::SharedPtr node_;
};

INSTANTIATE_TEST_CASE_P(
    TestWithParam, GetZeroVecocityAndAccelerationErrorTest,
    testing::Values(
        // Border value test
        TestParam{ kThreshold + kEpsilon, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ kThreshold, { 1, Level::kError, kName, "Velocities and accelerations are zero", kHardwareID } }));

// If the angle speed / acceleration is all below the threshold, an abnormal DIAG should be returned.
// The phenomenon confirmed by TMC_ADI_DRIVER
TEST_P(GetZeroVecocityAndAccelerationErrorTest, TestWithParam) {
  // Setup
  const auto param = GetParam();
  sensor_msgs::msg::Imu msg;
  msg.angular_velocity.x = param.value;
  msg.angular_velocity.y = param.value;
  msg.angular_velocity.z = param.value;
  msg.linear_acceleration.x = param.value;
  msg.linear_acceleration.y = param.value;
  msg.linear_acceleration.z = param.value;
  imu_pub_ = test_utils::GenerateFixedPublisher(node_, msg);

  // Exercise
  ASSERT_TRUE(WaitUntil(node_, imu_is_subscribed_, 3.0));
  imu_pub_->PublishPeriodically(100.0);
  diag_sub_->StartCaching();
  ASSERT_TRUE(WaitUntil(node_, diag_is_advertised_, 3.0));
  ASSERT_TRUE(WaitUntil(node_, cache_length_greater_than_3_, 5.0));
  diag_sub_->StopCaching();

  // Verify
  const auto diag_msg = diag_sub_->GetLatestMessage();
  EXPECT_EQ(param.expected.size, diag_msg.status.size());
  EXPECT_EQ(param.expected.level, diag_msg.status[0].level);
  EXPECT_STREQ(param.expected.name.c_str(), diag_msg.status[0].name.c_str());
  EXPECT_STREQ(param.expected.message.c_str(), diag_msg.status[0].message.c_str());
  EXPECT_STREQ(param.expected.hardware_id.c_str(), diag_msg.status[0].hardware_id.c_str());
}
}  // namespace tmc_imu_diag_updater


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
