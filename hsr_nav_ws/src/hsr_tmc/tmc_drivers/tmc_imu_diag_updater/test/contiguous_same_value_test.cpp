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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "test_utils.hpp"

using test_utils::DiagUpdaterNodeTest;
using test_utils::ResultPack;
using test_utils::Vector3Randomizer;
using test_utils::WaitUntil;

namespace {
const char* const kName = "imu_diag_updater: imu topic status";
const char* const kHardwareID = "imu";

enum Level {
  kOK = diagnostic_msgs::msg::DiagnosticStatus::OK,
  kError = diagnostic_msgs::msg::DiagnosticStatus::ERROR,
};

}  // anonymous namespace

namespace tmc_imu_diag_updater {

struct InputParam {
  std::string random_target_vel;
  std::string random_target_acc;
  uint32_t publish_num;
};

struct TestParam {
  InputParam p;
  ResultPack expected;
};


class ContiguousSameValueTest : public DiagUpdaterNodeTest, public ::testing::WithParamInterface<TestParam> {
 public:
  ContiguousSameValueTest() : DiagUpdaterNodeTest(rclcpp::Node::make_shared("contiguous_same_value_test")) {
    node_ = getNode();
  }
  rclcpp::Node::SharedPtr node_;
};

INSTANTIATE_TEST_CASE_P(
    ValidateBehaviorWithParam, ContiguousSameValueTest,
    testing::Values(
        // Since the test is retained, the result changes in the order of giving data.
        // Test groups that gradually reduce the number of fluctuations that fluctuates and make an error when crossing the threshold.
        TestParam{ { "xyz", "xyz", 100 }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { "xyz", "xy", 100 }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { "xyz", "x", 100 }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { "xyz", "", 100 }, { 1, Level::kError, kName, "Contiguous same value", kHardwareID } },
        // Is it possible to return to normal and go with another property?
        TestParam{ { "xyz", "xyz", 100 }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { "xy", "xyz", 100 }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { "x", "xyz", 100 }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { "", "xyz", 100 }, { 1, Level::kError, kName, "Contiguous same value", kHardwareID } },
        // Is it possible to return to normal and stop the angle speed and acceleration?
        TestParam{ { "xyz", "xyz", 100 }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { "xy", "xyz", 100 }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { "xy", "xy", 100 }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { "x", "xy", 100 }, { 1, Level::kError, kName, "Contiguous same value", kHardwareID } },
        // Return to normal and make an error when you cross the continuous number threshold.
        TestParam{ { "xyz", "xyz", 100 }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { "xyz", "", 1 }, { 1, Level::kOK, kName, "OK", kHardwareID } },                        // 0
        TestParam{ { "xyz", "", 1 }, { 1, Level::kOK, kName, "OK", kHardwareID } },                        // 1
        TestParam{ { "xyz", "", 1 }, { 1, Level::kOK, kName, "OK", kHardwareID } },                        // 2
        TestParam{ { "xyz", "", 1 }, { 1, Level::kOK, kName, "OK", kHardwareID } },                        // 3
        TestParam{ { "xyz", "", 1 }, { 1, Level::kOK, kName, "OK", kHardwareID } },                        // 4
        TestParam{ { "xyz", "", 1 }, { 1, Level::kError, kName, "Contiguous same value", kHardwareID } },  // 5(Error)
        // Returning
        TestParam{ { "xyz", "xyz", 100 }, { 1, Level::kOK, kName, "OK", kHardwareID } }));

TEST_P(ContiguousSameValueTest, ValidateBehaviorWithParam) {
  // Setup
  const auto param = GetParam();
  sensor_msgs::msg::Imu base_msg;
  const auto rand_vel = Vector3Randomizer(-1.0, 1.0, param.p.random_target_vel);
  const auto rand_acc = Vector3Randomizer(-1.0, 1.0, param.p.random_target_acc);
  imu_pub_ = test_utils::GenerateRandomPublisher(node_, base_msg, rand_vel, rand_acc);
  ASSERT_TRUE(WaitUntil(node_, imu_is_subscribed_, 3.0));
  diag_sub_->StartCaching();
  ASSERT_TRUE(WaitUntil(node_, diag_is_advertised_, 3.0));

  // Exercise
  rclcpp::Rate rate(100.0);
  for (int i = 0; i < param.p.publish_num; ++i) {
    imu_pub_->PublishOnce();
    rate.sleep();
  }
  ASSERT_TRUE(WaitUntil(node_, cache_length_greater_than_3_, 10.0));
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
