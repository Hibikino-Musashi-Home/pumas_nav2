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
#include <std_msgs/msg/empty.hpp>
#include "test_utils.hpp"

using test_utils::DiagUpdaterNodeTest;
using test_utils::ResultPack;
using test_utils::WaitForTopicExistence;
using test_utils::WaitUntil;

namespace {
const char* const kName = "imu_diag_updater: imu topic status";
const char* const kHardwareID = "imu";
const char* const kResetTopic = "reset_sample_count";
const char* const kErrMsg = "Initial Acceleration Norm is invalid";
constexpr uint32_t kPubNum = 10;
constexpr uint32_t kPubNumThd = 100;
constexpr double kImuRate = 100.0;
constexpr double kNormMin = 9.5;
constexpr double kNormMax = 11.5;
constexpr double kSigFig = 1e-15;
const std::array<std::string, 3> kPropertyNames{ "x", "y", "z" };

enum Level {
  kOK = diagnostic_msgs::msg::DiagnosticStatus::OK,
  kError = diagnostic_msgs::msg::DiagnosticStatus::ERROR,
};

}  // anonymous namespace

namespace tmc_imu_diag_updater {


struct InputParam {
  bool is_sample_reset;
  double value;
  uint32_t publish_num;
  std::string target;
};

struct TestParam {
  InputParam p;
  ResultPack expected;
};


class InitialAccelerationNormTest : public DiagUpdaterNodeTest, public ::testing::WithParamInterface<TestParam> {
 public:
  InitialAccelerationNormTest() : DiagUpdaterNodeTest(rclcpp::Node::make_shared("initial_acceleration_norm_test")) {
    node_ = getNode();
    reset_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(kResetTopic, 1);
  }
  ~InitialAccelerationNormTest() = default;
  rclcpp::Node::SharedPtr node_;

 protected:
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_publisher_;
};

INSTANTIATE_TEST_CASE_P(
    ValidateBehaviorWithParam, InitialAccelerationNormTest,
    testing::Values(
        // NORM boundary value test
        // norm_min - sig_fig < norm_min < norm_min + sig_fig
        TestParam{ { true, kNormMin - kSigFig, kPubNum, "z" }, { 1, Level::kError, kName, kErrMsg, kHardwareID } },
        TestParam{ { true, kNormMin, kPubNum, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { true, kNormMin + kSigFig, kPubNum, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        // norm_max - sig_fig < norm_max < norm_max + sig_fig
        TestParam{ { true, kNormMax - kSigFig, kPubNum, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { true, kNormMax, kPubNum, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { true, kNormMax + kSigFig, kPubNum, "z" }, { 1, Level::kError, kName, kErrMsg, kHardwareID } },
        // Normal (inside the sample) -> Abnormal (outside the sample)
        TestParam{ { true, kNormMax, kPubNumThd, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { false, kNormMax + kSigFig, kPubNum, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        // Abnormal (inside the sample) -> Normal (outside the sample)
        TestParam{ { true, kNormMax + kSigFig, kPubNumThd, "z" }, { 1, Level::kError, kName, kErrMsg, kHardwareID } },
        TestParam{ { false, kNormMax, kPubNum, "z" }, { 1, Level::kError, kName, kErrMsg, kHardwareID } },
        // Sample_num boundary value test
        // sample_num - 1
        TestParam{ { true, kNormMax, kPubNumThd - 1, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { false, kNormMax + kSigFig, kPubNum, "z" }, { 1, Level::kError, kName, kErrMsg, kHardwareID } },
        // sample_num
        TestParam{ { true, kNormMax, kPubNumThd, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { false, kNormMax + kSigFig, kPubNum, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        // sample_num + 1
        TestParam{ { true, kNormMax, kPubNumThd + 1, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { false, kNormMax + kSigFig, kPubNum, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        // Is the diag updated by the sample reset?
        TestParam{ { true, kNormMax + kSigFig, kPubNumThd, "z" }, { 1, Level::kError, kName, kErrMsg, kHardwareID } },
        TestParam{ { false, kNormMax, kPubNum, "z" }, { 1, Level::kError, kName, kErrMsg, kHardwareID } },
        TestParam{ { true, kNormMax, kPubNum, "z" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        // Can an error be detected with the axis X?
        TestParam{ { true, kNormMax, kPubNum, "x" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { true, kNormMax + kSigFig, kPubNum, "x" }, { 1, Level::kError, kName, kErrMsg, kHardwareID } },
        // Can an error be detected with the axis Y?
        TestParam{ { true, kNormMax, kPubNum, "y" }, { 1, Level::kOK, kName, "OK", kHardwareID } },
        TestParam{ { true, kNormMax + kSigFig, kPubNum, "y" }, { 1, Level::kError, kName, kErrMsg, kHardwareID } }));

TEST_P(InitialAccelerationNormTest, ValidateBehaviorWithParam) {
  // Setup
  const auto param = GetParam();
  sensor_msgs::msg::Imu msg;
  std::array<double*, 3> iteratable{ &(msg.linear_acceleration.x), &(msg.linear_acceleration.y),
                                     &(msg.linear_acceleration.z) };
  for (int i = 0; i < 3; ++i) {
    if (param.p.target.find(kPropertyNames[i]) != std::string::npos) {
      *(iteratable[i]) = param.p.value;
    }
  }
  imu_pub_ = test_utils::GenerateFixedPublisher(node_, msg);
  ASSERT_TRUE(WaitUntil(node_, imu_is_subscribed_, 5.0));

  // Exercise
  if (param.p.is_sample_reset) {
    ASSERT_TRUE(WaitForTopicExistence(node_, kResetTopic, std::chrono::seconds(3)));
    reset_publisher_->publish(std_msgs::msg::Empty());
  }
  diag_sub_->StartCaching();
  ASSERT_TRUE(WaitUntil(node_, diag_is_advertised_, 3.0));

  rclcpp::Rate rate(kImuRate);
  for (int i = 0; i < param.p.publish_num; ++i) {
    imu_pub_->PublishOnce();
    rate.sleep();
  }
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
