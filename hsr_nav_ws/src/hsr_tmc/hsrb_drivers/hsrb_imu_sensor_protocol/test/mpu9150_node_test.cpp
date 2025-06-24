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
/// @brief Inventsense's Gyro Sensor MPU9150 Easy -to -use API test

#include <chrono>
#include <vector>

#include <gtest/gtest.h>

#include <boost/math/constants/constants.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/empty.hpp>

#include "hsrb_imu_sensor_protocol/imu_protocol.hpp"
#include "hsrb_imu_sensor_protocol/mpu9150_protocol.hpp"
#include "serial_communication_for_test.hpp"
#include "subscriber.hpp"

using test_utils::WaitForTopicExistence;
using test_utils::WaitUntil;
using test_utils::WaitUntilTimuout;
using ImuSubscriber = test_utils::CacheSubscriber<sensor_msgs::msg::Imu>;

namespace hsrb_imu_sensor_protocol {

class MPU9150NodeTest : public ::testing::Test {
 protected:
  MPU9150NodeTest() { node_ = rclcpp::Node::make_shared("mpu9150node_test"); }
  virtual ~MPU9150NodeTest() {}
  virtual void SetUp() {
    imu_sub_.reset(new ImuSubscriber(node_, "/hsrb/base_imu", 1));
    imu_is_advertised_ = [this]() { return imu_sub_->IsPublished(); };
    imu_calibrate_client_ = node_->create_client<std_srvs::srv::Empty>("/hsrb/base_imu/calibrate");
    while (!imu_calibrate_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        break;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr imu_calibrate_client_;
  ImuSubscriber::SharedPtr imu_sub_;
  std::function<bool()> imu_is_advertised_;
};

const double kGravityAccel = 9.80665;
const double kEpsilon = 0.001;
const uint32_t kBaudRate = 57600;

/// Protocol condition
enum ProtocolStatus {
  /// Waiting for command
  kStatusWaiting,
  /// Wait for reset completion
  kStatusWaitForReset,
  /// Receive a reply to complete reset
  kStatusReceiveResetReturn,
};

// Sensor value reading test, normal system
TEST_F(MPU9150NodeTest, ReadStateNormal) {
  // Prepare
  SerialCommunication sensor_port("/tmp/mpu9150_protocol_sensor", kBaudRate);
  boost::system::error_code error;
  std::vector<uint8_t> send_packet(96);
  std::vector<uint8_t> reset_packet(4);

  ASSERT_TRUE(WaitForTopicExistence(node_, "hsrb/base_imu", std::chrono::seconds(5)));

  reset_packet[0] = 0x40;
  reset_packet[1] = 0x47;
  reset_packet[2] = 0x01;
  reset_packet[3] = 0x77;
  sensor_port.Send(reset_packet);


  imu_sub_->StartCaching();
  ASSERT_TRUE(WaitUntil(node_, imu_is_advertised_, 3.0));
  // Pour two measurement data
  // 1st measurement data
  // header
  send_packet[0] = 0x40;
  send_packet[1] = 0x47;
  // packet length (45byte)
  send_packet[2] = 0x2D;
  // Quaternion W
  send_packet[3] = 0x1D;
  send_packet[4] = 0x40;
  send_packet[5] = 0x40;
  send_packet[6] = 0x3E;
  // Quaternion X
  send_packet[7] = 0x23;
  send_packet[8] = 0x03;
  send_packet[9] = 0x10;
  send_packet[10] = 0x90;
  // Quaternion Y
  send_packet[11] = 0xE2;
  send_packet[12] = 0x03;
  send_packet[13] = 0x20;
  send_packet[14] = 0x00;
  // Quaternion Z
  send_packet[15] = 0x12;
  send_packet[16] = 0xBA;
  send_packet[17] = 0xD9;
  send_packet[18] = 0xA1;
  // Angular Velocity X
  send_packet[19] = 0x31;
  send_packet[20] = 0x91;
  send_packet[21] = 0x01;
  send_packet[22] = 0x01;
  // Angular Velocity Y
  send_packet[23] = 0x03;
  send_packet[24] = 0xA8;
  send_packet[25] = 0x0E;
  send_packet[26] = 0x0A;
  // Angular Velocity Z
  send_packet[27] = 0x09;
  send_packet[28] = 0x94;
  send_packet[29] = 0x10;
  send_packet[30] = 0x10;
  // Acceleration X
  send_packet[31] = 0x02;
  send_packet[32] = 0x12;
  send_packet[33] = 0x10;
  send_packet[34] = 0x09;
  // Acceleration Y
  send_packet[35] = 0x03;
  send_packet[36] = 0x21;
  send_packet[37] = 0x01;
  send_packet[38] = 0x09;
  // Acceleration Z
  send_packet[39] = 0x05;
  send_packet[40] = 0x03;
  send_packet[41] = 0x89;
  send_packet[42] = 0x09;
  // TimeStamp
  send_packet[43] = 0x11;
  send_packet[44] = 0xB4;
  send_packet[45] = 0x00;
  send_packet[46] = 0x01;
  // check sum
  send_packet[47] = 0x60;

  // The second measurement data
  // header
  send_packet[48] = 0x40;
  send_packet[49] = 0x47;
  // packet length (45byte)
  send_packet[50] = 0x2D;
  // Quaternion W
  send_packet[51] = 0x2D;
  send_packet[52] = 0x41;
  send_packet[53] = 0x20;
  send_packet[54] = 0x5B;
  // Quaternion X
  send_packet[55] = 0x20;
  send_packet[56] = 0x00;
  send_packet[57] = 0x00;
  send_packet[58] = 0x00;
  // Quaternion Y
  send_packet[59] = 0xE0;
  send_packet[60] = 0x00;
  send_packet[61] = 0x00;
  send_packet[62] = 0x00;
  // Quaternion Z
  send_packet[63] = 0xD2;
  send_packet[64] = 0xBE;
  send_packet[65] = 0xDF;
  send_packet[66] = 0xA5;
  // Angular Velocity X
  send_packet[67] = 0x01;
  send_packet[68] = 0x90;
  send_packet[69] = 0x00;
  send_packet[70] = 0x00;
  // Angular Velocity Y
  send_packet[71] = 0x00;
  send_packet[72] = 0xC8;
  send_packet[73] = 0x00;
  send_packet[74] = 0x00;
  // Angular Velocity Z
  send_packet[75] = 0x00;
  send_packet[76] = 0x64;
  send_packet[77] = 0x00;
  send_packet[78] = 0x00;
  // Acceleration X
  send_packet[79] = 0x00;
  send_packet[80] = 0x02;
  send_packet[81] = 0x00;
  send_packet[82] = 0x00;
  // Acceleration Y
  send_packet[83] = 0x00;
  send_packet[84] = 0x01;
  send_packet[85] = 0x00;
  send_packet[86] = 0x00;
  // Acceleration Z
  send_packet[87] = 0x00;
  send_packet[88] = 0x00;
  send_packet[89] = 0x80;
  send_packet[90] = 0x00;
  // TimeStamp
  send_packet[91] = 0x11;
  send_packet[92] = 0xB4;
  send_packet[93] = 0x00;
  send_packet[94] = 0x09;
  // check sum
  send_packet[95] = 0x40;
  // Send a packet
  WaitUntilTimuout(node_, 5.0);
  sensor_port.Send(send_packet);
  WaitUntilTimuout(node_, 5.0);
  imu_sub_->StopCaching();
  const auto imu_msg = imu_sub_->GetLatestMessage();

  const double g = 9.80665;
  const double degree = boost::math::constants::degree<double>();
  std::cerr << "imu_msg.orientation.x : " << imu_msg.orientation.x << std::endl;
  EXPECT_NEAR(0.5, imu_msg.orientation.x, kEpsilon);
  EXPECT_NEAR(-0.5, imu_msg.orientation.y, kEpsilon);
  EXPECT_NEAR(-0.7071, imu_msg.orientation.z, kEpsilon);
  EXPECT_NEAR(0.7071, imu_msg.orientation.w, kEpsilon);
  EXPECT_NEAR(400.0 * degree, imu_msg.angular_velocity.x, kEpsilon);
  EXPECT_NEAR(200.0 * degree, imu_msg.angular_velocity.y, kEpsilon);
  EXPECT_NEAR(100.0 * degree, imu_msg.angular_velocity.z, kEpsilon);
  EXPECT_NEAR(2.0 * g, imu_msg.linear_acceleration.x, kEpsilon);
  EXPECT_NEAR(1.0 * g, imu_msg.linear_acceleration.y, kEpsilon);
  EXPECT_NEAR(0.5 * g, imu_msg.linear_acceleration.z, kEpsilon);
}
// Sensor value reading test, normal system
TEST_F(MPU9150NodeTest, ResetStatusReceiveResetReturnNormal) {
  // Prepare
  SerialCommunication sensor_port("/tmp/mpu9150_protocol_sensor", kBaudRate);
  boost::system::error_code error;
  std::vector<uint8_t> reset_packet(4);

  ASSERT_TRUE(WaitForTopicExistence(node_, "hsrb/base_imu", std::chrono::seconds(5)));

  reset_packet[0] = 0x40;
  reset_packet[1] = 0x47;
  reset_packet[2] = 0x01;
  reset_packet[3] = 0x77;
  sensor_port.Send(reset_packet);

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = imu_calibrate_client_->async_send_request(request);
  sensor_port.Send(reset_packet);
  // Wait for the result.
  ASSERT_TRUE(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS);
}
}  // namespace hsrb_imu_sensor_protocol
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
