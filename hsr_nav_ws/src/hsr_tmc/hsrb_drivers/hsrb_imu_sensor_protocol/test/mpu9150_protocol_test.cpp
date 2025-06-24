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
#include <hsrb_imu_sensor_protocol/imu_protocol.hpp>
#include <hsrb_imu_sensor_protocol/mpu9150_protocol.hpp>
#include <rclcpp/rclcpp.hpp>
#include "serial_communication_for_test.hpp"

namespace hsrb_imu_sensor_protocol {

class MPU9150ProtocolTest : public ::testing::Test {
 protected:
  MPU9150ProtocolTest() { node_ = rclcpp::Node::make_shared("mpu9150_protocol_test"); }
  virtual ~MPU9150ProtocolTest() {}
  virtual void SetUp() {}
  rclcpp::Node::SharedPtr node_;
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

/// Assuming that the state of the protocol is waiting for the command to proceed to the internal state of the protocol
void SetProtocolStatus(ProtocolStatus status, MPU9150Protocol& protocol, SerialCommunication& sensor_port) {
  // Prepare
  boost::system::error_code error;
  std::vector<uint8_t> receive_packet;
  std::vector<uint8_t> send_packet(4);
  send_packet[0] = 0x40;
  send_packet[1] = 0x47;
  send_packet[2] = 0x01;
  send_packet[3] = 0x77;

  // If you are waiting for the command, do nothing
  if (status == kStatusWaiting) {
    return;
  }

  // Successfully send reset instructions and wait for reset completion
  MPU9150Protocol::ResetResult result;
  error = protocol.TryReset(result);
  ASSERT_FALSE(error);
  EXPECT_EQ(MPU9150Protocol::kContinue, result);

  if (status == kStatusWaitForReset) {
    sleep(1);
    sensor_port.Receive(receive_packet);
    return;
  }

  // Call the RESET function after more than 3 seconds after waiting for the reset
  // Receive the reply of the reset completion
  std::this_thread::sleep_for(std::chrono::seconds(3));
  error = protocol.TryReset(result);
  ASSERT_FALSE(error);
  EXPECT_EQ(MPU9150Protocol::kContinue, result);

  if (status == kStatusReceiveResetReturn) {
    sleep(1);
    sensor_port.Receive(receive_packet);
    return;
  }
  sleep(1);
  sensor_port.Receive(receive_packet);
  return;
}

// Sensor value reading test, normal system
TEST(MPU9150ProtocolTest, ReadStateNormal) {
  // Prepare
  SerialCommunication sensor_port("/tmp/mpu9150_protocol_sensor", kBaudRate);
  boost::system::error_code error;
  MPU9150Network network("/tmp/mpu9150_protocol_pc", error);
  MPU9150Protocol protocol(network);
  std::vector<uint8_t> send_packet(96);

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
  sensor_port.Send(send_packet);
  // Reading and execution
  ImuState state;
  error = protocol.ReadState(state);
  ASSERT_FALSE(error) << error.message();
  // Confirmation of the receiving sensor output value should have been removed from the second measurement data
  const double g = 9.80665;
  const double degree = boost::math::constants::degree<double>();
  EXPECT_NEAR(0.5, state.orientation[0], kEpsilon);
  EXPECT_NEAR(-0.5, state.orientation[1], kEpsilon);
  EXPECT_NEAR(-0.7071, state.orientation[2], kEpsilon);
  EXPECT_NEAR(0.7071, state.orientation[3], kEpsilon);
  EXPECT_NEAR(400.0 * degree, state.angular_velocity[0], kEpsilon);
  EXPECT_NEAR(200.0 * degree, state.angular_velocity[1], kEpsilon);
  EXPECT_NEAR(100.0 * degree, state.angular_velocity[2], kEpsilon);
  EXPECT_NEAR(2.0 * g, state.linear_acceleration[0], kEpsilon);
  EXPECT_NEAR(1.0 * g, state.linear_acceleration[1], kEpsilon);
  EXPECT_NEAR(0.5 * g, state.linear_acceleration[2], kEpsilon);
}

// Sensor value reading test, abnormal system
TEST(MPU9150ProtocolTest, ReadStateAbnormal) {
  // Prepare
  SerialCommunication sensor_port("/tmp/mpu9150_protocol_sensor", kBaudRate);
  boost::system::error_code error;
  MPU9150Network network("/tmp/mpu9150_protocol_pc", error);
  MPU9150Protocol protocol(network);

  std::vector<uint8_t> send_packet(48);
  // header
  send_packet[0] = 0x40;
  send_packet[1] = 0x47;
  // packet length (45byte)
  send_packet[2] = 0x2D;
  // Quaternion W
  send_packet[3] = 0x2D;
  send_packet[4] = 0x41;
  send_packet[5] = 0x20;
  send_packet[6] = 0x5B;
  // Quaternion X
  send_packet[7] = 0x20;
  send_packet[8] = 0x00;
  send_packet[9] = 0x00;
  send_packet[10] = 0x00;
  // Quaternion Y
  send_packet[11] = 0xE0;
  send_packet[12] = 0x00;
  send_packet[13] = 0x00;
  send_packet[14] = 0x00;
  // Quaternion Z
  send_packet[15] = 0xD2;
  send_packet[16] = 0xBE;
  send_packet[17] = 0xDF;
  send_packet[18] = 0xA5;
  // Angular Velocity X
  send_packet[19] = 0x01;
  send_packet[20] = 0x90;
  send_packet[21] = 0x00;
  send_packet[22] = 0x00;
  // Angular Velocity Y
  send_packet[23] = 0x00;
  send_packet[24] = 0xC8;
  send_packet[25] = 0x00;
  send_packet[26] = 0x00;
  // Angular Velocity Z
  send_packet[27] = 0x00;
  send_packet[28] = 0x64;
  send_packet[29] = 0x00;
  send_packet[30] = 0x00;
  // Acceleration X
  send_packet[31] = 0x00;
  send_packet[32] = 0x02;
  send_packet[33] = 0x00;
  send_packet[34] = 0x00;
  // Acceleration Y
  send_packet[35] = 0x00;
  send_packet[36] = 0x01;
  send_packet[37] = 0x00;
  send_packet[38] = 0x00;
  // Acceleration Z
  send_packet[39] = 0x00;
  send_packet[40] = 0x00;
  send_packet[41] = 0x80;
  send_packet[42] = 0x00;
  // TimeStamp
  send_packet[43] = 0x11;
  send_packet[44] = 0xB4;
  send_packet[45] = 0x00;
  send_packet[46] = 0x09;
  // check sum
  send_packet[47] = 0x40;

  ImuState state;
  std::vector<uint8_t> receive_packet;

  // Packets from the sensor are fraudulent
  --send_packet[47];
  sensor_port.Send(send_packet);

  // Reading and execution
  error = protocol.ReadState(state);
  ASSERT_TRUE(error);

  // Continue to reset
  MPU9150Protocol::ResetResult result;
  error = protocol.TryReset(result);
  ASSERT_EQ(MPU9150Protocol::kContinue, result);
  ASSERT_FALSE(error);

  // Do not read during reset, do not change State
  state.orientation[0] = 20.0;
  error = protocol.ReadState(state);
  ASSERT_FALSE(error);
  EXPECT_DOUBLE_EQ(20.0, state.orientation[0]);

  // Read so that no garbage remains on the port
  sleep(1);
  sensor_port.Receive(receive_packet);
}

// RESET while waiting for command, normal system
TEST(MPU9150ProtocolTest, ResetStatusWaitingNormal) {
  // Prepare
  SerialCommunication sensor_port("/tmp/mpu9150_protocol_sensor", kBaudRate);
  boost::system::error_code error;
  MPU9150Network network("/tmp/mpu9150_protocol_pc", error);
  MPU9150Protocol protocol(network);

  // Successfully send reset instructions and wait for reset
  MPU9150Protocol::ResetResult result;
  error = protocol.TryReset(result);
  ASSERT_FALSE(error);
  EXPECT_EQ(MPU9150Protocol::kContinue, result);

  // Confirm that the reset order was sent
  std::vector<uint8_t> receive_packet;
  sensor_port.Receive(receive_packet);

  ASSERT_EQ(5, receive_packet.size());
  EXPECT_EQ(0x40, receive_packet[0]);
  EXPECT_EQ(0x47, receive_packet[1]);
  EXPECT_EQ(0x02, receive_packet[2]);
  EXPECT_EQ(0x72, receive_packet[3]);
  EXPECT_EQ(0x04, receive_packet[4]);

  // Readstate doesn't do anything when waiting for reset
  ImuState state;
  state.orientation[0] = 10.0;
  error = protocol.ReadState(state);
  ASSERT_FALSE(error);
  EXPECT_NEAR(10.0, state.orientation[0], kEpsilon);
}

// RESET in a command waiting, abnormal system
TEST(MPU9150ProtocolTest, ResetStatusWaitingAbnormal) {
  // Prepare
  SerialCommunication sensor_port("/tmp/mpu9150_protocol_sensor", kBaudRate);
  boost::system::error_code error;
  MPU9150Network network("/tmp/mpu9150_protocol_pc", error);
  MPU9150Protocol protocol(network);

  // Fill the port with garbage data to fail to send reset instructions
  uint32_t counter = 0;
  while (!network.Send(0x70, NULL, 0)) {
    ++counter;
  }

  // Sending reset order, should fail
  MPU9150Protocol::ResetResult result;
  error = protocol.TryReset(result);
  ASSERT_TRUE(error);
  EXPECT_EQ(MPU9150Protocol::kError, result);

  // Remove trash data
  std::vector<uint8_t> receive_packet;
  int32_t byte_num = 5 * counter;
  counter = 1;
  while (byte_num >= 0) {
    sensor_port.Receive(receive_packet);
    byte_num -= receive_packet.size();
    ++counter;
  }
}

// Reset in waiting for reset
TEST(MPU9150ProtocolTest, ResetStatusWaitForReset) {
  // Prepare
  SerialCommunication sensor_port("/tmp/mpu9150_protocol_sensor", kBaudRate);
  boost::system::error_code error;
  MPU9150Network network("/tmp/mpu9150_protocol_pc", error);
  MPU9150Protocol protocol(network);

  // Waiting for the reset completion
  SetProtocolStatus(kStatusWaitForReset, protocol, sensor_port);

  // Call the RESET function within 3 seconds while waiting for the reset
  MPU9150Protocol::ResetResult result;
  error = protocol.TryReset(result);
  ASSERT_FALSE(error);
  EXPECT_EQ(MPU9150Protocol::kContinue, result);

  // Call a reset function after 3 seconds or more waiting for the reset to complete
  std::this_thread::sleep_for(std::chrono::seconds(3));
  error = protocol.TryReset(result);
  ASSERT_FALSE(error);
  EXPECT_EQ(MPU9150Protocol::kContinue, result);
}

// RESET, normal system in receiving the reply of reset completion
TEST(MPU9150ProtocolTest, ResetStatusReceiveResetReturnNormal) {
  // Prepare
  SerialCommunication sensor_port("/tmp/mpu9150_protocol_sensor", kBaudRate);
  boost::system::error_code error;
  MPU9150Network network("/tmp/mpu9150_protocol_pc", error);
  MPU9150Protocol protocol(network);

  std::vector<uint8_t> send_packet(4);
  send_packet[0] = 0x40;
  send_packet[1] = 0x47;
  send_packet[2] = 0x01;
  send_packet[3] = 0x77;

  // Receive the reply of the reset completion
  SetProtocolStatus(kStatusReceiveResetReturn, protocol, sensor_port);

  // Send a complete reply
  sensor_port.Send(send_packet);

  // Successful reply
  MPU9150Protocol::ResetResult result;
  error = protocol.TryReset(result);
  ASSERT_FALSE(error);
  EXPECT_EQ(MPU9150Protocol::kDone, result);
}

// RESET in a state where you receive a reply to the reset completion, anomalous system
TEST(MPU9150ProtocolTest, ResetStatusReceiveResetReturnAbnormal) {
  // Prepare
  SerialCommunication sensor_port("/tmp/mpu9150_protocol_sensor", kBaudRate);
  boost::system::error_code error;
  MPU9150Network network("/tmp/mpu9150_protocol_pc", error);
  MPU9150Protocol protocol(network);

  std::vector<uint8_t> send_packet(5);
  send_packet[0] = 0x40;
  send_packet[1] = 0x47;
  send_packet[2] = 0x02;
  send_packet[3] = 0x00;
  send_packet[4] = 0x76;

  // Receive the reply of the reset completion
  SetProtocolStatus(kStatusReceiveResetReturn, protocol, sensor_port);

  // Receiving the reset completion reply and failed within 5 seconds from the start of reset
  MPU9150Protocol::ResetResult result;
  error = protocol.TryReset(result);
  ASSERT_FALSE(error);
  EXPECT_EQ(MPU9150Protocol::kContinue, result);

  // Reception failed after 5 seconds or more
  std::this_thread::sleep_for(std::chrono::seconds(2));
  error = protocol.TryReset(result);
  ASSERT_TRUE(error);
  EXPECT_EQ(MPU9150Protocol::kError, result);

  // You should have returned to Waiting, so you will fail to read
  ImuState state;
  error = protocol.ReadState(state);
  ASSERT_TRUE(error);

  // Receive the reply of the reset completion
  SetProtocolStatus(kStatusReceiveResetReturn, protocol, sensor_port);

  // The reply of the reset completion is fraudulent
  sensor_port.Send(send_packet);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  error = protocol.TryReset(result);
  ASSERT_TRUE(error);
  EXPECT_EQ(MPU9150Protocol::kError, result);
}
}  // end of namespace hsrb_imu_sensor_protocol

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
