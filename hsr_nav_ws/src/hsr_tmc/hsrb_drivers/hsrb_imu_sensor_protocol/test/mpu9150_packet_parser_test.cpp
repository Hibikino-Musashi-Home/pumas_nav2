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
/// @brief Invensense's Gyro Sensor MPU9150 PARSER class test
#include <gtest/gtest.h>

#include <hsrb_imu_sensor_protocol/mpu9150_packet_parser.hpp>

namespace hsrb_imu_sensor_protocol {
// Normal, packet with data
TEST(MPU9150PacketParserTest, ParsePacketWithData) {
  // Prepare
  boost::array<uint8_t, 5> packet_5 = { { 0x40, 0x47, 0x02, 0x01, 0x75 } };
  boost::array<uint8_t, 6> packet_6 = { { 0x40, 0x47, 0x03, 0x01, 0x02, 0x72 } };
  MPU9150PacketParser parser;

  // PARSE execution
  ASSERT_EQ(0, parser.packets().size());
  for (uint32_t i = 0; i < 5; ++i) {
    ASSERT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(packet_5[i]));
  }

  // Confirmation of the contents
  ASSERT_EQ(5, parser.packets().size());
  for (uint32_t i = 0; i < 5; ++i) {
    EXPECT_EQ(packet_5[i], parser.packets().at(i));
  }

  // Reset
  parser.Reset();

  // PARSE execution
  for (uint32_t i = 0; i < 6; ++i) {
    ASSERT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(packet_6[i]));
  }

  // Confirmation of the contents
  ASSERT_EQ(6, parser.packets().size());
  for (uint32_t i = 0; i < 6; ++i) {
    EXPECT_EQ(packet_6[i], parser.packets().at(i));
  }

  // Test to keep accumulating packets until reset
  // PARSE execution
  for (uint32_t i = 0; i < 5; ++i) {
    ASSERT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(packet_5[i]));
  }

  // Confirmation of the contents
  ASSERT_EQ(11, parser.packets().size());
  for (uint32_t i = 0; i < 6; ++i) {
    EXPECT_EQ(packet_6[i], parser.packets().at(i));
  }
  for (uint32_t i = 0; i < 5; ++i) {
    EXPECT_EQ(packet_5[i], parser.packets().at(i + 6));
  }
}

// Normal, packet without data
TEST(MPU9150PacketParserTest, ParsePacketWithoutData) {
  // Prepare
  boost::array<uint8_t, 4> packet = { { 0x40, 0x47, 0x01, 0x77 } };
  MPU9150PacketParser parser;

  // Confirmation of reply flag
  EXPECT_FALSE(parser.is_reply_packet());

  // PARSE execution
  for (uint32_t i = 0; i < 3; ++i) {
    ASSERT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(packet[i]));
  }
  EXPECT_EQ(MPU9150PacketParser::kDone, parser.TryParse(packet[3]));

  // Confirmation of reply flag
  EXPECT_TRUE(parser.is_reply_packet());

  // Confirmation of the contents
  ASSERT_EQ(4, parser.packets().size());
  for (uint32_t i = 0; i < 4; ++i) {
    EXPECT_EQ(packet[i], parser.packets().at(i));
  }
}

// Normal system, reset
TEST(MPU9150PacketParserTest, ResetParser) {
  // Prepare
  boost::array<uint8_t, 4> packet = { { 0x40, 0x47, 0x01, 0x77 } };
  MPU9150PacketParser parser;

  // Confirmation of reply flag
  EXPECT_FALSE(parser.is_reply_packet());

  // PARSE execution
  for (uint32_t i = 0; i < 3; ++i) {
    ASSERT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(packet[i]));
  }

  // Reset on the way
  parser.Reset();

  // Execute again
  for (uint32_t i = 0; i < 3; ++i) {
    ASSERT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(packet[i]));
  }
  EXPECT_EQ(MPU9150PacketParser::kDone, parser.TryParse(packet[3]));

  // Confirmation of reply flag
  EXPECT_TRUE(parser.is_reply_packet());

  // Confirmation of the contents
  ASSERT_EQ(4, parser.packets().size());
  for (uint32_t i = 0; i < 4; ++i) {
    EXPECT_EQ(packet[i], parser.packets().at(i));
  }
}

// Normal system, checksum
TEST(MPU9150PacketParserTest, CheckSum) {
  // Prepare
  boost::array<uint8_t, 5> packet = { { 0x00, 0x7F, 0x80, 0x01, 0x40 } };

  EXPECT_EQ(0xFF, Checksum(packet.begin(), packet.begin() + 1));
  EXPECT_EQ(0x80, Checksum(packet.begin(), packet.begin() + 2));
  EXPECT_EQ(0x00, Checksum(packet.begin(), packet.begin() + 3));
  EXPECT_EQ(0xFF, Checksum(packet.begin(), packet.begin() + 4));
  EXPECT_EQ(0xBF, Checksum(packet.begin(), packet.begin() + 5));
}

// Abnormal system
TEST(MPU9150PacketParserTest, FailParser) {
  // Prepare
  boost::array<uint8_t, 5> packet = { { 0x40, 0x47, 0x02, 0x00, 0x76 } };
  MPU9150PacketParser parser;

  // Header 1 is strange
  EXPECT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(packet[0] + 1));

  // Header 2 is strange
  EXPECT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(packet[1] + 1));

  // The data length is 0
  parser.Reset();
  for (uint32_t i = 0; i < 2; ++i) {
    ASSERT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(packet[i]));
  }
  EXPECT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(0x00));

  // Checksum does not fit
  parser.Reset();
  for (uint32_t i = 0; i < 4; ++i) {
    ASSERT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(packet[i]));
  }
  EXPECT_EQ(MPU9150PacketParser::kContinue, parser.TryParse(packet[4] + 1));
}
}  // end of namespace hsrb_imu_sensor_protocol

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
