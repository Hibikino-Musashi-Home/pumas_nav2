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
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <gtest/gtest.h>

#include "../src/power_ecu_com_data_encoder.hpp"
#include "common_methods.hpp"

namespace {
const size_t kBufferSize = 1000;
}  // anonymous namespace

// PowerEcuComTimeDataEncoder
TEST(PowerEcuComTimeDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComTimeDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  std::string* start_time = encoder.GetParamPtr<std::string>("start_time");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "time_");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 26);

  std::string value = "YYYYMMDDHHMMSS";
  *start_time = value;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), (value + ",").c_str());

  value = "yyyymmddhhmmss";
  *start_time = value;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), (value + ",").c_str());
}
// PowerEcuComStartDataEncoder
TEST(PowerEcuComStartDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComStartDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  bool* is_enable_ecu1 = encoder.GetParamPtr<bool>("is_enable_ecu1");
  bool* is_enable_ecu2 = encoder.GetParamPtr<bool>("is_enable_ecu2");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "start");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 15);

  *is_enable_ecu1 = false;
  *is_enable_ecu2 = false;

  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h00,");

  *is_enable_ecu1 = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h01,");

  *is_enable_ecu2 = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h03,");
}

// PowerEcuComStopDataEncoder
TEST(PowerEcuComStopDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComStopDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "stop_");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 11);

  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "");
}

// PowerEcuComHeartDataEncoder
TEST(PowerEcuComHeartDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComHeartDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  uint16_t* error_state = encoder.GetParamPtr<uint16_t>("error_state");
  uint32_t* counts = encoder.GetParamPtr<uint32_t>("counts");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "heart");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 27);

  *error_state = 0;
  *counts = 0;

  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h0000,h00000000,");

  *error_state = 1;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h0001,h00000000,");

  *counts = 1;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h0001,h00000001,");
}

// PowerEcuComPumpDataEncoder
TEST(PowerEcuComPumpDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComPumpDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  uint8_t* is_pump_enable = encoder.GetParamPtr<uint8_t>("is_pump_enable");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "pump_");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 13);

  *is_pump_enable = false;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "0,");

  *is_pump_enable = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "1,");
}

// PowerEcuComPbmswDataEncoder
TEST(PowerEcuComPbmswDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComPbmswDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  uint8_t* is_motor_enable = encoder.GetParamPtr<uint8_t>("is_motor_enable");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "pbmsw");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 13);

  *is_motor_enable = false;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "0,");

  *is_motor_enable = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "1,");
}

// PowerEcuComLedcDataEncoder
TEST(PowerEcuComLedcDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComLedcDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  hsrb_power_ecu::Color<uint8_t>* led_color = encoder.GetParamPtr<hsrb_power_ecu::Color<uint8_t> >("led_color");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "ledc_");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 23);

  (*led_color).r = 0;
  (*led_color).g = 0;
  (*led_color).b = 0;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "000,000,000,");

  (*led_color).r = 1;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "001,000,000,");

  (*led_color).g = 1;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "001,001,000,");

  (*led_color).b = 1;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "001,001,001,");
}

// PowerEcuComGResDataEncoder
TEST(PowerEcuComGResDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComGResDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  bool* is_quaternion_reset = encoder.GetParamPtr<bool>("is_quaternion_reset");
  bool* is_gyro_reset = encoder.GetParamPtr<bool>("is_gyro_reset");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "g_res");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 15);

  *is_quaternion_reset = false;
  *is_gyro_reset = false;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h00,");

  *is_quaternion_reset = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h01,");

  *is_gyro_reset = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h03,");
}

// PowerEcuComSolswDataEncoder
TEST(PowerEcuComSolswDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComSolswDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  uint8_t* is_solenoid_enable = encoder.GetParamPtr<uint8_t>("is_solenoid_enable");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "solsw");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 15);

  *is_solenoid_enable = 0;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h00,");

  *is_solenoid_enable = 1;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h01,");
}

// PowerEcuComPdcmdDataEncoder
TEST(PowerEcuComPdcmdDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComPdcmdDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  bool* is_cpu_shutdown = encoder.GetParamPtr<bool>("is_cpu_shutdown");
  bool* is_gpu_shutdown = encoder.GetParamPtr<bool>("is_gpu_shutdown");
  bool* is_ex1_shutdown = encoder.GetParamPtr<bool>("is_ex1_shutdown");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "pdcmd");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 15);

  *is_cpu_shutdown = false;
  *is_gpu_shutdown = false;
  *is_ex1_shutdown = false;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h00,");

  *is_cpu_shutdown = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h01,");

  *is_gpu_shutdown = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h03,");

  *is_ex1_shutdown = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h07,");
}

// PowerEcuComMuteDataEncoder
TEST(PowerEcuComMuteDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComMuteDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  bool* is_amp_mute = encoder.GetParamPtr<bool>("is_amp_mute");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "mute_");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 15);

  *is_amp_mute = false;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h00,");

  *is_amp_mute = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h01,");
}

// PowerEcuComGetvDataEncoder
TEST(PowerEcuComGetvDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComGetvDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "getv_");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 15);

  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "h00,");
}

// PowerEcuComUndckDataEncoder
TEST(PowerEcuComUndckDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComUndckDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "undck");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 11);

  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "");
}

// PowerEcuCom12VuDataEncoder
TEST(PowerEcuCom12VuDataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuCom12VuDataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  bool* is_12vu_enable = encoder.GetParamPtr<bool>("is_12vu_enable");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "12vu_");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 13);

  *is_12vu_enable = false;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "0,");

  *is_12vu_enable = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "1,");
}

// PowerEcuCom5Vd3DataEncoder
TEST(PowerEcuCom5Vd3DataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuCom5Vd3DataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  bool* is_5vd3_enable = encoder.GetParamPtr<bool>("is_5vd3_enable");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "5vd3_");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 13);

  *is_5vd3_enable = false;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "0,");

  *is_5vd3_enable = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "1,");
}

// PowerEcuCom5Vd4DataEncoder
TEST(PowerEcuCom5Vd4DataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuCom5Vd4DataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  bool* is_5vd4_enable = encoder.GetParamPtr<bool>("is_5vd4_enable");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "5vd4_");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 13);

  *is_5vd4_enable = false;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "0,");

  *is_5vd4_enable = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "1,");
}

// PowerEcuCom5Vd5DataEncoder
TEST(PowerEcuCom5Vd5DataEncoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuCom5Vd5DataEncoder encoder;
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  bool* is_5vd5_enable = encoder.GetParamPtr<bool>("is_5vd5_enable");

  EXPECT_STREQ(encoder.GetPacketName().c_str(), "5vd5_");
  EXPECT_EQ(boost::lexical_cast<uint32_t>(encoder.GetPacketSizeStr()), 13);

  *is_5vd5_enable = false;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "0,");

  *is_5vd5_enable = true;
  buffer.clear();
  EXPECT_EQ(encoder.Encode(buffer), boost::system::errc::success);
  EXPECT_STREQ(read_buffer(buffer).c_str(), "1,");
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
