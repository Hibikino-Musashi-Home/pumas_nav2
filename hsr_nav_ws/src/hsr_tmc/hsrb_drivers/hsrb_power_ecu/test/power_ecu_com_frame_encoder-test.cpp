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
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>

#include "../src/power_ecu_com_data_encoder.hpp"
#include "../src/power_ecu_com_frame_encoder.hpp"
#include "common_methods.hpp"
namespace {
const size_t kBufferSize = 1000;
}  // anonymous namespace

class FrameEncoderTest : public ::testing::Test {
 public:
  FrameEncoderTest() : buffer_(kBufferSize) {
    // For Packet comparison, use STOP packets without parameters for testing
    stop_ = boost::make_shared<hsrb_power_ecu::PowerEcuComStopDataEncoder>();
    EXPECT_EQ(encoder_.RegisterDataEncoder(stop_), boost::system::errc::success);
    buffer_.clear();
  }

 protected:
  hsrb_power_ecu::PowerEcuComFrameEncoder encoder_;
  boost::shared_ptr<hsrb_power_ecu::PowerEcuComStopDataEncoder> stop_;
  hsrb_power_ecu::PacketBuffer buffer_;
};

TEST_F(FrameEncoderTest, NomalCase) {
  EXPECT_EQ(encoder_.Encode(buffer_, stop_->GetPacketName()), boost::system::errc::success);
  EXPECT_NE(buffer_.size(), 0);

  std::string packet_string = "";
  {
    std::stringstream sst;
    hsrb_power_ecu::PacketBuffer tmp_buffer(kBufferSize);

    sst << "H," << stop_->GetPacketName() << "," << stop_->GetPacketSizeStr() << ",";
    write_buffer(sst.str(), tmp_buffer);
    const uint32_t crc = hsrb_power_ecu::com_common::CalculateCrc32(tmp_buffer.begin(), tmp_buffer.end());
    sst << "h" << std::hex << std::uppercase << crc << ",\n";
    packet_string = sst.str();
  }

  EXPECT_STREQ(read_buffer(buffer_).c_str(), packet_string.c_str());
}

TEST_F(FrameEncoderTest, FailureEncodeNoRegisterDataEncoder) {
  write_buffer("test", buffer_);
  EXPECT_EQ(encoder_.Encode(buffer_, "start"), boost::system::errc::invalid_argument);
  EXPECT_STREQ(read_buffer(buffer_).c_str(), "test");
}

TEST_F(FrameEncoderTest, FailureRegisterSameDataEncoder) {
  EXPECT_EQ(encoder_.RegisterDataEncoder(stop_), boost::system::errc::invalid_argument);
}

TEST_F(FrameEncoderTest, FailureRegisterNullPtr) {
  boost::shared_ptr<hsrb_power_ecu::PowerEcuComTimeDataEncoder> time;
  time.reset();  // NULL
  EXPECT_EQ(encoder_.RegisterDataEncoder(time), boost::system::errc::invalid_argument);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
