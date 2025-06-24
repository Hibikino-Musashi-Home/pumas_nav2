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
#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>

#include <hsrb_power_ecu/i_network.hpp>

#include "../src/power_ecu_com_data_decoder.hpp"
#include "../src/power_ecu_com_frame_decoder.hpp"
#include "common_methods.hpp"
namespace {
const size_t kBufferSize = 1000;
}  // anonymous namespace

class FrameDecoderTest : public ::testing::Test {
 public:
  FrameDecoderTest() : data_decoder_(new hsrb_power_ecu::PowerEcuComRxackDataDecoder()), buffer_(kBufferSize) {
    EXPECT_EQ(decoder_.RegisterDataDecoder(data_decoder_), boost::system::errc::success);
  }

 protected:
  boost::shared_ptr<hsrb_power_ecu::PowerEcuComRxackDataDecoder> data_decoder_;
  hsrb_power_ecu::PowerEcuComFrameDecoder decoder_;
  hsrb_power_ecu::PacketBuffer buffer_;

  void FailureDecodeTestHelper(const std::string& correct_packet_string, const size_t pos, const std::string& err_str) {
    FailureDecodeTestHelper(correct_packet_string, pos, err_str, pos + err_str.length());
  }
  void FailureDecodeTestHelper(const std::string& correct_packet_string, const size_t pos, const std::string& err_str,
                               size_t encoded_index) {
    SCOPED_TRACE(err_str.c_str());
    size_t length = err_str.length();
    std::string bad_packet_string = correct_packet_string;
    bad_packet_string.replace(pos, length, err_str);
    std::cout << bad_packet_string << std::endl;

    write_buffer(bad_packet_string, buffer_);
    hsrb_power_ecu::PacketBuffer::const_iterator it = buffer_.begin();
    EXPECT_EQ(decoder_.Decode(buffer_.begin(), buffer_.end(), it), boost::system::errc::protocol_error);
    EXPECT_EQ(it, buffer_.begin() + encoded_index);
  }
};

TEST_F(FrameDecoderTest, NomalEncode) {
  std::string packet_string = "";
  {
    SCOPED_TRACE("Not received");
    packet_string = "testtesttesttesttesttest";

    write_buffer(packet_string, buffer_);
    hsrb_power_ecu::PacketBuffer::const_iterator it = buffer_.begin();
    EXPECT_EQ(decoder_.Decode(buffer_.begin(), buffer_.end(), it), boost::system::errc::result_out_of_range);
    EXPECT_EQ(it, buffer_.end());
  }

  {
    SCOPED_TRACE("Receiving");
    buffer_.clear();
    {
      std::stringstream sst;

      sst << "E," << data_decoder_->GetPacketName() << ",";

      packet_string = sst.str();
    }

    write_buffer(packet_string, buffer_);
    hsrb_power_ecu::PacketBuffer::const_iterator it = buffer_.begin();
    EXPECT_EQ(decoder_.Decode(buffer_.begin(), buffer_.end(), it), boost::system::errc::result_out_of_range);
    EXPECT_EQ(it, buffer_.begin());

    write_buffer("1234" + packet_string, buffer_);
    it = buffer_.begin();
    EXPECT_EQ(decoder_.Decode(buffer_.begin(), buffer_.end(), it), boost::system::errc::result_out_of_range);
    EXPECT_EQ(it, buffer_.begin() + 4);
  }

  {
    SCOPED_TRACE("Receiving");
    buffer_.clear();
    {
      std::stringstream sst;

      sst << std::setw(3) << std::setfill('0') << data_decoder_->GetPacketSize() << ",";
      sst << "h00,";

      packet_string += sst.str();
    }

    write_buffer(packet_string, buffer_);
    hsrb_power_ecu::PacketBuffer::const_iterator it = buffer_.begin();
    EXPECT_EQ(decoder_.Decode(buffer_.begin(), buffer_.end(), it), boost::system::errc::result_out_of_range);
    EXPECT_EQ(it, buffer_.begin());

    write_buffer("1234" + packet_string, buffer_);
    it = buffer_.begin();
    EXPECT_EQ(decoder_.Decode(buffer_.begin(), buffer_.end(), it), boost::system::errc::result_out_of_range);
    EXPECT_EQ(it, buffer_.begin() + 4);
  }

  {
    SCOPED_TRACE("Received");
    {
      std::stringstream sst;
      hsrb_power_ecu::PacketBuffer tmp_buffer(kBufferSize);
      write_buffer(packet_string, tmp_buffer);
      const uint32_t crc = hsrb_power_ecu::com_common::CalculateCrc32(tmp_buffer.begin(), tmp_buffer.end());
      sst << "h" << std::hex << std::uppercase << crc << ",\n";
      packet_string += sst.str();
    }

    write_buffer(packet_string, buffer_);
    hsrb_power_ecu::PacketBuffer::const_iterator it = buffer_.begin();
    EXPECT_EQ(decoder_.Decode(buffer_.begin(), buffer_.end(), it), boost::system::errc::success);
    EXPECT_EQ(it, buffer_.end());

    write_buffer("1234" + packet_string, buffer_);
    it = buffer_.begin();
    EXPECT_EQ(decoder_.Decode(buffer_.begin(), buffer_.end(), it), boost::system::errc::success);
    EXPECT_EQ(it, buffer_.end());

    write_buffer(packet_string + "1233", buffer_);
    it = buffer_.begin();
    EXPECT_EQ(decoder_.Decode(buffer_.begin(), buffer_.end(), it), boost::system::errc::success);
    EXPECT_EQ(it, buffer_.end() - 4);

    write_buffer("1234" + packet_string + "1234", buffer_);
    it = buffer_.begin();
    EXPECT_EQ(decoder_.Decode(buffer_.begin(), buffer_.end(), it), boost::system::errc::success);
    EXPECT_EQ(it, buffer_.end() - 4);
  }
  std::cout << packet_string << std::endl;
}

TEST_F(FrameDecoderTest, FailureDecode) {
  const std::string correct_packet_string = "E,rxack,015,h00,h83693205,\n";
  this->FailureDecodeTestHelper(correct_packet_string, 0, "E.");

  std::string tmp_str = "E,start,015,h00,";
  hsrb_power_ecu::PacketBuffer tmp_buf(kBufferSize);
  write_buffer(tmp_str, tmp_buf);
  uint32_t crc = hsrb_power_ecu::com_common::CalculateCrc32(tmp_buf.begin(), tmp_buf.end());
  std::stringstream sst;
  sst << tmp_str << "h" << std::hex << std::uppercase << crc << ",\n";
  tmp_str = sst.str();
  this->FailureDecodeTestHelper(tmp_str, 2, "start", tmp_str.length());

  this->FailureDecodeTestHelper(correct_packet_string, 7, ".");
  this->FailureDecodeTestHelper(correct_packet_string, 8, "fff");
  this->FailureDecodeTestHelper(correct_packet_string, 11, ".");
  this->FailureDecodeTestHelper(correct_packet_string, 16, "h00000000", correct_packet_string.length());
}

TEST_F(FrameDecoderTest, FailureRegisterSameDataDecoder) {
  EXPECT_EQ(decoder_.RegisterDataDecoder(data_decoder_), boost::system::errc::invalid_argument);
}

TEST_F(FrameDecoderTest, FailureRegisterNullPtr) {
  boost::shared_ptr<hsrb_power_ecu::PowerEcuComEcu1DataDecoder> ecu1;
  ecu1.reset();  // NULL
  EXPECT_EQ(decoder_.RegisterDataDecoder(ecu1), boost::system::errc::invalid_argument);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
