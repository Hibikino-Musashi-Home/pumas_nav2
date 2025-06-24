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
#include <vector>
#include <boost/cstdint.hpp>
#include <gtest/gtest.h>
#include <hsrb_servomotor_protocol/exxx_packet_parser.hpp>

using hsrb_servomotor_protocol::Checksum;
using hsrb_servomotor_protocol::ExxxPacketParser;

TEST(ExxxPacketParser, ParserErrorHeader2) {
  {
    std::array<uint8_t, 7> packet = {0xAAU, 0xFFU, 0x01U, 0x03U, 0x00U, 0x20U, 0x00U};

    std::array<uint8_t, 3> ids = {1};

    ExxxPacketParser parser;
    std::array<uint8_t, 18>::const_iterator it = packet.begin();

    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kError, parser.TryParse(*it++));
  }
}

TEST(ExxxPacketParser, ParserErrorInvalidID) {
  {
    std::array<uint8_t, 7> packet = {0xAAU, 0x55U, 0x02U, 0x03U, 0x00U, 0x20U, 0x00U};

    std::array<uint8_t, 3> ids = {1};

    ExxxPacketParser parser;
    std::array<uint8_t, 18>::const_iterator it = packet.begin();

    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kError, parser.TryParse(*it++));
  }
}


TEST(ExxxPacketParser, ParserErrorInvalidLength) {
  {
    std::array<uint8_t, 7> packet = {0xAAU, 0x55U, 0x01U, 0x01U, 0x00U, 0x20U, 0x00U};

    ExxxPacketParser parser;
    parser.AddID(1);
    std::array<uint8_t, 18>::const_iterator it = packet.begin();

    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kError, parser.TryParse(*it++));
  }
  {
    std::array<uint8_t, 7> packet = {0xAAU, 0x55U, 0x01U, 0x00U, 0x00U, 0x20U, 0x00U};


    ExxxPacketParser parser;
    parser.AddID(1);
    std::array<uint8_t, 18>::const_iterator it = packet.begin();

    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kError, parser.TryParse(*it++));
  }
}


TEST(ExxxPacketParser, ParserErrorChecksum) {
  {
    std::array<uint8_t, 9> packet = {0xAAU, 0x55U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x20U, 0x00U};


    ExxxPacketParser parser;
    parser.AddID(1);
    std::array<uint8_t, 18>::const_iterator it = packet.begin();

    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
    EXPECT_EQ(ExxxPacketParser::kError, parser.TryParse(*it++));
  }
}


TEST(ExxxPacketParser, ParseStatusPackets) {
  std::vector<std::vector<uint8_t> > packets;

  packets.resize(packets.size() + 1);
  packets.back().push_back(0xAAU);
  packets.back().push_back(0x55U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0x03U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0xFCU);

  packets.resize(packets.size() + 1);
  packets.back().push_back(0xAAU);
  packets.back().push_back(0x55U);
  packets.back().push_back(0x01U);
  packets.back().push_back(0x03U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0xFBU);

  packets.resize(packets.size() + 1);
  packets.back().push_back(0xAAU);
  packets.back().push_back(0x55U);
  packets.back().push_back(0x01U);
  packets.back().push_back(0x06U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0x01U);
  packets.back().push_back(0x02U);
  packets.back().push_back(0x03U);
  packets.back().push_back(0xF2U);

  packets.resize(packets.size() + 1);
  packets.back().push_back(0xAAU);
  packets.back().push_back(0x55U);
  packets.back().push_back(0x01U);
  packets.back().push_back(0x04U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0x00U);
  packets.back().push_back(0x20U);
  packets.back().push_back(0xDAU);


  ExxxPacketParser parser;
  parser.AddID(0);
  parser.AddID(1);
  std::vector<std::vector<uint8_t> >::iterator it = packets.begin();
  std::vector<std::vector<uint8_t> >::iterator end = packets.end();
  for (; it != end; ++it) {
    for (std::vector<uint8_t>::iterator buf_iter = it->begin(); buf_iter != it->end(); ++buf_iter) {
      if (std::distance(buf_iter, it->end()) > 1) {
        EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*buf_iter));
      } else {
        EXPECT_EQ(ExxxPacketParser::kDone, parser.TryParse(*buf_iter));
        parser.Reset();
      }
    }
  }
}


TEST(ExxxPacketParser, ParseNoisyPacket) {
  std::array<uint8_t, 23> packet = {
      0xAAU, 0x66U, 0x02U, 0x10U, 0x00U,
      0xAAU, 0x55U, 0x00U, 0x03U, 0x00U, 0x00U, 0x00U, 0xFCU,
      0xAAU, 0x00U,
      0xAAU, 0x55U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0xFBU};

  ExxxPacketParser parser;
  parser.AddID(0);
  parser.AddID(1);

  std::array<uint8_t, 23>::const_iterator it = packet.begin();

  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kError, parser.TryParse(*it++));
  parser.Reset();
  EXPECT_EQ(ExxxPacketParser::kError, parser.TryParse(*it++));
  parser.Reset();
  EXPECT_EQ(ExxxPacketParser::kError, parser.TryParse(*it++));
  parser.Reset();
  EXPECT_EQ(ExxxPacketParser::kError, parser.TryParse(*it++));
  parser.Reset();

  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kDone, parser.TryParse(*it++));
  parser.Reset();

  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kError, parser.TryParse(*it++));
  parser.Reset();

  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kDone, parser.TryParse(*it++));
  parser.Reset();
}


TEST(ExxxPacketParser, ParseSequencially) {
  std::array<uint8_t, 24> packet = {
      0xAAU, 0x55U, 0x64U, 0x03U, 0x00U, 0x10U, 0x00U, 0x00U,
      0xAAU, 0x55U, 0x65U, 0x03U, 0x00U, 0x10U, 0x00U, 0x00U,
      0xAAU, 0x55U, 0x66U, 0x03U, 0x00U, 0x10U, 0x00U, 0x00U};

  packet[7] = Checksum(&packet[2], &packet[6]);
  packet[15] = Checksum(&packet[10], &packet[14]);
  packet[23] = Checksum(&packet[18], &packet[22]);

  std::array<uint8_t, 3> ids = {100, 101, 102};

  ExxxPacketParser parser;
  parser.AddID(100);
  parser.AddID(101);
  parser.AddID(102);

  std::array<uint8_t, 12>::const_iterator it = packet.begin();

  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kDone, parser.TryParse(*it++));
  parser.Reset();

  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kDone, parser.TryParse(*it++));
  parser.Reset();

  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kContinue, parser.TryParse(*it++));
  EXPECT_EQ(ExxxPacketParser::kDone, parser.TryParse(*it++));
  parser.Reset();
}


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
