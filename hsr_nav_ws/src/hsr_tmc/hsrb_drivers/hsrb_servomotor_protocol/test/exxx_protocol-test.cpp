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
/// @file  hsr_protocol-test.cpp
/// @brief EXXXPROTOCOL test
#include <string>
#include <vector>

#include <boost/cstdint.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <hsrb_servomotor_protocol/exxx_protocol.hpp>

using hsrb_servomotor_protocol::ExxxProtocol;

// Custom Plain C array machines
// Maybe I don't need the GMOCK version
MATCHER_P2(ArrayEq, value, size, "") {
  bool match = true;
  for (uint32_t i = 0; i < size; ++i) {
    if (arg[i] != value[i]) {
      return false;
    }
  }
  return true;
}

/// MOCK of communication interface with devices
class MockNetwork : public hsrb_servomotor_protocol::INetwork {
 public:
  MOCK_METHOD4(Send, boost::system::error_code(uint8_t, uint8_t, const uint8_t*, uint16_t));
  MOCK_METHOD1(Receive, boost::system::error_code(uint8_t));
  MOCK_METHOD2(Receive, boost::system::error_code(uint8_t, std::vector<uint8_t>&));
  MOCK_CONST_METHOD0(last_packet, const std::vector<uint8_t>&());
};

/// Test -fixer
class ExxxProtocolTest : public ::testing::Test {
 public:
  ExxxProtocolTest() { mock_network_.reset(new MockNetwork()); }

 protected:
  std::shared_ptr<MockNetwork> mock_network_;
};

TEST_F(ExxxProtocolTest, SucceedReadBlock) {
  using ::testing::Return;
  using ::testing::SetArgReferee;
  using ::testing::ElementsAreArray;
  using ::testing::DoAll;
  using ::testing::Pointee;
  using ::testing::Eq;
  using ::testing::_;

  ExxxProtocol protocol(mock_network_);
  uint8_t id = 2;
  uint16_t addr = 0x1234;
  uint16_t size = 8;
  std::vector<uint8_t> data(size);
  std::vector<uint8_t> will_receive(size);

  for (uint8_t i = 0; i < size; ++i) {
    will_receive[i] = i;
  }

  typedef boost::system::error_code ErrorCode;
  ErrorCode success(boost::system::errc::success, boost::system::system_category());
  ErrorCode invalid_arg(boost::system::errc::invalid_argument, boost::system::system_category());
  ::testing::DefaultValue<ErrorCode>::Set(invalid_arg);
  const uint8_t send_data[] = { 0x34, 0x12, 0x08, 0x00 };
  // Send is called
  EXPECT_CALL(*mock_network_, Send(id, hsrb_servomotor_protocol::kInstructionReadData, ArrayEq(send_data, 4), 4))
      .Times(1)
      .WillOnce(Return(success));
  // Receive is also called
  EXPECT_CALL(*mock_network_, Receive(id, _)).Times(1).WillOnce(DoAll(SetArgReferee<1>(will_receive), Return(success)));
  ErrorCode error = protocol.ReadBlock(id, addr, size, &data[0]);
  EXPECT_EQ(will_receive, data);
  EXPECT_EQ(boost::system::errc::success, error.value());
  EXPECT_EQ(boost::system::system_category(), error.category());
}


TEST_F(ExxxProtocolTest, SucceedWriteBlock) {
  using ::testing::Return;
  using ::testing::SetArgReferee;
  using ::testing::ElementsAre;
  using ::testing::DoAll;
  using ::testing::Pointee;
  using ::testing::Eq;
  using ::testing::_;
  ExxxProtocol protocol(mock_network_);
  uint8_t id = 2;
  uint16_t addr = 0x1234;
  uint16_t size = 8;
  std::vector<uint8_t> data(size);

  for (uint8_t i = 0; i < size; ++i) {
    data[i] = i;
  }

  typedef boost::system::error_code ErrorCode;
  ErrorCode success(boost::system::errc::success, boost::system::system_category());
  ErrorCode invalid_arg(boost::system::errc::invalid_argument, boost::system::system_category());
  ::testing::DefaultValue<ErrorCode>::Set(invalid_arg);

  const uint8_t send_data[] = { 0x34, 0x12, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
  // Send is called
  EXPECT_CALL(*mock_network_,
              Send(id, hsrb_servomotor_protocol::kInstructionWriteData, ArrayEq(send_data, size + 2), size + 2))
      .Times(1)
      .WillOnce(Return(success));
  // RECIEVE is also called
  EXPECT_CALL(*mock_network_, Receive(id, _)).Times(1).WillOnce(Return(success));
  ErrorCode error = protocol.WriteBlock(id, addr, size, &data[0]);
  EXPECT_EQ(success, error);
  EXPECT_EQ(boost::system::system_category(), error.category());
}

TEST_F(ExxxProtocolTest, SucceedPing) {
  using ::testing::Return;
  using ::testing::ElementsAreArray;
  using ::testing::_;
  ExxxProtocol protocol(mock_network_);
  uint8_t id = 2;
  typedef boost::system::error_code ErrorCode;
  ErrorCode success(boost::system::errc::success, boost::system::system_category());

  // Send is called
  EXPECT_CALL(*mock_network_, Send(id, hsrb_servomotor_protocol::kInstructionPing, NULL, 0))
      .Times(1)
      .WillOnce(Return(success));
  // RECIEVE is also called
  EXPECT_CALL(*mock_network_, Receive(id, _)).Times(1).WillOnce(Return(success));
  ErrorCode error = protocol.Ping(id);
  EXPECT_EQ(boost::system::errc::success, error.value());
  EXPECT_EQ(boost::system::system_category(), error.category());
}


TEST_F(ExxxProtocolTest, SucceedReset) {
  using ::testing::Return;
  using ::testing::ElementsAreArray;
  using ::testing::_;
  ExxxProtocol protocol(mock_network_);
  uint8_t id = 2;
  typedef boost::system::error_code ErrorCode;
  ErrorCode success(boost::system::errc::success, boost::system::system_category());
  ErrorCode timed_out(boost::system::errc::timed_out, boost::system::system_category());

  std::array<uint8_t, 9> send_data = {0x52, 0x45, 0x53, 0x45, 0x54, 0x00, 0xFF, 0xAA, 0x55};
  // Send is called
  EXPECT_CALL(*mock_network_, Send(id, hsrb_servomotor_protocol::kInstructionReset, ArrayEq(send_data, 9), 9))
      .Times(1)
      .WillOnce(Return(success));
  // RECIEVE is also called
  EXPECT_CALL(*mock_network_, Receive(id, _)).Times(1).WillOnce(Return(timed_out));
  ErrorCode error = protocol.Reset(id);
  EXPECT_EQ(boost::system::errc::success, error.value());
  EXPECT_EQ(boost::system::system_category(), error.category());
}

TEST_F(ExxxProtocolTest, SucceedAvagoAvePos) {
  using ::testing::Return;
  using ::testing::ElementsAreArray;
  using ::testing::_;
  ExxxProtocol protocol(mock_network_);
  uint8_t id = 2;
  typedef boost::system::error_code ErrorCode;
  ErrorCode success(boost::system::errc::success, boost::system::system_category());

  // Send is called
  EXPECT_CALL(*mock_network_, Send(id, hsrb_servomotor_protocol::kInstructionAvagoAvePos, NULL, 0))
      .Times(1)
      .WillOnce(Return(success));
  // RECIEVE is also called
  EXPECT_CALL(*mock_network_, Receive(id, _)).Times(1).WillOnce(Return(success));
  ErrorCode error = protocol.AvagoAvePos(id);
  EXPECT_EQ(boost::system::errc::success, error.value());
  EXPECT_EQ(boost::system::system_category(), error.category());
}

TEST_F(ExxxProtocolTest, SucceedSyncParam) {
  using ::testing::Return;
  using ::testing::ElementsAreArray;
  using ::testing::_;
  ExxxProtocol protocol(mock_network_);
  uint8_t id = 2;
  typedef boost::system::error_code ErrorCode;
  ErrorCode success(boost::system::errc::success, boost::system::system_category());

  // Send is called
  EXPECT_CALL(*mock_network_, Send(id, hsrb_servomotor_protocol::kInstructionSyncParam, NULL, 0))
      .Times(1)
      .WillOnce(Return(success));
  // RECIEVE is also called
  EXPECT_CALL(*mock_network_, Receive(id, _)).Times(1).WillOnce(Return(success));
  ErrorCode error = protocol.SyncParam(id);
  EXPECT_EQ(boost::system::errc::success, error.value());
  EXPECT_EQ(boost::system::system_category(), error.category());
}


TEST_F(ExxxProtocolTest, SucceedWriteEeprom) {
  using ::testing::Return;
  using ::testing::ElementsAreArray;
  using ::testing::_;
  ExxxProtocol protocol(mock_network_);
  uint8_t id = 2;
  typedef boost::system::error_code ErrorCode;
  ErrorCode success(boost::system::errc::success, boost::system::system_category());

  // Send is called
  EXPECT_CALL(*mock_network_, Send(id, hsrb_servomotor_protocol::kInstructionWriteEeprom, NULL, 0))
      .Times(1)
      .WillOnce(Return(success));
  ErrorCode error = protocol.WriteEeprom(id);
  EXPECT_EQ(boost::system::errc::success, error.value());
  EXPECT_EQ(boost::system::system_category(), error.category());
}

TEST_F(ExxxProtocolTest, SucceedReadHash) {
  using ::testing::Return;
  using ::testing::SetArgReferee;
  using ::testing::DoAll;
  using ::testing::_;
  ExxxProtocol protocol(mock_network_);
  uint8_t id = 2;
  const uint32_t kMd5Size = 16;
  const uint32_t kSHA1Size = 20;
  std::vector<uint8_t> firm_hash;
  std::vector<uint8_t> table_hash;
  std::vector<uint8_t> received_data;
  std::vector<uint8_t> will_receive(kMd5Size + kSHA1Size);
  std::vector<uint8_t> table_hash_expected;
  std::vector<uint8_t> firm_hash_expected;

  for (uint8_t i = 0; i < kMd5Size + kSHA1Size; ++i) {
    if (i < kMd5Size) {
      table_hash_expected.push_back(i);
    } else {
      firm_hash_expected.push_back(i);
    }
    will_receive[i] = i;
  }

  typedef boost::system::error_code ErrorCode;
  ErrorCode success(boost::system::errc::success, boost::system::system_category());

  // Send is called
  EXPECT_CALL(*mock_network_, Send(id, hsrb_servomotor_protocol::kInstructionReadHash, NULL, 0))
      .Times(1)
      .WillOnce(Return(success));
  // RECIEVE is also called
  EXPECT_CALL(*mock_network_, Receive(id, _)).Times(1).WillOnce(DoAll(SetArgReferee<1>(will_receive), Return(success)));
  ErrorCode error = protocol.ReadHash(id, table_hash, firm_hash);
  EXPECT_EQ(kMd5Size, table_hash.size());
  EXPECT_EQ(kSHA1Size, firm_hash.size());
  EXPECT_EQ(table_hash_expected, table_hash);
  EXPECT_EQ(firm_hash_expected, firm_hash);
  EXPECT_EQ(boost::system::errc::success, error.value());
  EXPECT_EQ(boost::system::system_category(), error.category());
}


int main(int argc, char** argv) {
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
