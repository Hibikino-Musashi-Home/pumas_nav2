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
#include <algorithm>
#include <string>

#include <boost/make_shared.hpp>
#include <boost/move/move.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/system/error_code.hpp>

#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

#include "../include/hsrb_power_ecu/serial_network.hpp"
#include "common_methods.hpp"
#include "system_interface_mock.hpp"

class SerialNetworkTest : public ::testing::Test {
 public:
  SerialNetworkTest() : mock_(new ::testing::NiceMock<hsrb_power_ecu::SystemInterfaceMock>()), network_(mock_) {}

 public:
  boost::shared_ptr<hsrb_power_ecu::SystemInterfaceMock> mock_;
  hsrb_power_ecu::SerialNetwork network_;
};

TEST_F(SerialNetworkTest, NomalOpen) {
  // Open is called at Open
  EXPECT_CALL(*mock_, Open(::testing::_, ::testing::_)).Times(1);
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
}
TEST_F(SerialNetworkTest, FailureOpen) {
  EXPECT_CALL(*mock_, Open(::testing::_, ::testing::_)).WillOnce(::testing::Return(-1));
  errno = ENOENT;
  EXPECT_EQ(network_.Open(), boost::system::errc::no_such_file_or_directory);
}

TEST_F(SerialNetworkTest, NomalClose) {
  EXPECT_CALL(*mock_, Open(::testing::_, ::testing::_)).Times(1);
  EXPECT_EQ(network_.Open(), boost::system::errc::success);

  // CLOSE is called when it is open
  EXPECT_CALL(*mock_, Close(::testing::_)).Times(1);
  EXPECT_EQ(network_.Close(), boost::system::errc::success);
}

TEST_F(SerialNetworkTest, NomalClose2) {
  // Close is not called when not open
  EXPECT_CALL(*mock_, Close(::testing::_)).Times(0);
  EXPECT_EQ(network_.Close(), boost::system::errc::success);
}

TEST_F(SerialNetworkTest, Configure) {
  EXPECT_EQ(network_.Configure("receive_timeout_ms", "10"), boost::system::errc::success);
  EXPECT_EQ(network_.Configure("receive_timeout_ms", 10.0), boost::system::errc::success);
  EXPECT_EQ(network_.Configure("receive_timeout_ms", 10), boost::system::errc::success);
  EXPECT_EQ(network_.Configure("receive_timeout_ms", -1), boost::system::errc::invalid_argument);
  EXPECT_EQ(network_.Configure("device_name", "test"), boost::system::errc::success);
  EXPECT_EQ(network_.Configure("invalid", ""), boost::system::errc::invalid_argument);
  EXPECT_CALL(*mock_, Open(::testing::_, ::testing::_)).Times(1);
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
  EXPECT_EQ(network_.Configure("device_name", "test"), boost::system::errc::operation_in_progress);
}

TEST_F(SerialNetworkTest, NomalSend) {
  EXPECT_EQ(network_.Open(), boost::system::errc::success);

  std::string str = "test";
  hsrb_power_ecu::PacketBuffer buffer(1000);
  write_buffer(str, buffer);
  EXPECT_EQ(network_.Send(buffer), boost::system::errc::success);
}

TEST_F(SerialNetworkTest, FailureSend_SendBigData) {
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
  // Data larger than the export buffer
  hsrb_power_ecu::PacketBuffer buffer(5000);
  for (size_t i = 0; i < buffer.capacity(); ++i) {
    buffer.push_back('0');
  }
  EXPECT_EQ(network_.Send(buffer), boost::system::errc::invalid_argument);
}

TEST_F(SerialNetworkTest, FailureSend_Busy) {
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
  // Writing VII (VII canceled on the way)
  EXPECT_CALL(*mock_, Write(testing::_, testing::_, testing::_))
      .WillOnce(::testing::Return(-1))
      .WillRepeatedly(::testing::Return(4));
  EXPECT_CALL(*mock_, Clock_nanosleep(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .WillRepeatedly(::testing::Return(0));
  std::string str = "test";
  hsrb_power_ecu::PacketBuffer buffer(1000);
  write_buffer(str, buffer);
  errno = EAGAIN;
  EXPECT_EQ(network_.Send(buffer), boost::system::errc::success);
}

TEST_F(SerialNetworkTest, FailureSend_Clock_Nanosleep) {
  // CLOCK_NANOSLEEP failed
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
  EXPECT_CALL(*mock_, Write(testing::_, testing::_, testing::_)).WillOnce(::testing::Return(-1));
  hsrb_power_ecu::SystemInterfaceMock::error_code = EFAULT;
  EXPECT_CALL(*mock_, Clock_nanosleep(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .WillOnce(testing::InvokeWithoutArgs(&hsrb_power_ecu::SystemInterfaceMock::ErrorHelper));
  std::string str = "test";
  hsrb_power_ecu::PacketBuffer buffer(1000);
  write_buffer(str, buffer);
  errno = EAGAIN;
  EXPECT_EQ(network_.Send(buffer), boost::system::errc::bad_address);
}

TEST_F(SerialNetworkTest, FailureSend_Timeout) {
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
  // timeout
  EXPECT_CALL(*mock_, Write(testing::_, testing::_, testing::_)).WillRepeatedly(::testing::Return(-1));
  std::string str = "test";
  hsrb_power_ecu::PacketBuffer buffer(1000);
  write_buffer(str, buffer);
  errno = EAGAIN;
  EXPECT_EQ(network_.Send(buffer), boost::system::errc::timed_out);
}

TEST_F(SerialNetworkTest, FailureSend_WriteError) {
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
  // Other errors
  EXPECT_CALL(*mock_, Write(testing::_, testing::_, testing::_))
      .WillOnce(::testing::Return(-1))
      .WillRepeatedly(::testing::Invoke(mock_.get(), &hsrb_power_ecu::SystemInterfaceMock::WriteHelper));
  std::string str = "test";
  hsrb_power_ecu::PacketBuffer buffer(1000);
  write_buffer(str, buffer);
  errno = EPERM;
  EXPECT_EQ(network_.Send(buffer), boost::system::errc::operation_not_permitted);
}

TEST_F(SerialNetworkTest, NomalReceive) {
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
  hsrb_power_ecu::PacketBuffer buffer(1000);
  EXPECT_EQ(network_.Receive(buffer), boost::system::errc::success);
}

TEST_F(SerialNetworkTest, FailureReceive_TimeOut) {
  // timeout
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
  hsrb_power_ecu::PacketBuffer buffer(1000);
  EXPECT_CALL(*mock_, Ppoll(testing::_, testing::_, testing::_, testing::_)).WillRepeatedly(testing::Return(0));
  EXPECT_EQ(network_.Receive(buffer), boost::system::errc::timed_out);
}

TEST_F(SerialNetworkTest, FailureReceive_PpollError) {
  // ppoll error
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
  hsrb_power_ecu::PacketBuffer buffer(1000);
  errno = EPERM;
  EXPECT_CALL(*mock_, Ppoll(testing::_, testing::_, testing::_, testing::_)).WillRepeatedly(testing::Return(-1));
  EXPECT_EQ(network_.Receive(buffer), boost::system::errc::operation_not_permitted);
}

TEST_F(SerialNetworkTest, FailureReceive_ReadError) {
  // read error
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
  hsrb_power_ecu::PacketBuffer buffer(1000);
  errno = EPERM;
  EXPECT_CALL(*mock_, Read(testing::_, testing::_, testing::_)).WillRepeatedly(testing::Return(-1));
  EXPECT_EQ(network_.Receive(buffer), boost::system::errc::operation_not_permitted);
}

TEST_F(SerialNetworkTest, FailureReceive_BufferFull) {
  // buffer full
  EXPECT_EQ(network_.Open(), boost::system::errc::success);
  hsrb_power_ecu::PacketBuffer buffer(3);
  EXPECT_EQ(network_.Receive(buffer), boost::system::errc::success);
}

int main(int32_t argc, char* argv[]) {
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int32_t ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
