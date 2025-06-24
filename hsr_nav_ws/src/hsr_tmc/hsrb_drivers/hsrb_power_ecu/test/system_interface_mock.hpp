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
#ifndef HSRB_POWER_ECU_HSR_SYSTEM_INTERFACE_MOCK_HPP_
#define HSRB_POWER_ECU_HSR_SYSTEM_INTERFACE_MOCK_HPP_

#include <stdint.h>
#include <string>

#include <boost/foreach.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../include/hsrb_power_ecu/serial_network.hpp"
#include "../src/i_system_interface.hpp"
#include "../src/system.hpp"

namespace hsrb_power_ecu {

/**
 * @BRIEF Linux built -in function interface
 */
class SystemInterfaceMock : public ISystemInterface {
 public:
  static int32_t error_code;
  static int32_t ErrorHelper() {
    errno = error_code;
    return -error_code;
  }

 public:
  /**
   * @brief Now
   * @return
   */
  MOCK_CONST_METHOD0(Now, int64_t());

  // #include <termio.h>
  MOCK_METHOD2(Tcgetattr, int32_t(int32_t fd, termios *termios_p));
  MOCK_METHOD3(Tcsetattr, int32_t(int32_t fd, int32_t optional_actions, const termios *termios_p));
  MOCK_METHOD2(Tcflush, int32_t(int32_t fd, int32_t queue_selector));
  // #include <sys/ioctl.h>
  MOCK_METHOD3(Ioctl, int32_t(int32_t fd, uint32_t request, void *p));
  // #include <fcntl.h>
  MOCK_METHOD2(Open, int(const char *file, int oflag));
  // #include <unistd.h>
  MOCK_METHOD1(Close, int32_t(int32_t fd));
  MOCK_METHOD3(Read, ssize_t(int32_t fd, void *buf, size_t nbytes));
  MOCK_METHOD3(Write, ssize_t(int32_t fd, const void *buf, size_t n));
  // #include <time.h>
  MOCK_METHOD4(Clock_nanosleep, int32_t(clockid_t clock_id, int32_t flags, const timespec *req, timespec *rem));
  // #include<poll.h>
  MOCK_METHOD4(Ppoll, int32_t(pollfd *fds, nfds_t nfds, const timespec *timeout, const sigset_t *ss));

  SystemInterfaceMock() {
    ON_CALL(*this, Now()).WillByDefault(::testing::Invoke(&system_, &hsrb_power_ecu::System::Now));
    // Set the default return value of MockMethod to write
    ON_CALL(*this, Tcgetattr(::testing::_, ::testing::_)).WillByDefault(::testing::Return(0));
    ON_CALL(*this, Tcsetattr(::testing::_, ::testing::_, ::testing::_)).WillByDefault(::testing::Return(0));
    ON_CALL(*this, Tcflush(::testing::_, ::testing::_)).WillByDefault(::testing::Return(0));
    ON_CALL(*this, Ioctl(::testing::_, ::testing::_, ::testing::_)).WillByDefault(::testing::Return(0));
    ON_CALL(*this, Open(::testing::_, ::testing::_)).WillByDefault(::testing::Return(1));
    ON_CALL(*this, Close(::testing::_)).WillByDefault(::testing::Return(0));
    ON_CALL(*this, Read(::testing::_, ::testing::_, ::testing::_))
        .WillByDefault(::testing::Invoke(this, &SystemInterfaceMock::ReadHelper));
    ON_CALL(*this, Write(::testing::_, ::testing::_, ::testing::_))
        .WillByDefault(::testing::Invoke(this, &SystemInterfaceMock::WriteHelper));
    ON_CALL(*this, Clock_nanosleep(::testing::_, ::testing::_, ::testing::_, ::testing::_))
        .WillByDefault(::testing::Invoke(&system_, &hsrb_power_ecu::System::Clock_nanosleep));
    ON_CALL(*this, Ppoll(::testing::_, ::testing::_, ::testing::_, ::testing::_)).WillByDefault(::testing::Return(1));
  }
  ssize_t WriteHelper(int32_t fd, const void *buf, size_t n) {
    (void)(fd);   // unused
    (void)(buf);  // unused

    return n;
  }
  ssize_t ReadHelper(int32_t fd, void *buf, size_t nbytes) {
    (void)(fd);  // unused

    char *p = static_cast<char *>(buf);
    std::string sample_data = "01234567890";

    uint32_t count = 0;
    BOOST_FOREACH (char c, sample_data) {
      *p = c;
      ++p;
      ++count;
      if (count == nbytes) {
        break;
      }
    }
    return sample_data.length();
  }

 private:
  hsrb_power_ecu::System system_;
};

int32_t SystemInterfaceMock::error_code = 0;
}  // namespace hsrb_power_ecu

#endif  // HSRB_POWER_ECU_HSR_SYSTEM_INTERFACE_MOCK_HPP_
