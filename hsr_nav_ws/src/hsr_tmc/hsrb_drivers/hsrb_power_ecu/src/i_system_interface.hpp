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
#ifndef HSRB_POWER_ECU_HSR_I_SYSTEM_INTERFACE_HPP_
#define HSRB_POWER_ECU_HSR_I_SYSTEM_INTERFACE_HPP_

#include <stdint.h>
#include <time.h>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termio.h>
#include <unistd.h>

namespace hsrb_power_ecu {

/**
 * @brief Linux built -in function interface
 * Cut out to make it easier to perform the Mock test
 */
class ISystemInterface {
 public:
  /**
   * @brief Now
   * @return
   */
  virtual int64_t Now() const = 0;

  // #include <termio.h>
  virtual int32_t Tcgetattr(int32_t fd, termios *termios_p) = 0;
  virtual int32_t Tcsetattr(int32_t fd, int32_t optional_actions, const termios *termios_p) = 0;
  virtual int32_t Tcflush(int32_t fd, int32_t queue_selector) = 0;
  // #include <sys/ioctl.h>
  virtual int32_t Ioctl(int32_t fd, uint32_t request, void *p) = 0;
  // #include <fcntl.h>
  virtual int Open(const char *file, int oflag) = 0;
  // #include <unistd.h>
  virtual int32_t Close(int32_t fd) = 0;
  virtual ssize_t Read(int32_t fd, void *buf, size_t nbytes) = 0;
  virtual ssize_t Write(int32_t fd, const void *buf, size_t n) = 0;
  // #include <time.h>
  virtual int32_t Clock_nanosleep(clockid_t clock_id, int32_t flags, const timespec *req, timespec *rem) = 0;
  // #include<poll.h>
  virtual int32_t Ppoll(pollfd *fds, nfds_t nfds, const timespec *timeout, const sigset_t *ss) = 0;
};

}  // namespace hsrb_power_ecu

#endif  // HSRB_POWER_ECU_HSR_I_SYSTEM_INTERFACE_HPP_
