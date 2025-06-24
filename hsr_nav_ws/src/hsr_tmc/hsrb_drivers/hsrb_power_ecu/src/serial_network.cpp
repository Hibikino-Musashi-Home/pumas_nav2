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

#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/utsname.h>
#include <termio.h>
#include <unistd.h>

#include <hsrb_power_ecu/serial_network.hpp>
#include <rclcpp/rclcpp.hpp>

#include "i_system_interface.hpp"
#include "system.hpp"

namespace {
const char kPortName[] = "/dev/ttyUSB0";
const size_t kBufferSize = 4 * 1000;
const uint32_t kTimeoutNanoSec = 300000;
const int32_t kSleepTickNanoSec = 10000;

}  // anonymous namespace

namespace hsrb_power_ecu {

SerialNetwork::SerialNetwork()
    : fd_(0),
      receive_buffer_(kBufferSize),
      send_buffer_(kBufferSize),
      timeout_ns_(kTimeoutNanoSec),
      sleep_tick_(kSleepTickNanoSec),
      port_name_(kPortName),
      system_(boost::make_shared<System>()) {}
SerialNetwork::SerialNetwork(boost::shared_ptr<ISystemInterface> system)
    : fd_(0),
      receive_buffer_(kBufferSize),
      send_buffer_(kBufferSize),
      timeout_ns_(kTimeoutNanoSec),
      sleep_tick_(kSleepTickNanoSec),
      port_name_(kPortName),
      system_(system) {}
SerialNetwork::~SerialNetwork() {}

/**
 * @brief Open
 */
boost::system::error_code SerialNetwork::Open() {
  int32_t port = system_->Open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (port < 0) {
    return boost::system::error_code(errno, boost::system::system_category());
  }
  fd_ = port;

  termios term;
  if (system_->Tcgetattr(fd_, &term)) {
    return boost::system::error_code(errno, boost::system::system_category());
  }

  // 3Mbps
  // 8bit
  // stop bit 1
  // no parity
  // no modem control
  // enable receiving characters0
  term.c_iflag = IGNPAR;
  term.c_cflag = B3000000 | CS8 | CLOCAL | CREAD;
  term.c_oflag = OPOST;
  term.c_lflag = 0;
  term.c_cc[VTIME] = 0;
  term.c_cc[VMIN] = 1;
  if (system_->Tcsetattr(fd_, TCSANOW, &term)) {
    return boost::system::error_code(errno, boost::system::system_category());
  }

  // Set Low Latency (no settings required for Kernel5.4)
  struct utsname utsname;
  // Confirm whether the first three characters in the Kernel version are 4.4
  if ((uname(&utsname) == 0) && (std::string(utsname.release).compare(0, 3, "4.4") == 0)) {
    serial_struct serial;
    if (system_->Ioctl(fd_, TIOCGSERIAL, &serial)) {
      return boost::system::error_code(errno, boost::system::system_category());
    }
    serial.flags |= ASYNC_LOW_LATENCY;
    if (system_->Ioctl(fd_, TIOCSSERIAL, &serial)) {
      return boost::system::error_code(errno, boost::system::system_category());
    }
  }

  // Flushing port
  if (system_->Tcflush(fd_, TCIOFLUSH)) {
    return boost::system::error_code(errno, boost::system::system_category());
  }
  return boost::system::errc::make_error_code(boost::system::errc::success);
}

/**
 * @brief Close
 */
boost::system::error_code SerialNetwork::Close() {
  if (fd_ != 0) {
    (void)system_->Close(fd_);
    fd_ = 0;
  }
  return boost::system::errc::make_error_code(boost::system::errc::success);
}

/**
 * @brief network setting change
 */
boost::system::error_code SerialNetwork::Configure(const std::string &param, const int32_t value) {
  return Configure(param, boost::lexical_cast<std::string>(value));
}

/**
 * @brief network setting change
 */
boost::system::error_code SerialNetwork::Configure(const std::string &param, const double value) {
  return Configure(param, boost::lexical_cast<std::string>(value));
}

/**
 * @brief network setting change
 */
boost::system::error_code SerialNetwork::Configure(const std::string &param, const std::string &value) {
  if (fd_ != 0) {
    return boost::system::errc::make_error_code(boost::system::errc::operation_in_progress);
  }
  if (param == "receive_timeout_ms") {
    // timeout
    double const timeout = boost::lexical_cast<double>(value);
    if (timeout < 0) {
      return boost::system::errc::make_error_code(boost::system::errc::invalid_argument);
    }
    timeout_ns_ = static_cast<uint32_t>(timeout * 1000000.0);
  } else if (param == "device_name") {
    // Device name
    port_name_ = value;
  } else {
    return boost::system::errc::make_error_code(boost::system::errc::invalid_argument);
  }
  return boost::system::errc::make_error_code(boost::system::errc::success);
}

/**
 * @brief transmission
 */
boost::system::error_code SerialNetwork::Send(const PacketBuffer &data) {
  size_t const length = data.size();
  if (length > send_buffer_.size()) {
    return boost::system::error_code(boost::system::errc::invalid_argument, boost::system::system_category());
  }

  // Copy data to the transmission buffer
  std::copy(data.begin(), data.end(), send_buffer_.begin());

  size_t send_size = 0;
  int64_t const start = system_->Now();
  int64_t elapsed = start;
  while ((elapsed - start) < timeout_ns_) {
    // Try all data remaining in the transmission buffer
    ssize_t result = system_->Write(fd_, &send_buffer_.front() + send_size, length - send_size);
    if (result < 0) {
      if (errno == EAGAIN) {
        // In the event of a failure, if the device is in a vision state, retry the transmission within the timeout tolerance time.
        // wait for sleep_tick_ nanoseconds
        timespec duration = {0, sleep_tick_};
        while (system_->Clock_nanosleep(CLOCK_MONOTONIC, 0, &duration, &duration)) {
          if (errno == EINTR) {
            // TODO(kitsunai): テスト未実施
            continue;
          } else {
            return boost::system::error_code(errno, boost::system::system_category());
          }
        }
      } else {
        // Failure if other errors
        return boost::system::error_code(errno, boost::system::system_category());
      }
    } else {
      // If the data is left in the transmission buffer, re -try to send until the buffer becomes empty
      send_size += result;
      if (send_size == length) {
        return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
      } else if (send_size > length) {
        RCLCPP_FATAL(rclcpp::get_logger("serial_network"), "NOT REACHED");
      }
    }
    elapsed = system_->Now();
  }
  return boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
}

/**
 * @brief reception
 */
boost::system::error_code SerialNetwork::Receive(PacketBuffer &data) {
  int64_t const start = system_->Now();
  int64_t elapsed = start;
  pollfd poll_fd[1];
  timespec poll_timeout;

  poll_fd[0].fd = fd_;
  poll_fd[0].events = POLLIN | POLLPRI;

  while ((elapsed - start) < timeout_ns_) {
    // Wait until the received data comes
    int64_t remain_timeout = timeout_ns_ - (elapsed - start);
    poll_timeout.tv_sec = remain_timeout / 1000000000LL;
    poll_timeout.tv_nsec = remain_timeout % 1000000000LL;
    int32_t ready = system_->Ppoll(&poll_fd[0], 1, &poll_timeout, NULL);
    if (ready == 0) {
      // timeout
      break;
    } else if (ready < 0) {
      return boost::system::error_code(errno, boost::system::system_category());
    }
    // Reading
    uint32_t const limit_size = receive_buffer_.size() < data.reserve() ? receive_buffer_.size() : data.reserve();
    ssize_t const result = system_->Read(fd_, &receive_buffer_[0], limit_size);
    if (result < 0) {
      return boost::system::error_code(errno, boost::system::system_category());
    } else {
      for (ssize_t i = 0; i < result; i++) {
        if (data.full()) {
          // There are leftovers, but it is normal because it can be read normally until the buffer is full.
          return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
        }
        data.push_back(receive_buffer_[i]);
      }
      return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
    }
    elapsed = system_->Now();
  }
  return boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
}
}  // namespace hsrb_power_ecu
