/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
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
#include <iomanip>
#include <string>
#include <vector>

#include <sys/utsname.h>

#include <hsrb_servomotor_protocol/exxx_network.hpp>
#include <poll.h>

namespace {
// packetでのバイト数
const size_t kHeaderBytes = 2;
const size_t kIdBytes = 1;
const size_t kInstructionBytes = 1;
const size_t kLengthBytes = 2;
const size_t kErrorBytes = 2;
const size_t kChecksumBytes = 1;
const size_t kFooterBytes = 2;
// status packetでの項目の開始位置
const size_t kIDStartStatus = 2;
const size_t kLengthStartStatus = 3;
const size_t kErrorStartStatus = 5;
const size_t kParameterStartStatus = 7;
}  // anonymous namespace

namespace hsrb_servomotor_protocol {

ExxxNetwork::ExxxNetwork(std::string device_name, boost::system::error_code& error_out)
    : fd_(-1), timeout_(300000), sleep_tick_(10000), parser_(), buffer_(),
      logger_(rclcpp::get_logger("exxx_network")) {
  Init(device_name, error_out, true);
}

ExxxNetwork::ExxxNetwork(std::string device_name, boost::system::error_code& error_out, bool is_usb_rs485)
    : fd_(-1), timeout_(300000), sleep_tick_(10000), parser_(), buffer_(),
      logger_(rclcpp::get_logger("exxx_network")) {
  Init(device_name, error_out, is_usb_rs485);
}

ExxxNetwork::ExxxNetwork(std::string device_name, boost::system::error_code& error_out, bool is_usb_rs485,
                         int32_t timeout, int32_t sleep_tick)
    : fd_(-1), timeout_(timeout), sleep_tick_(sleep_tick), parser_(), buffer_(),
      logger_(rclcpp::get_logger("exxx_network")) {
  Init(device_name, error_out, is_usb_rs485);
}

void ExxxNetwork::Init(std::string device_name, boost::system::error_code& error_out, bool is_usb_rs485) {
  int port = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (port < 0) {
    error_out = boost::system::error_code(errno, boost::system::system_category());
    return;
  }
  fd_ = port;

  // Set raw mode
  termios term = { 0 };
  if (tcgetattr(fd_, &term)) {
    error_out = boost::system::error_code(errno, boost::system::system_category());
    return;
  }
  if (!is_usb_rs485) {
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
    if (tcsetattr(fd_, TCSANOW, &term)) {
      error_out = boost::system::error_code(errno, boost::system::system_category());
      return;
    }

    // set low latency (Kernel5.4では設定不要)
    struct utsname utsname;
    // kernelバージョンの先頭3文字が4.4か確認
    if ((uname(&utsname) == 0) && (std::string(utsname.release).compare(0, 3, "4.4") == 0)) {
      serial_struct serial = { 0 };
      if (ioctl(fd_, TIOCGSERIAL, &serial)) {
        error_out = boost::system::error_code(errno, boost::system::system_category());
        return;
      }
      serial.flags |= ASYNC_LOW_LATENCY;
      if (ioctl(fd_, TIOCSSERIAL, &serial)) {
        error_out = boost::system::error_code(errno, boost::system::system_category());
        return;
      }
    }

    // Flushing port
    if (tcflush(fd_, TCIOFLUSH)) {
      error_out = boost::system::error_code(errno, boost::system::system_category());
      return;
    }

    // RS485 Half-Duplexモードをセット
    int value = 0;
    if (ioctl(fd_, hsrb_servomotor_protocol::kCti485GetMode, &value) == -1) {
      error_out = boost::system::error_code(errno, boost::system::system_category());
      return;
    }
    if (value != hsrb_servomotor_protocol::kCti485HalfDuplexMode) {
      value = hsrb_servomotor_protocol::kCti485HalfDuplexMode;
      if (ioctl(fd_, hsrb_servomotor_protocol::kCti485SetMode, &value) < 0) {
        error_out = boost::system::error_code(errno, boost::system::system_category());
        return;
      }
    }
  } else {
    cfmakeraw(&term);
    if (cfsetispeed(&term, B3000000)) {
      error_out = boost::system::error_code(errno, boost::system::system_category());
      return;
    }
    if (tcsetattr(fd_, TCSANOW, &term)) {
      error_out = boost::system::error_code(errno, boost::system::system_category());
      return;
    }
  }
}

ExxxNetwork::~ExxxNetwork() { (void)close(fd_); }

/// Send
boost::system::error_code ExxxNetwork::Send(uint8_t id, uint8_t inst, const uint8_t* data, uint16_t data_bytes) {
  // header_bytes + id_bytes + legth_bytes + instruction_bytes +
  // + data_bytes + checksum_bytes + footer_bytes
  size_t length =
      kHeaderBytes + kIdBytes + kInstructionBytes + kLengthBytes + data_bytes + kChecksumBytes + kFooterBytes;
  buffer_[0] = kHeader1;
  buffer_[1] = kHeader2;
  buffer_[2] = id;
  // data_bytes + kInstractionBytes + kChecksumBytes の下位byte
  *reinterpret_cast<uint16_t*>(&buffer_[3]) = data_bytes + kInstructionBytes + kChecksumBytes;
  buffer_[5] = inst;
  for (int i = 0; i < data_bytes; ++i) {
    buffer_[i + 6] = data[i];
  }
  size_t header_to_data = kHeaderBytes + kIdBytes + kInstructionBytes + kLengthBytes + data_bytes;
  buffer_[header_to_data] = Checksum(buffer_.begin() + 2, buffer_.begin() + (length - 3));
  buffer_[header_to_data + kChecksumBytes] = kFooter1;
  buffer_[header_to_data + kChecksumBytes + 1] = kFooter2;
  std::stringstream buf;
  for (int i = 0; i < length; ++i) {
    buf << " " << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(buffer_[i]);
  }
  RCLCPP_DEBUG_STREAM(logger_, "Send:" << buf.str());

  int64_t start = Now();
  int64_t elapsed = start;
  size_t num_done = 0;
  while ((elapsed - start) < timeout_) {
    ssize_t result = write(fd_, &buffer_[num_done], length - num_done);
    if (result < 0) {
      if (errno == EAGAIN) {
        // wait for sleep_tick_ nanoseconds
        timespec duration = { 0, sleep_tick_ };
        while (clock_nanosleep(CLOCK_MONOTONIC, 0, &duration, &duration)) {
          if (errno == EINTR) {
            continue;
          } else {
            return boost::system::error_code(errno, boost::system::system_category());
          }
        }
      } else {
        return boost::system::error_code(errno, boost::system::system_category());
      }
    } else {
      num_done += result;
      if (num_done == length) {
        return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
      } else if (num_done > length) {
        RCLCPP_FATAL(logger_, "NOT REACHED");
      }
    }
    int64_t last_elapsed = elapsed;
    elapsed = Now();
    if (elapsed < last_elapsed) {
      // オーバーフロー
      std::cerr << "last=" << last_elapsed << " now=" << elapsed << std::endl;
      return boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
    }
  }
  return boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
}

boost::system::error_code ExxxNetwork::Receive(uint8_t id, std::vector<uint8_t>& data_out) {
  int64_t start = Now();
  int64_t elapsed = start;
  size_t num_done = 0;
  struct pollfd poll_fd[1];
  struct timespec poll_timeout;
  parser_.Reset();
  parser_.ClearIDs();
  parser_.AddID(id);
  poll_fd[0].fd = fd_;
  poll_fd[0].events = POLLIN | POLLPRI;
  while ((elapsed - start) < timeout_) {
    int64_t remain_timeout = timeout_ - (elapsed - start);
    poll_timeout.tv_sec = remain_timeout / 1000000000LL;
    poll_timeout.tv_nsec = remain_timeout % 1000000000LL;
    int ready = ppoll(&poll_fd[0], 1, &poll_timeout, NULL);
    if (ready == 0) {
      // timeout
      break;
    } else if (ready < 0) {
      return boost::system::error_code(errno, boost::system::system_category());
    }
    ssize_t result = read(fd_, &buffer_[0], buffer_.size());
    if (result < 0) {
      return boost::system::error_code(errno, boost::system::system_category());
    } else {
      {
        std::stringstream buf;
        for (int i = 0; i < result; ++i) {
          buf << " " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int32_t>(buffer_[i]);
        }
        RCLCPP_DEBUG_STREAM(logger_, "Read:" << buf.str());
      }
      for (int i = 0; i < result; ++i) {
        ExxxPacketParser::Status parse_result = parser_.TryParse(buffer_[i]);
        if (parse_result == ExxxPacketParser::kDone) {
          const std::vector<uint8_t>& packet = parser_.packet();
          const uint8_t* size_start = &packet[kLengthStartStatus];
          uint16_t parameter_size = *reinterpret_cast<const uint16_t*>(size_start) - (kErrorBytes + kChecksumBytes);
          RCLCPP_DEBUG_STREAM(logger_,
                              "id=" << static_cast<int32_t>(packet[kIDStartStatus]) << " size=" << parameter_size
                                    << " error=" << std::hex << static_cast<int32_t>(packet[kErrorStartStatus])
                                    << static_cast<int32_t>(packet[kErrorStartStatus + 1]));
          if (parameter_size > data_out.capacity()) {
            RCLCPP_DEBUG_STREAM(logger_, "capacity = " << data_out.capacity());
            return boost::system::error_code(boost::system::errc::no_buffer_space, boost::system::system_category());
          }
          data_out.resize(parameter_size);
          std::copy(&packet[kParameterStartStatus], &packet[kParameterStartStatus] + parameter_size, data_out.begin());
          const uint16_t error_status = *reinterpret_cast<const uint16_t*>(&packet[5]);
          if (error_status != 0) {
            return boost::system::error_code(error_status, ExxxWarningCategory());
          }
          return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
        } else if (parse_result == ExxxPacketParser::kError) {
          uint8_t b = buffer_[i];
          // ROS_WARN_STREAM("Packet parsing error: 0x" << std::hex << (int)b );
          parser_.Reset();
        }
      }
    }
    int64_t last_elapsed = elapsed;
    elapsed = Now();
    if (elapsed < last_elapsed) {
      // オーバーフロー
      std::cerr << "last=" << last_elapsed << " now=" << elapsed << std::endl;
      return boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
    }
  }
  return boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
}

}  // namespace hsrb_servomotor_protocol
