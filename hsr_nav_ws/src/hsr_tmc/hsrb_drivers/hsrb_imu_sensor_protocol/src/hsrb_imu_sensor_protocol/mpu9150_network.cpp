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
/// @brief Providing classes to communicate with InventeNSE's Gyro Sensor MPU9150
/// @brief The communication specifications are from Arduino_skethces/tmc_invenseNSENSE_MPU9150_firmware
#include <time.h>
#include <algorithm>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <boost/math/constants/constants.hpp>
#include "hsrb_imu_sensor_protocol/mpu9150_network.hpp"


namespace {
const double kGravitationalAcceleration = 9.80665;
}  // namespace

namespace hsrb_imu_sensor_protocol {

const double k30BitScale = 1.0 / (1 << 30);
const double k16BitScale = 1.0 / (1 << 16);
const int32_t kPacketSize = 48;

inline double g2acc(double gravity) { return kGravitationalAcceleration * gravity; }

inline double deg2rad(double degrees) { return boost::math::constants::degree<double>() * degrees; }

uint32_t SensorPacketToUInt32(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
  uint32_t value = p1 << 24 | p2 << 16 | p3 << 8 | p4;
  return value;
}

double SensorPacketToDouble(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, double scale) {
  int32_t value = p1 << 24 | p2 << 16 | p3 << 8 | p4;
  return static_cast<double>(value) * scale;
}

inline int64_t Now() {
  timespec t = { 0 };
  if (clock_gettime(CLOCK_MONOTONIC, &t)) {
    exit(EXIT_FAILURE);
  }
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL + static_cast<int64_t>(t.tv_nsec);
}

// constructor
MPU9150Network::MPU9150Network(std::string device_name, boost::system::error_code& error_out)
    : fd_(-1), timeout_(50000000), sleep_tick_(10000), parser_() {
  error_out = Init(device_name);
}

MPU9150Network::MPU9150Network(std::string device_name, boost::system::error_code& error_out, int32_t timeout,
                               int32_t sleep_tick)
    : fd_(-1), timeout_(timeout), sleep_tick_(sleep_tick), parser_() {
  error_out = Init(device_name);
}

boost::system::error_code MPU9150Network::Init(const std::string& device_name) {
  // Open the device
  int port = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (port < 0) {
    return boost::system::error_code(errno, boost::system::system_category());
  }
  fd_ = port;

  // Various settings
  termios term = { 0 };
  if (tcgetattr(fd_, &term)) {
    return boost::system::error_code(errno, boost::system::system_category());
  }
  term.c_iflag = 0;
  term.c_cflag = B460800 | CS8 | CLOCAL | CREAD;
  term.c_oflag = 0;
  term.c_lflag = 0;
  term.c_cc[VTIME] = 0;
  term.c_cc[VMIN] = 1;
  if (tcsetattr(fd_, TCSANOW, &term)) {
    return boost::system::error_code(errno, boost::system::system_category());
  }
  if (tcflush(fd_, TCIOFLUSH)) {
    return boost::system::error_code(errno, boost::system::system_category());
  }
  return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
}

// Destructor
MPU9150Network::~MPU9150Network() { (void)close(fd_); }

// Send
boost::system::error_code MPU9150Network::Send(uint8_t instruction, const uint8_t* data, uint16_t size) {
  // Size check
  if (size > 1019) {
    return boost::system::error_code(boost::system::errc::no_buffer_space, boost::system::system_category());
  }
  // Make a packet
  size_t length = size + 5;
  buffer_[0] = kMPU9150Header1Command;
  buffer_[1] = kMPU9150Header2Command;
  buffer_[2] = size + 2;
  buffer_[3] = instruction;
  for (int i = 0; i < size; ++i) {
    buffer_[i + 4] = data[i];
  }
  buffer_[size + 4] = Checksum(buffer_.begin(), buffer_.begin() + length - 1);
  // send
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
        return boost::system::error_code(boost::system::errc::protocol_error, boost::system::system_category());
      }
    }
    int64_t last_elapsed = elapsed;
    elapsed = Now();
    if (elapsed < last_elapsed) {
      return boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
    }
  }
  return boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
}

// Receive
boost::system::error_code MPU9150Network::Receive() {
  int64_t start = Now();
  int64_t elapsed = start;
  size_t num_done = 0;
  parser_.Reset();
  while ((elapsed - start) < timeout_) {
    ssize_t result = read(fd_, &buffer_[0], buffer_.size());
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
      for (int i = 0; i < result; ++i) {
        MPU9150PacketParser::ParseResult parse_result = parser_.TryParse(buffer_[i]);
        if (parse_result == MPU9150PacketParser::kDone) {
          return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
        } else if (parse_result == MPU9150PacketParser::kError) {
          parser_.Reset();
          return boost::system::error_code(boost::system::errc::protocol_error, boost::system::system_category());
        }
      }
      if (parser_.packets().empty()) {
        return boost::system::error_code(boost::system::errc::no_message_available, boost::system::system_category());
      }
      return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
    }
    int64_t last_elapsed = elapsed;
    elapsed = Now();
    if (elapsed < last_elapsed) {
      return boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
    }
  }
  return boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
}

/// @brief constructor
MPU9150PacketConverter::MPU9150PacketConverter() : last_timestamp_(0), last_packet_num_(0) {}

// Convert packet from the sensor to Sensorstate to output
void MPU9150PacketConverter::ToSensorState(const std::vector<uint8_t>& packet, boost::array<double, 4>& orientation,
                                           boost::array<double, 3>& angular_velocity,
                                           boost::array<double, 3>& linear_acceleration) {
  // Check if the packet meets the specifications
  if (packet.size() != kPacketSize) {
    return;
  }
  // conversion
  orientation[3] = SensorPacketToDouble(packet[3], packet[4], packet[5], packet[6], k30BitScale);
  orientation[0] = SensorPacketToDouble(packet[7], packet[8], packet[9], packet[10], k30BitScale);
  orientation[1] = SensorPacketToDouble(packet[11], packet[12], packet[13], packet[14], k30BitScale);
  orientation[2] = SensorPacketToDouble(packet[15], packet[16], packet[17], packet[18], k30BitScale);

  angular_velocity[0] = deg2rad(SensorPacketToDouble(packet[19], packet[20], packet[21], packet[22], k16BitScale));
  angular_velocity[1] = deg2rad(SensorPacketToDouble(packet[23], packet[24], packet[25], packet[26], k16BitScale));
  angular_velocity[2] = deg2rad(SensorPacketToDouble(packet[27], packet[28], packet[29], packet[30], k16BitScale));

  linear_acceleration[0] = g2acc(SensorPacketToDouble(packet[31], packet[32], packet[33], packet[34], k16BitScale));
  linear_acceleration[1] = g2acc(SensorPacketToDouble(packet[35], packet[36], packet[37], packet[38], k16BitScale));
  linear_acceleration[2] = g2acc(SensorPacketToDouble(packet[39], packet[40], packet[41], packet[42], k16BitScale));
}

// Convert packet from the sensor to Sensorstate to output
void MPU9150PacketConverter::ToSensorState(const std::vector<uint8_t>& packets, MPU9150State& sensor_state) {
  // If anything other than dripping data is mixed, it will be done without doing anything
  if (packets.size() % kPacketSize != 0) {
    return;
  }

  // Division for each packet
  for (uint32_t i = 0; i < packets.size() / kPacketSize; ++i) {
    // Time stamp removal
    uint32_t current_timestamp = SensorPacketToUInt32(packets[43 + i * kPacketSize], packets[44 + i * kPacketSize],
                                                      packets[45 + i * kPacketSize], packets[46 + i * kPacketSize]);
    // Compare the time stamp and take out the latest packet NO
    if (last_timestamp_ < current_timestamp) {
      last_timestamp_ = current_timestamp;
      last_packet_num_ = i;
    }
  }

  // Extract the latest packet
  std::vector<uint8_t> packet;
  // Extracted index
  uint32_t start = last_packet_num_ * kPacketSize;
  uint32_t end = last_packet_num_ * kPacketSize + kPacketSize;
  std::copy(packets.begin() + start, packets.begin() + end, back_inserter(packet));

  ToSensorState(packet, sensor_state.orientation, sensor_state.angular_velocity, sensor_state.linear_acceleration);
}
}  // end of namespace hsrb_imu_sensor_protocol
