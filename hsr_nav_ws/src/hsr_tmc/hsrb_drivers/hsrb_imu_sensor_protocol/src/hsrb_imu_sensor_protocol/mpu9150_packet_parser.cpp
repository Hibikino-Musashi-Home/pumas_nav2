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
/// @brief Providing PARSER class of InventeNSE Gyro Sensor MPU9150
/// @brief The communication specifications are from Arduino_skethces/tmc_invenseNSENSE_MPU9150_firmware
#include <hsrb_imu_sensor_protocol/mpu9150_packet_parser.hpp>

namespace hsrb_imu_sensor_protocol {

// Analyze
MPU9150PacketParser::ParseResult MPU9150PacketParser::TryParse(uint8_t input) {
  switch (state_) {
    case kStateHeader1:
      if (input == kMPU9150Header1Command) {
        state_ = kStateHeader2;
        packet_.push_back(input);
        return kContinue;
      } else {
        state_ = kStateHeader1;
        return kContinue;
      }
    case kStateHeader2:
      if (input == kMPU9150Header2Command) {
        state_ = kStateLength;
        packet_.push_back(input);
        return kContinue;
      } else {
        state_ = kStateHeader1;
        packet_.clear();
        return kContinue;
      }
    case kStateLength:
      if (input < 1) {
        state_ = kStateHeader1;
        packet_.clear();
        return kContinue;
      } else if (input == 1) {
        // If there is no data, migrate to checksam calculation
        length_ = input;
        packet_.push_back(input);
        state_ = kStateChecksum;
        return kContinue;
      } else {
        packet_.push_back(input);
        length_ = input;
        state_ = kStateData;
        return kContinue;
      }
    case kStateData:
      packet_.push_back(input);
      if (packet_.size() == (length_ + 2)) {
        state_ = kStateChecksum;
      }
      return kContinue;
    case kStateChecksum: {
      uint8_t sum = Checksum(packet_.begin(), packet_.end());
      if (input == sum) {
        packet_.push_back(input);
        // Stores all packets that were received
        all_packets_.insert(all_packets_.end(), packet_.begin(), packet_.end());
        packet_.clear();
        // There is no contents in the command reply packet
        if (length_ == 1) {
          is_reply_packet_ = true;
          return kDone;
        } else {
          state_ = kStateHeader1;
        }
        return kContinue;
      } else {
        state_ = kStateHeader1;
        packet_.clear();
        return kContinue;
      }
    }
    default:
      break;
  }
  return kError;
}

// Reset
void MPU9150PacketParser::Reset() {
  state_ = kStateHeader1;
  is_reply_packet_ = false;
  packet_.clear();
  all_packets_.clear();
}
}  // end of namespace hsrb_imu_sensor_protocol
