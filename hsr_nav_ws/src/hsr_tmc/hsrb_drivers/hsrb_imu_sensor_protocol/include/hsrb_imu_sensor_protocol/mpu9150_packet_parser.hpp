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
#ifndef HSRB_IMU_SENSOR_PROTOCOL_MPU9150_PACKET_PARSER_HPP_
#define HSRB_IMU_SENSOR_PROTOCOL_MPU9150_PACKET_PARSER_HPP_

#include <stdint.h>
#include <vector>

#include <boost/array.hpp>
#include <boost/noncopyable.hpp>

namespace hsrb_imu_sensor_protocol {

/// Control command header 1 (@)
const uint8_t kMPU9150Header1Command = 0x40;
/// Control command header 2 (g)
const uint8_t kMPU9150Header2Command = 0x47;


// Checksam calculation
template <class InputIterator>
uint8_t Checksum(InputIterator begin, InputIterator end) {
  size_t sum = 0;
  for (InputIterator it = begin; it != end; ++it) {
    sum += *it;
  }
  return ~static_cast<uint8_t>(sum);
}


/// INVENSENSE's Gyro Sensor MPU9150 PARSER class
/// See the TMC_INVENSENSENSE_MPU9150_firmware document for packet specifications
class MPU9150PacketParser : private boost::noncopyable {
 public:
  /// PARSE result
  enum ParseResult {
    /// fail
    kError,
    /// continuation
    kContinue,
    /// end
    kDone
  };

  /// @brief Candolactors and internal variables are initialized
  MPU9150PacketParser() : is_reply_packet_(false) { Reset(); }

  /// @brief Destructor
  ~MPU9150PacketParser() {}

  /// @brief Analyze
  ParseResult TryParse(uint8_t input);

  /// @brief Reset the state
  void Reset();

  /// @brief Return the packet
  const std::vector<uint8_t>& packets() const { return all_packets_; }

  bool is_reply_packet() const { return is_reply_packet_; }

 private:
  /// The state and state name of the parser indicate what the next I receive
  enum State {
    /// Header 1
    kStateHeader1,
    /// Header 2
    kStateHeader2,
    /// Data length
    kStateLength,
    /// 値
    kStateData,
    /// Checksum
    kStateChecksum,
    /// I didn't accept anything because I got an error
    kStateNothing
  };
  /// What is the next packet to receive
  State state_;
  /// The contents of the packet
  std::vector<uint8_t> packet_;
  /// All packets received
  std::vector<uint8_t> all_packets_;
  /// Packet data length
  uint8_t length_;
  /// Presentation of returned packets
  bool is_reply_packet_;
};
}  // end of namespace hsrb_imu_sensor_protocol
#endif  // HSRB_IMU_SENSOR_PROTOCOL_MPU9150_PACKET_PARSER_HPP_
