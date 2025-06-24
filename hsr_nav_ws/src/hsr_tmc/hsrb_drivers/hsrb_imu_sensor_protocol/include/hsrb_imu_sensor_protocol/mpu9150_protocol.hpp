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
/// @brief Easy -to -use API of InventeNSE's Gyro Sensor MPU9150
#ifndef HSRB_IMU_SENSOR_PROTOCOL_MPU9150_PROTOCOL_HPP_
#define HSRB_IMU_SENSOR_PROTOCOL_MPU9150_PROTOCOL_HPP_

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <hsrb_imu_sensor_protocol/imu_protocol.hpp>
#include <hsrb_imu_sensor_protocol/mpu9150_network.hpp>

namespace hsrb_imu_sensor_protocol {
class MPU9150Protocol : public IImuProtocol, private boost::noncopyable {
 public:
  /// @brief constructor
  explicit MPU9150Protocol(MPU9150Network& network);

  virtual ~MPU9150Protocol() {}

  /// @brief Read the current sensor value
  virtual ErrorCode ReadState(ImuState& state);

  /// @brief Reset the sensor
  virtual ErrorCode TryReset(ResetResult& result);

  /// @brief Reset the sensor
  virtual ErrorCode Reset(double timeout);

 private:
  /// Communicate with MPU9150
  MPU9150Network& network_;
  /// Packet converter from MPU9150
  MPU9150PacketConverter packet_converter_;

  /// Protocol condition
  enum ProtocolStatus {
    /// Waiting for command
    kStatusWaiting,
    /// Wait for reset completion
    kStatusWaitForReset,
    /// Receive a reply to complete reset
    kStatusReceiveResetReturn,
  };
  /// Current state of MPU9150
  ProtocolStatus status_;

  /// Time when the order to MPU9150 ends
  rclcpp::Time instruction_end_time_;
  /// Timeout waiting for the end order to MPU9150
  rclcpp::Time instruction_timeout_;
};

}  // namespace hsrb_imu_sensor_protocol
#endif  // HSRB_IMU_SENSOR_PROTOCOL_MPU9150_PROTOCOL_HPP_
