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
/// @brief The one for testing around communication with MPU9150
#ifndef SERIAL_COMMUNICATION_FOR_MPU9150_TEST_HPP_
#define SERIAL_COMMUNICATION_FOR_MPU9150_TEST_HPP_

#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/utility.hpp>

namespace hsrb_imu_sensor_protocol {
// Serial communication checker
class SerialCommunication : boost::noncopyable {
 public:
  // constructor
  explicit SerialCommunication(const std::string& port_name, uint32_t baud_rate)
      : serial_port_(io_service_, port_name) {
    // Serial communication settings
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control());
    serial_port_.set_option(boost::asio::serial_port_base::parity());
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits());
  }

  ~SerialCommunication() {}

  // Reception
  void Receive(std::vector<uint8_t>& data) {
    data.resize(serial_port_.read_some(boost::asio::buffer(receive_buffer_)));
    for (uint32_t i = 0; i < data.size(); ++i) {
      data[i] = receive_buffer_[i];
    }
  }
  // send
  void Send(const std::vector<uint8_t>& data) { serial_port_.write_some(boost::asio::buffer(data)); }

 private:
  // io_service
  boost::asio::io_service io_service_;
  // Serial port
  boost::asio::serial_port serial_port_;
  // Read buffer
  boost::array<uint8_t, 1024> receive_buffer_;
};

}  // end of namespace hsrb_imu_sensor_protocol
#endif  // SERIAL_COMMUNICATION_FOR_MPU9150_TEST_HPP_
