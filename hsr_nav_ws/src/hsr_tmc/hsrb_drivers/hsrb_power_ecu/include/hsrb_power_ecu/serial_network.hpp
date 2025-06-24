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
#ifndef HSRB_POWER_ECU_SERIAL_NETWORK_HPP_
#define HSRB_POWER_ECU_SERIAL_NETWORK_HPP_

#include <string>
#include <vector>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/system/error_code.hpp>

#include <hsrb_power_ecu/i_network.hpp>

namespace hsrb_power_ecu {

class ISystemInterface;

/**
 * @brief Serial port
 * Of the communication specifications, the following specifications are determined and cannot be changed.
 * - Borate 3Mbps
 * - Data bit 8bit
 * - No Parity Bit
 * - Stop bit 1bit
 *
 * The following specifications can be changed with the configure function
 * - Communication timeout time (ms)
 *   - Type double
 *   - Key "receive_timeout_ms"
 *   - Default value 0.3
 * - Device name
 *   - Type std::string
 *   - Key "device_name"
 *   - Default value "/dev/ttyUSB0"
 */
class SerialNetwork : private boost::noncopyable, public INetwork {
 public:
  SerialNetwork();
  explicit SerialNetwork(boost::shared_ptr<ISystemInterface> system);

  /**
   * @brief Destrist
   */
  virtual ~SerialNetwork();
  /**
   * @brief Open
   */
  virtual boost::system::error_code Open();
  /**
   * @brief Close
   */
  virtual boost::system::error_code Close();
  /**
   * @brief Network setting change
   * @param [in] param Setting name
   * @param [in] value Change value
   */
  virtual boost::system::error_code Configure(const std::string &param, int32_t value);
  /**
   * @brief Network setting change
   * @param [in] param Setting name
   * @param [in] value Change value
   */
  virtual boost::system::error_code Configure(const std::string &param, double value);
  /**
   * @brief Network setting change
   * @param [in] param Setting name
   * @param [in] value Change value
   */
  virtual boost::system::error_code Configure(const std::string &param, const std::string &value);
  /**
   * @brief Transmission
   * Send all the contents of the transmission data received by the argument within the timeout time
   * @param [in] data Transmission data buffer
   */
  virtual boost::system::error_code Send(const PacketBuffer &data);
  /**
   * @brief Reception
   * Store the transmission data behind the buffer.
   * If the received data does not exist, wait for reception within the timeout time.
   * @param [out] data Receiving buffer
   */
  virtual boost::system::error_code Receive(PacketBuffer &data);

 private:
  int fd_;
  std::vector<uint8_t> receive_buffer_;
  std::vector<uint8_t> send_buffer_;
  uint32_t timeout_ns_;
  int32_t sleep_tick_;
  std::string port_name_;
  boost::shared_ptr<ISystemInterface> system_;
};
}  // namespace hsrb_power_ecu

#endif  // HSRB_POWER_ECU_SERIAL_NETWORK_HPP_
