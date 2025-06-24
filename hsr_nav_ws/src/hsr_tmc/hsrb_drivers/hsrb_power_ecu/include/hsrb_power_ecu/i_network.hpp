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
#ifndef HSRB_POWER_ECU_I_NETWORK_HPP_
#define HSRB_POWER_ECU_I_NETWORK_HPP_

#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>
#include <boost/system/error_code.hpp>

namespace hsrb_power_ecu {
typedef boost::circular_buffer<char> PacketBuffer;

/**
 * @brief network interface
 */
class INetwork {
 public:
  /**
   * @brief Destrist
   */
  virtual ~INetwork() {}
  /**
   * @brief Open
   */
  virtual boost::system::error_code Open() = 0;
  /**
   * @brief Close
   */
  virtual boost::system::error_code Close() = 0;
  /**
   * @brief Network setting change
   * @param [in] param Setting name
   * @param [in] value Change value
   */
  virtual boost::system::error_code Configure(const std::string &param, const int32_t value) = 0;
  /**
   * @brief Network setting change
   * @param [in] param Setting name
   * @param [in] value Change value
   */
  virtual boost::system::error_code Configure(const std::string &param, const double value) = 0;
  /**
   * @brief Network setting change
   * @param [in] param Setting name
   * @param [in] value Change value
   */
  virtual boost::system::error_code Configure(const std::string &param, const std::string &value) = 0;
  /**
   * @brief Transmission
   * Send all the contents of the transmission data received by the argument within the timeout time
   * @param [in] data Transmission data buffer
   */
  virtual boost::system::error_code Send(const PacketBuffer &data) = 0;
  /**
   * @brief Reception
   * Store the transmission data behind the buffer.
   * If the received data does not exist, wait for reception within the timeout time.
   * @param [out] data Receiving buffer
   */
  virtual boost::system::error_code Receive(PacketBuffer &data) = 0;
};
}  // namespace hsrb_power_ecu
#endif  // HSRB_POWER_ECU_I_NETWORK_HPP_
