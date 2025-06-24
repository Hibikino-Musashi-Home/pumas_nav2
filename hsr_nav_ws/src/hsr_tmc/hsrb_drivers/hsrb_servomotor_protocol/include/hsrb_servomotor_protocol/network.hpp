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
#ifndef HSRB_SERVOMOTOR_PROTOCOL_NETWORK_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_NETWORK_HPP_

#include <memory>
#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/system/error_code.hpp>

namespace hsrb_servomotor_protocol {

class INetwork {
 public:
  using Ptr = std::shared_ptr<INetwork>;
  virtual ~INetwork() {}

  /// Send a packet to nodes.
  /// @param [in] id    The ID of the target node
  /// @param [in] inst  Instruction type
  /// @param [in] data  Byte array of parameters
  /// @param [in] data_bytes Size of parameters
  virtual boost::system::error_code Send(uint8_t id, uint8_t inst, const uint8_t* data, uint16_t data_bytes) = 0;

  /// Receive a packet to nodes.
  /// @param [in] id  The expected responder ID
  virtual boost::system::error_code Receive(uint8_t id, std::vector<uint8_t>& data_out) = 0;

  virtual const std::vector<uint8_t>& last_packet() const = 0;
};

}  // namespace hsrb_servomotor_protocol

#endif /*HSRB_SERVOMOTOR_PROTOCOL_NETWORK_HPP_*/
