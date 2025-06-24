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

#include "power_ecu_com_common.hpp"

#include <boost/crc.hpp>

namespace {
const uint32_t kCrc32PolynomialValue = 0x04C11DB7;
const uint32_t kCrc32InitializeValue = 0xffffffff;
}  // anonymous namespace

namespace hsrb_power_ecu {
namespace com_common {

/**
 * @brief CRC32 Checksam calculation
 * @param [in] start_it Checksam string leader
 * @param [in] end_it Checksam string endorator
 */
uint32_t CalculateCrc32(std::string::const_iterator start_it, std::string::const_iterator end_it) {
  hsrb_power_ecu::PacketBuffer buf(std::distance(start_it, end_it));
  std::copy(start_it, end_it, std::back_inserter(buf));
  return CalculateCrc32(buf.begin(), buf.end());
}

/**
 * @brief CRC32 Checksam calculation
 * @param [in] start_it Checksam string leader
 * @param [in] end_it Checksam string endorator
 */
uint32_t CalculateCrc32(hsrb_power_ecu::PacketBuffer::const_iterator start_it,
                        hsrb_power_ecu::PacketBuffer::const_iterator end_it) {
  hsrb_power_ecu::PacketBuffer::const_iterator current_it = start_it;
  boost::crc_optimal<32, kCrc32PolynomialValue, kCrc32InitializeValue, 0, false, false> crc32;
  // Endian conversion is required (current power supply ECU performs CRC operations as shown below)
  while (current_it != end_it) {
    int32_t const distance = std::distance(current_it, end_it);
    if (distance >= 4) {
      crc32.process_byte(*(current_it + 3));
    } else {
      crc32.process_byte(0);
    }
    if (distance >= 3) {
      crc32.process_byte(*(current_it + 2));
    } else {
      crc32.process_byte(0);
    }
    if (distance >= 2) {
      crc32.process_byte(*(current_it + 1));
    } else {
      crc32.process_byte(0);
    }
    if (distance >= 1) {
      crc32.process_byte(*(current_it));
    } else {
      crc32.process_byte(0);
    }
    if (distance >= 4) {
      current_it += 4;
    } else {
      current_it = end_it;
    }
  }
  return crc32.checksum();
}

}  // namespace com_common
}  // namespace hsrb_power_ecu
