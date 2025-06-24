/*
Copyright (c) 2017 TOYOTA MOTOR CORPORATION
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
#ifndef HSRB_SERVOMOTOR_PROTOCOL_EXXX_REPROGRAMING_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_EXXX_REPROGRAMING_HPP_

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>
#include <boost/static_assert.hpp>
#include <boost/system/error_code.hpp>
#include <boost/type_traits.hpp>

#include <hsrb_servomotor_protocol/exxx_common.hpp>

namespace hsrb_servomotor_protocol {

class ExxxReprograming : private boost::noncopyable {
 public:
  typedef boost::system::error_code ErrorCode;
  ExxxReprograming(const std::string& device_name, bool is_usb_rs485, uint32_t baudrate,
                   int64_t timeout, int64_t sleep_tick);
  virtual ~ExxxReprograming();

  ErrorCode Open();
  ErrorCode WaitBoot(const int32_t bootloader_version, const uint8_t id, const int64_t boot_timeout);
  ErrorCode Erase();
  ErrorCode Flush(const std::string& flush_data);
  ErrorCode Run();

 private:
  int fd_;
  std::string device_name_;
  bool is_usb_rs485_;
  uint32_t baudrate_;
  int64_t timeout_;
  int64_t sleep_tick_;

  ErrorCode FlushXmodem(const std::string& flush_data);
  ErrorCode CheckReceivedMessage(const std::string& check_message);
  ErrorCode SendCommand(const uint8_t command, const std::string& check_message);
  ErrorCode SendFlushBlockData(const std::string& flush_data, uint32_t send_pos, uint8_t block_number);
  ErrorCode SendData(const uint8_t* data, uint32_t data_bytes);
  ErrorCode ReceiveLine(std::string& receive_line);
  ErrorCode ReceiveByte(uint8_t& data_out);
};

}  // namespace hsrb_servomotor_protocol

#endif /*HSRB_SERVOMOTOR_PROTOCOL_EXXX_REPROGRAMING_HPP_*/
