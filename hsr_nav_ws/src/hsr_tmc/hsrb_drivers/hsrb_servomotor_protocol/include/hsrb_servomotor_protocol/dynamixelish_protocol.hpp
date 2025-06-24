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
#ifndef HSRB_SERVOMOTOR_PROTOCOL_DYNAMIXELISH_PROTOCOL_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_DYNAMIXELISH_PROTOCOL_HPP_

#include <vector>
#include <boost/system/error_code.hpp>

namespace hsrb_servomotor_protocol {

/// Dynamixel通信プロトコル
/// (http://support.robotis.com/jp/product/dynamixel/dxl_communication.htm)に似た
/// オブジェクトディクショナリを読み書きするプロトコルのインターフェース。
/// ID16bit, サイズ共に16bitであることを想定している。
class IDynamixelishProtocol {
 public:
  typedef boost::system::error_code ErrorCode;
  virtual ~IDynamixelishProtocol() {}

  virtual ErrorCode ReadInt8(uint8_t id, uint16_t addr, int8_t& data_out) = 0;
  virtual ErrorCode ReadInt16(uint8_t id, uint16_t addr, int16_t& data_out) = 0;
  virtual ErrorCode ReadInt32(uint8_t id, uint16_t addr, int32_t& data_out) = 0;
  virtual ErrorCode ReadInt64(uint8_t id, uint16_t addr, int64_t& data_out) = 0;
  virtual ErrorCode ReadUInt8(uint8_t id, uint16_t addr, uint8_t& data_out) = 0;
  virtual ErrorCode ReadUInt16(uint8_t id, uint16_t addr, uint16_t& data_out) = 0;
  virtual ErrorCode ReadUInt32(uint8_t id, uint16_t addr, uint32_t& data_out) = 0;
  virtual ErrorCode ReadUInt64(uint8_t id, uint16_t addr, uint64_t& data_out) = 0;
  virtual ErrorCode ReadFloat32(uint8_t id, uint16_t addr, float& data_out) = 0;
  virtual ErrorCode ReadFloat64(uint8_t id, uint16_t addr, double& data_out) = 0;
  virtual ErrorCode ReadBlock(uint8_t id, uint16_t addr, uint16_t size, uint8_t* data_out) = 0;

  virtual ErrorCode WriteInt8(uint8_t id, uint16_t addr, int8_t data) = 0;
  virtual ErrorCode WriteInt16(uint8_t id, uint16_t addr, int16_t data) = 0;
  virtual ErrorCode WriteInt32(uint8_t id, uint16_t addr, int32_t data) = 0;
  virtual ErrorCode WriteInt64(uint8_t id, uint16_t addr, int64_t data) = 0;
  virtual ErrorCode WriteUInt8(uint8_t id, uint16_t addr, uint8_t data) = 0;
  virtual ErrorCode WriteUInt16(uint8_t id, uint16_t addr, uint16_t data) = 0;
  virtual ErrorCode WriteUInt32(uint8_t id, uint16_t addr, uint32_t data) = 0;
  virtual ErrorCode WriteUInt64(uint8_t id, uint16_t addr, uint64_t data) = 0;
  virtual ErrorCode WriteFloat32(uint8_t id, uint16_t addr, float data) = 0;
  virtual ErrorCode WriteFloat64(uint8_t id, uint16_t addr, double data) = 0;
  virtual ErrorCode WriteBlock(uint8_t id, uint16_t addr, uint16_t size, const uint8_t* data) = 0;

  virtual ErrorCode AvagoAvePos(uint8_t id) = 0;
  virtual ErrorCode ReadHash(uint8_t id, std::vector<uint8_t>& control_table_hash_out,
                             std::vector<uint8_t>& firmware_hash_out) = 0;
  virtual ErrorCode SyncParam(uint8_t id) = 0;
};


}  // namespace hsrb_servomotor_protocol

#endif /*HSRB_SERVOMOTOR_PROTOCOL_DYNAMIXELISH_PROTOCOL_HPP_*/
