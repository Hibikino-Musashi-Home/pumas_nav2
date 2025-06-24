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
#ifndef HSRB_SERVOMOTOR_PROTOCOL_EXXX_PROTOCOL_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_EXXX_PROTOCOL_HPP_

#include <algorithm>
#include <iterator>
#include <vector>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>

#include <hsrb_servomotor_protocol/dynamixelish_protocol.hpp>
#include <hsrb_servomotor_protocol/exxx_common.hpp>
#include <hsrb_servomotor_protocol/network.hpp>

namespace hsrb_servomotor_protocol {

const uint32_t kMaxPacketBuffer = 0xFFFF;
const uint32_t kControlTableHashByte = 16;
const uint32_t kDateByte = 8;
const uint32_t kFirmwareHashByte = 20;

class ExxxProtocol : public IDynamixelishProtocol, private boost::noncopyable {
 public:
  explicit ExxxProtocol(const INetwork::Ptr& network) : network_(network) {
    send_buffer_.reserve(kMaxPacketBuffer);
    receive_buffer_.reserve(kMaxPacketBuffer);
  }

  virtual ~ExxxProtocol() {}

  virtual ErrorCode ReadInt8(uint8_t id, uint16_t addr, int8_t& data_out) {
    return ReadData<int8_t>(id, addr, data_out);
  }

  virtual ErrorCode ReadInt16(uint8_t id, uint16_t addr, int16_t& data_out) {
    return ReadData<int16_t>(id, addr, data_out);
  }

  virtual ErrorCode ReadInt32(uint8_t id, uint16_t addr, int32_t& data_out) {
    return ReadData<int32_t>(id, addr, data_out);
  }

  virtual ErrorCode ReadInt64(uint8_t id, uint16_t addr, int64_t& data_out) {
    return ReadData<int64_t>(id, addr, data_out);
  }

  virtual ErrorCode ReadUInt8(uint8_t id, uint16_t addr, uint8_t& data_out) {
    return ReadData<uint8_t>(id, addr, data_out);
  }

  virtual ErrorCode ReadUInt16(uint8_t id, uint16_t addr, uint16_t& data_out) {
    return ReadData<uint16_t>(id, addr, data_out);
  }

  virtual ErrorCode ReadUInt32(uint8_t id, uint16_t addr, uint32_t& data_out) {
    return ReadData<uint32_t>(id, addr, data_out);
  }

  virtual ErrorCode ReadUInt64(uint8_t id, uint16_t addr, uint64_t& data_out) {
    return ReadData<uint64_t>(id, addr, data_out);
  }

  virtual ErrorCode ReadFloat32(uint8_t id, uint16_t addr, float& data_out) {
    return ReadData<float>(id, addr, data_out);
  }

  virtual ErrorCode ReadFloat64(uint8_t id, uint16_t addr, double& data_out) {
    return ReadData<double>(id, addr, data_out);
  }

  virtual ErrorCode ReadBlock(uint8_t id, uint16_t addr, uint16_t size, uint8_t* data_out) {
    return ReadBlock(id, addr, data_out, data_out + size);
  }

  virtual ErrorCode WriteInt8(uint8_t id, uint16_t addr, int8_t data) { return WriteData<int8_t>(id, addr, data); }

  virtual ErrorCode WriteInt16(uint8_t id, uint16_t addr, int16_t data) { return WriteData<int16_t>(id, addr, data); }

  virtual ErrorCode WriteInt32(uint8_t id, uint16_t addr, int32_t data) { return WriteData<int32_t>(id, addr, data); }

  virtual ErrorCode WriteInt64(uint8_t id, uint16_t addr, int64_t data) { return WriteData<int64_t>(id, addr, data); }

  virtual ErrorCode WriteUInt8(uint8_t id, uint16_t addr, uint8_t data) { return WriteData<uint8_t>(id, addr, data); }

  virtual ErrorCode WriteUInt16(uint8_t id, uint16_t addr, uint16_t data) {
    return WriteData<uint16_t>(id, addr, data);
  }

  virtual ErrorCode WriteUInt32(uint8_t id, uint16_t addr, uint32_t data) {
    return WriteData<uint32_t>(id, addr, data);
  }

  virtual ErrorCode WriteUInt64(uint8_t id, uint16_t addr, uint64_t data) {
    return WriteData<uint64_t>(id, addr, data);
  }

  virtual ErrorCode WriteFloat32(uint8_t id, uint16_t addr, float data) { return WriteData<float>(id, addr, data); }

  virtual ErrorCode WriteFloat64(uint8_t id, uint16_t addr, double data) { return WriteData<double>(id, addr, data); }

  virtual ErrorCode WriteBlock(uint8_t id, uint16_t addr, uint16_t size, const uint8_t* data) {
    return WriteBlock(id, addr, data, data + size);
  }

  template <typename T>
  ErrorCode ReadData(uint8_t id, uint16_t addr, T& data_out) {
    uint8_t* begin = reinterpret_cast<uint8_t*>(&data_out);
    uint8_t* end = begin + sizeof(data_out);
    return ReadBlock(id, addr, begin, end);
  }

  template <class OutputIterator>
  ErrorCode ReadBlock(uint8_t id, uint16_t addr, OutputIterator begin, OutputIterator end) {
    BOOST_STATIC_ASSERT((boost::is_same<uint8_t, typename std::iterator_traits<OutputIterator>::value_type>::value));
    const uint16_t length = std::distance(begin, end);
    send_buffer_.resize(4);
    *reinterpret_cast<uint16_t*>(&send_buffer_[0]) = addr;
    *reinterpret_cast<uint16_t*>(&send_buffer_[2]) = length;

    boost::system::error_code error;
    boost::system::error_code exxx_error;
    error = network_->Send(id, kInstructionReadData, &send_buffer_[0], send_buffer_.size());
    if (error) return error;

    error = network_->Receive(id, receive_buffer_);
    // system errorは直ちに返す
    if (error.category() == boost::system::system_category() && error) {
      return error;
    }

    if (receive_buffer_.size() != length) {
      return boost::system::error_code(boost::system::errc::protocol_error, boost::system::system_category());
    }
    std::copy(receive_buffer_.begin(), receive_buffer_.end(), begin);
    if (error) {
      return error;
    }
    return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
  }

  template <typename T>
  ErrorCode WriteData(uint8_t id, uint16_t addr, T data) {
    const uint8_t* begin = reinterpret_cast<const uint8_t*>(&data);
    const uint8_t* end = begin + sizeof(data);
    return WriteBlock(id, addr, begin, end);
  }

  /// 複数のノードにデータを書き込む
  /// @param[in] ids 書き込むidの配列
  /// @param[in] addr 書き込むアドレス（ノードで共通）
  /// @param[in] data 書き込むデータの配列
  /// @return ErrorCode
  /// @pre idsとdataのサイズは同じ
  template <typename T>
  ErrorCode SyncWriteData(std::vector<uint8_t> ids, uint16_t addr, std::vector<T> data) {
    if (ids.size() != data.size()) {
      return boost::system::error_code(boost::system::errc::invalid_argument, boost::system::system_category());
    }
    uint16_t data_length = sizeof(T);
    uint8_t num_nodes = ids.size();
    send_buffer_.resize((data_length + 1) * num_nodes + 4);
    *reinterpret_cast<uint16_t*>(&send_buffer_[0]) = addr;
    *reinterpret_cast<uint16_t*>(&send_buffer_[2]) = data_length;
    for (size_t i = 0; i < ids.size(); ++i) {
      send_buffer_[4 + i * (data_length + 1)] = ids[i];
      *reinterpret_cast<T>(&send_buffer_[4 + i * (data_length + 1) + 1]) = data[i];
    }
    boost::system::error_code error;
    error = network_->Send(kBroadcastID, kInstructionSyncWrite, &send_buffer_[0], send_buffer_.size());
    if (error) return error;

    // sync_writeは返事はこない
    return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
  }

  template <class InputIterator>
  ErrorCode WriteBlock(uint8_t id, uint16_t addr, InputIterator begin, InputIterator end) {
    BOOST_STATIC_ASSERT((boost::is_same<uint8_t, typename std::iterator_traits<InputIterator>::value_type>::value));

    const uint8_t length = std::distance(begin, end);
    send_buffer_.resize(length + 2);
    *reinterpret_cast<uint16_t*>(&send_buffer_[0]) = addr;
    std::copy(begin, end, &send_buffer_[2]);

    boost::system::error_code error;
    error = network_->Send(id, kInstructionWriteData, &send_buffer_[0], send_buffer_.size());
    if (error) return error;

    error = network_->Receive(id, receive_buffer_);
    if (error) {
      return error;
    }
    return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
  }

  ErrorCode Ping(uint8_t id);
  ErrorCode Reset(uint8_t id);
  ErrorCode SyncParam(uint8_t id);
  ErrorCode AvagoAvePos(uint8_t id);
  ErrorCode WriteEeprom(uint8_t id);
  ErrorCode ReadHash(uint8_t id, std::vector<uint8_t>& control_table_hash_out, std::vector<uint8_t>& firmware_hash_out);

 private:
  INetwork::Ptr network_;
  std::vector<uint8_t> send_buffer_;
  std::vector<uint8_t> receive_buffer_;
};

}  // namespace hsrb_servomotor_protocol

#endif /*HSRB_SERVOMOTOR_PROTOCOL_EXXX_PROTOCOL_HPP_*/
