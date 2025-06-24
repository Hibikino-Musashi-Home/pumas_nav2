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
#ifndef HSRB_SERVOMOTOR_PROTOCOL_EXXX_PACKET_PARSER_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_EXXX_PACKET_PARSER_HPP_

#include <vector>
#include <boost/noncopyable.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hsrb_servomotor_protocol {

template <class InputIterator>
uint8_t Checksum(InputIterator begin, InputIterator end) {
  // チェックサム計算
  size_t sum = 0;
  for (InputIterator it = begin; it != end; ++it) {
    sum += *it;
  }
  return ~static_cast<uint8_t>(sum);
}


class ExxxPacketParser : private boost::noncopyable {
 public:
  enum Status {
    kError,
    kContinue,
    kDone,
  };

  ExxxPacketParser() : state_(kStateHeader1), logger_(rclcpp::get_logger("exxx_packet_parser")) {}

  void Reset() {
    state_ = kStateHeader1;
    packet_.clear();
  }

  Status TryParse(uint8_t input) {
    switch (state_) {
      case kStateHeader1: {
        if (input == 0xAAU) {
          RCLCPP_DEBUG_STREAM(logger_, "Header1 0x" << std::hex << static_cast<int>(input));
          state_ = kStateHeader2;
          packet_.push_back(input);
          return kContinue;
        } else {
          return kError;
        }
      }
      case kStateHeader2: {
        if (input == 0x55U) {
          RCLCPP_DEBUG_STREAM(logger_, "Header2 0x" << std::hex << static_cast<int>(input));
          state_ = kStateNodeID;
          packet_.push_back(input);
          return kContinue;
        } else {
          state_ = kStateParserError;
          return kError;
        }
      }
      case kStateNodeID: {
        assert(packet_.size() == 2);
        std::vector<uint8_t>::const_iterator it = std::find(valid_ids_.begin(), valid_ids_.end(), input);
        if (it != valid_ids_.end()) {
          RCLCPP_DEBUG_STREAM(logger_, "NodeID " << static_cast<int>(input));
          packet_.push_back(input);
          state_ = kStateLength1;
          return kContinue;
        } else {
          state_ = kStateParserError;
          return kError;
        }
      }
      case kStateLength1: {
        assert(packet_.size() == 3);
        packet_.push_back(input);
        state_ = kStateLength2;
        return kContinue;
      }
      case kStateLength2: {
        assert(packet_.size() == 4);
        packet_.push_back(input);
        RCLCPP_DEBUG_STREAM(logger_, "Length " << *reinterpret_cast<uint16_t*>(&packet_[3]));

        if (*reinterpret_cast<uint16_t*>(&packet_[3]) < 3) {
          state_ = kStateParserError;
          return kError;
        }
        state_ = kStateErrorStatus1;
        return kContinue;
      }
      case kStateErrorStatus1: {
        assert(packet_.size() == 5);
        packet_.push_back(input);
        state_ = kStateErrorStatus2;
        return kContinue;
      }
      case kStateErrorStatus2: {
        assert(packet_.size() == 6);
        const uint16_t length = *reinterpret_cast<uint16_t*>(&packet_[3]);
        packet_.push_back(input);
        if (length == 3) {
          state_ = kStateChecksum;
        } else {
          state_ = kStateData;
        }
        return kContinue;
      }
      case kStateData: {
        RCLCPP_DEBUG_STREAM(logger_, "StateData 0x" << std::hex << static_cast<int>(input));
        const uint16_t length = *reinterpret_cast<uint16_t*>(&packet_[3]);
        packet_.push_back(input);
        // 全て読み終わったか判定
        if (packet_.size() >= (length + 4)) {
          state_ = kStateChecksum;
        }
        return kContinue;
      }
      case kStateChecksum: {
        uint8_t sum = Checksum(packet_.begin() + 2, packet_.end());
        if (input == sum) {
          RCLCPP_DEBUG_STREAM(logger_, "Checksum 0x" << std::hex << static_cast<int>(input) << " OK");
          packet_.push_back(input);
          return kDone;
        } else {
          RCLCPP_DEBUG_STREAM(logger_, "Checksum mismatch 0x" << std::hex << static_cast<int>(input) << " != 0x"
                                       << static_cast<int>(sum));
          return kError;
        }
      }
      case kStateParserError: {
        return kError;
      }
      default: { assert(false); }
    }
  }

  const std::vector<uint8_t>& packet() const { return packet_; }

  void AddID(uint8_t id) { valid_ids_.push_back(id); }

  void ClearIDs() { valid_ids_.clear(); }

 private:
  enum State {
    kStateHeader1,
    kStateHeader2,
    kStateNodeID,
    kStateLength1,
    kStateLength2,
    kStateErrorStatus1,
    kStateErrorStatus2,
    kStateData,
    kStateChecksum,
    kStateParserError
  };
  State state_;
  std::vector<uint8_t> packet_;
  std::vector<uint8_t> valid_ids_;

  rclcpp::Logger logger_;
};

}  // namespace hsrb_servomotor_protocol

#endif /*HSRB_SERVOMOTOR_PROTOCOL_EXXX_PACKET_PARSER_HPP_*/
