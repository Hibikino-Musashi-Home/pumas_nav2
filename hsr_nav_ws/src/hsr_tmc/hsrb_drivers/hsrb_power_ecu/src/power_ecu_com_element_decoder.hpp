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
#ifndef POWER_ECU_COM_ELEMENT_DECODER_HPP_
#define POWER_ECU_COM_ELEMENT_DECODER_HPP_

#include <stdint.h>
#include <algorithm>
#include <limits>
#include <string>

#include <boost/smart_ptr/shared_ptr.hpp>

#include <hsrb_power_ecu/i_network.hpp>
#include "power_ecu_com_frame_decoder.hpp"

namespace hsrb_power_ecu {

/**
 * @brief Uint型のデコーダ
 */
template <class Output>
class ElementUintDecoder : public IElementDecoder {
 private:
  ElementUintDecoder(ElementUintDecoder const&);             // = delete;
  ElementUintDecoder& operator=(ElementUintDecoder const&);  // = delete;

 public:
  /**
   * @brief コンストラクタ
   * @param value デコードした値の代入先の参照
   * @param length 桁数
   */
  ElementUintDecoder(Output& value, const size_t length) : value_(value), length_(length) {}
  /**
   * @brief デストラクタ
   */
  virtual ~ElementUintDecoder() {}
  /**
   * @brief デコード
   * @param start_iterator 読み出し先頭の文字のイテレータ
   * @param end_iterator 読み出す最後の文字の次のイテレータ
   * @return 成功時 true
   */
  inline virtual bool Decode(const PacketBuffer::const_iterator start_iterator,
                             const PacketBuffer::const_iterator end_iterator) {
    // 桁数チェック
    if (std::distance(start_iterator, end_iterator) != length_) {
      return false;
    }
    // 代入処理
    uint32_t tmp_value = 0;
    for (PacketBuffer::const_iterator it = start_iterator; it != end_iterator; ++it) {
      PacketBuffer::value_type v = *it;
      if ((v < '0') || (v > '9')) {
        return false;
      }
      v -= '0';
      tmp_value = (tmp_value * 10) + v;
    }
    value_ = static_cast<Output>(tmp_value);
    return true;
  }

 private:
  Output& value_;        //!< デコードした値の代入先の参照
  const size_t length_;  //!< 桁数
};

/**
 * @brief Uint型のデコーダ
 */
template <class Output>
class ElementHexUintDecoder : public IElementDecoder {
 private:
  ElementHexUintDecoder(ElementHexUintDecoder const&);             // = delete;
  ElementHexUintDecoder& operator=(ElementHexUintDecoder const&);  // = delete;

 public:
  /**
   * @brief コンストラクタ
   * @param value デコードした値の代入先の参照
   * @param length 桁数
   */
  ElementHexUintDecoder(Output& value, const size_t length) : value_(value), length_(length) {}
  /**
   * @brief デストラクタ
   */
  virtual ~ElementHexUintDecoder() {}
  /**
   * @brief デコード
   * @param start_iterator 読み出し先頭の文字のイテレータ
   * @param end_iterator 読み出す最後の文字の次のイテレータ
   * @return 成功時 true
   */
  inline virtual bool Decode(const PacketBuffer::const_iterator start_iterator,
                             const PacketBuffer::const_iterator end_iterator) {
    // 桁数チェック
    // @uend
    // 計算順序が、演算子の優先順位を考慮されている場合でも、丸括弧で厳格に記載してください（他の箇所も確認してください）
    if (((std::distance(start_iterator, end_iterator) - 1) != length_) || (*start_iterator != 'h')) {
      return false;
    }
    // 代入処理
    uint32_t tmp_value = 0;
    for (PacketBuffer::const_iterator it = (start_iterator + 1);  // 'h'読み飛ばし
         it != end_iterator; ++it) {
      PacketBuffer::value_type v = *it;
      if ((v >= '0') && (v <= '9')) {
        v -= '0';
      } else if ((v >= 'A') && (v <= 'F')) {
        v -= ('A' - 10);
      } else {
        return false;
      }
      tmp_value = (tmp_value * 16) + v;
    }
    value_ = static_cast<Output>(tmp_value);
    return true;
  }

 private:
  Output& value_;        //!< デコードした値の代入先の参照
  const size_t length_;  //!< 桁数
};

/**
 * @brief int型のデコーダ
 */
template <class Output>
class ElementIntDecoder : public IElementDecoder {
 private:
  ElementIntDecoder(ElementIntDecoder const&);             // = delete;
  ElementIntDecoder& operator=(ElementIntDecoder const&);  // = delete;

 public:
  /**
   * @brief コンストラクタ
   * @param value デコードした値の代入先の参照
   * @param length 桁数
   */
  ElementIntDecoder(Output& value, const size_t length) : value_(value), length_(length) {}
  /**
   * @brief デストラクタ
   */
  virtual ~ElementIntDecoder() {}
  /**
   * @brief デコード
   * @param start_iterator 読み出し先頭の文字のイテレータ
   * @param end_iterator 読み出す最後の文字の次のイテレータ
   * @return 成功時 true
   */
  inline virtual bool Decode(const PacketBuffer::const_iterator start_iterator,
                             const PacketBuffer::const_iterator end_iterator) {
    // 桁数確認
    if ((std::distance(start_iterator, end_iterator) - 1) != length_) {
      return false;
    }
    // 符号計算
    bool is_positive_number;
    PacketBuffer::value_type v = *start_iterator;
    if (v == ' ') {
      is_positive_number = true;
    } else if (v == '-') {
      is_positive_number = false;
    } else {
      return false;  // TODO(kitsunai): 未テスト
    }
    // 値計算
    int32_t tmp_value = 0;
    for (PacketBuffer::const_iterator it = (start_iterator + 1);  // 符号読み飛ばし
         it != end_iterator; ++it) {
      v = *it;
      if ((v < '0') || (v > '9')) {
        return false;  // TODO(kitsunai): 未テスト
      }
      v -= '0';
      tmp_value = tmp_value * 10 + v;
    }
    if (!is_positive_number) {
      tmp_value *= -1;
    }
    if (tmp_value > std::numeric_limits<Output>::max()) {
      tmp_value = std::numeric_limits<Output>::max();
    } else if (tmp_value < std::numeric_limits<Output>::min()) {
      tmp_value = std::numeric_limits<Output>::min();
    }

    value_ = static_cast<Output>(tmp_value);
    return true;
  }

 private:
  Output& value_;        //!< デコードした値の代入先の参照
  const size_t length_;  //!< 桁数
};

/**
 * @brief Uint型のデコーダ
 */
class ElementStringDecoder : public IElementDecoder {
 private:
  ElementStringDecoder(ElementStringDecoder const&);             // = delete;
  ElementStringDecoder& operator=(ElementStringDecoder const&);  // = delete;

 public:
  /**
   * @brief コンストラクタ
   * @param value デコードした値の代入先の参照
   * @param length 桁数
   */
  ElementStringDecoder(std::string& value, const size_t length) : value_(value), length_(length) {
    value_.reserve(length_ + 1);
  }
  /**
   * @brief デストラクタ
   */
  virtual ~ElementStringDecoder() {}
  /**
   * @brief デコード
   * @param start_iterator 読み出し先頭の文字のイテレータ
   * @param end_iterator 読み出す最後の文字の次のイテレータ
   * @return 成功時 true
   */
  inline virtual bool Decode(const PacketBuffer::const_iterator start_iterator,
                             const PacketBuffer::const_iterator end_iterator) {
    // 桁数チェック
    if ((std::distance(start_iterator, end_iterator) != static_cast<int32_t>(length_)) ||
        (value_.capacity() < length_)) {
      return false;
    }
    // 代入処理
    value_.clear();
    std::copy((start_iterator), end_iterator, std::back_inserter(value_));
    return true;
  }

 private:
  std::string& value_;   //!< デコードした値の代入先の参照
  const size_t length_;  //!< 桁数
};

}  // namespace hsrb_power_ecu
#endif  // POWER_ECU_COM_ELEMENT_DECODER_HPP_
