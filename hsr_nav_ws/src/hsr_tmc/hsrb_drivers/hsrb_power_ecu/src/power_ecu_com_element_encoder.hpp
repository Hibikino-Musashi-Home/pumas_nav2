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
#ifndef HSRB_POWER_ECU_POWER_ECU_COM_ELEMENT_ENCODER_HPP_
#define HSRB_POWER_ECU_POWER_ECU_COM_ELEMENT_ENCODER_HPP_

#include <algorithm>
#include <string>

#include <boost/foreach.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <hsrb_power_ecu/i_network.hpp>
#include "power_ecu_com_frame_encoder.hpp"

namespace hsrb_power_ecu {

/**
 * @brief 16進数のパケット用エンコーダ
 * 内部でuint32_tを使った変換を行っているため、
 * 桁数の上限は8桁
 */
template <class Input>
class ElementHexUintEncoder : public IElementEncoder {
 private:
  ElementHexUintEncoder(ElementHexUintEncoder const&);             // = delete;
  ElementHexUintEncoder& operator=(ElementHexUintEncoder const&);  // = delete;

 public:
  /**
   * @brief コンストラクタ
   * @param value 変換元の値
   * @param length 変換後の文字列の桁数
   */
  ElementHexUintEncoder(const Input& value, const size_t length) : value_(value), length_(length) {
    convert_buffer_.reserve(length + 2);  // 桁数+\0+'h'
  }
  /**
   * @brief デストラクタ
   */
  virtual ~ElementHexUintEncoder() {}

  /**
   * @brief エンコード
   * @param[out] buffer 出力先のバッファ
   * @return エンコード成功時 true
   */
  inline virtual bool Encode(PacketBuffer& buffer) {
    uint32_t current_value = static_cast<uint32_t>(value_);
    convert_buffer_.clear();

    // 16進文字を下の桁から作る
    for (size_t i = 0; i < length_; ++i) {
      uint32_t v = current_value % 0x10;
      char h;
      if (v > 15) {
        h = '0';  // TODO(kitsunai): 未テスト
      } else if (v < 10) {
        h = v + '0';
      } else {
        h = v + 'A' - 10;
      }
      convert_buffer_.push_back(h);
      current_value /= 0x10;
    }
    convert_buffer_.push_back('h');
    if (current_value != 0) {
      return false;
    }

    // 逆さまに代入
    std::copy(convert_buffer_.rbegin(), convert_buffer_.rend(), std::back_inserter(buffer));
    return true;
  }

 private:
  const Input& value_;          //!< エンコードする値
  const size_t length_;         //!< 桁数
  std::string convert_buffer_;  //!< 変換用バッファ
};

/**
 * @brief 16進メッセージをビット指定で作成するクラス
 * 内部でuint32_tを使った変換を行っているため、
 * 桁数の上限は8桁
 */
class ElementHexUintBitsEncoder : public ElementHexUintEncoder<uint32_t> {
 private:
  ElementHexUintBitsEncoder(ElementHexUintBitsEncoder const&);             // = delete;
  ElementHexUintBitsEncoder& operator=(ElementHexUintBitsEncoder const&);  // = delete;

  typedef boost::unordered_map<uint32_t, const bool*> BitmapType;

 public:
  /**
   * @brief コンストラクタ
   * @param length 変換後の文字列の桁数
   */
  explicit ElementHexUintBitsEncoder(const size_t length)
      : ElementHexUintEncoder<uint32_t>(value_, length), length_(length) {}
  /**
   * @brief デストラクタ
   */
  virtual ~ElementHexUintBitsEncoder() {}

  /**
   * @brief エンコード
   * @param[out] buffer 出力先のバッファ
   * @return エンコード成功時 true
   */
  inline virtual bool Encode(PacketBuffer& buffer) {
    value_ = 0;
    BOOST_FOREACH (BitmapType::value_type const pair, bit_map_) {
      if (*pair.second) {
        value_ += (1 << pair.first);
      }
    }
    return ElementHexUintEncoder<uint32_t>::Encode(buffer);
  }

  /**
   * @brief ビット番地と評価対象となるbool値を登録
   *
   * @param bit 送信時の最下位ビット(通信仕様書では最上位bit)を0としたときの番地
   * @param flag 評価対象のbool値
   *
   * @return
   */
  inline bool RegisterBit(const uint32_t bit, const bool* const flag) {
    if (bit > ((length_ * 4) - 1)) {
      return false;  // TODO(kitsunai): 未テスト
    }
    if (bit_map_.count(bit) != 0) {
      return false;  // TODO(kitsunai): 未テスト
    }
    bit_map_[bit] = flag;
    return true;
  }

 private:
  size_t length_;
  BitmapType bit_map_;  //!< ビット番地とフラグ値のマップ
  uint32_t value_;      //!< バッファ
};

/**
 * @brief 10進数のパケット用エンコーダ
 * 内部でuint32_tを使った変換を行っているため、
 * 桁数の上限は9桁
 */
template <class Input>
class ElementUintEncoder : public IElementEncoder {
 private:
  ElementUintEncoder(ElementUintEncoder const&);             // = delete;
  ElementUintEncoder& operator=(ElementUintEncoder const&);  // = delete;

 public:
  /**
   * @brief コンストラクタ
   * @param value 変換元の値
   * @param length 変換後の文字列の桁数
   */
  ElementUintEncoder(const Input& value, const size_t length) : value_(value), length_(length) {
    convert_buffer_.reserve(length + 1);  // 桁数+\0
  }
  /**
   * @brief デストラクタ
   */
  virtual ~ElementUintEncoder() {}

  /**
   * @brief エンコード
   * @param[out] buffer 出力先のバッファ
   * @return エンコード成功時 true
   */
  inline virtual bool Encode(PacketBuffer& buffer) {
    uint32_t current_value = static_cast<uint32_t>(value_);
    convert_buffer_.clear();

    // 下の桁から作成
    for (size_t i = 0; i < length_; ++i) {
      uint32_t v = current_value % 10;
      char d;
      if (v > 10) {
        d = '0';  // TODO(kitsunai): 未テスト
      } else {
        d = v + '0';
      }
      convert_buffer_.push_back(d);
      current_value /= 10;
    }
    if (current_value != 0) {
      return false;
    }

    // 逆さまに代入
    std::copy(convert_buffer_.rbegin(), convert_buffer_.rend(), std::back_inserter(buffer));
    return true;
  }

 private:
  const Input& value_;          //!< エンコードする値
  const size_t length_;         //!< 桁数
  std::string convert_buffer_;  //!< 変換用バッファ
};

class ElementStringEncoder : public IElementEncoder {
 private:
  ElementStringEncoder(ElementStringEncoder const&);             // = delete;
  ElementStringEncoder& operator=(ElementStringEncoder const&);  // = delete;

 public:
  /**
   * @brief コンストラクタ
   * @param value 変換元の値
   * @param length 変換後の文字列の桁数
   */
  ElementStringEncoder(const std::string& value, const size_t length) : value_(value), length_(length) {}
  /**
   * @brief デストラクタ
   */
  virtual ~ElementStringEncoder() {}

  /**
   * @brief エンコード
   * @param[out] buffer 出力先のバッファ
   * @return エンコード成功時 true
   */
  inline virtual bool Encode(PacketBuffer& buffer) {
    if (value_.size() != length_) return false;
    std::copy(value_.begin(), value_.end(), std::back_inserter(buffer));
    return true;
  }

 private:
  const std::string& value_;  //!< エンコードする値
  const size_t length_;       //!< 桁数
};
}  // namespace hsrb_power_ecu
#endif  // HSRB_POWER_ECU_POWER_ECU_COM_ELEMENT_ENCODER_HPP_
