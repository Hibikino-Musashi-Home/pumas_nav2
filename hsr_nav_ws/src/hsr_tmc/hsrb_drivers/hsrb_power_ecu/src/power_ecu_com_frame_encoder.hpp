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
#ifndef POWER_ECU_COM_FRAME_ENCODER_HPP_
#define POWER_ECU_COM_FRAME_ENCODER_HPP_

#include <stdint.h>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/system/error_code.hpp>
#include <boost/unordered/unordered_map.hpp>

#include "any_type_pointer_map.hpp"
#include "power_ecu_com_common.hpp"

namespace hsrb_power_ecu {
/**
 * @brief パケット要素のエンコーダのインターフェース
 */
class IElementEncoder {
 public:
  /**
   * @brief IElementEncoderのスマートポインタ
   */
  typedef boost::shared_ptr<IElementEncoder> Ptr;
  /**
   * @brief デストラクタ
   */
  virtual ~IElementEncoder() {}
  /**
   * @brief エンコード
   * @param[out] buffer 出力先のバッファ
   * @return エンコード成功時 true
   */
  virtual bool Encode(PacketBuffer &buffer) = 0;
};

/**
 * @brief データエンコーダのインターフェース
 */
class IPowerEcuComDataEncoder {
 private:
  IPowerEcuComDataEncoder(IPowerEcuComDataEncoder const &);             // = delete;
  IPowerEcuComDataEncoder &operator=(IPowerEcuComDataEncoder const &);  // = delete;

 protected:
  /**
   * @brief コンストラクタ
   * @param packet_size パケットヘッダ部のサイズ
   * @param packet_name パケットヘッダ部のパケット種別
   */
  IPowerEcuComDataEncoder(const std::string &packet_size, const std::string &packet_name)
      : packet_size_(packet_size), packet_name_(packet_name) {}

 public:
  /**
   * @brief デストラクタ
   */
  virtual ~IPowerEcuComDataEncoder() {}
  /**
   * @brief エンコード
   * @param[out] buffer 出力先のバッファ
   * @return
   * 正常終了 boost::system::errc::success
   * エンコード失敗 boost::system::errc::protocol_error
   */
  virtual inline boost::system::error_code Encode(PacketBuffer &buffer) {
    BOOST_FOREACH (IElementEncoder::Ptr const p, element_encoder_list_) {
      if (!p->Encode(buffer)) {
        return boost::system::errc::make_error_code(
            boost::system::errc::protocol_error);  // TODO(kitsunai): テスト未実施
      }
      buffer.push_back(',');
    }
    return boost::system::errc::make_error_code(boost::system::errc::success);
  }
  /**
   * @brief パケットヘッダ部のサイズ取得
   * @return パケットサイズ
   */
  virtual inline std::string GetPacketSizeStr() const { return packet_size_; }
  /**
   * @brief パケットヘッダ部のパケット種別取得
   * @return パケット種別
   */
  inline std::string GetPacketName() const { return packet_name_; }

  /**
   * @brief データのポインタを取得
   *
   * @tparam T データの型
   * @param[in] name データの名前
   *
   * @return 成功時 : データのポインタ<br>
   *         失敗時 : NULL<br>
   *         未登録のデータ名、データの型が不一致の場合失敗となる
   */
  template <typename T>
  T* GetParamPtr(const std::string& name) const {
    return parameter_map_.GetPtr<T>(name);
  }

 protected:
  std::vector<IElementEncoder::Ptr> element_encoder_list_;  //!< 要素毎のエンコード指示リスト
  const std::string packet_size_;                           //!< ヘッダ部のパケットサイズ
  const std::string packet_name_;                           //!< ヘッダ部のパケット種別
  any_type_pointer_map::Map parameter_map_;                 //!< 制御コマンドのパラメータMap
};

/**
 * @brief 送信パケットのフレームエンコーダ
 * 通信フォーマットをframe, data, elementにの要素に分類分けしている。
 *
 * 例) コマンド"H,ledc_,23,000,100,255,h12345678,\0" の場合、
 * - frame :
 *   通信パケット全体を指す 例) "H,ledc_,23,000,100,255,h12345678,\0"
 *   フレームエンコーダはコマンドのヘッダ、フッダの計算を担当し、
 *   フレームエンコーダは通信パケットのヘッダ、フッダの仕様を知っており、
 *   コマンド名をキーとてデータエンコーダを管理している。
 *
 * - data :
 *   通信パケットのヘッダ、フッダを取り除いたものを示す、 例) "000,100,255,"
 *   データエンコーダはdataのelementに切り分けと、エレメントエンコーダの呼び出しを行う。
 *   また、全てのelementをエンコードした後、物理量への変換など後処理を行う。
 *   データエンコーダはコマンド毎に定義され、コマンド名、コマンドサイズ、
 *   elementの構成(並び順、各elementの型と桁数)の他、
 *   エンコードに必要な情報を格納したPacketDataの参照を知っている。
 *   また、コマンド毎にエンコードの元となる情報をPacketData型として持つ。
 *
 * - element :
 *   通信パケットのヘッダ、フッダを取り除いたものを示す 例) "000"
 *   エレメントエンコーダはパケット文字列<=>各要素の型の変換を行う。
 *   エレメントエンコーダは表記(符号付き10進、16進等)毎に定義され、
 *   各要素のフォーマット(符号10進は[+-][0-9]+)を知っている。
 *
 * それぞれのエンコーダはframe->data->elementの親子関係を持つ。
 * また、エンコーダとデコーダの設計は対象性がある。
 *
 * RobotHWはPacketDataの値の更新した後
 * フレームエンコーダのEncodeメソッドを呼び出しエンコードを行う。
 *
 */
class PowerEcuComFrameEncoder {
 public:
  typedef boost::shared_ptr<hsrb_power_ecu::IPowerEcuComDataEncoder> DataEncoderType;

 private:
  PowerEcuComFrameEncoder(PowerEcuComFrameEncoder const &);             // = delete;
  PowerEcuComFrameEncoder &operator=(PowerEcuComFrameEncoder const &);  // = delete;

  typedef boost::unordered_map<std::string, DataEncoderType> DataEncoderMap;  //!< データエンコーダの辞書

 public:
  /**
   * @brief コンストラクタ
   */
  PowerEcuComFrameEncoder();
  /**
   * @brief デストラクタ
   */
  ~PowerEcuComFrameEncoder();
  /**
   * @brief エンコード
   * @param[out] buffer     出力先のバッファ
   * @param[in] packet_name データ検索用のエンコードするパケットの名前
   * @return 成功時 boost::system::error::success
   */
  boost::system::error_code Encode(PacketBuffer &buffer, const std::string &packet_name);
  /**
   * @brief データエンコーダ登録
   * @param[in] encoder 登録するエンコーダ
   * @return 成功時 boost::system::errc::success
   */
  boost::system::error_code RegisterDataEncoder(DataEncoderType encoder);

  /**
   * @brief データのポインタを取得
   *
   * @tparam T データの型
   * @param[in] name データの名前
   *
   * @return 成功時 : データのポインタ<br>
   *         失敗時 : NULL<br>
   *         未登録のデータ名、データの型が不一致の場合失敗となる
   */
  template <typename T>
  T* GetParamPtr(const std::string& name) const {
    BOOST_FOREACH (DataEncoderMap::value_type const encoder, data_encoder_map_) {
      T* ret = encoder.second->GetParamPtr<T>(name);
      if (ret != NULL) {
        return ret;
      }
    }
    return NULL;
  }

 private:
  DataEncoderMap data_encoder_map_;         //!< データエンコーダの辞書
  uint32_t check_sum_;                      //!< チェックサム計算用の一時格納データ
  IElementEncoder::Ptr check_sum_encoder_;  //!< チェックサム値を文字列にするエンコーダ
};
}  // namespace hsrb_power_ecu
#endif  // POWER_ECU_COM_FRAME_ENCODER_HPP_
