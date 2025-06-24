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
#ifndef HSRB_POWER_ECU_POWER_ECU_COM_FRAME_DECODER_HPP_
#define HSRB_POWER_ECU_POWER_ECU_COM_FRAME_DECODER_HPP_

#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/unordered/unordered_map.hpp>

#include "any_type_pointer_map.hpp"
#include "power_ecu_com_common.hpp"

namespace hsrb_power_ecu {
/**
 * @brief パケット要素のデコーダのインターフェース
 */
class IElementDecoder {
 public:
  /**
   * @brief IElementDecoderのスマートポインタ
   */
  typedef boost::shared_ptr<IElementDecoder> Ptr;
  /**
   * @brief デストラクタ
   */
  virtual ~IElementDecoder() {}
  /**
   * @brief デコード
   * @param[in] start_iterator パケットの先頭
   * @param[in] end_iterator パケットの終点
   * @return デコード成功時 true
   */
  virtual bool Decode(const PacketBuffer::const_iterator start_iterator,
                      const PacketBuffer::const_iterator end_iterator) = 0;
};

/**
 * @brief データデコーダのインターフェース
 */
class IPowerEcuComDataDecoder {
 private:
  IPowerEcuComDataDecoder(IPowerEcuComDataDecoder const&);             // = delete;
  IPowerEcuComDataDecoder& operator=(IPowerEcuComDataDecoder const&);  // = delete;

 protected:
  /**
   * @brief コンストラクタ
   * @param packet_size パケットヘッダ部のサイズ
   * @param packet_name パケットヘッダ部のパケット種別
   */
  IPowerEcuComDataDecoder(const size_t packet_size, const std::string& packet_name)
      : packet_size_(packet_size), packet_name_(packet_name) {}

 public:
  /**
   * @brief デストラクタ
   */
  virtual ~IPowerEcuComDataDecoder() {}

  /**
   * @brief デコード
   * @param[in] start_iterator パケットデータ部の先頭イテレータ
   * @param[in] end_iterator パケットデータ部の終点イテレータ
   * @return
   * 正常終了 boost::system::errc::success
   * デコード失敗 boost::system::errc::protocol_error
   */
  virtual boost::system::error_code Decode(const PacketBuffer::const_iterator start_iterator,
                                           const PacketBuffer::const_iterator end_iterator);
  /**
   * @brief パケットヘッダ部のサイズ取得
   * @return パケットサイズ
   */
  inline size_t GetPacketSize() const { return packet_size_; }

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
  /**
   * @brief 事後処理
   * Decode関数の後に呼ばれる関数
   * @return 成功時 true
   */
  inline virtual bool Update() { return true; }

  std::vector<IElementDecoder::Ptr> element_decoder_list_;  //!< 要素ごとのデコード指示リスト
  const size_t packet_size_;                                //!< パケットのサイズ
  const std::string packet_name_;                           //!< パケットの種別
  any_type_pointer_map::Map parameter_map_;                 //!< 制御コマンドのパラメータMap
};

/**
 * @brief フレームデコーダ
 * 通信フォーマットをframe, data, elementにの要素に分類分けしている。
 *
 * 例) コマンド"H,ledc_,23,000,100,255,h12345678,\0" の場合、
 * - frame :
 *   通信パケット全体を指す 例) "H,ledc_,23,000,100,255,h12345678,\0"
 *   フレームデコーダはコマンドのヘッダ、フッダの計算を担当し、
 *   フレームデコーダは通信パケットのヘッダ、フッダの仕様を知っており、
 *   コマンド名をキーとてデータデコーダを管理している。
 *
 * - data :
 *   通信パケットのヘッダ、フッダを取り除いたものを示す、 例) "000,100,255,"
 *   データデコーダはdataのelementに切り分けと、エレメントデコーダの呼び出しを行う。
 *   また、全てのelementをデコードした後、物理量への変換など後処理を行う。
 *   データデコーダ、はコマンド毎に定義され、コマンド名、コマンドサイズ、
 *   elementの構成(並び順、各elementの型と桁数)の他、
 *   デコードした情報の格納するPacketDataの参照を知っている。
 *   また、コマンド毎にデコードの後の格納先となる情報をPacketData型として持つ
 *
 * - element :
 *   通信パケットのヘッダ、フッダを取り除いたものを示す 例) "000"
 *   エレメントデコーダはパケット文字列<=>各要素の型の変換を行う。
 *   エレメントデコーダは表記(符号付き10進、16進等)毎に定義され、
 *   各要素のフォーマット(符号10進は[+-][0-9]+)を知っている。
 *
 * それぞれのデコーダはframe->data->elementの親子関係を持つ。
 * また、デコーダとエンコーダの設計は対象性がある。
 *
 * RobotHWは受信バッファをフレームエンコーダのDecodeメソッドを呼び出しデコードを行う。
 * デコード結果はデータデコーダに紐付けたPacketDataに自動的に格納される。
 *
 */
class PowerEcuComFrameDecoder {
 public:
  typedef boost::shared_ptr<hsrb_power_ecu::IPowerEcuComDataDecoder> DataDecoderType;

 private:
  PowerEcuComFrameDecoder(PowerEcuComFrameDecoder const&);                    // = delete;
  PowerEcuComFrameDecoder& operator=(PowerEcuComFrameDecoder const&);         // = delete;
  typedef boost::unordered_map<std::string, DataDecoderType> DataDecoderMap;  //!< データデコーダの辞書型

 public:
  /**
   * @brief コンストラクタ
   */
  PowerEcuComFrameDecoder();
  /**
   * @brief デコード
   * パケット受信バッファを先頭から読み込み、先頭のパケット1つをデコードする。
   * パケット受信バッファの書き換えは行わず、デコード済みのインデックスをencoded_iteratorで返す。
   * @param[in] start_iterator デコード対象の先頭イテレータ
   * @param[in] end_iterator デコード対象の終点イテレータ
   * @param[out] encoded_iterator デコード済みの場所を示すイテレータ
   * @return
   * 正常終了 boost::system::errc::success
   * デコード完了 boost::system::errc::result_out_of_range
   * デコードできないパケットがあった boost::system::errc::protocol_error
   */
  boost::system::error_code Decode(const hsrb_power_ecu::PacketBuffer::const_iterator& start_iterator,
                                   const hsrb_power_ecu::PacketBuffer::const_iterator& end_iterator,
                                   hsrb_power_ecu::PacketBuffer::const_iterator& encoded_iterator);
  /**
   * @brief データデコーダ登録
   * @param [in] decoder 登録するデコーダ
   * @return
   * 成功時 boost::system::errc::success
   * 登録済み or nullptr boost::system::errc::invalid_argument
   */
  boost::system::error_code RegisterDataDecoder(DataDecoderType decoder);

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
    BOOST_FOREACH (DataDecoderMap::value_type const decoder, data_decoder_map_) {
      T* ret = decoder.second->GetParamPtr<T>(name);
      if (ret != NULL) {
        return ret;
      }
    }
    return NULL;
  }

 private:
  /**
   * @brief 文字列を取得
   * @param[in] start_it 先頭のイテレータ
   * @param[in] end_it 終点のイテレータ
   * @param[in] size 読み出しサイズ
   * @param[out] output_string 取得した文字列の格納先
   * @return 成功時 True
   */
  bool GetString(const size_t size, const hsrb_power_ecu::PacketBuffer::const_iterator& end_it,
                 hsrb_power_ecu::PacketBuffer::const_iterator& start_it, std::string& output_string);
  /**
   * @brief 文字列をスキップ
   * @param[in] skip_string スキップする文字列
   * @param[out] it 先頭のイテレータ
   * @return スキップした文字列が、引数で受け取った文字列と等しいなら true
   */
  bool SkipString(const std::string& skip_string, hsrb_power_ecu::PacketBuffer::const_iterator& it);

  DataDecoderMap data_decoder_map_;      //!< データデコーダの辞書
  std::string packet_name_buffer_;       //!< パケットデコード時のヘッダ名格納用バッファ
  std::string packet_size_buffer_;       //!< パケットデコード時のパケットサイズ格納用バッファ
  std::string packet_check_sum_buffer_;  //!< パケットデコード時のチェックサム格納用バッファ
};
}  // namespace hsrb_power_ecu
#endif  // HSRB_POWER_ECU_POWER_ECU_COM_FRAME_DECODER_HPP_
