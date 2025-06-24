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
#include "power_ecu_com_frame_decoder.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>

#include <boost/algorithm/minmax_element.hpp>
#include <boost/crc.hpp>
#include <boost/foreach.hpp>

#include <rclcpp/rclcpp.hpp>

#include "power_ecu_com_common.hpp"
#include "ros2_msg_utils.hpp"


namespace hsrb_power_ecu {
/**
 * @brief デコード
 * @param[in] start_iterator パケットデータ部の先頭イテレータ
 * @param[in] end_iterator パケットデータ部の終点イテレータ
 * @return
 * 正常終了 boost::system::errc::success
 * デコード失敗 boost::system::errc::protocol_error
 */
boost::system::error_code IPowerEcuComDataDecoder::Decode(const PacketBuffer::const_iterator start_iterator,
                                                          const PacketBuffer::const_iterator end_iterator) {
  // デコードスタート
  PacketBuffer::const_iterator current_iterator = start_iterator;
  BOOST_FOREACH (IElementDecoder::Ptr const decoder, element_decoder_list_) {
    // 次のカンマのイテレータを取得
    PacketBuffer::const_iterator const element_end_iterator = std::find(current_iterator, end_iterator, ',');
    // カンマが見つからなかったらエラー
    if (element_end_iterator == end_iterator) {
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
    // 要素デコード
    bool const ret = decoder->Decode(current_iterator, element_end_iterator);

    if (!ret) {
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
    // カンマの次のイテレータを先頭のイテレータに指定
    current_iterator = element_end_iterator + 1;
  }

  if (!Update()) {
    return boost::system::errc::make_error_code(boost::system::errc::protocol_error);  // TODO(kitsunai): テスト未実施
  }
  return boost::system::errc::make_error_code(boost::system::errc::success);
}

/**
 * @brief コンストラクタ
 */
PowerEcuComFrameDecoder::PowerEcuComFrameDecoder()
    : data_decoder_map_(), packet_name_buffer_(), packet_size_buffer_(), packet_check_sum_buffer_() {
  // バッファの確保
  packet_name_buffer_.reserve(hsrb_power_ecu::com_common::kPacketNameBufferSize);
  packet_size_buffer_.reserve(hsrb_power_ecu::com_common::kPacketSizeBufferSize);
  packet_check_sum_buffer_.reserve(hsrb_power_ecu::com_common::kPacketCheckSumBufferSize);
}

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
boost::system::error_code PowerEcuComFrameDecoder::Decode(
    const hsrb_power_ecu::PacketBuffer::const_iterator& start_iterator,
    const hsrb_power_ecu::PacketBuffer::const_iterator& end_iterator,
    hsrb_power_ecu::PacketBuffer::const_iterator& encoded_iterator) {
  // パケット先頭文字を検索する
  hsrb_power_ecu::PacketBuffer::const_iterator current_iterator = std::find(start_iterator, end_iterator, 'E');
  if (current_iterator == end_iterator) {
    // バッファ中に'E'がない=読み込み中のメッセージは存在しない
    encoded_iterator = end_iterator;
    return boost::system::errc::make_error_code(boost::system::errc::result_out_of_range);
  }
  hsrb_power_ecu::PacketBuffer::const_iterator const last_iterator = current_iterator;

  // ヘッダ解析
  size_t packet_size = 0;
  //// ヘッダサイズ分の受信が終わっているか確認
  if (static_cast<uint32_t>(std::distance(current_iterator, end_iterator)) <
      hsrb_power_ecu::com_common::kPacketHeaderLength) {
    encoded_iterator = last_iterator;
    return boost::system::errc::make_error_code(boost::system::errc::result_out_of_range);
  } else {
    // パケットヘッダ
    if (!SkipString("E,", current_iterator)) {
      encoded_iterator = current_iterator;
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
    // パケット種別
    //// 予めバッファサイズを確認しているので、falseが返ることは想定しない
    bool ret;
    ret = GetString(5, end_iterator, current_iterator, packet_name_buffer_);
    hsrb_power_ecu::Assert(ret, "GetString return false.");
    if (!SkipString(",", current_iterator)) {
      encoded_iterator = current_iterator;
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
    // パケットサイズ
    ret = GetString(3, end_iterator, current_iterator, packet_size_buffer_);
    hsrb_power_ecu::Assert(ret, "GetString return false.");
    packet_size = std::atoi(packet_size_buffer_.c_str());
    if (packet_size < 11) {  // フッタ長(11)未満のパケットは仕様上ありえない
      encoded_iterator = current_iterator;
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
    if (!SkipString(",", current_iterator)) {
      encoded_iterator = current_iterator;
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
  }

  // パケットの受信が不完全な場合result_out_of_rangeを返す
  if (static_cast<uint32_t>(std::distance(current_iterator, end_iterator)) < packet_size) {
    encoded_iterator = last_iterator;
    return boost::system::errc::make_error_code(boost::system::errc::result_out_of_range);
  }

  // フッタ解析
  size_t const footer_length = hsrb_power_ecu::com_common::kPacketFooterLength;
  size_t const frame_size = hsrb_power_ecu::com_common::kPacketHeaderLength + packet_size;

  // パケットのチェックサム計算
  uint32_t const check_sum_calc = hsrb_power_ecu::com_common::CalculateCrc32(
      last_iterator,                                    // 先頭文字列
      current_iterator + packet_size - footer_length);  // 読み込む文字の次のイテレータ

  // チェックサム文字列取得
  hsrb_power_ecu::PacketBuffer::const_iterator checksum_iterator =
      current_iterator + packet_size - footer_length + 1;  // チェックサム文字列先頭の"h"を読みこまない
  bool ret = GetString(footer_length - 3,              // "h" + ",\n"を読まない
                       end_iterator, checksum_iterator, packet_check_sum_buffer_);
  hsrb_power_ecu::Assert(ret, "GetString resturn false.");
  uint32_t const check_sum_res = static_cast<uint32_t>(std::strtol(packet_check_sum_buffer_.c_str(), NULL, 16));

  // チェックサム比較
  if (check_sum_calc != check_sum_res) {
    encoded_iterator = last_iterator + frame_size;
    return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
  }

  // 対応するデコーダを検索
  DataDecoderMap::iterator const map_it = data_decoder_map_.find(packet_name_buffer_);

  // 対応するデコーダが存在しない時、protocol_errorを返す
  if ((map_it == data_decoder_map_.end()) || (map_it->second->GetPacketSize() != packet_size)) {
    encoded_iterator = last_iterator + frame_size;
    return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
  }

  // デコード
  boost::system::error_code const result =
      map_it->second->Decode(current_iterator,
                             (current_iterator + packet_size - footer_length));  // 読み込む文字の次のイテレータ

  encoded_iterator = last_iterator + frame_size;
  return result;
}

/**
 * @brief データデコーダ登録
 * @param [in] decoder 登録するデコーダ
 * @return
 * 成功時 boost::system::errc::success
 * 登録済み or nullptr boost::system::errc::invalid_argument
 */
boost::system::error_code PowerEcuComFrameDecoder::RegisterDataDecoder(DataDecoderType decoder) {
  if (decoder == NULL || data_decoder_map_.find(decoder->GetPacketName()) != data_decoder_map_.end()) {
    return boost::system::errc::make_error_code(boost::system::errc::invalid_argument);
  }

  data_decoder_map_[decoder->GetPacketName()] = decoder;

  return boost::system::errc::make_error_code(boost::system::errc::success);
}

/**
 * @brief 文字列を取得
 * @param[in] start_it 先頭のイテレータ
 * @param[in] end_it 終点のイテレータ
 * @param[in] size 読み出しサイズ
 * @param[out] output_string 取得した文字列の格納先
 * @return 成功時 True
 */
bool PowerEcuComFrameDecoder::GetString(const size_t size, const hsrb_power_ecu::PacketBuffer::const_iterator& end_it,
                                        hsrb_power_ecu::PacketBuffer::const_iterator& start_it,
                                        std::string& output_string) {
  size_t const distance = std::distance(start_it, end_it);
  if (size > distance) {
    return false;
  }

  // string型の内部バッファが足りないとき警告を表示
  if (output_string.capacity() < distance) {
    output_string.reserve(distance + 1);
  }

  output_string.clear();

  std::copy(start_it, start_it + size, std::back_inserter(output_string));
  start_it += size;
  return true;
}

/**
 * @brief 文字列をスキップ
 * @param[in] skip_string スキップする文字列
 * @param[out] it 先頭のイテレータ
 * @return スキップした文字列が、引数で受け取った文字列と等しいなら true
 */
bool PowerEcuComFrameDecoder::SkipString(const std::string& skip_string,
                                         hsrb_power_ecu::PacketBuffer::const_iterator& it) {
  bool is_equal = true;
  for (size_t i = 0; i < skip_string.length(); ++i) {
    is_equal &= (static_cast<char>(*it) == skip_string[i]);
    ++it;
  }
  return is_equal;
}

}  // namespace hsrb_power_ecu
