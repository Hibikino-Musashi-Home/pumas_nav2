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
#include "power_ecu_com_frame_encoder.hpp"

#include <algorithm>
#include <string>

#include <boost/make_shared.hpp>
#include <boost/range/algorithm.hpp>
#include "power_ecu_com_element_encoder.hpp"

namespace hsrb_power_ecu {

PowerEcuComFrameEncoder::PowerEcuComFrameEncoder() : check_sum_(0) {
  // チェックサム作成用のバッファ確保
  check_sum_encoder_ = boost::make_shared<hsrb_power_ecu::ElementHexUintEncoder<uint32_t> >(check_sum_, 8);
}

PowerEcuComFrameEncoder::~PowerEcuComFrameEncoder() {}

/**
 * @brief エンコード
 * @param[out] buffer     出力先のバッファ
 * @param[in] packet_name データデコーダ検索用のエンコードするパケットの名前
 * @return 成功時 boost::system::error::success
 */
boost::system::error_code PowerEcuComFrameEncoder::Encode(PacketBuffer &buffer, const std::string &packet_name) {
  //// 対応するエンコーダを検索
  DataEncoderMap::iterator map_it = data_encoder_map_.find(packet_name);
  if (map_it == data_encoder_map_.end()) {
    return boost::system::errc::make_error_code(boost::system::errc::invalid_argument);
  }

  // フレーム作成
  //// 先頭文字列
  buffer.push_back('H');
  buffer.push_back(',');

  //// ヘッダ名
  std::copy(packet_name.begin(), packet_name.end(), std::back_inserter(buffer));
  buffer.push_back(',');

  //// サイズ
  const std::string packet_size = map_it->second->GetPacketSizeStr();
  std::copy(packet_size.begin(), packet_size.end(), std::back_inserter(buffer));
  buffer.push_back(',');

  // データ部エンコード
  //// データ部エンコード
  map_it->second->Encode(buffer);

  // チェックサム作成
  check_sum_ = hsrb_power_ecu::com_common::CalculateCrc32(buffer.begin(), buffer.end());
  check_sum_encoder_->Encode(buffer);
  buffer.push_back(',');
  buffer.push_back('\n');

  return boost::system::errc::make_error_code(boost::system::errc::success);
}

/**
 * @brief データエンコーダ登録
 * @param[in] encoder 登録するエンコーダ
 * @return 成功時 boost::system::errc::success
 */
boost::system::error_code PowerEcuComFrameEncoder::RegisterDataEncoder(DataEncoderType encoder) {
  // 無効なポインタ、データエンコーダが重複しているときエラー
  if (encoder == NULL || data_encoder_map_.find(encoder->GetPacketName()) != data_encoder_map_.end()) {
    return boost::system::errc::make_error_code(boost::system::errc::invalid_argument);
  }

  // 登録
  data_encoder_map_[encoder->GetPacketName()] = encoder;

  return boost::system::errc::make_error_code(boost::system::errc::success);
}

}  // namespace hsrb_power_ecu
