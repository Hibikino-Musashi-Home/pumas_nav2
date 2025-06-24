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

#ifndef HSRB_POWER_ECU_POWER_ECU_COM_REPRO_DATA_ENCODER_HPP_
#define HSRB_POWER_ECU_POWER_ECU_COM_REPRO_DATA_ENCODER_HPP_

#include <algorithm>
#include <iomanip>
#include <string>

#include "power_ecu_com_common.hpp"
#include "power_ecu_com_element_encoder.hpp"
#include "power_ecu_com_frame_encoder.hpp"
#include "power_ecu_protocol.hpp"

namespace hsrb_power_ecu {

/**
 * @brief リプロ開始コマンド
 */
class PowerEcuComRprosDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComRprosDataEncoder(PowerEcuComRprosDataEncoder const&);             // = delete;
  PowerEcuComRprosDataEncoder& operator=(PowerEcuComRprosDataEncoder const&);  // = delete;

 public:
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComRprosDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComRprosDataEncoder() {}
  /**
   * @brief パラメータ登録
   *
   * @param[out] parameter_map 登録先
   */
  virtual bool RegisterParameter(hsrb_power_ecu::any_type_pointer_map::Map& parameter_map);

 private:
  std::string repro_version_;
  uint8_t reserve_;
};

/**
 * @brief 時計合わせコマンド
 */
class PowerEcuComRprodDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComRprodDataEncoder(PowerEcuComRprodDataEncoder const&);             // = delete;
  PowerEcuComRprodDataEncoder& operator=(PowerEcuComRprodDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    std::string start_time;  //!< 現在時刻
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComRprodDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComRprodDataEncoder() {}
  /**
   * @brief パラメータ登録
   *
   * @param[out] parameter_map 登録先
   */
  virtual bool RegisterParameter(hsrb_power_ecu::any_type_pointer_map::Map& parameter_map);
  /**
   * @brief エンコード
   * @param[out] buffer 出力先のバッファ
   * @return
   * 正常終了 boost::system::errc::success
   * エンコード失敗 boost::system::errc::protocol_error
   */
  virtual inline boost::system::error_code Encode(PacketBuffer& buffer) {
    std::copy(repro_data_.begin(), repro_data_.end(), std::back_inserter(buffer));
    return boost::system::errc::make_error_code(boost::system::errc::success);
  }
  /**
   * @brief パケットヘッダ部のサイズ取得
   * @return パケットサイズ
   */
  virtual inline std::string GetPacketSizeStr() const {
    std::stringstream sst;
    sst << std::dec << std::setw(3) << std::setfill('0') << (repro_data_.size() + 11);
    return sst.str();
  }

 private:
  std::string repro_data_;
};

/**
 * @brief リプログラム終了コマンド
 */
class PowerEcuComRproeDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComRproeDataEncoder(PowerEcuComRproeDataEncoder const&);             // = delete;
  PowerEcuComRproeDataEncoder& operator=(PowerEcuComRproeDataEncoder const&);  // = delete;
 public:
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComRproeDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComRproeDataEncoder() {}
  /**
   * @brief パラメータ登録
   *
   * @param[out] parameter_map 登録先
   */
  virtual bool RegisterParameter(hsrb_power_ecu::any_type_pointer_map::Map& parameter_map);

 private:
  uint8_t reserve_;
};

}  // namespace hsrb_power_ecu

#endif  // HSRB_POWER_ECU_POWER_ECU_COM_REPRO_DATA_ENCODER_HPP_
