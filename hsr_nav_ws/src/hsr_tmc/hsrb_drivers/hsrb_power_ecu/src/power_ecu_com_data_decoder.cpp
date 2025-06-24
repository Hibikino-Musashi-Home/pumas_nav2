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
#include "power_ecu_com_data_decoder.hpp"

#include <cstdlib>

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/smart_ptr/make_shared.hpp>

#include <rclcpp/rclcpp.hpp>

namespace {
const size_t kEcu1DataDecoderPacketSize = 326;        //!< 基本情報パケットのパケットサイズ
const char kEcu1DataDecoderPacketName[] = "ecu1_";    //!< 基本情報パケットのパケット種別
const size_t kEcu1DateLengh = 14 + 1;                 //!< 日時の文字数 + \0
const size_t kEcu1DiagStatusLength = 65 + 1;          //!< ダイアグ情報の文字数 + \0
const size_t kEcu1PowerEcuStatusFlagLength = 3 + 1;   //!< 電源ECUステータスの文字数 + \0
const size_t kEcu1EcuStatusLength = 17 + 1;           //!< 電源ECUステータスの文字数 + \0
const size_t kEcu1GyroStatusLength = 17 + 1;          //!< ジャイロ姿勢角演算ステータスの文字数 + /0
const size_t kEcu1TempStringLength = 9 + 1;           //!< テンポラリ文字列の最大文字数
const size_t kEcu2DataDecoderPacketSize = 291;        //!< オプション情報パケットパケットサイズ
const char kEcu2DataDecoderPacketName[] = "ecu2_";    //!< オプション情報パケットパケット種別
const size_t kEcu2PowerEcuVersionLength = 41 + 1;     //!< 電源ECUファームVerの文字数 + \0
const size_t kEcu2PowerEcuComVersionLength = 41 + 1;  //!< 電源ECU通信構造HASHの文字数 + \0
const size_t kRxackDataDecoderPacketSize = 15;        //!< 返信パケットパケットサイズ
const char kRxackDataDecoderPacketName[] = "rxack";   //!< 返信パケットパケット種別
const size_t kVerDataDecoderPacketSize = 95;          //!< バージョン情報パケットのパケットサイズ
const char kVerDataDecoderPacketName[] = "ver__";     //!< バージョン情報パケットのパケットサイズ
}  // anonymous namespace

namespace hsrb_power_ecu {
/**
 * @brief コンストラクタ
 * @param[in] packet_data パケットデータ
 */
PowerEcuComEcu1DataDecoder::PowerEcuComEcu1DataDecoder()
    : IPowerEcuComDataDecoder(kEcu1DataDecoderPacketSize, kEcu1DataDecoderPacketName), packet_out_() {
  packet_out_.ecu1_date.reserve(kEcu1DateLengh);
  packet_raw_data_.date.reserve(kEcu1DateLengh);
  packet_out_.diag_status.reserve(kEcu1DiagStatusLength);
  packet_raw_data_.diag_status.reserve(kEcu1DiagStatusLength);
  packet_out_.power_ecu_status_flag.reserve(kEcu1PowerEcuStatusFlagLength);
  packet_raw_data_.power_ecu_status_flag.reserve(kEcu1PowerEcuStatusFlagLength);
  packet_out_.gyro_status.reserve(kEcu1GyroStatusLength);
  packet_raw_data_.gyro_status.reserve(kEcu1GyroStatusLength);
  packet_raw_data_.power_ecu_status.reserve(kEcu1EcuStatusLength);
  temp_str.reserve(kEcu1TempStringLength);

  // デコード指示リスト作成
  // パケットデータの先頭からpush_backしていく
  // 第1引数 要素デコードクラスとデコード後の型指定
  // 第2引数 デコード後の代入先
  // 第3引数 桁数
  //!< タイムスタンプ           10進10桁 [ms]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintDecoder<uint32_t> >(
      new hsrb_power_ecu::ElementUintDecoder<uint32_t>(packet_raw_data_.time_stamp, 10)));
  //!< 日時                     文字 （YYYYMMDDhhmmss）
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_raw_data_.date, 14)));
  //!< 電源ECUステータス        16進2桁      -
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_raw_data_.power_ecu_status_flag, 2 + 1)));
  //!< 電源ECU状態 16進数16桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_raw_data_.power_ecu_status, 16 + 1)));
  //!< ダイアグ情報             16進64桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_raw_data_.diag_status, 64 + 1)));
  //!< バッテリ総容量 符号10進5桁  [mAh]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_raw_data_.battery_total_capacity, 5)));
  //!< バッテリ残容量 符号10進5桁  [mAh]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_raw_data_.battery_remaining_capacity, 5)));
  //!< 電流値                   符号10進5桁  [mA]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_raw_data_.electric_current, 5)));
  //!< 電池電圧                 符号10進5桁  [mV]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_raw_data_.battery_voltage, 5)));
  //!< 電池温度                 符号10進3桁  [℃]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int8_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int8_t>(packet_raw_data_.battery_temperature, 3)));
  //!< バッテリ状態フラグ       16進4桁      -
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementHexUintDecoder<uint16_t>(packet_raw_data_.battery_state_flag, 4)));
  //!< バッテリ初期学習容量 10進5桁      [mAh]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementUintDecoder<uint16_t>(packet_raw_data_.battery_initial_learning_capacity, 5)));
  //!< バッテリ異常ステータス   16進4桁 -
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementHexUintDecoder<uint16_t>(packet_raw_data_.battery_error_status, 4)));
  //!< 相対容量                 10進3桁 [%]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintDecoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintDecoder<uint8_t>(packet_raw_data_.battery_relative_capacity, 3)));
  //!< 近接、バンパセンサ状態   16進2桁      -
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintDecoder<uint8_t> >(
      new hsrb_power_ecu::ElementHexUintDecoder<uint8_t>(packet_raw_data_.bumper_status, 2)));
  //!< ジャイロ姿勢各演算ステータス 16進数16桁 uint8*8
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_raw_data_.gyro_status, 16 + 1)));
  //!< quaternion_t             符号10進10桁 x10^-1
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.quaternion_t, 10)));
  //!< quaternion_x             符号10進10桁 x10^-1
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.quaternion_x, 10)));
  //!< quaternion_y             符号10進10桁 x10^-1
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.quaternion_y, 10)));
  //!< quaternion_z             符号10進10桁 x10^-1
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.quaternion_z, 10)));
  //!< 角速度x                  符号10進10桁 x10^-2[rad/s]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.angular_velocity_x, 10)));
  //!< 角速度y                  符号10進10桁 x10^-2[rad/s]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.angular_velocity_y, 10)));
  //!< 角速度z                  符号10進10桁 x10^-2[rad/s]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.angular_velocity_z, 10)));
  //!< 加速度x                  符号10進10桁 x10^-2[m/s^2]
  element_decoder_list_.push_back(
      boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(new hsrb_power_ecu::ElementIntDecoder<int32_t>(
          packet_raw_data_.acceleration_x, 10)));  //!< 加速度y                  符号10進10桁 x10^-2[m/s^2]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.acceleration_y, 10)));
  //!< 加速度z                  符号10進10桁 x10^-2[m/s^2]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.acceleration_z, 10)));
  //!< 自動充電ステータス         16進8桁      -
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintDecoder<uint8_t> >(
      new hsrb_power_ecu::ElementHexUintDecoder<uint8_t>(packet_raw_data_.charger_state, 2)));

  parameter_map_.Register("time_stamp", &packet_out_.time_stamp);
  parameter_map_.Register("ecu1_date", &packet_out_.ecu1_date);
  parameter_map_.Register("power_ecu_status_flag", &packet_out_.power_ecu_status_flag);
  parameter_map_.Register("diag_status", &packet_out_.diag_status);
  parameter_map_.Register("battery_total_capacity", &packet_out_.battery_total_capacity);
  parameter_map_.Register("battery_remaining_capacity", &packet_out_.battery_remaining_capacity);
  parameter_map_.Register("electric_current", &packet_out_.electric_current);
  parameter_map_.Register("battery_voltage", &packet_out_.battery_voltage);
  parameter_map_.Register("battery_temperature", &packet_out_.battery_temperature);
  parameter_map_.Register("is_battery_crgov", &packet_out_.is_battery_crgov);
  parameter_map_.Register("is_battery_23par", &packet_out_.is_battery_23par);
  parameter_map_.Register("is_battery_std", &packet_out_.is_battery_std);
  parameter_map_.Register("is_battery_full", &packet_out_.is_battery_full);
  parameter_map_.Register("is_battery_discov", &packet_out_.is_battery_discov);
  parameter_map_.Register("is_battery_chg", &packet_out_.is_battery_chg);
  parameter_map_.Register("is_battery_disc", &packet_out_.is_battery_disc);
  parameter_map_.Register("is_battery_0per", &packet_out_.is_battery_0per);
  parameter_map_.Register("is_battery_45par", &packet_out_.is_battery_45par);
  parameter_map_.Register("is_battery_sel", &packet_out_.is_battery_sel);
  parameter_map_.Register("is_battery_bal", &packet_out_.is_battery_bal);
  parameter_map_.Register("battery_initial_learning_capacity", &packet_out_.battery_initial_learning_capacity);
  parameter_map_.Register("battery_error_status", &packet_out_.battery_error_status);
  parameter_map_.Register("battery_relative_capacity", &packet_out_.battery_relative_capacity);
  parameter_map_.Register("power_ecu_internal_state", &packet_out_.power_ecu_internal_state);
  parameter_map_.Register("is_powerecu_bat_stat", &packet_out_.is_powerecu_bat_stat);
  parameter_map_.Register("is_powerecu_sw_kinoko", &packet_out_.is_powerecu_sw_kinoko);
  parameter_map_.Register("is_powerecu_sw_pwr", &packet_out_.is_powerecu_sw_pwr);
  parameter_map_.Register("is_powerecu_sw_drv", &packet_out_.is_powerecu_sw_drv);
  parameter_map_.Register("is_powerecu_sw_latch", &packet_out_.is_powerecu_sw_latch);
  parameter_map_.Register("is_powerecu_sw_w_sel", &packet_out_.is_powerecu_sw_w_sel);
  parameter_map_.Register("is_powerecu_sw_w_stop", &packet_out_.is_powerecu_sw_w_stop);
  parameter_map_.Register("is_bumper_bumper2", &packet_out_.is_bumper_bumper2);
  parameter_map_.Register("is_bumper_bumper1", &packet_out_.is_bumper_bumper1);
  parameter_map_.Register("is_bumper_prox5", &packet_out_.is_bumper_prox5);
  parameter_map_.Register("is_bumper_prox4", &packet_out_.is_bumper_prox4);
  parameter_map_.Register("is_bumper_prox3", &packet_out_.is_bumper_prox3);
  parameter_map_.Register("is_bumper_prox2", &packet_out_.is_bumper_prox2);
  parameter_map_.Register("is_bumper_prox1", &packet_out_.is_bumper_prox1);
  parameter_map_.Register("gyro_status", &packet_out_.gyro_status);
  parameter_map_.Register("imu_quaternions", &packet_out_.imu_quaternions);
  parameter_map_.Register("imu_angular_velocities", &packet_out_.imu_angular_velocities);
  parameter_map_.Register("imu_accelerations", &packet_out_.imu_accelerations);
  parameter_map_.Register("charger_state", &packet_out_.charger_state);
}

/**
 * @brief デコード後処理
 * @return 成功時 True
 */
bool PowerEcuComEcu1DataDecoder::Update() {
  //!< タイムスタンプ           10進10桁 [ms]
  packet_out_.time_stamp = packet_raw_data_.time_stamp;
  //!< 日時                     文字 （YYYYMMDDhhmmss）
  if (packet_raw_data_.date.size() <= packet_out_.ecu1_date.capacity()) {
    packet_out_.ecu1_date.clear();
    std::copy(packet_raw_data_.date.begin(), packet_raw_data_.date.end(), std::back_inserter(packet_out_.ecu1_date));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("power_ecu_com_data_decoder"), "ecu1 date have not enough buffer.");
  }
  //!< 電源ECUステータス        16進2桁      -
  if ((packet_raw_data_.power_ecu_status_flag.size() - 1) <= packet_out_.power_ecu_status_flag.capacity()) {
    packet_out_.power_ecu_status_flag.clear();
    std::copy(packet_raw_data_.power_ecu_status_flag.begin() + 1, packet_raw_data_.power_ecu_status_flag.end(),
              std::back_inserter(packet_out_.power_ecu_status_flag));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("power_ecu_com_data_decoder"), "ecu1 power_ecu_status have not enough buffer.");
  }

  // ダイアグ情報 ダイアグ
  // diag[0]      DIAG-P-001
  // diag[1]      DIAG-P-002
  // diag[2]      DIAG-P-003
  // diag[3]      DIAG-P-004
  // diag[4]      DIAG-P-005
  // diag[5]      DIAG-P-006
  // diag[6]      DIAG-P-007
  // diag[7]      DIAG-P-008
  //
  // diag[8]      DIAG-P-009
  // diag[9]      DIAG-P-010
  // diag[10]     DIAG-P-011
  // diag[11]     DIAG-P-012
  // diag[12]     DIAG-P-013
  // diag[13]     DIAG-P-014
  // diag[14]     DIAG-P-015
  // diag[15]     DIAG-P-016
  //
  // diag[16]     DIAG-P-017
  // diag[17]     DIAG-P-018
  // diag[18]     DIAG-P-019
  // diag[19]     DIAG-P-020
  // diag[20]     DIAG-P-021
  // diag[21]     DIAG-P-022
  // diag[22]     DIAG-P-023
  // diag[23]     DIAG-P-024
  //
  // diag[24]     DIAG-P-025
  // diag[25]     DIAG-P-026
  // diag[26]     DIAG-P-027
  // diag[27]     DIAG-P-028
  // diag[28]     DIAG-P-029
  // diag[29]     DIAG-P-030
  // diag[30]     DIAG-P-031
  // diag[31]     DIAG-P-032
  //
  // diag[32]     DIAG-P-033
  // diag[33]     DIAG-P-034
  // diag[34]     DIAG-P-035
  // diag[35]     DIAG-P-036
  // diag[36]     DIAG-P-037
  // diag[37]     DIAG-P-038
  // diag[38]     DIAG-P-039
  // diag[39]     DIAG-P-040
  //
  // diag[40]     DIAG-P-041
  // diag[41]     DIAG-E-001
  // diag[42]     DIAG-E-002
  // diag[43]     DIAG-C-001
  // diag[44]     DIAG-C-002
  // diag[45]     DIAG-C-003
  // diag[46]     DIAG-V-001
  // diag[47]
  //
  // diag[48]
  // diag[49]
  // diag[50]     DIAG-I-001
  // diag[51]     DIAG-I-002
  // diag[52]     DIAG-B-001
  // diag[53]     DIAG-B-002
  if ((packet_raw_data_.diag_status.size() - 1) <= packet_out_.diag_status.capacity()) {
    packet_out_.diag_status.clear();
    std::copy(packet_raw_data_.diag_status.begin() + 1, packet_raw_data_.diag_status.end(),
              std::back_inserter(packet_out_.diag_status));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("power_ecu_com_data_decoder"), "ecu2 diag_status have not enough buffer.");
  }

  // ①バッテリ総容量   2byte 1mAH 00000～65535  0～65535mAH
  packet_out_.battery_total_capacity = static_cast<double>(packet_raw_data_.battery_total_capacity) * 1e-3;
  // ②バッテリ残容量   2byte 1mAH 00000～65535  0～65535mAH
  packet_out_.battery_remaining_capacity = static_cast<double>(packet_raw_data_.battery_remaining_capacity) * 1e-3;
  // ③バッテリ電流値   2byte 2mA  -65536～65534 -65536～65534mA ( - :充電, +:放電)
  packet_out_.electric_current = static_cast<double>(packet_raw_data_.electric_current) * 1e-3;
  // ④バッテリ電圧値   2byte 1mV  00000～65535  0～65535mV
  packet_out_.battery_voltage = static_cast<double>(packet_raw_data_.battery_voltage) * 1e-3;
  // ⑤バッテリ電池温度 1byte --   -128～127     -128～127℃
  packet_out_.battery_temperature = static_cast<double>(packet_raw_data_.battery_temperature);
  // ⑦バッテリ：状態フラグ
  // 1byte目
  // ビット シンボル ビット名 機能
  // bit7   CRGOV    過充電   0:過充電以外     1:過充電
  // bit6   23PAR    並列数   0:2並            1:3並
  // bit5   STD      学習許可 0:学習禁止       1:学習許可
  // bit4   FULL     満充電   0:満充電状態以外 1:満充電状態
  // bit3   DISCOV   過放電   0:過放電以外     1:過放電
  // bit2   CHG      充電許可 0:充電停止       1:充電許可
  // bit1   DISC     放電許可 0:放電停止       1:放電許可
  // bit0   0PER     0%検出   0:0%検出状態以外 1:0%検出状態
  //
  // 2byte目
  // ビット     シンボル ビット名               機能
  // bit7～bit3 reserve  予約ビット             0:固定値
  // bit2       45PAR    並列数                 0:4並 1:5並
  // bit1       SEL      最小セル電圧0%検出状態 0:最小セル電圧0%検出状態以外
  // 1:最小セル電圧0%検出状態
  // bit0       BAL      セルバランス崩れ       0:セルバランス崩れ以外
  // 1:セルバランス崩れ
  packet_out_.is_battery_45par = ((packet_raw_data_.battery_state_flag & (0x1 << (8 + 2))) > 0);
  packet_out_.is_battery_sel = ((packet_raw_data_.battery_state_flag & (0x1 << (8 + 1))) > 0);
  packet_out_.is_battery_bal = ((packet_raw_data_.battery_state_flag & (0x1 << (8 + 0))) > 0);
  packet_out_.is_battery_crgov = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 7))) > 0);
  packet_out_.is_battery_23par = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 6))) > 0);
  packet_out_.is_battery_std = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 5))) > 0);
  packet_out_.is_battery_full = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 4))) > 0);
  packet_out_.is_battery_discov = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 3))) > 0);
  packet_out_.is_battery_chg = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 2))) > 0);
  packet_out_.is_battery_disc = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 1))) > 0);
  packet_out_.is_battery_0per = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 0))) > 0);
  //!< バッテリ初期学習容量 10進5桁      [mAh]
  packet_out_.battery_initial_learning_capacity = packet_raw_data_.battery_initial_learning_capacity;
  //!< バッテリ異常ステータス   16進4桁 - 内容は仕様書で未定義
  packet_out_.battery_error_status = packet_raw_data_.battery_error_status;
  //!< 相対容量                 10進3桁 [%]
  packet_out_.battery_relative_capacity = static_cast<double>(packet_raw_data_.battery_relative_capacity);

  // 電源ECU状態
  // ビット | シンボル        | ビット名                         | 機能
  // 24～63 | reserve         | 予約ビット                       | 0:固定値
  // 16～23 | ECU_PROG_STATUS | "4.5.3のS**の数 電源ECU内部状態" | 0～12
  // 7～15  | reserve         | 予約ビット                       |
  // 6      | BAT_STAT        | バッテリ充電状態                 | 0:充電されていない 1:充電されている
  // 5      | SW_KINOKO       | 有線緊急停止SW                   | 0:押されていない 1:押されている
  // 4      | SW_PWR          | 電源（プリウスSW)                | 0:押されていない 1:押されている
  // 3      | SW_DRV          | 駆動SW                           | 0:駆動系が出力されている 1:駆動系が出力されていない
  // 2      | SW_LATCH        | ラッチ解除SW                     | 0:押されていない 1:押されている
  // 1      | SW_W_SEL        | 無線切り替えSW                   | 0:無線緊急停止有効 1:無線緊急停止無効
  // 0      | SW_W_STOP       | 無線緊急停止SW                   | 0:押されていない 1:押されている
  temp_str.clear();
  std::copy(packet_raw_data_.power_ecu_status.end() - 6, packet_raw_data_.power_ecu_status.end() - 4,
            std::back_inserter(temp_str));
  packet_out_.power_ecu_internal_state = std::strtol(temp_str.c_str(), NULL, 16);
  temp_str.clear();
  std::copy(packet_raw_data_.power_ecu_status.end() - 2, packet_raw_data_.power_ecu_status.end(),
            std::back_inserter(temp_str));
  uint8_t bits = std::strtol(temp_str.c_str(), NULL, 16);
  packet_out_.is_powerecu_bat_stat = (bits & (0x1 << 6)) != 0;
  // 有線緊急停止SWのECUからの出力が0と1が逆になっている
  // ECU側では変更を行わないので上位ソフトにて判定を逆にして対応する
  packet_out_.is_powerecu_sw_kinoko = (bits & (0x1 << 5)) == 0;
  packet_out_.is_powerecu_sw_pwr = (bits & (0x1 << 4)) != 0;
  // 本来is_powerecu_sw_drvは駆動電源が停止しているかどうかが出力されているべきだが
  // 2018.03現在、is_powerecu_sw_kinokoとまったく同じ状態がでてしまっているため
  // 正しい情報として扱うことができない。
  // そのため別途ECUの内部状態で、駆動電源の出力状態を判定する。
  // TODO(T.Nishino) 電源ECUのファームウェアが修正された段階で修正する
  packet_out_.is_powerecu_sw_drv = (packet_out_.power_ecu_internal_state != 9);
  packet_out_.is_powerecu_sw_latch = (bits & (0x1 << 2)) != 0;
  packet_out_.is_powerecu_sw_w_sel = (bits & (0x1 << 1)) != 0;
  // 無線緊急停止SWのECUからの出力が0と1が逆になっている
  // ECU側では変更を行わないので上位ソフトにて判定を逆にして対応する
  packet_out_.is_powerecu_sw_w_stop = (bits & (0x1 << 0)) == 0;

  // ⑨電源ボード：近接センサ、バンパセンサ状態
  // ビット シンボル ビット名              機能
  // bit7   reserve  予約ビット            0:固定値
  // bit6   BUMPER2  バンパセンサ状態2     0:接触なし   1:接触あり
  // bit5   BUMPER1  バンパセンサ状態1     0:接触なし   1:接触あり
  // bit4   PROX5    近接センサラッチ状態5 0:近接物なし 1:近接物あり
  // bit3   PROX4    近接センサラッチ状態4 0:近接物なし 1:近接物あり
  // bit2   PROX3    近接センサラッチ状態3 0:近接物なし 1:近接物あり
  // bit1   PROX2    近接センサラッチ状態2 0:近接物なし 1:近接物あり
  // bit0   PROX1    近接センサラッチ状態1 0:近接物なし 1:近接物あり
  packet_out_.is_bumper_bumper2 = ((packet_raw_data_.bumper_status & (0x1 << 6)) > 0);
  packet_out_.is_bumper_bumper1 = ((packet_raw_data_.bumper_status & (0x1 << 5)) > 0);
  packet_out_.is_bumper_prox5 = ((packet_raw_data_.bumper_status & (0x1 << 4)) > 0);
  packet_out_.is_bumper_prox4 = ((packet_raw_data_.bumper_status & (0x1 << 3)) > 0);
  packet_out_.is_bumper_prox3 = ((packet_raw_data_.bumper_status & (0x1 << 2)) > 0);
  packet_out_.is_bumper_prox2 = ((packet_raw_data_.bumper_status & (0x1 << 1)) > 0);
  packet_out_.is_bumper_prox1 = ((packet_raw_data_.bumper_status & (0x1 << 0)) > 0);

  //!< ジャイロ姿勢角演算ステータス 16進数16桁
  packet_out_.gyro_status.clear();
  std::copy(packet_raw_data_.gyro_status.begin() + 1, packet_raw_data_.gyro_status.end(),
            std::back_inserter(packet_out_.gyro_status));

  //!< quaternion x,y,z,t -1.0~1.0
  packet_out_.imu_quaternions[0] = packet_raw_data_.quaternion_x * 1e-9;  //!< quaternion_x        符号10進10桁 x10^-1
  packet_out_.imu_quaternions[1] = packet_raw_data_.quaternion_y * 1e-9;  //!< quaternion_y        符号10進10桁 x10^-1
  packet_out_.imu_quaternions[2] = packet_raw_data_.quaternion_z * 1e-9;  //!< quaternion_z        符号10進10桁 x10^-1
  packet_out_.imu_quaternions[3] = packet_raw_data_.quaternion_t * 1e-9;  //!< quaternion_t        符号10進10桁 x10^-1

  //!< 角速度 x,y,z [rad/s]
  packet_out_.imu_angular_velocities[0] =
      packet_raw_data_.angular_velocity_x * 1e-8;  //!< 角速度x             符号10進10桁 x10^-2[rad/s]
  packet_out_.imu_angular_velocities[1] =
      packet_raw_data_.angular_velocity_y * 1e-8;  //!< 角速度y             符号10進10桁 x10^-2[rad/s]
  packet_out_.imu_angular_velocities[2] =
      packet_raw_data_.angular_velocity_z * 1e-8;  //!< 角速度z             符号10進10桁 x10^-2[rad/s]

  //!< 加速度 x,y,z [m/s^2]
  packet_out_.imu_accelerations[0] =
      packet_raw_data_.acceleration_x * 1e-8;  //!< 加速度x             符号10進10桁 x10^-2[m/s^2]
  packet_out_.imu_accelerations[1] =
      packet_raw_data_.acceleration_y * 1e-8;  //!< 加速度y             符号10進10桁 x10^-2[m/s^2]
  packet_out_.imu_accelerations[2] =
      packet_raw_data_.acceleration_z * 1e-8;  //!< 加速度z             符号10進10桁 x10^-2[m/s^2]

  //!< 自動充電ステータス
  packet_out_.charger_state = packet_raw_data_.charger_state;

  return true;
}

/**
 * @brief コンストラクタ
 * @param[in] packet_data パケットデータ
 */
PowerEcuComEcu2DataDecoder::PowerEcuComEcu2DataDecoder()
    : IPowerEcuComDataDecoder(kEcu2DataDecoderPacketSize, kEcu2DataDecoderPacketName), packet_data_() {
  // std::string* date;     //!<  日時（YYYYMMDDhhmmss） 文字
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_data_.ecu2_date, 14)));
  // uint16_t* d12V_D0_V;   //!<  12Vd0電圧        [mV] 符号1桁+10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_D0_V, 5)));
  // int16_t* d12V_D0_A;    //!<  12Vd0電流        [mA] 符号1桁+10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_D0_A, 5)));
  // uint16_t* d12V_D1_V;   //!<  12Vd1電圧        [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_D1_V, 5)));
  // int16_t* d12V_D1_A;    //!<  12Vd1電流        [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_D1_A, 5)));
  // uint16_t* d12V_D2_V;   //!<  12Vd2電圧        [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_D2_V, 5)));
  // int16_t* d12V_D2_A;    //!<  12Vd2電流        [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_D2_A, 5)));
  // uint16_t* d12V_D3_V;   //!<  12Vd3電圧        [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_D3_V, 5)));
  // int16_t* d12V_D3_A;    //!<  12Vd3電流        [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_D3_A, 5)));
  // uint16_t* d12V_O1_V;   //!<  12Vo1電圧        [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_O1_V, 5)));
  // int16_t* d12V_O1_A;    //!<  12Vo1電流        [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_O1_A, 5)));
  // uint16_t* d12V_O2_V;   //!<  12Vo2電圧        [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_O2_V, 5)));
  // int16_t* d12V_O2_A;    //!<  12Vo2電流        [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_O2_A, 5)));
  // uint16_t* d5VA_V;      //!<  5Va電圧          [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VA_V, 5)));
  // uint16_t* d5VD1_V;     //!<  5Vd1電圧         [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VD1_V, 5)));
  // int16_t* d5VD1_A;      //!<  5Vd1電流         [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d5VD1_A, 5)));
  // uint16_t* d5VD2_V;     //!<  5Vd2電圧         [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VD2_V, 5)));
  // int16_t* d5VD2_A;      //!<  5Vd2電流         [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d5VD2_A, 5)));
  // uint16_t* d5VD3_V;     //!<  5Vd3電圧         [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VD3_V, 5)));
  // int16_t* d5VD3_A;      //!<  5Vd3電流         [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d5VD3_A, 5)));
  // uint16_t* d5VD4_V;     //!<  5Vd4電圧         [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VD4_V, 5)));
  // int16_t* d5VD4_A;      //!<  5Vd4電流         [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d5VD4_A, 5)));
  // uint16_t* d5VD5_V;     //!<  5Vd5電圧         [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VD5_V, 5)));
  // int16_t* d5VD5_A;      //!<  5Vd5電流         [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d5VD5_A, 5)));
  // int16_t* Chgsense;     //!<  自動順電挿抜端子電圧 [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.Chgsense, 5)));
  // uint16_t* d2V5VDA1_V;  //!<  2.5Va1電圧(A/D1) [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d2V5VDA1_V, 5)));
  // uint16_t* d2V5VDA2_V;  //!<  2.5Va2電圧(A/D2) [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d2V5VDA2_V, 5)));
  // uint16_t* ACDC_V;      //!<  ACDC電圧         [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.ACDC_V, 5)));
  // int16_t* ADCD_A;       //!<  ACDC電流         [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.ADCD_A, 5)));
  // uint16_t* BATT_V;      //!<  BATT電圧         [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.BATT_V, 5)));
  // int16_t* BATT_A;       //!<  BATT電流         [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.BATT_A, 5)));
  // int16_t* BATT_A2;      //!<  BATT電流2        [10mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.BATT_A2, 5)));
  // uint16_t* PBM_V;       //!<  PBM電圧          [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.PBM_V, 5)));
  // int16_t* PBM_A;        //!<  PBM電流          [mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.PBM_A, 5)));
  // int16_t* PBM_A2;       //!<  PBM電流2        [10mA] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.PBM_A2, 5)));
  // uint16_t* PUMP_V;      //!<  ポンプセンサ電圧 [mV] 符号1桁10進数5桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.PUMP_V, 5)));
  // int8_t* ECU_TEMP;      //!<  電源ECU温度     [℃] 符号1桁10進数3桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.ECU_TEMP, 3)));
  // int8_t* ECU_TEMP1;     //!< 電源ECU温度1     [℃] 符号1桁10進数3桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.ECU_TEMP1, 3)));
  // int8_t* ECU_TEMP2;     //!< 電源ECU温度2     [℃] 符号1桁10進数3桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.ECU_TEMP2, 3)));
  // int8_t* ECU_TEMP3;     //!< 電源ECU温度3     [℃] 符号1桁10進数3桁
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.ECU_TEMP3, 3)));

  parameter_map_.Register("ecu2_date", &packet_data_.ecu2_date);
  parameter_map_.Register("d12V_D0_V", &packet_data_.d12V_D0_V);
  parameter_map_.Register("d12V_D0_A", &packet_data_.d12V_D0_A);
  parameter_map_.Register("d12V_D1_V", &packet_data_.d12V_D1_V);
  parameter_map_.Register("d12V_D1_A", &packet_data_.d12V_D1_A);
  parameter_map_.Register("d12V_D2_V", &packet_data_.d12V_D2_V);
  parameter_map_.Register("d12V_D2_A", &packet_data_.d12V_D2_A);
  parameter_map_.Register("d12V_D3_V", &packet_data_.d12V_D3_V);
  parameter_map_.Register("d12V_D3_A", &packet_data_.d12V_D3_A);
  parameter_map_.Register("d12V_O1_V", &packet_data_.d12V_O1_V);
  parameter_map_.Register("d12V_O1_A", &packet_data_.d12V_O1_A);
  parameter_map_.Register("d12V_O2_V", &packet_data_.d12V_O2_V);
  parameter_map_.Register("d12V_O2_A", &packet_data_.d12V_O2_A);
  parameter_map_.Register("d5VA_V", &packet_data_.d5VA_V);
  parameter_map_.Register("d5VD1_V", &packet_data_.d5VD1_V);
  parameter_map_.Register("d5VD1_A", &packet_data_.d5VD1_A);
  parameter_map_.Register("d5VD2_V", &packet_data_.d5VD2_V);
  parameter_map_.Register("d5VD2_A", &packet_data_.d5VD2_A);
  parameter_map_.Register("d5VD3_V", &packet_data_.d5VD3_V);
  parameter_map_.Register("d5VD3_A", &packet_data_.d5VD3_A);
  parameter_map_.Register("d5VD4_V", &packet_data_.d5VD4_V);
  parameter_map_.Register("d5VD4_A", &packet_data_.d5VD4_A);
  parameter_map_.Register("d5VD5_V", &packet_data_.d5VD5_V);
  parameter_map_.Register("d5VD5_A", &packet_data_.d5VD5_A);
  parameter_map_.Register("Chgsense", &packet_data_.Chgsense);
  parameter_map_.Register("d2V5VDA1_V", &packet_data_.d2V5VDA1_V);
  parameter_map_.Register("d2V5VDA2_V", &packet_data_.d2V5VDA2_V);
  parameter_map_.Register("ACDC_V", &packet_data_.ACDC_V);
  parameter_map_.Register("ADCD_A", &packet_data_.ADCD_A);
  parameter_map_.Register("BATT_V", &packet_data_.BATT_V);
  parameter_map_.Register("BATT_A", &packet_data_.BATT_A);
  parameter_map_.Register("BATT_A2", &packet_data_.BATT_A2);
  parameter_map_.Register("PBM_V", &packet_data_.PBM_V);
  parameter_map_.Register("PBM_A", &packet_data_.PBM_A);
  parameter_map_.Register("PBM_A2", &packet_data_.PBM_A2);
  parameter_map_.Register("PUMP_V", &packet_data_.PUMP_V);
  parameter_map_.Register("ECU_TEMP", &packet_data_.ECU_TEMP);
  parameter_map_.Register("ECU_TEMP1", &packet_data_.ECU_TEMP1);
  parameter_map_.Register("ECU_TEMP2", &packet_data_.ECU_TEMP2);
  parameter_map_.Register("ECU_TEMP3", &packet_data_.ECU_TEMP3);
}

/**
 * @brief デコード後処理
 * @return 成功時 True
 */
bool PowerEcuComEcu2DataDecoder::Update() {
  return true;
}

/**
 * @brief コンストラクタ
 * @param[in] packet_data パケットデータ
 */
PowerEcuComRxackDataDecoder::PowerEcuComRxackDataDecoder()
    : IPowerEcuComDataDecoder(kRxackDataDecoderPacketSize, kRxackDataDecoderPacketName), packet_data_() {
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintDecoder<uint8_t> >(
      new hsrb_power_ecu::ElementHexUintDecoder<uint8_t>(packet_data_.ack_value, 2)));

  parameter_map_.Register("is_receive_ack", &packet_data_.is_receive_ack);
  parameter_map_.Register("ack_value", &packet_data_.ack_value);
}

/**
 * @brief デコード後処理
 * @return 成功時 True
 */
bool PowerEcuComRxackDataDecoder::Update() {
  packet_data_.is_receive_ack = true;
  return true;
}

/**
 * @brief コンストラクタ
 * @param[in] packet_data パケットデータ
 */
PowerEcuComVerDataDecoder::PowerEcuComVerDataDecoder()
    : IPowerEcuComDataDecoder(kVerDataDecoderPacketSize, kVerDataDecoderPacketName), packet_data_() {
  // ・バージョン情報ver__(getv_コマンドへの返信はrxackの代わりにこれを返す）
  // オフセット | バイト数 | 記述例                                     | 内容                      | 表記       | 評価
  // | 単位
  // 12         | 42       | hb5b862f5072d516a86e7c469bb015898e7cc631b, | バージョン情報(ECUVER)    | 16進数40桁 | uint8
  // | -
  // 54         | 42       | hb5b862f5072d516a86e7c469bb015898e7cc632b, | バージョン情報(ECUCOMVER) | 16進数40桁 | uint9
  // | -

  power_ecu_version_raw_.reserve(kEcu2PowerEcuVersionLength);
  packet_data_.ver_power_ecu_version.reserve(kEcu2PowerEcuVersionLength);
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(power_ecu_version_raw_, 40 + 1)));

  power_ecu_com_version_raw_.reserve(kEcu2PowerEcuComVersionLength);
  packet_data_.ver_power_ecu_com_version.reserve(kEcu2PowerEcuComVersionLength);
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(power_ecu_com_version_raw_, 40 + 1)));

  parameter_map_.Register("ver_power_ecu_version", &packet_data_.ver_power_ecu_version);
  parameter_map_.Register("ver_power_ecu_com_version", &packet_data_.ver_power_ecu_com_version);
  parameter_map_.Register("is_receive_version", &packet_data_.is_receive_version);
}

/**
 * @brief デコード後処理
 * @return 成功時 True
 */
bool PowerEcuComVerDataDecoder::Update() {
  packet_data_.is_receive_version = true;
  packet_data_.ver_power_ecu_version.clear();
  std::copy(power_ecu_version_raw_.begin() + 1, power_ecu_version_raw_.end(),
            std::back_inserter(packet_data_.ver_power_ecu_version));
  packet_data_.ver_power_ecu_com_version.clear();
  std::copy(power_ecu_com_version_raw_.begin() + 1, power_ecu_com_version_raw_.end(),
            std::back_inserter(packet_data_.ver_power_ecu_com_version));
  return true;
}

}  // namespace hsrb_power_ecu
