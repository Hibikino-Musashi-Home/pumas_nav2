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
#include "power_ecu_com_data_encoder.hpp"

#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared.hpp>

#include "ros2_msg_utils.hpp"

namespace {
const char kTimePacketSize[] = "026";     //!< 時刻合わせコマンドのパケットサイズ
const char kTimePacketName[] = "time_";   //!< 時刻合わせコマンドのパケット種別
const char kStartPacketSize[] = "015";    //!< 定期通信開始コマンドのパケットサイズ
const char kStartPacketName[] = "start";  //!< 定期通信開始コマンドのパケット種別
const char kStopPacketSize[] = "011";     //!< 定期通信終了コマンドのパケットサイズ
const char kStopPacketName[] = "stop_";   //!< 定期通信終了コマンドのパケット種別
const char kHeartpacketSize[] = "027";    //!< ハートビートコマンドのパケットサイズ
const char kHeartPacketName[] = "heart";  //!< ハートビートコマンドのパケット種別
const char kPumpPacketSize[] = "013";     //!< ポンプスイッチコマンドのパケットサイズ
const char kPumpPacketName[] = "pump_";   //!< ポンプスイッチコマンドのパケット種別
const char kPbmswPacketSize[] = "013";    //!< 駆動系スイッチコマンドのパケットサイズ
const char kPbmswPacketName[] = "pbmsw";  //!< 駆動系スイッチコマンドのパケット種別
const char kLedcPacketSize[] = "023";     //!< 多用途LEDの色指定コマンドのパケットサイズ
const char kLedcPacketName[] = "ledc_";   //!< 多用途LEDの色指定コマンドのパケット種別
const char kGResPacketSize[] = "015";     //!< 姿勢角演算リセットコマンドのパケットサイズ
const char kGResPacketName[] = "g_res";   //!< 姿勢角演算リセットコマンドのパケット種別
const char kSolswPacketSize[] = "015";    //!< ソレロイドスイッチコマンドのパケットサイズ
const char kSolswPacketName[] = "solsw";  //!< ソレロイドスイッチコマンドのパケット種別
const char kPdcmdPacketSize[] = "015";    //!< 電源シャットダウンコマンドのパケットサイズ
const char kPdcmdPacketName[] = "pdcmd";  //!< 電源シャットダウンコマンドのパケット種別
const char kMutePacketSize[] = "015";     //!< オーディオアンプMuteコマンドのパケットサイズ
const char kMutePacketName[] = "mute_";   //!< オーディオアンプMuteコマンドのパケット種別
const char kGetvPacketSize[] = "015";     //!< バージョン情報取得コマンドのパケットサイズ
const char kGetvPacketName[] = "getv_";   //!< バージョン情報取得コマンドのパケット種別
const uint8_t kGetvpacketData = 0;        //!< バージョン情報取得コマンドのパケット要素の予約値
const char kUndckPacketSize[] = "011";    //!< アンドック指令コマンドのパケットサイズ
const char kUndckPacketName[] = "undck";  //!< アンドック指令コマンドのパケット種別
const char k12VuPacketSize[] = "013";     //!< 12V USBイネーブルコマンドのパケットサイズ
const char k12VuPacketName[] = "12vu_";   //!< 12V USBイネーブルコマンドのパケット種別
const char k5Vd3PacketSize[] = "013";     //!< 5Vd3 イネーブルコマンドのパケットサイズ
const char k5Vd3PacketName[] = "5vd3_";   //!< 5Vd3 イネーブルコマンドのパケット種別
const char k5Vd4PacketSize[] = "013";     //!< 5Vd4 イネーブルコマンドのパケットサイズ
const char k5Vd4PacketName[] = "5vd4_";   //!< 5Vd4 イネーブルコマンドのパケット種別
const char k5Vd5PacketSize[] = "013";     //!< 5Vd5 イネーブルコマンドのパケットサイズ
const char k5Vd5PacketName[] = "5vd5_";   //!< 5Vd5 イネーブルコマンドのパケット種別
}  // anonymous namespace

namespace hsrb_power_ecu {
/**
 * @brief コンストラクタ
 * @param packet_data パケットデータ
 */
PowerEcuComTimeDataEncoder::PowerEcuComTimeDataEncoder()
    : IPowerEcuComDataEncoder(kTimePacketSize, kTimePacketName), packet_data_() {
  // 要素登録
  //!< 日付 文字列 14桁
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringEncoder>(
      new hsrb_power_ecu::ElementStringEncoder(packet_data_.start_time, 14)));

  parameter_map_.Register("start_time", &packet_data_.start_time);
}

/**
 * @brief コンストラクタ
 * @param packet_data パケットデータ
 */
PowerEcuComStartDataEncoder::PowerEcuComStartDataEncoder()
    : IPowerEcuComDataEncoder(kStartPacketSize, kStartPacketName), packet_data_() {
  // 要素登録
  boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder> p =
      boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder>(
          new hsrb_power_ecu::ElementHexUintBitsEncoder(2));  // 16進2桁
  // 6bit目 ecu2
  bool ret;
  ret = p->RegisterBit(1, &packet_data_.is_enable_ecu2);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  // 7bit目 ecu1
  ret = p->RegisterBit(0, &packet_data_.is_enable_ecu1);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  element_encoder_list_.push_back(p);

  parameter_map_.Register("is_enable_ecu1", &packet_data_.is_enable_ecu1);
  parameter_map_.Register("is_enable_ecu2", &packet_data_.is_enable_ecu2);
}

/**
 * @brief コンストラクタ
 */
PowerEcuComStopDataEncoder::PowerEcuComStopDataEncoder() : IPowerEcuComDataEncoder(kStopPacketSize, kStopPacketName) {}

/**
 * @brief コンストラクタ
 * @param packet_data パケットデータ
 */
PowerEcuComHeartDataEncoder::PowerEcuComHeartDataEncoder()
    : IPowerEcuComDataEncoder(kHeartpacketSize, kHeartPacketName), packet_data_() {
  //!< エラー状態                16進数4桁 uint16
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintEncoder<uint16_t> >(
      new hsrb_power_ecu::ElementHexUintEncoder<uint16_t>(packet_data_.error_state, 4)));
  //!< カウント値（送信ごとに+1) 16進数8桁 uint32
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintEncoder<uint32_t> >(
      new hsrb_power_ecu::ElementHexUintEncoder<uint32_t>(packet_data_.counts, 8)));

  parameter_map_.Register("error_state", &packet_data_.error_state);
  parameter_map_.Register("counts", &packet_data_.counts);
}

/**
 * @brief コンストラクタ
 * @param packet_data パケットデータ
 */
PowerEcuComPumpDataEncoder::PowerEcuComPumpDataEncoder()
    : IPowerEcuComDataEncoder(kPumpPacketSize, kPumpPacketName), packet_data_() {
  //!< ポンプスイッチ(0:OFF 1:ON) 10進数1桁 uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintEncoder<uint8_t>(packet_data_.is_pump_enable, 1)));

  parameter_map_.Register("is_pump_enable", &packet_data_.is_pump_enable);
}

/**
 * @brief コンストラクタ
 * @param packet_data パケットデータ
 */
PowerEcuComPbmswDataEncoder::PowerEcuComPbmswDataEncoder()
    : IPowerEcuComDataEncoder(kPbmswPacketSize, kPbmswPacketName), packet_data_() {
  //!< 駆動系スイッチ(0 : OFF 1 : ON) 10進数1桁 uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintEncoder<uint8_t>(packet_data_.is_motor_enable, 1)));

  parameter_map_.Register("is_motor_enable", &packet_data_.is_motor_enable);
}

/**
 * @brief コンストラクタ
 * @param packet_data パケットデータ
 */
PowerEcuComLedcDataEncoder::PowerEcuComLedcDataEncoder()
    : IPowerEcuComDataEncoder(kLedcPacketSize, kLedcPacketName), packet_data_() {
  //!< R強度(0～255) 10進数3桁 uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintEncoder<uint8_t>(packet_data_.led_color.r, 3)));
  //!< G強度(0～255) 10進数3桁 uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintEncoder<uint8_t>(packet_data_.led_color.g, 3)));
  //!< B強度(0～255) 10進数3桁 uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintEncoder<uint8_t>(packet_data_.led_color.b, 3)));

  parameter_map_.Register("led_color", &packet_data_.led_color);
}

PowerEcuComGResDataEncoder::PowerEcuComGResDataEncoder()
    : IPowerEcuComDataEncoder(kGResPacketSize, kGResPacketName), packet_data_() {
  // オフセット | バイト数 | 記述例 | 内容         | 表記      | 評価  | 単位
  // 12         | 4        | h00,   | リセット種別 | 16進数2桁 | uint8 | -

  // リセット種別
  // Bit  | Label   | 内容
  // 0    | res_q   | クオタニオンを0にする
  // 1    | res_g   | ジャイロのオフセットを0にする
  boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder> p =
      boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder>(new hsrb_power_ecu::ElementHexUintBitsEncoder(2));
  bool ret;
  ret = p->RegisterBit(0, &packet_data_.is_quaternion_reset);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  ret = p->RegisterBit(1, &packet_data_.is_gyro_reset);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  element_encoder_list_.push_back(p);

  parameter_map_.Register("is_quaternion_reset", &packet_data_.is_quaternion_reset);
  parameter_map_.Register("is_gyro_reset", &packet_data_.is_gyro_reset);
}

PowerEcuComSolswDataEncoder::PowerEcuComSolswDataEncoder()
    : IPowerEcuComDataEncoder(kSolswPacketSize, kSolswPacketName), packet_data_() {
  // オフセット | バイト数 | 記述例 | 内容                           | 表記      | 評価  | 単位
  // 12         | 4        | h00,   | ソレノイドスイッチ(0:OFF 1:ON) | 16進数1桁 | uint8 | -
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementHexUintEncoder<uint8_t>(packet_data_.is_solenoid_enable, 2)));

  parameter_map_.Register("is_solenoid_enable", &packet_data_.is_solenoid_enable);
}

PowerEcuComPdcmdDataEncoder::PowerEcuComPdcmdDataEncoder()
    : IPowerEcuComDataEncoder(kPdcmdPacketSize, kPdcmdPacketName), packet_data_() {
  // オフセット | バイト数 | 記述例 | 内容               | 表記      | 評価  | 単位
  // 12         | 4        | h00,   | シャットダウン種別 | 16進数2桁 | uint8 | -

  // シャットダウン種別
  // Bit  | Label   | 内容
  // 0    | pdcpu   | 1で内部CPUをシャットダウン
  // 1    | pdgpu   | 1でGPUをシャットダウン
  // 2    | pdex1   | 1で外部CPUをシャットダウン
  boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder> p =
      boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder>(new hsrb_power_ecu::ElementHexUintBitsEncoder(2));
  bool ret;
  ret = p->RegisterBit(0, &packet_data_.is_cpu_shutdown);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  p->RegisterBit(1, &packet_data_.is_gpu_shutdown);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  p->RegisterBit(2, &packet_data_.is_ex1_shutdown);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  element_encoder_list_.push_back(p);

  parameter_map_.Register("is_cpu_shutdown", &packet_data_.is_cpu_shutdown);
  parameter_map_.Register("is_gpu_shutdown", &packet_data_.is_gpu_shutdown);
  parameter_map_.Register("is_ex1_shutdown", &packet_data_.is_ex1_shutdown);
}
PowerEcuComMuteDataEncoder::PowerEcuComMuteDataEncoder()
    : IPowerEcuComDataEncoder(kMutePacketSize, kMutePacketName), packet_data_() {
  // オフセット | バイト数 | 記述例 | 内容     | 表記      | 評価  | 単位
  // 12         | 4        | h00,   | MUTE種別 | 16進数2桁 | uint8 | -

  // MUTE種別
  // Bit  | Label   | 内容
  // 0    | mutex   | 0:音を出す 1:音を出さない
  boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder> p =
      boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder>(new hsrb_power_ecu::ElementHexUintBitsEncoder(2));
  bool ret = p->RegisterBit(0, &packet_data_.is_amp_mute);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  element_encoder_list_.push_back(p);

  parameter_map_.Register("is_amp_mute", &packet_data_.is_amp_mute);
}

PowerEcuComGetvDataEncoder::PowerEcuComGetvDataEncoder()
    : IPowerEcuComDataEncoder(kGetvPacketSize, kGetvPacketName), packet_data_() {
  // オフセット | バイト数 | 記述例 | 内容                  | 表記      | 評価   | 単位
  // 12         | 4        | h00,   | バージョン種別(常に0) | 16進数2桁 | uint8  | -
  packet_data_.reserved = kGetvpacketData;
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementHexUintEncoder<uint8_t>(packet_data_.reserved, 2)));
}

/**
 * @brief コンストラクタ
 */
PowerEcuComUndckDataEncoder::PowerEcuComUndckDataEncoder()
    : IPowerEcuComDataEncoder(kUndckPacketSize, kUndckPacketName) {
  parameter_map_.Register("is_undck", &packet_data_.is_undck);
}

/**
 * @brief コンストラクタ
 */
PowerEcuCom12VuDataEncoder::PowerEcuCom12VuDataEncoder()
    : IPowerEcuComDataEncoder(k12VuPacketSize, k12VuPacketName), packet_data_() {
  //!< 12V USB(0:OFF 1:ON) 10進数1桁 uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<bool> >(
      new hsrb_power_ecu::ElementUintEncoder<bool>(packet_data_.is_12vu_enable, 1)));

  parameter_map_.Register("is_12vu_enable", &packet_data_.is_12vu_enable);
}

/**
 * @brief コンストラクタ
 */
PowerEcuCom5Vd3DataEncoder::PowerEcuCom5Vd3DataEncoder()
    : IPowerEcuComDataEncoder(k5Vd3PacketSize, k5Vd3PacketName), packet_data_() {
  //!< 5Vd3 (0:OFF 1:ON) 10進数1桁 uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<bool> >(
      new hsrb_power_ecu::ElementUintEncoder<bool>(packet_data_.is_5vd3_enable, 1)));

  parameter_map_.Register("is_5vd3_enable", &packet_data_.is_5vd3_enable);
}

/**
 * @brief コンストラクタ
 */
PowerEcuCom5Vd4DataEncoder::PowerEcuCom5Vd4DataEncoder()
    : IPowerEcuComDataEncoder(k5Vd4PacketSize, k5Vd4PacketName), packet_data_() {
  //!< 5Vd4 (0:OFF 1:ON) 10進数1桁 uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<bool> >(
      new hsrb_power_ecu::ElementUintEncoder<bool>(packet_data_.is_5vd4_enable, 1)));

  parameter_map_.Register("is_5vd4_enable", &packet_data_.is_5vd4_enable);
}

/**
 * @brief コンストラクタ
 */
PowerEcuCom5Vd5DataEncoder::PowerEcuCom5Vd5DataEncoder()
    : IPowerEcuComDataEncoder(k5Vd5PacketSize, k5Vd5PacketName), packet_data_() {
  //!< 5Vd5 (0:OFF 1:ON) 10進数1桁 uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<bool> >(
      new hsrb_power_ecu::ElementUintEncoder<bool>(packet_data_.is_5vd5_enable, 1)));

  parameter_map_.Register("is_5vd5_enable", &packet_data_.is_5vd5_enable);
}

}  // namespace hsrb_power_ecu
