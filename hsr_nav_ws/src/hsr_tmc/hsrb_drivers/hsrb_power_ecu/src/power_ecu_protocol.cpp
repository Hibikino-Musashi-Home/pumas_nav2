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
#include "power_ecu_protocol.hpp"

#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>

#include "power_ecu_com_common.hpp"
#include "power_ecu_com_data_decoder.hpp"
#include "power_ecu_com_data_encoder.hpp"
#include "power_ecu_com_repro_data_encoder.hpp"
#include "ros2_msg_utils.hpp"


namespace {
const size_t kBufferSize = 4 * 1000;             //!< バッファサイズ
const size_t kCommandQueueSize = 1000;           //!< コマンドキューの最大数
const uint32_t kErrorCounterSize = 1000;         //!< エラーレートのバッファサイズ
const uint32_t kRetryCount = 10;                 //!< 許容リトライカウント
const double kRetryRate = 0.9;                   //!< 許容エラーレート
const uint32_t kProcessCommandQueueTimeOut = 5;  //!< コマンドキュー解決のタイムアウト時間(sec)
const double kCycleHz = 100.0;                   //!< ポーリング周期(Hz)
const double kCommandTimeout = 10;               //!< タイムアウト時間
// ハートビート送信周期 (タイムアウト時間10秒の半分)
const rclcpp::Duration kHeartbeatDuration = rclcpp::Duration::from_seconds(kCommandTimeout * 0.5);

const char kEcuComVersion1String[] = "B7335B767D0FA2E6925BC8E965E443291A16A26A";  //!< プロトコルバージョン1

const char kIsReceiveAckName[] = "is_receive_ack";
const char kAckValue[] = "ack_value";
const char kVerPowerEcuVersionName[] = "ver_power_ecu_version";
const char kVerPowerEcuComVersionName[] = "ver_power_ecu_com_version";
const char kIsReceiveVersionName[] = "is_receive_version";
const char kCounts[] = "counts";

}  // anonymous namespace

namespace hsrb_power_ecu {

PowerEcuProtocol::PowerEcuProtocol(boost::shared_ptr<hsrb_power_ecu::INetwork> network,
                                   const rclcpp::Node::SharedPtr& node)
    : network_(network),
      receive_buffer_(kBufferSize),
      send_buffer_(kBufferSize),
      frame_decoder_(),
      frame_encoder_(),
      command_queue_(kCommandQueueSize),
      is_waiting_ack_(false),
      read_error_counter_(kErrorCounterSize),
      write_error_counter_(kErrorCounterSize),
      clock_(node->get_clock()),
      last_heartbeat_time_(clock_->now()) {
  // データデコーダ登録
  // デコーダの登録に失敗することを設計上ありえない
  RegisterDataDecoder<hsrb_power_ecu::PowerEcuComRxackDataDecoder>();
  RegisterDataDecoder<hsrb_power_ecu::PowerEcuComVerDataDecoder>();

  // フレームエンコーダ登録
  // エンコーダの登録に失敗することを設計上ありえない
  //// heart
  heart_command_name_ = RegisterDataEncoder<hsrb_power_ecu::PowerEcuComHeartDataEncoder>();
  getv_command_name_ = RegisterDataEncoder<hsrb_power_ecu::PowerEcuComGetvDataEncoder>();

  // コマンド生成
  //// ハートビートコマンド
  counts = GetParamPtr<uint32_t>(kCounts);
  assert(counts != NULL);
  is_receive_ack_ = GetParamPtr<bool>(kIsReceiveAckName);
  assert(is_receive_ack_ != NULL);
  ack_value_ = GetParamPtr<uint8_t>(kAckValue);
  assert(ack_value_ != NULL);
  ver_power_ecu_version_ = GetParamPtr<std::string>(kVerPowerEcuVersionName);
  assert(ver_power_ecu_version_ != NULL);
  ver_power_ecu_com_version_ = GetParamPtr<std::string>(kVerPowerEcuComVersionName);
  assert(ver_power_ecu_com_version_ != NULL);
  is_receive_version_ = GetParamPtr<bool>(kIsReceiveVersionName);
  assert(is_receive_version_ != NULL);
}

bool PowerEcuProtocol::Open() {
  // シリアルポート初期化
  if (network_->Open() != boost::system::errc::success) {
    RCLCPP_FATAL(rclcpp::get_logger("power_ecu_protocol"), "network open Failed");
    return false;
  }
  return true;
}

void PowerEcuProtocol::Close() {
  // シリアルポートクローズ
  network_->Close();
}

bool PowerEcuProtocol::Init() {
  // バージョン取得
  PowerEcuVersions version;
  if (!GetPowerEcuVersions(version)) {
    return false;
  }

  // バージョンに対応するコマンドを登録
  // バージョン切り替え機能追加時はこの処理をクラスとして取り出す
  if (version.power_ecu_com_version == kEcuComVersion1String) {
    // 登録処理
    std::string dummy;
    RegisterDataDecoder<hsrb_power_ecu::PowerEcuComEcu1DataDecoder>();
    RegisterDataDecoder<hsrb_power_ecu::PowerEcuComEcu2DataDecoder>();

    time_command_name_ = RegisterDataEncoder<hsrb_power_ecu::PowerEcuComTimeDataEncoder>();
    start_command_name_ = RegisterDataEncoder<hsrb_power_ecu::PowerEcuComStartDataEncoder>();
    stop_command_name_ = RegisterDataEncoder<hsrb_power_ecu::PowerEcuComStopDataEncoder>();
    mute_command_name_ = RegisterDataEncoder<hsrb_power_ecu::PowerEcuComMuteDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuComPumpDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuComPbmswDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuComLedcDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuComGResDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuComSolswDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuComPdcmdDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuComRprosDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuComRprodDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuComRproeDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuComUndckDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuCom12VuDataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuCom5Vd3DataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuCom5Vd4DataEncoder>();
    RegisterDataEncoder<hsrb_power_ecu::PowerEcuCom5Vd5DataEncoder>();

    uint32_t* time_stamp = GetParamPtr<uint32_t>("time_stamp");
    std::string* ecu1_date = GetParamPtr<std::string>("ecu1_date");
    std::string* power_ecu_status_flag = GetParamPtr<std::string>("power_ecu_status_flag");
    std::string* diag_status = GetParamPtr<std::string>("diag_status");
    double* battery_total_capacity = GetParamPtr<double>("battery_total_capacity");
    double* battery_remaining_capacity = GetParamPtr<double>("battery_remaining_capacity");
    double* electric_current = GetParamPtr<double>("electric_current");
    double* battery_voltage = GetParamPtr<double>("battery_voltage");
    double* battery_temperature = GetParamPtr<double>("battery_temperature");
    bool* is_battery_crgov = GetParamPtr<bool>("is_battery_crgov");
    bool* is_battery_23par = GetParamPtr<bool>("is_battery_23par");
    bool* is_battery_std = GetParamPtr<bool>("is_battery_std");
    bool* is_battery_full = GetParamPtr<bool>("is_battery_full");
    bool* is_battery_discov = GetParamPtr<bool>("is_battery_discov");
    bool* is_battery_chg = GetParamPtr<bool>("is_battery_chg");
    bool* is_battery_disc = GetParamPtr<bool>("is_battery_disc");
    bool* is_battery_0per = GetParamPtr<bool>("is_battery_0per");
    bool* is_battery_45par = GetParamPtr<bool>("is_battery_45par");
    bool* is_battery_sel = GetParamPtr<bool>("is_battery_sel");
    bool* is_battery_bal = GetParamPtr<bool>("is_battery_bal");
    uint16_t* battery_initial_learning_capacity = GetParamPtr<uint16_t>("battery_initial_learning_capacity");
    uint16_t* battery_error_status = GetParamPtr<uint16_t>("battery_error_status");
    double* battery_relative_capacity = GetParamPtr<double>("battery_relative_capacity");
    uint32_t* power_ecu_internal_state = GetParamPtr<uint32_t>("power_ecu_internal_state");
    bool* is_powerecu_bat_stat = GetParamPtr<bool>("is_powerecu_bat_stat");
    bool* is_powerecu_sw_kinoko = GetParamPtr<bool>("is_powerecu_sw_kinoko");
    bool* is_powerecu_sw_pwr = GetParamPtr<bool>("is_powerecu_sw_pwr");
    bool* is_powerecu_sw_drv = GetParamPtr<bool>("is_powerecu_sw_drv");
    bool* is_powerecu_sw_latch = GetParamPtr<bool>("is_powerecu_sw_latch");
    bool* is_powerecu_sw_w_sel = GetParamPtr<bool>("is_powerecu_sw_w_sel");
    bool* is_powerecu_sw_w_stop = GetParamPtr<bool>("is_powerecu_sw_w_stop");
    bool* is_bumper_bumper2 = GetParamPtr<bool>("is_bumper_bumper2");
    bool* is_bumper_bumper1 = GetParamPtr<bool>("is_bumper_bumper1");
    bool* is_bumper_prox5 = GetParamPtr<bool>("is_bumper_prox5");
    bool* is_bumper_prox4 = GetParamPtr<bool>("is_bumper_prox4");
    bool* is_bumper_prox3 = GetParamPtr<bool>("is_bumper_prox3");
    bool* is_bumper_prox2 = GetParamPtr<bool>("is_bumper_prox2");
    bool* is_bumper_prox1 = GetParamPtr<bool>("is_bumper_prox1");
    std::string* gyro_status = GetParamPtr<std::string>("gyro_status");
    boost::array<double, 4>* imu_quaternions = GetParamPtr<boost::array<double, 4> >("imu_quaternions");
    boost::array<double, 3>* imu_angular_velocities = GetParamPtr<boost::array<double, 3> >("imu_angular_velocities");
    boost::array<double, 3>* imu_accelerations = GetParamPtr<boost::array<double, 3> >("imu_accelerations");
    uint8_t* charger_state = GetParamPtr<uint8_t>("charger_state");
    std::string* ecu2_date = GetParamPtr<std::string>("ecu2_date");
    uint16_t* d12V_D0_V = GetParamPtr<uint16_t>("d12V_D0_V");
    int16_t* d12V_D0_A = GetParamPtr<int16_t>("d12V_D0_A");
    uint16_t* d12V_D1_V = GetParamPtr<uint16_t>("d12V_D1_V");
    int16_t* d12V_D1_A = GetParamPtr<int16_t>("d12V_D1_A");
    uint16_t* d12V_D2_V = GetParamPtr<uint16_t>("d12V_D2_V");
    int16_t* d12V_D2_A = GetParamPtr<int16_t>("d12V_D2_A");
    uint16_t* d12V_D3_V = GetParamPtr<uint16_t>("d12V_D3_V");
    int16_t* d12V_D3_A = GetParamPtr<int16_t>("d12V_D3_A");
    uint16_t* d12V_O1_V = GetParamPtr<uint16_t>("d12V_O1_V");
    int16_t* d12V_O1_A = GetParamPtr<int16_t>("d12V_O1_A");
    uint16_t* d12V_O2_V = GetParamPtr<uint16_t>("d12V_O2_V");
    int16_t* d12V_O2_A = GetParamPtr<int16_t>("d12V_O2_A");
    uint16_t* d5VA_V = GetParamPtr<uint16_t>("d5VA_V");
    uint16_t* d5VD1_V = GetParamPtr<uint16_t>("d5VD1_V");
    int16_t* d5VD1_A = GetParamPtr<int16_t>("d5VD1_A");
    uint16_t* d5VD2_V = GetParamPtr<uint16_t>("d5VD2_V");
    int16_t* d5VD2_A = GetParamPtr<int16_t>("d5VD2_A");
    uint16_t* d5VD3_V = GetParamPtr<uint16_t>("d5VD3_V");
    int16_t* d5VD3_A = GetParamPtr<int16_t>("d5VD3_A");
    uint16_t* d5VD4_V = GetParamPtr<uint16_t>("d5VD4_V");
    int16_t* d5VD4_A = GetParamPtr<int16_t>("d5VD4_A");
    uint16_t* d5VD5_V = GetParamPtr<uint16_t>("d5VD5_V");
    int16_t* d5VD5_A = GetParamPtr<int16_t>("d5VD5_A");
    uint16_t* Chgsense = GetParamPtr<uint16_t>("Chgsense");
    uint16_t* d2V5VDA1_V = GetParamPtr<uint16_t>("d2V5VDA1_V");
    uint16_t* d2V5VDA2_V = GetParamPtr<uint16_t>("d2V5VDA2_V");
    uint16_t* ACDC_V = GetParamPtr<uint16_t>("ACDC_V");
    int16_t* ADCD_A = GetParamPtr<int16_t>("ADCD_A");
    uint16_t* BATT_V = GetParamPtr<uint16_t>("BATT_V");
    int16_t* BATT_A = GetParamPtr<int16_t>("BATT_A");
    int16_t* BATT_A2 = GetParamPtr<int16_t>("BATT_A2");
    uint16_t* PBM_V = GetParamPtr<uint16_t>("PBM_V");
    int16_t* PBM_A = GetParamPtr<int16_t>("PBM_A");
    int16_t* PBM_A2 = GetParamPtr<int16_t>("PBM_A2");
    uint16_t* PUMP_V = GetParamPtr<uint16_t>("PUMP_V");
    int16_t* ECU_TEMP = GetParamPtr<int16_t>("ECU_TEMP");
    int16_t* ECU_TEMP1 = GetParamPtr<int16_t>("ECU_TEMP1");
    int16_t* ECU_TEMP2 = GetParamPtr<int16_t>("ECU_TEMP2");
    int16_t* ECU_TEMP3 = GetParamPtr<int16_t>("ECU_TEMP3");
    bool* is_receive_ack = GetParamPtr<bool>("is_receive_ack");
    uint8_t* ack_value = GetParamPtr<uint8_t>("ack_value");
    std::string* ver_power_ecu_version = GetParamPtr<std::string>("ver_power_ecu_version");
    std::string* ver_power_ecu_com_version = GetParamPtr<std::string>("ver_power_ecu_com_version");
    bool* is_receive_version = GetParamPtr<bool>("is_receive_version");
    std::string* start_time = GetParamPtr<std::string>("start_time");
    bool* is_enable_ecu1 = GetParamPtr<bool>("is_enable_ecu1");
    bool* is_enable_ecu2 = GetParamPtr<bool>("is_enable_ecu2");
    uint16_t* error_state = GetParamPtr<uint16_t>("error_state");
    uint32_t* counts = GetParamPtr<uint32_t>("counts");
    uint8_t* is_pump_enable = GetParamPtr<uint8_t>("is_pump_enable");
    uint8_t* is_motor_enable = GetParamPtr<uint8_t>("is_motor_enable");
    Color<uint8_t>* led_color = GetParamPtr<Color<uint8_t> >("led_color");
    bool* is_quaternion_reset = GetParamPtr<bool>("is_quaternion_reset");
    bool* is_gyro_reset = GetParamPtr<bool>("is_gyro_reset");
    uint8_t* is_solenoid_enable = GetParamPtr<uint8_t>("is_solenoid_enable");
    bool* is_cpu_shutdown = GetParamPtr<bool>("is_cpu_shutdown");
    bool* is_gpu_shutdown = GetParamPtr<bool>("is_gpu_shutdown");
    bool* is_ex1_shutdown = GetParamPtr<bool>("is_ex1_shutdown");
    bool* is_undck = GetParamPtr<bool>("is_undck");
    bool* is_amp_mute = GetParamPtr<bool>("is_amp_mute");
    bool* is_12vu_enable = GetParamPtr<bool>("is_12vu_enable");
    bool* is_5vd3_enable = GetParamPtr<bool>("is_5vd3_enable");
    bool* is_5vd4_enable = GetParamPtr<bool>("is_5vd4_enable");
    bool* is_5vd5_enable = GetParamPtr<bool>("is_5vd5_enable");
    std::string* repro_version = GetParamPtr<std::string>("repro_version");
    std::string* repro_data = GetParamPtr<std::string>("repro_data");
  }

  return true;
}

boost::system::error_code PowerEcuProtocol::Start() {
  // バージョン切り替え機能追加時はこの処理をクラスとして取り出す
  {
    // RTC合わせ
    //// 現在時刻セット
    auto in_time_t = static_cast<time_t>((int32_t)(clock_->now().seconds()));
    std::stringstream string_stream;
    string_stream << std::put_time(std::localtime(&in_time_t), "%Y%m%d%H%M%S");
    std::string time_string = string_stream.str();

    // 20170210095651のようなフォーマットを想定
    hsrb_power_ecu::Assert((time_string.size() == 14), "time_string format error");
    *(this->GetParamPtr<std::string>("start_time")) = time_string;
    AddCommandQueue(time_command_name_);

    // mute 解除
    *(this->GetParamPtr<bool>("is_amp_mute")) = false;
    AddCommandQueue(mute_command_name_);

    // 定期通信開始
    *(this->GetParamPtr<bool>("is_enable_ecu1")) = true;
    *(this->GetParamPtr<bool>("is_enable_ecu2")) = true;
    AddCommandQueue(start_command_name_);
  }

  return ProcessCommandQueue(true, kCycleHz, kRetryRate);
}

boost::system::error_code PowerEcuProtocol::Stop() {
  // バージョン切り替え機能追加時はこの処理をクラスとして取り出す
  { AddCommandQueue(stop_command_name_); }
  return ProcessCommandQueue(true, kCycleHz, kRetryRate);
}

bool PowerEcuProtocol::GetPowerEcuVersions(PowerEcuVersions& result) {
  rclcpp::WallRate loop_rate(100.0);
  const auto start_time = clock_->now();

  // コマンド送信
  *is_receive_version_ = false;
  size_t retry = 0;
  while (true) {
    boost::system::error_code ret = SendCommand(command_map_[getv_command_name_]);
    if (ret == boost::system::errc::success) {
      break;
    }
    ++retry;
    if (retry <= kRetryCount) {
      RCLCPP_FATAL(rclcpp::get_logger("power_ecu_protocol"), "getv_command send fault.");
      return false;
    }
  }

  // コマンド受信
  while (true) {
    if (ReceiveAll() != boost::system::errc::success) {
      if (GetReceiveErrorRate() > kRetryRate) {
        RCLCPP_FATAL(rclcpp::get_logger("power_ecu_protocol"), "receive fault.");
        return false;
      }
    }
    if (*is_receive_version_) {
      result.power_ecu_version.assign(*ver_power_ecu_version_);
      result.power_ecu_com_version.assign(*ver_power_ecu_com_version_);
      // バージョンチェック
      break;
    }
    // タイムアウトチェック
    if ((clock_->now() - start_time).seconds() > kCommandTimeout) {
      RCLCPP_FATAL(rclcpp::get_logger("power_ecu_protocol"), "receive timeout.");
      return false;
    }
    loop_rate.sleep();
  }

  return true;
}

boost::system::error_code PowerEcuProtocol::ProcessCommandQueue(bool check_ros,
                                                                double cycle_hz,
                                                                double error_rate) {
  // コマンド処理
  const auto start_time = clock_->now();

  // ポーリング周期チェック - コマンドキュー解決時間以上の場合は引数異常とする
  if (cycle_hz < (1.0 / kProcessCommandQueueTimeOut)) {
    // ポーリング周期0以下は引数異常
    return boost::system::errc::make_error_code(boost::system::errc::invalid_argument);
  }
  rclcpp::WallRate loop_rate(cycle_hz);

  while ((command_queue_.size() != 0) && (!check_ros || rclcpp::ok())) {
    if (SendAll() != boost::system::errc::success) {
      RCLCPP_WARN(rclcpp::get_logger("power_ecu_protocol"), "Send failed");
      if (GetSendErrorRate() > error_rate) {
        // 許容エラーレートを超えた時、失敗を返す
        return boost::system::errc::make_error_code(boost::system::errc::operation_canceled);
      }
    }
    loop_rate.sleep();

    if (ReceiveAll() != boost::system::errc::success) {
      RCLCPP_WARN(rclcpp::get_logger("power_ecu_protocol"), "Receive failed");
      if (GetReceiveErrorRate() > error_rate) {
        // 許容エラーレートを超えた時、失敗を返す
        return boost::system::errc::make_error_code(boost::system::errc::operation_canceled);
      }
    }
    // 時間のチェックは直前に行う
    if ((clock_->now() - start_time).seconds() > kProcessCommandQueueTimeOut) {
      RCLCPP_WARN(rclcpp::get_logger("power_ecu_protocol"), "ProcessCommandQueue Timeout");
      return boost::system::errc::make_error_code(boost::system::errc::timed_out);
    }
  }
  return boost::system::errc::make_error_code(boost::system::errc::success);
}


boost::system::error_code PowerEcuProtocol::ReceiveAll() {
  const auto time = clock_->now();
  // リードバッファの解析を行う
  // 受信処理
  boost::system::error_code ret = network_->Receive(receive_buffer_);

  if (ret != boost::system::errc::success) {
    // 受信失敗
    // serial_networkを通信デバイスとしていた場合、下記が失敗の原因である
    //   - 受信タイムアウト
    //     シリアルポートの受信バッファが空でも起こる為、正常な動作の可能性もある
    //   - その他シリアルポートのエラー
    //
    // networkは成功時successとだけ挙動が定義されているため、
    // success以外をnetwork_downとして上位へ通知する
    read_error_counter_.Register(false);
    return boost::system::errc::make_error_code(boost::system::errc::network_down);
  }

  boost::system::error_code result = boost::system::errc::make_error_code(boost::system::errc::success);
  // decode
  hsrb_power_ecu::PacketBuffer::const_iterator current_it = receive_buffer_.begin();
  hsrb_power_ecu::PacketBuffer::const_iterator encoded_it = receive_buffer_.begin();
  while (receive_buffer_.size() > 0 && ret != boost::system::errc::result_out_of_range) {
    ret = frame_decoder_.Decode(current_it, receive_buffer_.end(), encoded_it);
    if (ret != boost::system::errc::success && ret != boost::system::errc::result_out_of_range) {
      // 受信パケットが化けているとき protocol_error
      result = boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
    current_it = encoded_it;
  }

  // デコード済みの領域をクリア
  hsrb_power_ecu::PacketBuffer::const_iterator start_it = receive_buffer_.begin();
  size_t size = std::distance(start_it, encoded_it);
  receive_buffer_.erase_begin(size);


  // Ack受信確認
  // 1. 返信が来ない
  //    タウムアウト時間返信を待つ
  //    タイムアウト発生時は、RCLCPP_ERRORを吐く
  // 2. 失敗が返ってきた
  //    RCLCPP_ERRORを吐く
  //
  // is_waiting_ack_フラグを立てるのはWriteメソッドを使ってコマンド送信をした時。
  // 直接SendCommandメソッドを使ってコマンド送信をした場合、この処理は行われない。
  // getv_コマンドや、infoコマンド等Rxackを返さないコマンドは直接SendCommandメソッドを
  // 使ってコマンド送信を行う
  if (is_waiting_ack_) {     // Ack受信待ちの時
    if (*is_receive_ack_) {  // 返信が返ってきている時
      is_waiting_ack_ = false;
      CommandBuffer::iterator current_command = command_queue_.begin();
      if (*ack_value_ != 0) {  // 失敗が返ってきたら
        // コマンド処理終了
        result = boost::system::errc::make_error_code(boost::system::errc::operation_canceled);
        RCLCPP_ERROR(rclcpp::get_logger("power_ecu_protocol"),
                     "Failed send command. %s",
                     (*current_command)->GetCommandName().c_str());
      }
      // 返信が返ってきている場合は、キューから削除する
      command_queue_.pop_front();
    } else if ((time - last_send_command_time_).seconds() > kCommandTimeout) {
      // Ackタイムアウト時は、キューから削除する
      is_waiting_ack_ = false;
      RCLCPP_ERROR(rclcpp::get_logger("power_ecu_protocol"),
                   "command timeout: %s",
                   command_queue_.front()->GetCommandName().c_str());
      command_queue_.pop_front();
      result = boost::system::errc::make_error_code(boost::system::errc::timed_out);
    }
  }


  read_error_counter_.Register(result == boost::system::errc::success);
  return result;
}

boost::system::error_code PowerEcuProtocol::SendAll() {
  rclcpp::Time time = clock_->now();
  boost::system::error_code ret;
  if (command_queue_.size() != 0 && !is_waiting_ack_) {
    // 新しいコマンドの作成、送信
    CommandBuffer::iterator current_command = command_queue_.begin();
    ret = SendCommand(*current_command);
    if (ret != boost::system::errc::success) {
      write_error_counter_.Register(false);
      return ret;  // network_down
    }
    last_send_command_time_ = time;
    is_waiting_ack_ = true;
  } else {
    if (time - last_heartbeat_time_ > kHeartbeatDuration) {
      ++(*counts);
      ret = SendCommand(command_map_[heart_command_name_]);
      if (ret != boost::system::errc::success) {
        write_error_counter_.Register(false);
        return ret;  // network_down
      }
      last_heartbeat_time_ += kHeartbeatDuration;
    }
  }
  write_error_counter_.Register(true);
  return boost::system::errc::make_error_code(boost::system::errc::success);
}

void PowerEcuProtocol::AddCommandQueue(hsrb_power_ecu::CommandState::Ptr command) {
  command->SetProcessingStatus(true);  // 処理中フラグOn
  command->ClearRetryCount();          // リトライカウント0

  // コマンドキュー追加
  command_queue_.push_back(command);
}

boost::system::error_code PowerEcuProtocol::SendCommand(hsrb_power_ecu::CommandState::Ptr command) {
  // 変数初期化
  send_buffer_.clear();

  // 送信コマンド作成
  if (!frame_encoder_.Encode(send_buffer_, command->GetCommandName()) == boost::system::errc::success) {
    // コマンド追加はこのクラス内で完結しているため、エンコーダが存在しない子をは設計上ありえない
    RCLCPP_FATAL(rclcpp::get_logger("power_ecu_protocol"), "Packet encode failed.");
    exit(EXIT_FAILURE);
  }

  // コマンド送信
  if (network_->Send(send_buffer_) != boost::system::errc::success) {
    // コマンド送信失敗時はネットワークがダウンしたと判断する
    return boost::system::errc::make_error_code(boost::system::errc::network_down);
  }

  // ack受信フラグを落とす
  *is_receive_ack_ = false;
  return boost::system::errc::make_error_code(boost::system::errc::success);
}

}  // namespace hsrb_power_ecu
