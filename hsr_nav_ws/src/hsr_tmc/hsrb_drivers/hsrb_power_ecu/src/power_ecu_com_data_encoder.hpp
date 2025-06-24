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
#ifndef POWER_ECU_COM_DATA_ENCODER_HPP_
#define POWER_ECU_COM_DATA_ENCODER_HPP_

#include <stdint.h>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/smart_ptr/make_shared.hpp>
#include <boost/system/error_code.hpp>

#include "power_ecu_com_common.hpp"
#include "power_ecu_com_element_encoder.hpp"
#include "power_ecu_com_frame_encoder.hpp"

namespace hsrb_power_ecu {
/**
 * @brief 色指定用のコンテナ
 *
 * @tparam T 色情報の型
 */
template <typename T>
struct Color {
  T r;
  T g;
  T b;
};

// TODO(kitsunai): 送信コマンドで未実装の物がある
// 電源シャットダウンコマンド       pdown, 電源ECUのシャットダウンを行う
// 基本情報読出しコマンド           info1, 電源ECUの基本情報を１回読み出す
// オプション情報読出しコマンド     info2, 電源ECUのオプション情報を１回読み出す
// リプログラム開始コマンド         rpros, リプログラムを開始する
// リプログラムデータコマンド       rprod, リプログラムするデータを送信する
// リプログラム終了コマンド         rproe, リプログラムを終了する

/**
 * @brief 時計合わせコマンド
 */
class PowerEcuComTimeDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComTimeDataEncoder(PowerEcuComTimeDataEncoder const&);             // = delete;
  PowerEcuComTimeDataEncoder& operator=(PowerEcuComTimeDataEncoder const&);  // = delete;

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
  PowerEcuComTimeDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComTimeDataEncoder() {}

 private:
  PacketData packet_data_;  //!< パケットデータ
};

// 基本情報定期送信スタートコマンド start, 電源ECUの連続送信を開始する
class PowerEcuComStartDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComStartDataEncoder(PowerEcuComStartDataEncoder const&);             // = delete;
  PowerEcuComStartDataEncoder& operator=(PowerEcuComStartDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    bool is_enable_ecu1;  //!< ECU1を定期送信するかどうか
    bool is_enable_ecu2;  //!< ECU2を定期送信するかどうか
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComStartDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComStartDataEncoder() {}

 private:
  PacketData packet_data_;  //!< パケットデータ
};

// 基本情報定期送信ストップコマンド stop_, 電源ECUの連続送信を停止する
class PowerEcuComStopDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComStopDataEncoder(PowerEcuComStopDataEncoder const&);             // = delete;
  PowerEcuComStopDataEncoder& operator=(PowerEcuComStopDataEncoder const&);  // = delete;

 public:
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComStopDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComStopDataEncoder() {}
};

// ハートビート                     heart, メインCPUが電源ECUの生存確認を行う
class PowerEcuComHeartDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComHeartDataEncoder(PowerEcuComHeartDataEncoder const&);             // = delete;
  PowerEcuComHeartDataEncoder& operator=(PowerEcuComHeartDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    uint16_t error_state;  //!< エラー状態                16進数4桁 uint16
    uint32_t counts;       //!< カウント値（送信ごとに+1) 16進数8桁 uint32
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComHeartDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComHeartDataEncoder() {}

 private:
  PacketData packet_data_;  //!< パケットデータ
};

// ポンプスイッチ                   pump_, ポンプのスイッチを制御する
class PowerEcuComPumpDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComPumpDataEncoder(PowerEcuComPumpDataEncoder const&);             // = delete;
  PowerEcuComPumpDataEncoder& operator=(PowerEcuComPumpDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    uint8_t is_pump_enable;  //!< ポンプスイッチ(0:OFF 1:ON) 10進数1桁 uint8
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComPumpDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComPumpDataEncoder() {}

 private:
  PacketData packet_data_;  //!< パケットデータ
};

// 駆動系スイッチ                   pbmsw, 駆動系の電源を制御する
class PowerEcuComPbmswDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComPbmswDataEncoder(PowerEcuComPbmswDataEncoder const&);             // = delete;
  PowerEcuComPbmswDataEncoder& operator=(PowerEcuComPbmswDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    uint8_t is_motor_enable;  //!< 駆動系スイッチ(0 : OFF 1 : ON) 10進数1桁 uint8
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComPbmswDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComPbmswDataEncoder() {}

 private:
  PacketData packet_data_;  //!< パケットデータ
};

// 多用途LEDの色指定                ledc_, 多用途LEDの点灯／点滅や色を指定する
class PowerEcuComLedcDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComLedcDataEncoder(PowerEcuComLedcDataEncoder const&);             // = delete;
  PowerEcuComLedcDataEncoder& operator=(PowerEcuComLedcDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    Color<uint8_t> led_color;  //!< 色強度(0～255) 10進数3桁 uint8
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComLedcDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComLedcDataEncoder() {}

 private:
  PacketData packet_data_;  //!< パケットデータ
};

// 姿勢角演算リセット(g_res)
class PowerEcuComGResDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComGResDataEncoder(PowerEcuComGResDataEncoder const&);             // = delete;
  PowerEcuComGResDataEncoder& operator=(PowerEcuComGResDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    bool is_quaternion_reset;
    bool is_gyro_reset;
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComGResDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComGResDataEncoder() {}

 private:
  PacketData packet_data_;
};

// ソレノイドスイッチ(solsw)
class PowerEcuComSolswDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComSolswDataEncoder(PowerEcuComSolswDataEncoder const&);             // = delete;
  PowerEcuComSolswDataEncoder& operator=(PowerEcuComSolswDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    uint8_t is_solenoid_enable;  //!< ソレノイドスイッチ(0:OFF 1:ON) 10進数1桁 uint8
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComSolswDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComSolswDataEncoder() {}

 private:
  PacketData packet_data_;
};

// 電源シャットダウン(個別）コマンド(pdcmd)
class PowerEcuComPdcmdDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComPdcmdDataEncoder(PowerEcuComPdcmdDataEncoder const&);             // = delete;
  PowerEcuComPdcmdDataEncoder& operator=(PowerEcuComPdcmdDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    bool is_cpu_shutdown;
    bool is_gpu_shutdown;
    bool is_ex1_shutdown;
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComPdcmdDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComPdcmdDataEncoder() {}

 private:
  PacketData packet_data_;
};

// オーディオアンプMUTE(mute_)
class PowerEcuComMuteDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComMuteDataEncoder(PowerEcuComMuteDataEncoder const&);             // = delete;
  PowerEcuComMuteDataEncoder& operator=(PowerEcuComMuteDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    bool is_amp_mute;
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComMuteDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComMuteDataEncoder() {}

 private:
  PacketData packet_data_;
};

// バージョン情報の取得コマンド(getv_)
class PowerEcuComGetvDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComGetvDataEncoder(PowerEcuComGetvDataEncoder const&);             // = delete;
  PowerEcuComGetvDataEncoder& operator=(PowerEcuComGetvDataEncoder const&);  // = delete;

  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    uint8_t reserved;  //!< バージョン種別(常に0) 16進数1桁 uint8
  };

 public:
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComGetvDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComGetvDataEncoder() {}

 private:
  PacketData packet_data_;
};

// アンドック指令コマンド(undck)
class PowerEcuComUndckDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComUndckDataEncoder(PowerEcuComUndckDataEncoder const&);             // = delete;
  PowerEcuComUndckDataEncoder& operator=(PowerEcuComUndckDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    bool is_undck;
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComUndckDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComUndckDataEncoder() {}

 private:
  PacketData packet_data_;
};

// 12Vu_イネーブルコマンド(12vu_)
class PowerEcuCom12VuDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuCom12VuDataEncoder(PowerEcuCom12VuDataEncoder const&);             // = delete;
  PowerEcuCom12VuDataEncoder& operator=(PowerEcuCom12VuDataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    bool is_12vu_enable;
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuCom12VuDataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuCom12VuDataEncoder() {}

 private:
  PacketData packet_data_;
};

// 5Vd3_イネーブルコマンド(5vd3_)
class PowerEcuCom5Vd3DataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuCom5Vd3DataEncoder(PowerEcuCom5Vd3DataEncoder const&);             // = delete;
  PowerEcuCom5Vd3DataEncoder& operator=(PowerEcuCom5Vd3DataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    bool is_5vd3_enable;
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuCom5Vd3DataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuCom5Vd3DataEncoder() {}

 private:
  PacketData packet_data_;
};

// 5Vd4_イネーブルコマンド(5vd4_)
class PowerEcuCom5Vd4DataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuCom5Vd4DataEncoder(PowerEcuCom5Vd4DataEncoder const&);             // = delete;
  PowerEcuCom5Vd4DataEncoder& operator=(PowerEcuCom5Vd4DataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    bool is_5vd4_enable;
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuCom5Vd4DataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuCom5Vd4DataEncoder() {}

 private:
  PacketData packet_data_;
};

// 5Vd5_イネーブルコマンド(5vd5_)
class PowerEcuCom5Vd5DataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuCom5Vd5DataEncoder(PowerEcuCom5Vd5DataEncoder const&);             // = delete;
  PowerEcuCom5Vd5DataEncoder& operator=(PowerEcuCom5Vd5DataEncoder const&);  // = delete;

 public:
  /**
   * @brief パケットの内部構造
   */
  struct PacketData {
    bool is_5vd5_enable;
  };
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuCom5Vd5DataEncoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuCom5Vd5DataEncoder() {}

 private:
  PacketData packet_data_;
};

}  // namespace hsrb_power_ecu
#endif  // POWER_ECU_COM_DATA_ENCODER_HPP_
