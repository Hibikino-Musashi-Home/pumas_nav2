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
#ifndef POWER_ECU_COM_DATA_DECODER_HPP_
#define POWER_ECU_COM_DATA_DECODER_HPP_
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/function.hpp>
#include <boost/system/error_code.hpp>

#include "power_ecu_com_common.hpp"
#include "power_ecu_com_element_decoder.hpp"
#include "power_ecu_com_frame_decoder.hpp"

namespace hsrb_power_ecu {
/* *
* @brief ecu1コマンドのデコーダ
*/
class PowerEcuComEcu1DataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComEcu1DataDecoder(PowerEcuComEcu1DataDecoder const&);             // = delete;
  PowerEcuComEcu1DataDecoder& operator=(PowerEcuComEcu1DataDecoder const&);  // = delete;

 public:
  /**
   * @brief ecu1コマンドのhwクラスのメンバ変数ポインタコンテナ
   */
  struct PacketData {
    uint32_t time_stamp;                             //!< タイムスタンプ [ms]
    std::string ecu1_date;                           //!< 日時 YYYYMMDDhhmmss
    std::string power_ecu_status_flag;               //!< 電源ECUステータス Sの部分
    std::string diag_status;                         //!< ダイアグ情報 16進32桁
    double battery_total_capacity;                   //!< バッテリ総容量 [mAh]
    double battery_remaining_capacity;               //!< バッテリ残容量 [mAh]
    double electric_current;                         //!< 電流値 [mA]
    double battery_voltage;                          //!< 電池電圧 [mV]
    double battery_temperature;                      //!< 電池温度 [C]
    bool is_battery_crgov;                           //!< 過充電 1:過充電
    bool is_battery_23par;                           //!< 並列数 0:2並 1:3並
    bool is_battery_std;                             //!< 学習許可 1:学習許可
    bool is_battery_full;                            //!< 満充電 1:満充電状態
    bool is_battery_discov;                          //!< 過放電 1:過放電
    bool is_battery_chg;                             //!< 充電許可 1:充電許可
    bool is_battery_disc;                            //!< 放電許可 1:放電許可
    bool is_battery_0per;                            //!< 0%検出 1:0%検出状態
    bool is_battery_45par;                           //!< 並列数 0:4並 1:5並
    bool is_battery_sel;                             //!< 最小セル電圧0%検出状態 1:0%検出状態
    bool is_battery_bal;                             //!< セルバランス崩れ 1:崩れ
    uint16_t battery_initial_learning_capacity;      //!< バッテリ初期学習容量 [mAh]
    uint16_t battery_error_status;                   //!< バッテリ異常ステータス 仕様未定義
    double battery_relative_capacity;                //!< 相対容量 [%]
    uint32_t power_ecu_internal_state;               //!< (新規)4.5.3のS**の数 電源ECU内部状態
    bool is_powerecu_bat_stat;                       //!< (新規)バッテリ充電状態
    bool is_powerecu_sw_kinoko;                      //!< (新規)有線緊急停止SW
    bool is_powerecu_sw_pwr;                         //!< (新規)電源（プリウスSW)
    bool is_powerecu_sw_drv;                         //!< (新規)駆動SW
    bool is_powerecu_sw_latch;                       //!< (新規)ラッチ解除SW
    bool is_powerecu_sw_w_sel;                       //!< (新規)無線切り替えSW
    bool is_powerecu_sw_w_stop;                      //!< (新規)無線緊急停止SW
    bool is_bumper_bumper2;                          //!< バンパセンサ状態2 1:接触あり
    bool is_bumper_bumper1;                          //!< バンパセンサ状態1 1:接触あり
    bool is_bumper_prox5;                            //!< 近接センサラッチ状態5 1:近接物あり
    bool is_bumper_prox4;                            //!< 近接センサラッチ状態4 1:近接物あり
    bool is_bumper_prox3;                            //!< 近接センサラッチ状態3 1:近接物あり
    bool is_bumper_prox2;                            //!< 近接センサラッチ状態2 1:近接物あり
    bool is_bumper_prox1;                            //!< 近接センサラッチ状態1 1:近接物あり
    std::string gyro_status;                         //!< (新規)ジャイロ姿勢角演算ステータス
    boost::array<double, 4> imu_quaternions;         //!< quaternion x,y,z,t -1.0~1.0
    boost::array<double, 3> imu_angular_velocities;  //!< 角速度 x,y,z [rad/s]
    boost::array<double, 3> imu_accelerations;       //!< 加速度 x,y,z [m/s^2]
    uint8_t charger_state;                           //!< 自動充電ステータス
  };

 private:
  /**
   * @brief パケットデータ受け取り用の内部コンテナ
   */
  struct PacketRawData {
    uint32_t time_stamp;                         //!< タイムスタンプ 10進10桁
    std::string date;                            //!< 日時 文字
    std::string power_ecu_status_flag;           //!< 電源ECUステータス 16進2桁
    std::string power_ecu_status;                //!< (新規)電源ECU状態 16進数16桁
    std::string diag_status;                     //!< uint816ダイアグ情報 16進32桁
    uint16_t battery_total_capacity;             //!< バッテリ総容量 符号10進5桁
    uint16_t battery_remaining_capacity;         //!< バッテリ残容量 符号10進5桁
    int16_t electric_current;                    //!< 電流値 符号10進5桁
    uint16_t battery_voltage;                    //!< 電池電圧 符号10進5桁
    int8_t battery_temperature;                  //!< 電池温度 符号10進3桁
    uint16_t battery_state_flag;                 //!< バッテリ状態フラグ 16進4桁
    uint16_t battery_initial_learning_capacity;  //!< バッテリ初期学習容量 10進5桁
    uint16_t battery_error_status;               //!< バッテリ異常ステータス 16進4桁
    uint8_t battery_relative_capacity;           //!< 相対容量 10進3桁
    uint8_t bumper_status;                       //!< 近接、バンパセンサ状態 16進2桁
    std::string gyro_status;                     //!< (新規)ジャイロ姿勢角演算ステータス 16進数16桁
    int32_t quaternion_t;                        //!< quaternion_t 符号10進10桁
    int32_t quaternion_x;                        //!< quaternion_x 符号10進10桁
    int32_t quaternion_y;                        //!< quaternion_y 符号10進10桁
    int32_t quaternion_z;                        //!< quaternion_z 符号10進10桁
    int32_t angular_velocity_x;                  //!< 角速度x 符号10進10桁
    int32_t angular_velocity_y;                  //!< 角速度y 符号10進10桁
    int32_t angular_velocity_z;                  //!< 角速度z 符号10進10桁
    int32_t acceleration_x;                      //!< 加速度x 符号10進10桁
    int32_t acceleration_y;                      //!< 加速度y 符号10進10桁
    int32_t acceleration_z;                      //!< 加速度z 符号10進10桁
    uint8_t charger_state;                       //!< 自動充電ステータス
  };

 public:
  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComEcu1DataDecoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComEcu1DataDecoder() {}

 private:
  /**
   * @brief デコード後処理
   * @return 成功時 True
   */
  virtual bool Update();

  PacketRawData packet_raw_data_;  //!< Hwクラス変数ポインタコンテナ
  PacketData packet_out_;          //!< 受信データ受け取り用バッファ
  std::string temp_str;            //!< テンポラリ
  uint32_t temp_uint;
};

/**
* @brief ecu2コマンドのデコーダ
*/
class PowerEcuComEcu2DataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComEcu2DataDecoder(PowerEcuComEcu2DataDecoder const&);             // = delete;
  PowerEcuComEcu2DataDecoder& operator=(PowerEcuComEcu2DataDecoder const&);  // = delete;

 public:
  /**
  * @brief ecu2コマンドのパケットデータ
  */
  struct PacketData {
    std::string ecu2_date;               //!< 日時（YYYYMMDDhhmmss） 文字
    uint16_t d12V_D0_V;                  //!< 12Vd0電圧        [mV] 符号1桁+10進数5桁
    int16_t d12V_D0_A;                   //!< 12Vd0電流        [mA] 符号1桁+10進数5桁
    uint16_t d12V_D1_V;                  //!< 12Vd1電圧        [mV] 符号1桁10進数5桁
    int16_t d12V_D1_A;                   //!< 12Vd1電流        [mA] 符号1桁10進数5桁
    uint16_t d12V_D2_V;                  //!< 12Vd2電圧        [mV] 符号1桁10進数5桁
    int16_t d12V_D2_A;                   //!< 12Vd2電流        [mA] 符号1桁10進数5桁
    uint16_t d12V_D3_V;                  //!< 12Vd3電圧        [mV] 符号1桁10進数5桁
    int16_t d12V_D3_A;                   //!< 12Vd3電流        [mA] 符号1桁10進数5桁
    uint16_t d12V_O1_V;                  //!< 12Vo1電圧        [mV] 符号1桁10進数5桁
    int16_t d12V_O1_A;                   //!< 12Vo1電流        [mA] 符号1桁10進数5桁
    uint16_t d12V_O2_V;                  //!< 12Vo2電圧        [mV] 符号1桁10進数5桁
    int16_t d12V_O2_A;                   //!< 12Vo2電流        [mA] 符号1桁10進数5桁
    uint16_t d5VA_V;                     //!< 5Va電圧          [mV] 符号1桁10進数5桁
    uint16_t d5VD1_V;                    //!< 5Vd1電圧         [mV] 符号1桁10進数5桁
    int16_t d5VD1_A;                     //!< 5Vd1電流         [mA] 符号1桁10進数5桁
    uint16_t d5VD2_V;                    //!< 5Vd2電圧         [mV] 符号1桁10進数5桁
    int16_t d5VD2_A;                     //!< 5Vd2電流         [mA] 符号1桁10進数5桁
    uint16_t d5VD3_V;                    //!< 5Vd3電圧         [mV] 符号1桁10進数5桁
    int16_t d5VD3_A;                     //!< 5Vd3電流         [mA] 符号1桁10進数5桁
    uint16_t d5VD4_V;                    //!< 5Vd4電圧         [mV] 符号1桁10進数5桁
    int16_t d5VD4_A;                     //!< 5Vd4電流         [mA] 符号1桁10進数5桁
    uint16_t d5VD5_V;                    //!< 5Vd5電圧         [mV] 符号1桁10進数5桁
    int16_t d5VD5_A;                     //!< 5Vd5電流         [mA] 符号1桁10進数5桁
    uint16_t Chgsense;                   //!< 自動順電挿抜端子電圧 [mV] 符号1桁10進数5桁
    uint16_t d2V5VDA1_V;                 //!< 2.5Va1電圧(A/D1) [mV] 符号1桁10進数5桁
    uint16_t d2V5VDA2_V;                 //!< 2.5Va2電圧(A/D2) [mV] 符号1桁10進数5桁
    uint16_t ACDC_V;                     //!< ACDC電圧         [mV] 符号1桁10進数5桁
    int16_t ADCD_A;                      //!< ACDC電流         [mA] 符号1桁10進数5桁
    uint16_t BATT_V;                     //!< BATT電圧         [mV] 符号1桁10進数5桁
    int16_t BATT_A;                      //!< BATT電流         [mA] 符号1桁10進数5桁
    int16_t BATT_A2;                     //!< BATT電流2        [10mA] 符号1桁10進数5桁
    uint16_t PBM_V;                      //!< PBM電圧          [mV] 符号1桁10進数5桁
    int16_t PBM_A;                       //!< PBM電流          [mA] 符号1桁10進数5桁
    int16_t PBM_A2;                      //!< PBM電流2         [10mA] 符号1桁10進数5桁
    uint16_t PUMP_V;                     //!< ポンプセンサ電圧 [mV] 符号1桁10進数5桁
    int16_t ECU_TEMP;                    //!< 電源ECU温度      [℃] 符号1桁10進数3桁
    int16_t ECU_TEMP1;                   //!< 電源ECU温度1     [℃] 符号1桁10進数3桁
    int16_t ECU_TEMP2;                   //!< 電源ECU温度2     [℃] 符号1桁10進数3桁
    int16_t ECU_TEMP3;                   //!< 電源ECU温度3     [℃] 符号1桁10進数3桁
  };

  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComEcu2DataDecoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComEcu2DataDecoder() {}

 private:
  /**
   * @brief デコード後処理
   * @return 成功時 True
   */
  virtual bool Update();
  PacketData packet_data_;  //!< パケットデータ
};

/**
 * @brief RXACKコマンドのデコーダ
 */
class PowerEcuComRxackDataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComRxackDataDecoder(PowerEcuComRxackDataDecoder const&);             // = delete;
  PowerEcuComRxackDataDecoder& operator=(PowerEcuComRxackDataDecoder const&);  // = delete;

 public:
  /**
   * @brief Rxackのパケットデータ
   */
  struct PacketData {
    /**
     * @brief コンストラクタ
     */
    PacketData() : is_receive_ack(false), ack_value(0) {}
    bool is_receive_ack;  //!< Ackが返ってきたかどうか
    uint8_t ack_value;    //!< 返信コマンドの戻り値
  };

  /**
   * @brief コンストラクタ
   * @param packet_data パケットデータ
   */
  PowerEcuComRxackDataDecoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComRxackDataDecoder() {}

 private:
  /**
   * @brief デコード後処理
   * @return 成功時 True
   */
  virtual bool Update();

  PacketData packet_data_;  //!< パケットデータ
};

/**
 * @brief Verコマンドのデコーダ
 */
class PowerEcuComVerDataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComVerDataDecoder(PowerEcuComVerDataDecoder const&);             // = delete;
  PowerEcuComVerDataDecoder& operator=(PowerEcuComVerDataDecoder const&);  // = delete;

 public:
  /**
   * @brief Verのパケットデータ
   */
  struct PacketData {
    PacketData() : is_receive_version(false) {}
    std::string ver_power_ecu_version;      //!< 電源ECUファームVer[git hash 20byte] 16進数40桁
    std::string ver_power_ecu_com_version;  //!< 電源ECU通信構造HASH[hash 20byte] 16進数40桁
    bool is_receive_version;                //!< Verコマンドを受信したかどうか
  };

  /**
   * @brief コンストラクタ
   * @param[in] packet_data パケットデータ
   */
  PowerEcuComVerDataDecoder();
  /**
   * @brief デストラクタ
   */
  virtual ~PowerEcuComVerDataDecoder() {}

 private:
  /**
   * @brief デコード後処理
   * @return 成功時 True
   */
  virtual bool Update();

  PacketData packet_data_;                 //!< パケットデータ
  std::string power_ecu_version_raw_;      //!< 電源ECUファームVer[git hash 20byte] 16進数40桁
  std::string power_ecu_com_version_raw_;  //!< 電源ECU通信構造HASH[hash 20byte] 16進数40桁
};

}  // namespace hsrb_power_ecu
#endif  // POWER_ECU_COM_DATA_DECODER_HPP_
