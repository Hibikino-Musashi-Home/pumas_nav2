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
#include <algorithm>
#include <bitset>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <hsrb_power_ecu/i_network.hpp>
#include <hsrb_power_ecu/serial_network.hpp>

#include "get_parameter.hpp"
#include "power_ecu_protocol.hpp"

const std::map<uint32_t, std::string> kDiagnosticsErrors = {
    {0, "SPI1通信異常"},
    {1, "A/D変換器（制御系）異常"},
    {2, "A/D変換器（駆動系）異常"},
    {3, "A/D変換器（絶縁2次側）異常"},
    {4, "12Vd0異常（上昇）"},
    {5, "12Vd0異常（低下）"},
    {6, "12Vd0過電流異常"},
    {7, "12Vd1異常（上昇）"},
    {8, "12Vd1異常（低下）"},
    {9, "12Vd1過電流異常"},
    {10, "12Vd2異常（上昇）"},
    {11, "12Vd2異常（低下）"},
    {12, "12Vd2過電流異常"},
    {13, "12Vd3異常（上昇）"},
    {14, "12Vd3異常（低下）"},
    {15, "12Vd3過電流異常"},
    {16, "5Vd1異常（上昇）"},
    {17, "5Vd1異常（低下）"},
    {18, "5Vd1過電流異常"},
    {19, "5Vd2異常（上昇）"},
    {20, "5Vd2異常（低下）"},
    {21, "5Vd2過電流異常"},
    {22, "5Vd3異常（上昇）"},
    {23, "5Vd3異常（低下）"},
    {24, "5Vd3過電流異常"},
    {25, "5Vd4異常（上昇）"},
    {26, "5Vd4異常（低下）"},
    {27, "5Vd4過電流異常"},
    {28, "5Vd5異常（上昇）"},
    {29, "5Vd5異常（低下）"},
    {30, "5Vd5過電流異常"},
    {31, "5Va異常（上昇）"},
    {32, "5Va異常（低下）"},
    {33, "12Vo1異常（上昇）"},
    {34, "12Vo1異常（低下）"},
    {35, "12Vo1過電流異常"},
    {36, "12Vo2異常（上昇）"},
    {37, "12Vo2異常（低下）"},
    {38, "12Vo2過電流異常"},
    {39, "ACDC異常（上昇）"},
    {40, "ACDC異常（低下）"},
    {41, "ACDC過電流異常"},
    {42, "ACDC電流過負荷Level1"},
    {43, "ACDC電流過負荷Level2"},
    {44, "BATT電圧異常（上昇：充電）"},
    {45, "BATT電圧異常（上昇：放電）"},
    {46, "BATT電圧異常（低下）"},
    {47, "BATT放電電流異常"},
    {48, "BATT充電電流異常"},
    {49, "PBM電圧異常（上昇）"},
    {50, "PBM電圧異常（低下）"},
    {51, "PBM過電流異常"},
    {52, "PBM過電流異常Level2"},
    {53, "BATTショート異常"},
    {54, "PBMショート異常"},
    {55, "ポンプ出力異常"},
    {56, "12Vd1過電流異常"},
    {57, "12Vd2過電流異常"},
    {58, "12Vd3過電流異常"},
    {59, "12Vo1過電流異常"},
    {60, "12Vo2過電流異常"},
    {61, "5Vd1過電流異常"},
    {62, "5Vd2過電流異常"},
    {63, "5Vd3過電流異常"},
    {64, "5Vd4過電流異常"},
    {65, "5Vd5過電流異常"},
    {66, "PBM過電流異常"},
    {67, "プリチャージ異常"},
    {68, "12Vd2フォルト"},
    {69, "12Vd3フォルト"},
    {70, "12Vo2フォルト"},
    {71, "12Vd0/Vd1/Vo1フォルト"},
    {80, "回生回路異常"},
    {81, "回生回路過負荷異常"},
    {104, "SPI4通信異常"},
    {105, "I2C通信異常"},
    {106, "CPU通信異常"},
    {107, "A/D変換器（制御系）異常"},
    {108, "OFF処理異常12Vd2（ACDC電源）"},
    {109, "OFF処理異常12Vd2（バッテリ）"},
    {144, "ログファイル作成不可能"},
    {145, "フォルダ作成不可能"},
    {146, "ログ記録 RTC異常"},
    {147, "FATファイルシステム異常"},
    {148, "SDカード未挿入"},
    {160, "SPI6通信異常"},
    {161, "s1初期異常（イニシャル）"},
    {162, "s2初期異常（イニシャル）"},
    {163, "s3初期異常（イニシャル）"},
    {164, "ジャイロPitch張付き異常"},
    {165, "ジャイロRoll張付き異常"},
    {166, "ジャイロYaw張付き異常"},
    {167, "加速度X張付き異常"},
    {168, "加速度Y張付き異常"},
    {169, "加速度Z張付き異常"},
    {170, "ジャイロ2重故障"},
    {171, "加速度2重故障"},
    {172, "クオタニオンリセット異常"},
    {173, "S1加速度姿勢異常（イニシャル）"},
    {174, "S2加速度姿勢異常（イニシャル）"},
    {175, "S3加速度姿勢異常（イニシャル）"},
    {176, "雰囲気温度異常"},
    {177, "基板過熱異常1"},
    {178, "基板過熱異常2"},
    {179, "MCU温度異常"},
    {180, "温度センサ比較異常"},
    {208, "バッテリ通信異常"},
    {209, "バッテリ状態異常"},
    {210, "バッテリ放電温度異常（高温）"},
    {211, "バッテリ放電温度異常（低温）"},
    {212, "バッテリ温度比較異常"},
    {213, "セル充電温度警告"},
    {224, "イニシャルチェック 電圧信号①回路異常"},
    {225, "イニシャルチェック 電圧信号②回路異常"},
    {226, "イニシャルチェック 電圧信号③回路異常"},
    {227, "ACDC電源A/D変換器異常"},
    {228, "ACDC電源(PCA) 通信異常"},
    {229, "電源ECU通信異常"},
    {230, "出力コネクタ異常"},
    {231, "ACDC電源 動作異常"},
    {232, "PWR-003 電源MOS異常"},
    {233, "DC出力過電圧"},
    {234, "DC出力電圧低下"},
    {235, "出力過電流"},
    {236, "接続異常"},
    {237, "電流値異常"},
    {238, "ロボ接続確認 入力電圧低下"},
    {239, "通信異常"}
};

std::vector<uint32_t> ParseDiagStatus(const std::string& diag_status) {
  std::vector<uint32_t> result;
  for (auto i = 0u; i < diag_status.size(); ++i) {
    const auto bitset_value = std::bitset<4>(
        std::stoi(diag_status.substr(diag_status.size() - i - 1, 1), nullptr, 16));
    for (auto j = 0u; j < 4; ++j) {
      if (bitset_value.test(j)) {
        result.push_back(i * 4 + j);
      }
    }
  }
  return result;
}


void PrintDiagStatus(const std::string& diag_status) {
  std::cout << "diag_status: ";

  // A multiple assumption of 16bit like 256bit
  uint32_t space_count = 4;
  for (auto i = 0u; i < diag_status.size(); ++i) {
    if (space_count == 0) {
      space_count = 3;
      std::cout << " ";
    } else {
      --space_count;
    }
    std::cout << diag_status[i];
  }
  std::cout << std::endl;

  const auto error_bits = ParseDiagStatus(diag_status);
  for (const auto x : error_bits) {
    std::cout << "  " << kDiagnosticsErrors.at(x) << std::endl;
  }
}


/**
 * @bRIEF version confirmation command
 */
int32_t main(int32_t argc, char** argv) {
  // Command line analysis
  std::string port_name;

  {
    bool can_run = true;
    bool is_display_help = false;

    if (argc == 2) {
      // When there is one option
      if (std::string(argv[1]) == "--help") {
        // -Help does not return abnormalities when specified
        can_run = false;
        is_display_help = true;
      } else {
        port_name = argv[1];
      }
    } else {
      // There is no command line argument or two or more errors
      can_run = false;
    }

    if (!can_run) {
      std::cout << std::endl;
      std::cout << "This program gets the version of power ecu." << std::endl << std::endl;
      std::cout << "Usage: ros2 run hsrb_power_ecu info /dev/tty***" << std::endl << std::endl;
      if (is_display_help) {
        return 0;
      } else {
        return 1;
      }
    }
  }

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("power_ecu_info_command");

  const auto network = boost::make_shared<hsrb_power_ecu::SerialNetwork>();
  network->Configure("device_name", port_name);
  network->Configure("receive_timeout_ms", 1);
  auto protocol = boost::make_shared<hsrb_power_ecu::PowerEcuProtocol>(network, node);

  if (!protocol->Open()) {
    RCLCPP_FATAL(node->get_logger(), "Protocol Open failed.");
    exit(EXIT_FAILURE);
  }
  if (!protocol->Init()) {
    RCLCPP_FATAL(node->get_logger(), "Protocol Init Failed");
    exit(EXIT_FAILURE);
  }
  if (protocol->Start() != boost::system::errc::success) {
    RCLCPP_FATAL(node->get_logger(), "start failed");
    exit(EXIT_FAILURE);
  }

  hsrb_power_ecu::PowerEcuVersions versions;
  if (protocol->GetPowerEcuVersions(versions)) {
    std::cout << "power ecu version : " << versions.power_ecu_version << std::endl;
    std::cout << "protocol version : " << versions.power_ecu_com_version << std::endl << std::endl;

    const auto diag_status = *hsrb_power_ecu::ExistParamPtr<std::string>(protocol, "diag_status");
    PrintDiagStatus(diag_status);
  } else {
    RCLCPP_FATAL(node->get_logger(), "GetPowerEcuVersions failed.");
    exit(EXIT_FAILURE);
  }

  protocol->Close();
  return EXIT_SUCCESS;
}
