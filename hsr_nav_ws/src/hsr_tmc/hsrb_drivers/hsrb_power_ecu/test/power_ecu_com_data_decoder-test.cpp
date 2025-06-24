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
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <gtest/gtest.h>

#include "../src/power_ecu_com_data_decoder.hpp"
#include "common_methods.hpp"

namespace {
const size_t kBufferSize = 1000;
void CompareEcu1PacketData(hsrb_power_ecu::PowerEcuComEcu1DataDecoder &decoder,
                           hsrb_power_ecu::PowerEcuComEcu1DataDecoder::PacketData dist) {
  EXPECT_EQ(*decoder.GetParamPtr<uint32_t>("time_stamp"), dist.time_stamp);
  EXPECT_STREQ((*decoder.GetParamPtr<std::string>("ecu1_date")).c_str(), dist.ecu1_date.c_str());
  EXPECT_STREQ((*decoder.GetParamPtr<std::string>("power_ecu_status_flag")).c_str(),
    dist.power_ecu_status_flag.c_str());
  EXPECT_STREQ((*decoder.GetParamPtr<std::string>("diag_status")).c_str(), dist.diag_status.c_str());
  EXPECT_DOUBLE_EQ(*decoder.GetParamPtr<double>("battery_total_capacity"), dist.battery_total_capacity);
  EXPECT_DOUBLE_EQ(*decoder.GetParamPtr<double>("battery_remaining_capacity"), dist.battery_remaining_capacity);
  EXPECT_DOUBLE_EQ(*decoder.GetParamPtr<double>("electric_current"), dist.electric_current);
  EXPECT_DOUBLE_EQ(*decoder.GetParamPtr<double>("battery_voltage"), dist.battery_voltage);
  EXPECT_DOUBLE_EQ(*decoder.GetParamPtr<double>("battery_temperature"), dist.battery_temperature);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_battery_crgov"), dist.is_battery_crgov);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_battery_23par"), dist.is_battery_23par);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_battery_std"), dist.is_battery_std);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_battery_full"), dist.is_battery_full);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_battery_discov"), dist.is_battery_discov);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_battery_chg"), dist.is_battery_chg);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_battery_disc"), dist.is_battery_disc);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_battery_0per"), dist.is_battery_0per);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_battery_45par"), dist.is_battery_45par);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_battery_sel"), dist.is_battery_sel);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_battery_bal"), dist.is_battery_bal);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("battery_initial_learning_capacity"),
    dist.battery_initial_learning_capacity);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("battery_error_status"), dist.battery_error_status);
  EXPECT_DOUBLE_EQ(*decoder.GetParamPtr<double>("battery_relative_capacity"), dist.battery_relative_capacity);
  EXPECT_EQ(*decoder.GetParamPtr<uint32_t>("power_ecu_internal_state"), dist.power_ecu_internal_state);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_powerecu_bat_stat"), dist.is_powerecu_bat_stat);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_powerecu_sw_kinoko"), dist.is_powerecu_sw_kinoko);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_powerecu_sw_pwr"), dist.is_powerecu_sw_pwr);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_powerecu_sw_drv"), dist.is_powerecu_sw_drv);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_powerecu_sw_latch"), dist.is_powerecu_sw_latch);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_powerecu_sw_w_sel"), dist.is_powerecu_sw_w_sel);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_powerecu_sw_w_stop"), dist.is_powerecu_sw_w_stop);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_bumper_bumper2"), dist.is_bumper_bumper2);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_bumper_bumper1"), dist.is_bumper_bumper1);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_bumper_prox5"), dist.is_bumper_prox5);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_bumper_prox4"), dist.is_bumper_prox4);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_bumper_prox3"), dist.is_bumper_prox3);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_bumper_prox2"), dist.is_bumper_prox2);
  EXPECT_EQ(*decoder.GetParamPtr<bool>("is_bumper_prox1"), dist.is_bumper_prox1);
  EXPECT_STREQ((*decoder.GetParamPtr<std::string>("gyro_status")).c_str(), dist.gyro_status.c_str());

  boost::array<double, 4>* imu_quaternions = decoder.GetParamPtr<boost::array<double, 4> >("imu_quaternions");
  EXPECT_DOUBLE_EQ((*imu_quaternions)[0], dist.imu_quaternions[0]);
  EXPECT_DOUBLE_EQ((*imu_quaternions)[1], dist.imu_quaternions[1]);
  EXPECT_DOUBLE_EQ((*imu_quaternions)[2], dist.imu_quaternions[2]);
  EXPECT_DOUBLE_EQ((*imu_quaternions)[3], dist.imu_quaternions[3]);

  boost::array<double, 3>* imu_angular_velocities =
    decoder.GetParamPtr<boost::array<double, 3> >("imu_angular_velocities");
  EXPECT_DOUBLE_EQ((*imu_angular_velocities)[0], dist.imu_angular_velocities[0]);
  EXPECT_DOUBLE_EQ((*imu_angular_velocities)[1], dist.imu_angular_velocities[1]);
  EXPECT_DOUBLE_EQ((*imu_angular_velocities)[2], dist.imu_angular_velocities[2]);

  boost::array<double, 3>* imu_accelerations = decoder.GetParamPtr<boost::array<double, 3> >("imu_accelerations");
  EXPECT_DOUBLE_EQ((*imu_accelerations)[0], dist.imu_accelerations[0]);
  EXPECT_DOUBLE_EQ((*imu_accelerations)[1], dist.imu_accelerations[1]);
  EXPECT_DOUBLE_EQ((*imu_accelerations)[2], dist.imu_accelerations[2]);

  EXPECT_EQ(*decoder.GetParamPtr<uint8_t>("charger_state"), dist.charger_state);
}

void CompareEcu2PacketData(hsrb_power_ecu::PowerEcuComEcu2DataDecoder &decoder,
                           hsrb_power_ecu::PowerEcuComEcu2DataDecoder::PacketData dist) {
  EXPECT_STREQ((*decoder.GetParamPtr<std::string>("ecu2_date")).c_str(), dist.ecu2_date.c_str());
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d12V_D0_V"), dist.d12V_D0_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("d12V_D0_A"), dist.d12V_D0_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d12V_D1_V"), dist.d12V_D1_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("d12V_D1_A"), dist.d12V_D1_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d12V_D2_V"), dist.d12V_D2_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("d12V_D2_A"), dist.d12V_D2_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d12V_D3_V"), dist.d12V_D3_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("d12V_D3_A"), dist.d12V_D3_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d12V_O1_V"), dist.d12V_O1_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("d12V_O1_A"), dist.d12V_O1_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d12V_O2_V"), dist.d12V_O2_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("d12V_O2_A"), dist.d12V_O2_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d5VA_V"), dist.d5VA_V);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d5VD1_V"), dist.d5VD1_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("d5VD1_A"), dist.d5VD1_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d5VD2_V"), dist.d5VD2_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("d5VD2_A"), dist.d5VD2_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d5VD3_V"), dist.d5VD3_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("d5VD3_A"), dist.d5VD3_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d5VD4_V"), dist.d5VD4_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("d5VD4_A"), dist.d5VD4_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d5VD5_V"), dist.d5VD5_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("d5VD5_A"), dist.d5VD5_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("Chgsense"), dist.Chgsense);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d2V5VDA1_V"), dist.d2V5VDA1_V);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("d2V5VDA2_V"), dist.d2V5VDA2_V);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("ACDC_V"), dist.ACDC_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("ADCD_A"), dist.ADCD_A);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("BATT_V"), dist.BATT_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("BATT_A"), dist.BATT_A);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("BATT_A2"), dist.BATT_A2);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("PBM_V"), dist.PBM_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("PBM_A"), dist.PBM_A);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("PBM_A2"), dist.PBM_A2);
  EXPECT_EQ(*decoder.GetParamPtr<uint16_t>("PUMP_V"), dist.PUMP_V);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("ECU_TEMP"), dist.ECU_TEMP);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("ECU_TEMP1"), dist.ECU_TEMP1);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("ECU_TEMP2"), dist.ECU_TEMP2);
  EXPECT_EQ(*decoder.GetParamPtr<int16_t>("ECU_TEMP3"), dist.ECU_TEMP3);
}
void CompareVerPacketData(const hsrb_power_ecu::PowerEcuComVerDataDecoder &decoder,
                          const hsrb_power_ecu::PowerEcuComVerDataDecoder::PacketData& dist) {
  EXPECT_STREQ((*decoder.GetParamPtr<std::string>("ver_power_ecu_version")).c_str(),
    dist.ver_power_ecu_version.c_str());
  EXPECT_STREQ((*decoder.GetParamPtr<std::string>("ver_power_ecu_com_version")).c_str(),
    dist.ver_power_ecu_com_version.c_str());
}
}  // anonymous namespace

TEST(PowerEcuComEcu1DataDecoderTest, NormalCase) {
  hsrb_power_ecu::PowerEcuComEcu1DataDecoder decoder;

  EXPECT_EQ(decoder.GetPacketSize(), 326);
  EXPECT_STREQ(decoder.GetPacketName().c_str(), "ecu1_");

  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);
  hsrb_power_ecu::PowerEcuComEcu1DataDecoder::PacketData result;
  result.time_stamp = 0;
  result.ecu1_date = "00000000000000";
  result.power_ecu_status_flag = "00";
  result.diag_status = "0000000000000000000000000000000000000000000000000000000000000000";
  result.battery_total_capacity = 0.0;
  result.battery_remaining_capacity = 0.0;
  result.electric_current = 0.0;
  result.battery_voltage = 0.0;
  result.battery_temperature = 0.0;
  result.is_battery_crgov = false;
  result.is_battery_23par = false;
  result.is_battery_std = false;
  result.is_battery_full = false;
  result.is_battery_discov = false;
  result.is_battery_chg = false;
  result.is_battery_disc = false;
  result.is_battery_0per = false;
  result.is_battery_45par = false;
  result.is_battery_sel = false;
  result.is_battery_bal = false;
  result.battery_initial_learning_capacity = 0;
  result.battery_error_status = 0;
  result.battery_relative_capacity = 0.0;
  result.power_ecu_internal_state = 0;
  result.is_powerecu_bat_stat = false;
  result.is_powerecu_sw_kinoko = true;
  result.is_powerecu_sw_pwr = false;
  result.is_powerecu_sw_drv = true;
  result.is_powerecu_sw_latch = false;
  result.is_powerecu_sw_w_sel = false;
  result.is_powerecu_sw_w_stop = true;
  result.is_bumper_bumper2 = false;
  result.is_bumper_bumper1 = false;
  result.is_bumper_prox5 = false;
  result.is_bumper_prox4 = false;
  result.is_bumper_prox3 = false;
  result.is_bumper_prox2 = false;
  result.is_bumper_prox1 = false;
  result.gyro_status = "0000000000000000";
  result.imu_quaternions[0] = 0.0;
  result.imu_quaternions[1] = 0.0;
  result.imu_quaternions[2] = 0.0;
  result.imu_quaternions[3] = 0.0;
  result.imu_angular_velocities[0] = 0.0;
  result.imu_angular_velocities[1] = 0.0;
  result.imu_angular_velocities[2] = 0.0;
  result.imu_accelerations[0] = 0.0;
  result.imu_accelerations[1] = 0.0;
  result.imu_accelerations[2] = 0.0;
  result.charger_state = 0;

  {
    SCOPED_TRACE("init");
    std::string packet =
        "0000000000,00000000000000,h00,h0000000000000000,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("time_stamp");
    std::string packet =
        "0000000001,00000000000000,h00,h0000000000000000,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.time_stamp = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("data");
    std::string packet =
        "0000000001,00000000000001,h00,h0000000000000000,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.ecu1_date = "00000000000001";
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("power_ecu_status");
    std::string packet =
        "0000000001,00000000000001,h01,h0000000000000000,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.power_ecu_status_flag = "01";
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("power_ecu_internal_state");
    std::string packet =
        "0000000001,00000000000001,h01,h0000000000010000,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.power_ecu_internal_state = 1;
    CompareEcu1PacketData(decoder, result);
    packet =
        "0000000001,00000000000001,h01,h0000000000110000,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.power_ecu_internal_state = 17;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_powerecu_bat_stat");
    std::string packet =
        "0000000001,00000000000001,h01,h0000000000110040,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_powerecu_bat_stat = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_powerecu_sw_kinoko");
    std::string packet =
        "0000000001,00000000000001,h01,h0000000000110060,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_powerecu_sw_kinoko = false;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_powerecu_sw_pwr");
    std::string packet =
        "0000000001,00000000000001,h01,h0000000000110070,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_powerecu_sw_pwr = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_powerecu_sw_drv");
    std::string packet =
        "0000000001,00000000000001,h01,h0000000000090078,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.power_ecu_internal_state = 9;
    result.is_powerecu_sw_drv = false;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_powerecu_sw_latch");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007C,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_powerecu_sw_latch = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_powerecu_sw_w_sel");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007E,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_powerecu_sw_w_sel = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_powerecu_sw_w_stop");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000000, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_powerecu_sw_w_stop = false;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("diag_status");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 00000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.diag_status = "0000000000000000000000000000000000000000000000000000000000000001";
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("battery_total_capacity");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 00000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.battery_total_capacity = 1.0;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("battery_remaining_capacity");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 00000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.battery_remaining_capacity = 1.0;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("electric_current");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.electric_current = 1.0;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("battery_voltage");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 000,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.battery_voltage = 1.0;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("battery_temperature");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h0000,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.battery_temperature = 1.0;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_battery_crgov");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h0080,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_battery_crgov = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_battery_23par");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h00C0,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_battery_23par = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_battery_std");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h00E0,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_battery_std = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_battery_full");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h00F0,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_battery_full = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_battery_discov");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h00F8,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_battery_discov = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_battery_chg");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h00FC,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_battery_chg = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_battery_disc");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h00FE,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_battery_disc = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_battery_0per");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h00FF,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_battery_0per = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_battery_45par");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h04FF,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_battery_45par = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_battery_sel");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h06FF,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_battery_sel = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_battery_bal");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00000,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_battery_bal = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("battery_initial_learning_capacity");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0000,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.battery_initial_learning_capacity = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("battery_error_status");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,000,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.battery_error_status = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("battery_relative_capacity");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h00,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.battery_relative_capacity = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_bumper_bumper2");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h40,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_bumper_bumper2 = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_bumper_bumper1");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h60,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_bumper_bumper1 = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_bumper_prox5");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h70,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_bumper_prox5 = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_bumper_prox4");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h78,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_bumper_prox4 = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_bumper_prox3");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7C,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_bumper_prox3 = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_bumper_prox2");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7E,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_bumper_prox2 = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("is_bumper_prox1");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000000, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.is_bumper_prox1 = true;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("gyro_status");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 0000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.gyro_status = "0000000000000001";
    CompareEcu1PacketData(decoder, result);
  }
  {
    SCOPED_TRACE("imu_quaternions[3]");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000,-0000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.imu_quaternions[3] = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("imu_quaternions[0]");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 0000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.imu_quaternions[0] = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("imu_quaternions[1]");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 1000000000,"
        " 0000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.imu_quaternions[1] = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("imu_quaternions[2]");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 1000000000,"
        " 1000000000,-0000000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.imu_quaternions[2] = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("imu_angular_velocities[0]");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 1000000000,"
        " 1000000000, 0100000000, 0000000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.imu_angular_velocities[0] = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("imu_angular_velocities[1]");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 1000000000,"
        " 1000000000, 0100000000, 0100000000, 0000000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.imu_angular_velocities[1] = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("imu_angular_velocities[2]");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 1000000000,"
        " 1000000000, 0100000000, 0100000000, 0100000000,-0000000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.imu_angular_velocities[2] = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("imu_accelerations[0]");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 1000000000,"
        " 1000000000, 0100000000, 0100000000, 0100000000, 0100000000, 0000000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.imu_accelerations[0] = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("imu_accelerations[1]");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 1000000000,"
        " 1000000000, 0100000000, 0100000000, 0100000000, 0100000000, 0100000000,-0000000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.imu_accelerations[1] = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("imu_accelerations[2]");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 1000000000,"
        " 1000000000, 0100000000, 0100000000, 0100000000, 0100000000, 0100000000, 0100000000,h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.imu_accelerations[2] = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("charger_state");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 1000000000,"
        " 1000000000, 0100000000, 0100000000, 0100000000, 0100000000, 0100000000, 0100000000,h01,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    result.charger_state = 1;
    CompareEcu1PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("failure  \",\" ");
    std::string packet =
        "0000000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 1000000000,"
        " 1000000000, 0100000000, 0100000000, 0100000000, 0100000000, 0100000000, 0100000000,h01";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::protocol_error);
  }

  {
    SCOPED_TRACE("failure data broaken");
    std::string packet =
        "zyx0000001,00000000000001,h01,h000000000009007F,"
        "h0000000000000000000000000000000000000000000000000000000000000001, 01000, 01000, 01000,"
        " 01000, 001,h07FF,00001,h0001,001,h7F,h0000000000000001, 1000000000, 1000000000, 1000000000,"
        " 1000000000, 0100000000, 0100000000, 0100000000, 0100000000, 0100000000, 0100000000,h01,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::protocol_error);
  }
}

TEST(PowerEcuComEcu2DataDecoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComEcu2DataDecoder decoder;

  EXPECT_EQ(decoder.GetPacketSize(), 291);
  EXPECT_STREQ(decoder.GetPacketName().c_str(), "ecu2_");

  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);
  hsrb_power_ecu::PowerEcuComEcu2DataDecoder::PacketData result;
  result.ecu2_date = "00000000000000";
  result.d12V_D0_V = 0;
  result.d12V_D0_A = 0;
  result.d12V_D1_V = 0;
  result.d12V_D1_A = 0;
  result.d12V_D2_V = 0;
  result.d12V_D2_A = 0;
  result.d12V_D3_V = 0;
  result.d12V_D3_A = 0;
  result.d12V_O1_V = 0;
  result.d12V_O1_A = 0;
  result.d12V_O2_V = 0;
  result.d12V_O2_A = 0;
  result.d5VA_V = 0;
  result.d5VD1_V = 0;
  result.d5VD1_A = 0;
  result.d5VD2_V = 0;
  result.d5VD2_A = 0;
  result.d5VD3_V = 0;
  result.d5VD3_A = 0;
  result.d5VD4_V = 0;
  result.d5VD4_A = 0;
  result.d5VD5_V = 0;
  result.d5VD5_A = 0;
  result.Chgsense = 0;
  result.d2V5VDA1_V = 0;
  result.d2V5VDA2_V = 0;
  result.ACDC_V = 0;
  result.ADCD_A = 0;
  result.BATT_V = 0;
  result.BATT_A = 0;
  result.BATT_A2 = 0;
  result.PBM_V = 0;
  result.PBM_A = 0;
  result.PBM_A2 = 0;
  result.PUMP_V = 0;
  result.ECU_TEMP = 0;
  result.ECU_TEMP1 = 0;
  result.ECU_TEMP2 = 0;
  result.ECU_TEMP3 = 0;

  {
    SCOPED_TRACE("init");
    std::string packet =
        "00000000000000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("date");
    std::string packet =
        "00000000000001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.ecu2_date = "00000000000001";
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d12V_D0_V");
    std::string packet =
        "00000000000001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_D0_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d12V_D0_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_D0_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d12V_D1_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_D1_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }
  {
    SCOPED_TRACE("d12V_D1_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_D1_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d12V_D2_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_D2_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }
  {
    SCOPED_TRACE("d12V_D2_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_D2_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d12V_D3_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_D3_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }
  {
    SCOPED_TRACE("d12V_D3_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_D3_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d12V_O1_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_O1_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }
  {
    SCOPED_TRACE("d12V_O1_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_O1_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d12V_O2_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_O2_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }
  {
    SCOPED_TRACE("d12V_O2_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d12V_O2_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d5VA_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d5VA_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }
  {
    SCOPED_TRACE("d5VD1_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d5VD1_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d5VD1_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d5VD1_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d5VD2_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d5VD2_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d5VD2_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d5VD2_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d5VD3_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d5VD3_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d5VD3_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d5VD3_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d5VD4_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d5VD4_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d5VD4_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d5VD4_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d5VD5_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d5VD5_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d5VD5_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d5VD5_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("Chgsense");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.Chgsense = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d2V5VDA1_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d2V5VDA1_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("d2V5VDA2_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.d2V5VDA2_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("ACDC_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00000, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.ACDC_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("ADCD_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00000, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.ADCD_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("BATT_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00000, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.BATT_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("BATT_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00000, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.BATT_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("BATT_A2");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00000, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.BATT_A2 = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("PBM_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00000, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.PBM_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("PBM_A");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00000, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.PBM_A = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("PBM_A2");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00000, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.PBM_A2 = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("PUMP_V");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 000, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.PUMP_V = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("ECU_TEMP");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 001, 000, 000, 000,";
    write_buffer(packet, buffer);
    result.ECU_TEMP = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("ECU_TEMP1");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 001, 001, 000, 000,";
    write_buffer(packet, buffer);
    result.ECU_TEMP1 = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("ECU_TEMP2");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 001, 001, 001, 000,";
    write_buffer(packet, buffer);
    result.ECU_TEMP2 = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("ECU_TEMP3");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 001, 001, 001, 001,";
    write_buffer(packet, buffer);
    result.ECU_TEMP3 = 1;
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareEcu2PacketData(decoder, result);
  }

  {
    SCOPED_TRACE("failure \",\" ");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 001, 001, 001, 001";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::protocol_error);
  }

  {
    SCOPED_TRACE("failure data broaken ");
    std::string packet =
        "00000000000001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, 00001, "
        "00001, 00001, 00001, 00001, 00001, 00001, 00001, 001, 001,  001, 001";

    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::protocol_error);
  }
}

TEST(PowerEcuComRxackDataDecoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComRxackDataDecoder decoder;

  bool* is_receive_ack = decoder.GetParamPtr<bool>("is_receive_ack");
  uint8_t* ack_value = decoder.GetParamPtr<uint8_t>("ack_value");

  EXPECT_EQ(decoder.GetPacketSize(), 15);
  EXPECT_STREQ(decoder.GetPacketName().c_str(), "rxack");

  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  {
    SCOPED_TRACE("init");
    std::string packet = "h00,";
    write_buffer(packet, buffer);
    EXPECT_EQ(*is_receive_ack, false);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    EXPECT_EQ(*ack_value, 0);
    EXPECT_EQ(*is_receive_ack, true);
  }

  {
    SCOPED_TRACE("ack_value");
    std::string packet = "h01,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    EXPECT_EQ(*ack_value, 1);
  }

  {
    SCOPED_TRACE("failure \",\" ");
    std::string packet = "h01";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::protocol_error);
  }

  {
    SCOPED_TRACE("failure data broaken ");
    std::string packet = "h0f,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::protocol_error);
  }
}

TEST(PowerEcuComVerDataDecoderTest, NomalCase) {
  hsrb_power_ecu::PowerEcuComVerDataDecoder decoder;

  bool* is_receive_version = decoder.GetParamPtr<bool>("is_receive_version");

  EXPECT_EQ(decoder.GetPacketSize(), 95);
  EXPECT_STREQ(decoder.GetPacketName().c_str(), "ver__");

  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);

  hsrb_power_ecu::PowerEcuComVerDataDecoder::PacketData result;
  result.ver_power_ecu_version = "0000000000000000000000000000000000000000";
  result.ver_power_ecu_com_version = "0000000000000000000000000000000000000000";

  {
    SCOPED_TRACE("init");
    std::string packet = "h0000000000000000000000000000000000000000,h0000000000000000000000000000000000000000,";
    write_buffer(packet, buffer);
    EXPECT_EQ(*is_receive_version, false);
    decoder.Decode(buffer.begin(), buffer.end());
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    EXPECT_EQ(*is_receive_version, true);
    CompareVerPacketData(decoder, result);
  }

  {
    SCOPED_TRACE("power_ecu_version");
    std::string packet = "h0000000000000000000000000000000000000001,h0000000000000000000000000000000000000000,";
    write_buffer(packet, buffer);
    result.ver_power_ecu_version = "0000000000000000000000000000000000000001";
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareVerPacketData(decoder, result);
  }

  {
    SCOPED_TRACE("power_ecu_com_version");
    std::string packet = "h0000000000000000000000000000000000000001,h0000000000000000000000000000000000000001,";
    write_buffer(packet, buffer);
    result.ver_power_ecu_com_version = "0000000000000000000000000000000000000001";
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::success);
    CompareVerPacketData(decoder, result);
  }

  {
    SCOPED_TRACE("failure \",\" ");
    std::string packet = "h0000000000000000000000000000000000000001,h0000000000000000000000000000000000000001";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::protocol_error);
  }

  {
    SCOPED_TRACE("failure data broaken ");
    std::string packet = "h 0000000000000000000000000000000000000001,h0000000000000000000000000000000000000001,";
    write_buffer(packet, buffer);
    EXPECT_EQ(decoder.Decode(buffer.begin(), buffer.end()), boost::system::errc::protocol_error);
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
