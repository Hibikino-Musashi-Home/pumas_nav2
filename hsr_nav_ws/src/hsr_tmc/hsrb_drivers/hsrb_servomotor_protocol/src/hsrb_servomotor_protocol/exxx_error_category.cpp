/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
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
#include <hsrb_servomotor_protocol/exxx_error_category.hpp>

#include <sstream>
#include <string>

#include "message_builder.hpp"

namespace {

const char* const kExxxErrorNamespace = "ExxxError";
const char* const kExxxSafetyErrorNamespace = "ExxxSafetyError";

}  // namespace

namespace hsrb_servomotor_protocol {

namespace exxx_error_code {

/// @brief Exxxサーボアンプのワーニングカテゴリ
/// @c error code はステータスパケットの @c error status
/// @c error condition は ExxxErrorCondition
class ExxxErrorCategoryImpl : public boost::system::error_category {
 public:
  ExxxErrorCategoryImpl() {}

#if (defined __GNUC__ && __GNUC__ >= 7)
  const char* name() const noexcept override { return "ExxxError"; }
#else
  virtual const char* name() const { return "ExxxError"; }
#endif

  /// @param [in] ev An error code in this category.
  virtual std::string message(int ev) const {
    if (0 < ev && ev <= kExxxErrorCodeForceMinimumSize) {
      MessageBuilder builder(kExxxErrorNamespace);
      // See doc/error_message.md for detail
      if (ev & kExxxErrorCodeAvagoSensorError) {
        builder.Append("Avago", "Avago sensor trouble.");
      }
      if (ev & kExxxErrorCodeJointEncorderError) {
        builder.Append("Encoder", "Unexpected encorder value.");
      }
      if (ev & kExxxErrorCodeCommunicationError) {
        builder.Append("Communication", "Invalid packet size.");
      }
      if (ev & kExxxErrorCodeLogicError) {
        builder.Append("Logic", "Abnormal state in motor driver.");
      }
      if (ev & kExxxErrorCodeFailWriteEEPROM) {
        builder.Append("Eeprom", "EEPROM failure.");
      }
      if (ev & kExxxErrorCodeHallSensorFailure) {
        builder.Append("HallSensor", "Hall sensor failure.");
      }
      if (ev & kExxxErrorCodeJumping) {
        builder.Append("Jumping", "Timing belt jumped.");
      }
      if (ev & kExxxErrorCodeCurrentSensorFailure) {
        builder.Append("CurrentSensor", "Current sensor failure.");
      }
      if (ev & kExxxErrorCodeOverReferenceDeviation) {
        builder.Append("ReferenceDeviation", "Position error too large.");
      }
      if (ev & kExxxErrorCodeOverControlDeviation) {
        builder.Append("ControlDeviation", "Velocity integration error too large.");
      }
      if (ev & kExxxErrorCodeOverLoad) {
        builder.Append("CoreTemp", "Motor over temperature.");
      }
      if (ev & kExxxErrorCodeOverCurrent) {
        builder.Append("Current", "Over current.");
      }
      if (ev & kExxxErrorCodeOverPosition) {
        builder.Append("OverPosition", "Abnormal position.");
      }
      if (ev & kExxxErrorCodeVoltageError) {
        builder.Append("Voltage", "Supplied voltage out of range.");
      }
      if (ev & kExxxErrorCodeOverTermpreature) {
        builder.Append("Temperature", "Board over temperature.");
      }
      if (ev & kExxxErrorCodeFunctionalSafetyError) {
        builder.Append("FunctionalSafety", "Functional safety error.");
      }
      return builder.Build();
    } else {
      return "Invalid error status.";
    }
  }
};

/// @brief Exxxサーボアンプのワーニングカテゴリ
/// @c error code はステータスパケットの @c safety error status
/// @c error condition は ExxxErrorCondition
class ExxxSafetyErrorCategoryImpl : public boost::system::error_category {
 public:
  ExxxSafetyErrorCategoryImpl() {}

#if (defined __GNUC__ && __GNUC__ >= 7)
  const char* name() const noexcept override { return "ExxxSafetyError"; }
#else
  virtual const char* name() const { return "ExxxSafetyError"; }
#endif

  /// @param [in] ev An error code in this category.
  virtual std::string message(int ev) const {
    if (0 < ev && ev <= kExxxSafetyErrorCodeForceMinimumSize) {
      MessageBuilder builder(kExxxSafetyErrorNamespace);
      // See doc/error_message.md for detail
      if (ev & kExxxSafetyErrorCodePowerSupplyMOSFETShortError) {
        builder.Append("MOSFET", "Short error.");
      }
      if (ev & kExxxSafetyErrorCodePowerSupplyMOSFETOpenError) {
        builder.Append("MOSFET", "Open error.");
      }
      if (ev & kExxxSafetyErrorCodeWatchdogSlowMonitoringFunction) {
        builder.Append("Watchdog", "Slow monitoring error.");
      }
      if (ev & kExxxSafetyErrorCodeWatchdogFastMonitoringFunction) {
        builder.Append("Watchdog", "Fast monitoring error.");
      }
      if (ev & kExxxSafetyErrorCodeROMCheckError) {
        builder.Append("ROM", "Check error.");
      }
      if (ev & kExxxSafetyErrorCodeSequenceMonitorError) {
        builder.Append("Monitor", "Sequence error.");
      }
      if (ev & kExxxSafetyErrorCodeInverterPowerSupplyPBMError) {
        builder.Append("Inverter", "PBM supply error.");
      }
      if (ev & kExxxSafetyErrorCodeInverterPowerSupply12VError) {
        builder.Append("Inverter", "12V Power supply error.");
      }
      if (ev & kExxxSafetyErrorCodeThreePhaseVoltageAdditionError) {
        builder.Append("Inverter", "Three phase voltage addition error.");
      }
      if (ev & kExxxSafetyErrorCodeResolverSensorOffsetError) {
        builder.Append("ResolverSensor", "[BLDC]Unexpected sensor value.");
      }
      if (ev & kExxxSafetyErrorCodePositionSensorError) {
        builder.Append("PositionSensor", "[BLDC]Two sensor value difference.");
      }
      if (ev & kExxxSafetyErrorCodeResolverSensorOutOfRangeError) {
        builder.Append("ResolverSensor", "Out of range.");
      }
      if (ev & kExxxSafetyErrorCodeThreePhaseCurrentAdditionError) {
        builder.Append("Current", "Three phase current addition error.");
      }
      if (ev & kExxxSafetyErrorCodeTwoPhaseVoltageAdditionError) {
        builder.Append("Inverter", "Two phase voltage addition error.");
      }
      if (ev & kExxxSafetyErrorCodeResolverSensorModelError) {
        builder.Append("ResolverSensor", "[DC]Unexpected sensor value.");
      }
      if (ev & kExxxSafetyErrorCodeDCPositionSensorError) {
        builder.Append("PositionSensor", "[DC]Two sensor value difference.");
      }
      if (ev & kExxxSafetyErrorCodeMotorShortCircuitError) {
        builder.Append("Motor", "Short circuit error.");
      }
      return builder.Build();
    } else {
      return "Invalid safety error status.";
    }
  }
};

}  // namespace exxx_error_code

const boost::system::error_category& ExxxErrorCategory() {
  static exxx_error_code::ExxxErrorCategoryImpl s_exxx_error_category_instance;
  return s_exxx_error_category_instance;
}

const boost::system::error_category& ExxxSafetyErrorCategory() {
  static exxx_error_code::ExxxSafetyErrorCategoryImpl s_exxx_safety_error_category_instance;
  return s_exxx_safety_error_category_instance;
}

}  // namespace hsrb_servomotor_protocol
