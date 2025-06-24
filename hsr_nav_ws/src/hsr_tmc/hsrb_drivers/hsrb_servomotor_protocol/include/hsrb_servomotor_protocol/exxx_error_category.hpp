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
#ifndef HSRB_SERVOMOTOR_PROTOCOL_EXXX_ERROR_CATEGORY_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_EXXX_ERROR_CATEGORY_HPP_

#include <boost/system/error_code.hpp>

namespace hsrb_servomotor_protocol {

const boost::system::error_category& ExxxErrorCategory();
const boost::system::error_category& ExxxSafetyErrorCategory();

/// 独自のerror_codeを定義
namespace exxx_error_code {


/// bitフィールドで立っているフラグを表す
/// enumはとびとびの値だが，複合的な値をとる
/// 0bit: OverTemperature
/// 1bit: VoltageError
/// 2bit: OverPosition
/// 3bit: OverCurrent
/// 4bit: OverLoad
/// 5bit: OverControlDeviation
/// 6bit: OverReferenceDeviation
/// 7bit: CurrentSensorFailure
/// 8bit: Jumping
/// 9bit: HallSensorFailure
/// 10bit: FailWriteEEPROM
/// 11bit: LogicError
/// 12bit: CommunicationError
/// 13bit: JointEncorderError
/// 14bit: AvagoSensorError
/// 15bit: FunctionalSafetyError
enum ExxxErrorCode {
  kExxxErrorCodeOverTermpreature = 0x0001,
  kExxxErrorCodeVoltageError = 0x0002,
  kExxxErrorCodeOverPosition = 0x0004,
  kExxxErrorCodeOverCurrent = 0x0008,
  kExxxErrorCodeOverLoad = 0x0010,
  kExxxErrorCodeOverControlDeviation = 0x0020,
  kExxxErrorCodeOverReferenceDeviation = 0x0040,
  kExxxErrorCodeCurrentSensorFailure = 0x0080,
  kExxxErrorCodeJumping = 0x0100,
  kExxxErrorCodeHallSensorFailure = 0x0200,
  kExxxErrorCodeFailWriteEEPROM = 0x0400,
  kExxxErrorCodeLogicError = 0x0800,
  kExxxErrorCodeCommunicationError = 0x1000,
  kExxxErrorCodeJointEncorderError = 0x2000,
  kExxxErrorCodeAvagoSensorError = 0x4000,
  kExxxErrorCodeFunctionalSafetyError = 0x8000,
  kExxxErrorCodeForceMinimumSize = 0xFFFF
};


/// bitフィールドで立っているフラグを表す
/// enumはとびとびの値だが，複合的な値をとる
/// 0bit: 未使用
/// 1bit: PowerSupplyMOSFETShortError
/// 2bit: PowerSupplyMOSFETOpenError
/// 3bit: 未使用
/// 4bit: WatchdogSlowMonitoringFunction
/// 5bit: WatchdogFastMonitoringFunction
/// 6bit: ROMCheckError
/// 7bit: SequenceMonitorError
/// 8bit: InverterPowerSupplyPBMError
/// 9bit: InverterPowerSupply12VError
/// 10bit: ThreePhaseVoltageAdditionError
/// 11bit: ResolverSensorOffsetError
/// 12bit: PositionSensorError
/// 13bit: ResolverSensorOutOfRangeError
/// 14bit: ThreePhaseCurrentAdditionError
/// 15bit: TwoPhaseVoltageAdditionError
/// 16bit: ResolverSensorModelError
/// 17bit: DCPositionSensorError
/// 18bit: MotorShortCircuitError
enum ExxxSafetyErrorCode {
  kExxxSafetyErrorCodePowerSupplyMOSFETShortError = 0x0002,
  kExxxSafetyErrorCodePowerSupplyMOSFETOpenError = 0x0004,
  kExxxSafetyErrorCodeWatchdogSlowMonitoringFunction = 0x0010,
  kExxxSafetyErrorCodeWatchdogFastMonitoringFunction = 0x0020,
  kExxxSafetyErrorCodeROMCheckError = 0x0040,
  kExxxSafetyErrorCodeSequenceMonitorError = 0x0080,
  kExxxSafetyErrorCodeInverterPowerSupplyPBMError = 0x0100,
  kExxxSafetyErrorCodeInverterPowerSupply12VError = 0x0200,
  kExxxSafetyErrorCodeThreePhaseVoltageAdditionError = 0x0400,
  kExxxSafetyErrorCodeResolverSensorOffsetError = 0x0800,
  kExxxSafetyErrorCodePositionSensorError = 0x1000,
  kExxxSafetyErrorCodeResolverSensorOutOfRangeError = 0x2000,
  kExxxSafetyErrorCodeThreePhaseCurrentAdditionError = 0x4000,
  kExxxSafetyErrorCodeTwoPhaseVoltageAdditionError = 0x8000,
  kExxxSafetyErrorCodeResolverSensorModelError = 0x10000,
  kExxxSafetyErrorCodeDCPositionSensorError = 0x20000,
  kExxxSafetyErrorCodeMotorShortCircuitError = 0x40000,
  kExxxSafetyErrorCodeForceMinimumSize = 0x7FFFF
};

}  // namespace exxx_error_code

}  // namespace hsrb_servomotor_protocol


namespace boost {
namespace system {

template <>
struct is_error_code_enum<hsrb_servomotor_protocol::exxx_error_code::ExxxErrorCode> {
  static const bool value = true;
};

template <>
struct is_error_code_enum<hsrb_servomotor_protocol::exxx_error_code::ExxxSafetyErrorCode> {
  static const bool value = true;
};

}  // namespace system
}  // namespace boost

inline boost::system::error_code make_error_code(hsrb_servomotor_protocol::exxx_error_code::ExxxErrorCode e) {
  return boost::system::error_code(e, hsrb_servomotor_protocol::ExxxErrorCategory());
}

inline boost::system::error_code make_safety_error_code(
         hsrb_servomotor_protocol::exxx_error_code::ExxxSafetyErrorCode e) {
  return boost::system::error_code(e, hsrb_servomotor_protocol::ExxxSafetyErrorCategory());
}

/// Construct an error_condition from value
inline boost::system::error_condition make_error_condition(hsrb_servomotor_protocol::exxx_error_code::ExxxErrorCode e) {
  return boost::system::error_condition(e, hsrb_servomotor_protocol::ExxxErrorCategory());
}

inline boost::system::error_condition make_safety_error_condition(
         hsrb_servomotor_protocol::exxx_error_code::ExxxSafetyErrorCode e) {
  return boost::system::error_condition(e, hsrb_servomotor_protocol::ExxxSafetyErrorCategory());
}


#endif /*HSRB_SERVOMOTOR_PROTOCOL_EXXX_ERROR_CATEGORY_HPP_*/
