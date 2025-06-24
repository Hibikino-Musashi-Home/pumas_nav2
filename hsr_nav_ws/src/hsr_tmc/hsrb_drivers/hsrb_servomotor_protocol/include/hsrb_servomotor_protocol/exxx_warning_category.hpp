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
#ifndef HSRB_SERVOMOTOR_PROTOCOL_EXXX_WARNING_CATEGORY_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_EXXX_WARNING_CATEGORY_HPP_

#include <boost/system/error_code.hpp>

namespace hsrb_servomotor_protocol {

const boost::system::error_category& ExxxWarningCategory();

/// 独自のwarning_codeを定義
namespace exxx_warning_code {

/// bitフィールドで立っているフラグを表す
/// 0bit: alarm_status
/// 1bit: VelocityDown
/// 2bit: VelocityRestriction
/// 3bit: CurrentDown
/// 4bit: CurrentRestriction
/// 5bit: DutyDown
/// 6bit: DutyRestriction
/// 7bit: PositionRestriction
/// 8bit: ElectricalOverload
/// 9bit: PositionVelocityRestriction
/// 10bit: CheckSumError
/// 11bit: InvalidInstruction
/// 12bit: OutOfAddressRange
/// 13bit: Avago2MrFail
enum ExxxWarningCode {
  kExxxWarningCodeAlarmStatus = 0x0001,
  kExxxWarningCodeVelocityDown = 0x0002,
  kExxxWarningCodeVelocityRestriction = 0x0004,
  kExxxWarningCodeCurrentDown = 0x0008,
  kExxxWarningCodeCurrentRestriction = 0x0010,
  kExxxWarningCodeDutyDown = 0x0020,
  kExxxWarningCodeDutyRestriction = 0x0040,
  kExxxWarningCodePositionRestriction = 0x0080,
  kExxxWarningCodeElectricalOverload = 0x0100,
  kExxxWarningCodePositionVelocityRestriction = 0x0200,
  kExxxWarningCodeCheckSumError = 0x0400,
  kExxxWarningCodeInvalidInstruction = 0x0800,
  kExxxWarningCodeOutOfAddressRange = 0x1000,
  kExxxWarningCodeAvago2MrFail = 0x2000,
  kExxxWarningCodeHandRestriction = 0x4000,
  kExxxWarningCodeOtherDriveMode = 0x8000,
  kExxxWarningCodeForceMinimumSize = 0xFFFF
};

}  // namespace exxx_warning_code

}  // namespace hsrb_servomotor_protocol


namespace boost {
namespace system {

template <>
struct is_error_code_enum<hsrb_servomotor_protocol::exxx_warning_code::ExxxWarningCode> {
  static const bool value = true;
};

}  // namespace system
}  // namespace boost

inline boost::system::error_code make_warning_code(hsrb_servomotor_protocol::exxx_warning_code::ExxxWarningCode e) {
  return boost::system::error_code(e, hsrb_servomotor_protocol::ExxxWarningCategory());
}

/// Construct an warning_condition from value
inline boost::system::error_condition make_warning_condition(
    hsrb_servomotor_protocol::exxx_warning_code::ExxxWarningCode e) {
  return boost::system::error_condition(e, hsrb_servomotor_protocol::ExxxWarningCategory());
}


#endif /*HSRB_SERVOMOTOR_PROTOCOL_EXXX_WARNING_CATEGORY_HPP_*/
