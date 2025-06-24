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
#include <hsrb_servomotor_protocol/exxx_warning_category.hpp>

#include <sstream>
#include <string>

#include "message_builder.hpp"

namespace {

const char* const kExxxWarningNamespace = "ExxxWarning";

}  // namespace

namespace hsrb_servomotor_protocol {

namespace exxx_warning_code {

/// @brief Exxxサーボアンプのエラーカテゴリ
/// @c error code はステータスパケットの @c error status
/// @c error condition は ExxxWarningCondition
class ExxxWarningCategoryImpl : public boost::system::error_category {
 public:
  ExxxWarningCategoryImpl() {}

#if (defined __GNUC__ && __GNUC__ >= 7)
  const char* name() const noexcept override { return "ExxxWarning"; }
#else
  virtual const char* name() const { return "ExxxWarning"; }
#endif

  /// @param [in] ev An error code in this category.
  virtual std::string message(int ev) const {
    if (0 < ev && ev <= kExxxWarningCodeForceMinimumSize) {
      MessageBuilder builder(kExxxWarningNamespace);
      // See doc/error_message.md for detail
      if (ev & kExxxWarningCodeOtherDriveMode) {
        builder.Append("OhterDriveMode", "Invalid drive mode.");
      }
      if (ev & kExxxWarningCodeHandRestriction) {
        builder.Append("HandRestriction", "Hand effort reference out of range.");
      }
      if (ev & kExxxWarningCodeAvago2MrFail) {
        builder.Append("Avago2Mr", "Motor position update failure.");
      }
      if (ev & kExxxWarningCodeOutOfAddressRange) {
        builder.Append("AddressRange", "Invalid address on control table.");
      }
      if (ev & kExxxWarningCodeInvalidInstruction) {
        builder.Append("Inst", "Invalid instruction.");
      }
      if (ev & kExxxWarningCodeCheckSumError) {
        builder.Append("CheckSum", "Checksum error.");
      }
      if (ev & kExxxWarningCodePositionVelocityRestriction) {
        builder.Append("PositionVelocityRestriction", "Velocity reference out of range.");
      }
      if (ev & kExxxWarningCodeElectricalOverload) {
        builder.Append("ElectricalOverload", "Electrical overload.");
      }
      if (ev & kExxxWarningCodePositionRestriction) {
        builder.Append("PositionRestriction", "Position reference out of range.");
      }
      if (ev & kExxxWarningCodeDutyRestriction) {
        builder.Append("DutyRestriction", "Voltage reference out of range.");
      }
      if (ev & kExxxWarningCodeDutyDown) {
        builder.Append("DutyDown", "Voltage command interrupted.");
      }
      if (ev & kExxxWarningCodeCurrentRestriction) {
        builder.Append("CurrentRestriction", "Current reference out of range.");
      }
      if (ev & kExxxWarningCodeCurrentDown) {
        builder.Append("CurrentDown", "Current command interrupted.");
      }
      if (ev & kExxxWarningCodeVelocityRestriction) {
        builder.Append("VelocityRestriction", "Velocity reference out of range.");
      }
      if (ev & kExxxWarningCodeVelocityDown) {
        builder.Append("VelocityDown", "Velocity command interrupted.");
      }
      if (ev & kExxxWarningCodeAlarmStatus) {
        builder.Append("AlarmStatus", "Error occurred.");
      }
      return builder.Build();
    } else {
      return "Invalid error status.";
    }
  }
};

}  // namespace exxx_warning_code

const boost::system::error_category& ExxxWarningCategory() {
  static exxx_warning_code::ExxxWarningCategoryImpl s_exxx_warning_category_instance;
  return s_exxx_warning_category_instance;
}

}  // namespace hsrb_servomotor_protocol
