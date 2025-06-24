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
#ifndef HSRB_SERVOMOTOR_PROTOCOL_EXXX_COMMON_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_EXXX_COMMON_HPP_

#include <cstdio>
#include <time.h>
#include <boost/cstdint.hpp>

namespace hsrb_servomotor_protocol {

inline int64_t Now() {
  timespec t = { 0 };
  if (clock_gettime(CLOCK_MONOTONIC, &t)) {
    perror("clock_gettime");
    exit(EXIT_FAILURE);
  }
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL + static_cast<int64_t>(t.tv_nsec);
}


enum Instruction {
  kInstructionPing = 0x01,
  kInstructionReadData = 0x02,
  kInstructionWriteData = 0x03,
  kInstructionRegWrite = 0x04,
  kInstructionAction = 0x05,
  kInstructionReset = 0x10,
  kInstructionWriteEeprom = 0x09,
  kInstructionAvagoAvePos = 0x0a,
  kInstructionReadDate = 0x0b,
  kInstructionReadHash = 0x0c,
  kInstructionSyncParam = 0x0d,
  kInstructionSyncWrite = 0x83,
  kInstructionSyncRegWrite = 0x85,
};


enum DriveMode {
  kDriveModeNoControl = 0,
  kDriveModeVoltage = 1,
  kDriveModeCurrent = 2,
  kDriveModeVelocity = 3,
  kDriveModePosition = 4,
  kDriveModeActPositionAndActVelocity = 5,
  kDriveModeJntVelocity = 6,
  kDriveModeJntPositionAndJntVelocity = 7,
  kDriveModeJntPositionAndActVelocity = 8,
  kDriveModeImpedance = 9,
  kDriveModeHandGrasp = 20,
  kDriveModeHandPosition = 21,
  kDriveModeHandSE = 22
};

const uint8_t kByteDriveMode = 1;
const uint8_t kHeader1 = 0xAAU;
const uint8_t kHeader2 = 0x55U;
const uint8_t kFooter1 = 0x00U;
const uint8_t kFooter2 = 0x00U;
static const uint8_t kBroadcastID = 0xFEU;

}  // namespace hsrb_servomotor_protocol

#endif /*HSRB_SERVOMOTOR_PROTOCOL_EXXX_COMMON_HPP_*/
