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
#ifndef TMC_UTILS_QOS_HPP_
#define TMC_UTILS_QOS_HPP_

#include <rclcpp/qos.hpp>

namespace tmc_utils {

// Actually, it's hard to imagine TransientLocal at Bestefort, so it's hard to imagine.
class BestEffortQoS : public rclcpp::QoS {
 public:
  BestEffortQoS() : BestEffortQoS(1) {}

  explicit BestEffortQoS(size_t history_depth) : QoS(history_depth) {
    best_effort();
    durability_volatile();
  }
};

// Used for either Pub/Sub
class ReliableVolatileQoS : public rclcpp::QoS {
 public:
  ReliableVolatileQoS() : ReliableVolatileQoS(1) {}

  explicit ReliableVolatileQoS(size_t history_depth) : QoS(history_depth) {
    reliable();
    durability_volatile();
  }
};

// Equivalent to ROS1 Latched Publisher
class ReliableTransientLocalQoS : public rclcpp::QoS {
 public:
  ReliableTransientLocalQoS() : ReliableTransientLocalQoS(1) {}

  explicit ReliableTransientLocalQoS(size_t history_depth) : QoS(history_depth) {
    reliable();
    transient_local();
  }
};

}  // namespace tmc_utils
#endif  // TMC_UTILS_QOS_HPP_

