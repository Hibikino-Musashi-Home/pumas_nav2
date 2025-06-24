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

#include "battery_state_publisher.hpp"

namespace hsrb_power_ecu {

BatteryStatePublisher::BatteryStatePublisher(const rclcpp::Node::SharedPtr& node,
                                             const PowerEcuProtocol::Ptr protocol)
    : StatePublisherBase(node, "battery_state"), clock_(node->get_clock()) {
  battery_relative_capacity_ = ExistParamPtr<double>(protocol, "battery_relative_capacity");
  battery_temperature_ = ExistParamPtr<double>(protocol, "battery_temperature");
  battery_voltage_ = ExistParamPtr<double>(protocol, "battery_voltage");
  electric_current_ = ExistParamPtr<double>(protocol, "electric_current");
  battery_remaining_capacity_ = ExistParamPtr<double>(protocol, "battery_remaining_capacity");
  battery_total_capacity_ = ExistParamPtr<double>(protocol, "battery_total_capacity");
}

sensor_msgs::msg::BatteryState BatteryStatePublisher::CreateMessage() {
  sensor_msgs::msg::BatteryState message;
  message.percentage = *battery_relative_capacity_ / 100.0;
  message.temperature = *battery_temperature_;
  message.voltage = *battery_voltage_;
  message.current = *electric_current_;
  message.charge = *battery_remaining_capacity_;
  message.capacity = *battery_total_capacity_;
  message.header.stamp = clock_->now();
  return message;
}

}  // namespace hsrb_power_ecu
