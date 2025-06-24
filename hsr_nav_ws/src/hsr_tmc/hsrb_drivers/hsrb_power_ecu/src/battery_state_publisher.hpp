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
#ifndef HSRB_POWER_ECU_BATTERY_STATE_PUBLISHER_HPP_
#define HSRB_POWER_ECU_BATTERY_STATE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "power_ecu_protocol.hpp"
#include "ros2_msg_utils.hpp"
#include "state_publisher_base.hpp"

namespace hsrb_power_ecu {

class BatteryStatePublisher
    : public StatePublisherBase<sensor_msgs::msg::BatteryState> {
 public:
  BatteryStatePublisher(const rclcpp::Node::SharedPtr& node,
                        const PowerEcuProtocol::Ptr protocol);
  virtual ~BatteryStatePublisher() = default;

 protected:
  sensor_msgs::msg::BatteryState CreateMessage() override;

 private:
  rclcpp::Clock::SharedPtr clock_;
  double* battery_relative_capacity_;
  double* battery_temperature_;
  double* battery_voltage_;
  double* electric_current_;
  double* battery_remaining_capacity_;
  double* battery_total_capacity_;
};

}  // namespace hsrb_power_ecu

#endif  // HSRB_POWER_ECU_BATTERY_STATE_PUBLISHER_HPP_
