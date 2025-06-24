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
#ifndef HSRB_POWER_ECU_STATE_PUBLISHER_BASE_HPP_
#define HSRB_POWER_ECU_STATE_PUBLISHER_BASE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "get_parameter.hpp"
#include "power_ecu_protocol.hpp"

namespace hsrb_power_ecu {

class IStatePublisher {
 public:
  virtual ~IStatePublisher() = default;
  virtual void Publish() = 0;
};

template<typename MSG>
class StatePublisherBase : public IStatePublisher {
 public:
  StatePublisherBase(const rclcpp::Node::SharedPtr& node,
                     const std::string& topic_name) : publish_period_(0, 0) {
    clock_ = node->get_clock();
    publisher_ = node->create_publisher<MSG>(topic_name, 10);

    double publish_rate = GetPositiveParameter<double>(node, topic_name + "/publish_rate", 1.0);
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);
    next_publish_time_ = clock_->now() + publish_period_;
  }
  virtual ~StatePublisherBase() = default;

  void Publish() final {
    rclcpp::Time current_time = clock_->now();
    if (current_time < next_publish_time_) {
      return;
    }

    MSG message = CreateMessage();
    publisher_->publish(message);

    next_publish_time_ = clock_->now() + publish_period_;
  }

 protected:
  virtual MSG CreateMessage() = 0;

 private:
  rclcpp::Clock::SharedPtr clock_;
  typename rclcpp::Publisher<MSG>::SharedPtr publisher_;
  rclcpp::Duration publish_period_;
  rclcpp::Time next_publish_time_;
};

}  // namespace hsrb_power_ecu

#endif  // HSRB_POWER_ECU_STATE_PUBLISHER_BASE_HPP_
