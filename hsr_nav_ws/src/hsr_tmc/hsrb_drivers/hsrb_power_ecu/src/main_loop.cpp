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
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <hsrb_power_ecu/i_network.hpp>
#include <hsrb_power_ecu/serial_network.hpp>
#include "battery_state_publisher.hpp"
#include "bool_state_publisher.hpp"
#include "get_parameter.hpp"
#include "imu_state_publisher.hpp"
#include "led_command_subscriber.hpp"
#include "power_ecu_protocol.hpp"
#include "runstop_state_publisher.hpp"

namespace {
const char* const kDefaultPortName = "/dev/ttyCTI3";
const double kPortReceiveTimeoutMs = 0.5;
const char* const kDefaultImuFrame = "base_imu_frame";

// ECU parameter name and topic name issued
const std::vector<std::pair<std::string, std::string>> kParamAndTopicNames {
  {"is_bumper_bumper1", "base_f_bumper_sensor"}, {"is_bumper_bumper2", "base_b_bumper_sensor"},
  {"is_powerecu_sw_kinoko", "emergency_stop_button"}, {"is_powerecu_sw_w_stop", "wireless_stop_button"},
  {"is_powerecu_sw_w_sel", "wireless_stop_enable"}, {"is_powerecu_bat_stat", "battery_charging"}
};
}  // anonymous namespace


int32_t main(int32_t argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("power_ecu_main_loop");

  const std::string port_name = hsrb_power_ecu::GetParameter(node, "port_name", std::string(&(*kDefaultPortName)));
  const double port_receive_timeout = hsrb_power_ecu::GetParameter(
    node, "port_receive_timeout_ms", kPortReceiveTimeoutMs);
  const std::string imu_frame = hsrb_power_ecu::GetParameter(node, "imu_frame", std::string(&(*kDefaultImuFrame)));

  // TODO(Masayuki Masuda): 以前のhw-implにあった色々から，当座必要なものだけを用意
  const auto network = boost::make_shared<hsrb_power_ecu::SerialNetwork>();
  network->Configure("device_name", port_name);
  network->Configure("receive_timeout_ms", port_receive_timeout);
  auto protocol = boost::make_shared<hsrb_power_ecu::PowerEcuProtocol>(network, node);

  if (!protocol->Open()) {
    RCLCPP_FATAL(node->get_logger(), "Protocol Open failed.");
    exit(EXIT_FAILURE);
  }
  if (!protocol->Init()) {
    RCLCPP_FATAL(node->get_logger(), "Protocol Init Failed");
    exit(EXIT_FAILURE);
  }
  if (protocol->Start() != boost::system::errc::success) {
    RCLCPP_FATAL(node->get_logger(), "start failed");
    exit(EXIT_FAILURE);
  }

  auto led_command_subscriber = std::make_shared<hsrb_power_ecu::LedCommandSubscriber>(node, protocol);

  std::vector<std::shared_ptr<hsrb_power_ecu::IStatePublisher>> publishers;
  publishers.push_back(std::make_shared<hsrb_power_ecu::BatteryStatePublisher>(node, protocol));
  publishers.push_back(std::make_shared<hsrb_power_ecu::RunstopStatePublisher>(node, protocol));
  publishers.push_back(std::make_shared<hsrb_power_ecu::ImuStatePublisher>(node, protocol, imu_frame));

  for (const auto& name : kParamAndTopicNames) {
    publishers.push_back(std::make_shared<hsrb_power_ecu::BoolStatePublisher>(
                         node, protocol, name.first, name.second));
  }

  rclcpp::WallRate loop_rate(100.0);  // Hz

  while (rclcpp::ok()) {
    protocol->ReceiveAll();
    protocol->SendAll();
    for (auto& publisher : publishers) {
      publisher->Publish();
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  if (protocol->Stop() != boost::system::errc::success) {
    RCLCPP_FATAL(node->get_logger(), "stop failed");
    exit(EXIT_FAILURE);
  }

  protocol->Close();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
