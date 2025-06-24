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
#include <iostream>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <hsrb_power_ecu/i_network.hpp>
#include <hsrb_power_ecu/serial_network.hpp>
#include "bool_state_publisher.hpp"
#include "power_ecu_protocol.hpp"

int32_t main(int32_t argc, char** argv) {
  // Command line analysis
  std::string port_name;

  {
    bool can_run = true;
    bool is_display_help = false;

    if (argc == 2) {
      // When there is one option
      if (std::string(argv[1]) == "--help") {
        // -Help does not return abnormalities when specified
        can_run = false;
        is_display_help = true;
      } else {
        port_name = argv[1];
      }
    } else {
      // There is no command line argument or two or more errors
      can_run = false;
    }

    if (!can_run) {
      std::cout << std::endl;
      std::cout << "This program check bool_state_publisher of power ecu." << std::endl << std::endl;
      std::cout << "Usage: ros2 run hsrb_power_ecu bools /dev/tty***" << std::endl << std::endl;
      if (is_display_help) {
        return 0;
      } else {
        return 1;
      }
    }
  }

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("power_ecu_bool_state_publisher_loop");

  const auto network = boost::make_shared<hsrb_power_ecu::SerialNetwork>();
  network->Configure("device_name", port_name);
  network->Configure("receive_timeout_ms", 1);
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

  std::vector<std::pair<std::string, std::string>> param_and_topic_names {
    {"is_bumper_bumper1", "base_f_bumper_sensor"}, {"is_bumper_bumper2", "base_b_bumper_sensor"},
    {"is_powerecu_sw_kinoko", "emergency_stop_button"}, {"is_powerecu_sw_w_stop", "wireless_stop_button"},
    {"is_powerecu_sw_w_sel", "wireless_stop_enable"}, {"is_powerecu_bat_stat", "battery_charging"}
  };

  std::vector<std::shared_ptr<hsrb_power_ecu::IStatePublisher>> publishers;
  for (const auto& name : param_and_topic_names) {
    publishers.push_back(std::make_shared<hsrb_power_ecu::BoolStatePublisher>(
                         node, protocol, name.first, name.second));
  }
  rclcpp::WallRate loop_rate(100.0);

  RCLCPP_INFO(node->get_logger(), "loop start.");
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
