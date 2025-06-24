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
/**
 * @file util_function.hpp
 * Provide functions that perform well in the @brew test
 *
 *
 */
#ifndef TEST_UTILS_UTIL_FUNCTION_HPP_
#define TEST_UTILS_UTIL_FUNCTION_HPP_
#include <functional>
#include <limits>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace test_utils {

using WaitFunctionType = std::function<bool()>;

/**
 * @BRIEF Wait until some conditions are achieved
 *
 * @Param Condition_function conditional function
 * @param timeout_sec maximum standby time (SEC)
 * @Param Rate_hz Confirmation Calp (Hz) Default 100.0 (Hz)
 *
 * @Return Conditions to achieve or not to reach
 */
bool WaitUntil(rclcpp::Node::SharedPtr node, WaitFunctionType condition_function, double timeout_sec,
               double rate_hz = 100.0) {
  // An argument error check
  if (!condition_function) {
    throw std::invalid_argument("Function for waiting is empty.");
  }
  if (timeout_sec < 0.0) {
    throw std::invalid_argument("Timeout must must have fully value");
  }
  if (rate_hz < std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("Rate to validate must have fully value");
  }
  const rclcpp::Time start_time = rclcpp::Clock{ RCL_ROS_TIME }.now();
  rclcpp::Rate rate(rate_hz);
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    if (condition_function()) return true;

    if ((rclcpp::Clock{ RCL_ROS_TIME }.now() - start_time) >= timeout) break;

    rate.sleep();
  }
  return false;
}

bool WaitForTopicExistence(rclcpp::Node::SharedPtr node, const std::string& topic_name, std::chrono::seconds timeout) {
  auto start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    // Get a service list
    auto topic_names_and_types = node->get_topic_names_and_types();

    // Check if there is a service
    for (const auto& topic : topic_names_and_types) {
      if (topic.first == ("/" + topic_name)) {
        return true;
      }
    }

    // Check the timeout
    if (std::chrono::steady_clock::now() - start_time > timeout) {
      return false;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

// Wait until timeout
bool WaitUntilTimuout(rclcpp::Node::SharedPtr node, double timeout_sec = 5.0) {
  const rclcpp::Time start_time = rclcpp::Clock{ RCL_ROS_TIME }.now();
  rclcpp::Rate rate(100.0);
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);
  while (rclcpp::Clock{ RCL_ROS_TIME }.now() - start_time < timeout) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  return true;
}

}  // end of namespace test_utils
#endif  // TEST_UTILS_UTIL_FUNCTION_HPP_
