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
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <hsrb_power_ecu/i_network.hpp>
#include <hsrb_power_ecu/serial_network.hpp>

#include "../src/battery_state_publisher.hpp"
#include "../src/bool_state_publisher.hpp"
#include "../src/get_parameter.hpp"
#include "../src/led_command_subscriber.hpp"
#include "../src/power_ecu_protocol.hpp"
#include "../src/runstop_state_publisher.hpp"

#include "network_mock.hpp"


namespace {
// ECU parameter name and topic name issued
const std::vector<std::pair<std::string, std::string>> kParamAndTopicNames {
  {"is_bumper_bumper1", "base_f_bumper_sensor"}, {"is_bumper_bumper2", "base_b_bumper_sensor"},
  {"is_powerecu_sw_kinoko", "emergency_stop_button"}, {"is_powerecu_sw_w_stop", "wireless_stop_button"},
  {"is_powerecu_sw_w_sel", "wireless_stop_enable"}, {"is_powerecu_bat_stat", "battery_charging"}
};

const rclcpp::Duration kLoopTimeout = rclcpp::Duration::from_seconds(1.0);

class IPublishChecker {
 public:
  IPublishChecker() : is_update_(false) {}
  virtual ~IPublishChecker() = default;
  void Reset() {is_update_ = false;}
  bool CheckUpdated() const {return is_update_;}
 protected:
  bool is_update_;
};

class BatteryStatePublishChecker : public IPublishChecker{
 public:
  BatteryStatePublishChecker(const rclcpp::Node::SharedPtr& node,
                             const std::string& topic_name) {
    subscriber_ = node->create_subscription<sensor_msgs::msg::BatteryState>(topic_name, rclcpp::QoS(1),
        std::bind(&BatteryStatePublishChecker::Callback, this, std::placeholders::_1));
  }

  sensor_msgs::msg::BatteryState GetValue() const {return last_battery_state_;}

 private:
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscriber_;
  sensor_msgs::msg::BatteryState last_battery_state_;

  void Callback(const sensor_msgs::msg::BatteryState::SharedPtr message) {
    last_battery_state_ = *message;
    is_update_ = true;
  }
};

class BoolPublishChecker : public IPublishChecker{
 public:
  BoolPublishChecker(const rclcpp::Node::SharedPtr& node,
                     const std::string& topic_name) {
    subscriber_ = node->create_subscription<std_msgs::msg::Bool>(topic_name, rclcpp::QoS(1),
        std::bind(&BoolPublishChecker::Callback, this, std::placeholders::_1));
  }

  bool CheckExpected(const bool expected_value) const {return last_value_ == expected_value;}

 private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
  bool last_value_;

  void Callback(const std_msgs::msg::Bool::SharedPtr message) {
    last_value_ = message->data;
    is_update_ = true;
  }
};
}  // anonymous namespace

namespace hsrb_power_ecu {
class MainLoopTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_node_ = rclcpp::Node::make_shared("test_node");
    loop_node_ = rclcpp::Node::make_shared("loop_node");
    loop_node_->declare_parameter<double>("runstop_button/publish_rate", 1000.0);
    loop_node_->declare_parameter<double>("battery_state/publish_rate", 1000.0);
    for (const auto& name : kParamAndTopicNames) {
      loop_node_->declare_parameter<double>(name.second + "/publish_rate", 1000.0);
    }
    PrepareMainLoop();
  }

  void TearDown() override {
    if (protocol_->Stop() != boost::system::errc::success) {
      RCLCPP_FATAL(loop_node_->get_logger(), "stop failed");
      FAIL();
    }
    protocol_->Close();
  }

  void PrepareMainLoop() {
    network_ = boost::make_shared<hsrb_power_ecu::NetworkMock>();
    protocol_ = boost::make_shared<hsrb_power_ecu::PowerEcuProtocol>(network_, loop_node_);

    if (!protocol_->Open()) {
      RCLCPP_FATAL(loop_node_->get_logger(), "protocol_ Open failed.");
      FAIL();
    }
    if (!protocol_->Init()) {
      RCLCPP_FATAL(loop_node_->get_logger(), "protocol_ Init Failed");
      FAIL();
    }
    if (protocol_->Start() != boost::system::errc::success) {
      RCLCPP_FATAL(loop_node_->get_logger(), "start failed");
      FAIL();
    }

    led_command_subscriber_ = std::make_shared<hsrb_power_ecu::LedCommandSubscriber>(loop_node_, protocol_);

    publishers_.push_back(std::make_shared<hsrb_power_ecu::BatteryStatePublisher>(loop_node_, protocol_));
    publishers_.push_back(std::make_shared<hsrb_power_ecu::RunstopStatePublisher>(loop_node_, protocol_));
    for (const auto& name : kParamAndTopicNames) {
      publishers_.push_back(std::make_shared<hsrb_power_ecu::BoolStatePublisher>(
          loop_node_, protocol_, name.first, name.second));
    }
  }

  void RunMainLoop(std::function<bool()> done_func) {
    auto clock = loop_node_->get_clock();
    const auto start_time = clock->now();
    rclcpp::WallRate loop_rate(10.0);  // Hz
    while (rclcpp::ok() && !done_func()) {
      protocol_->ReceiveAll();
      protocol_->SendAll();
      for (auto& publisher : publishers_) {
        publisher->Publish();
      }
      rclcpp::spin_some(loop_node_);
      rclcpp::spin_some(test_node_);
      loop_rate.sleep();

      if (clock->now() - start_time > kLoopTimeout) {
        throw std::runtime_error("mainloop timeout");
      }
    }
  }

  // What is used in main_loop
  rclcpp::Node::SharedPtr loop_node_;
  boost::shared_ptr<hsrb_power_ecu::NetworkMock> network_;
  boost::shared_ptr<hsrb_power_ecu::PowerEcuProtocol> protocol_;
  std::shared_ptr<hsrb_power_ecu::LedCommandSubscriber> led_command_subscriber_;
  std::vector<std::shared_ptr<hsrb_power_ecu::IStatePublisher>> publishers_;

  // What you are using in the test
  rclcpp::Node::SharedPtr test_node_;
};


TEST_F(MainLoopTest, LedCommandSubscribeTest) {
  auto command_status_led_publisher_ =
      test_node_->create_publisher<std_msgs::msg::ColorRGBA>("command_status_led_rgb", 1);

  std_msgs::msg::ColorRGBA message;
  message.r = 1.0;
  message.g = 1.0;
  message.b = 1.0;
  message.a = 1.0;
  network_->ResetSendBuffer();
  command_status_led_publisher_->publish(message);

  std::function<bool()> is_buffer_sent = [&] {return (network_->GetSendBuffer() != "");};
  EXPECT_NO_THROW(RunMainLoop(is_buffer_sent));
  ASSERT_TRUE((network_->GetSendBuffer().find("H,ledc_,023,255,255,255,") != -1));

  message.g = 0.0;
  message.a = 0.0;
  network_->ResetSendBuffer();
  command_status_led_publisher_->publish(message);

  EXPECT_NO_THROW(RunMainLoop(is_buffer_sent));
  ASSERT_TRUE((network_->GetSendBuffer().find("H,ledc_,023,255,000,255,") != -1));

  // The command is not sent even if you subscribe to the same message as the current state
  network_->ResetSendBuffer();
  command_status_led_publisher_->publish(message);

  EXPECT_THROW(RunMainLoop(is_buffer_sent), std::runtime_error);
  ASSERT_TRUE((network_->GetSendBuffer().find("H,ledc_,023,255,000,255,") == -1));

  // The expected input value is 0 to 1, and 255 is applied to int to the command value.
  // If the command value is outside the range of 0 to 255, the maximum value and the minimum value are clipped.
  message.r = -10.0;
  message.g = 10.0;
  network_->ResetSendBuffer();
  command_status_led_publisher_->publish(message);

  EXPECT_NO_THROW(RunMainLoop(is_buffer_sent));
  ASSERT_TRUE((network_->GetSendBuffer().find("H,ledc_,023,000,255,255,") != -1));
}

TEST_F(MainLoopTest, PublishedDataTest) {
  auto battery_state_checker =
      std::make_shared<BatteryStatePublishChecker>(test_node_, "battery_state");
  auto runstop_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "runstop_button");
  auto base_f_bumper_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "base_f_bumper_sensor");
  auto base_b_bumper_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "base_b_bumper_sensor");
  auto emergency_stop_button_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "emergency_stop_button");
  auto wireless_stop_button_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "wireless_stop_button");
  auto wireless_stop_enable_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "wireless_stop_enable");
  auto battery_charging_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "battery_charging");

  std::vector<std::shared_ptr<IPublishChecker>> publish_checkers = {
    battery_state_checker, runstop_checker, base_f_bumper_checker, base_b_bumper_checker, emergency_stop_button_checker,
    wireless_stop_button_checker, wireless_stop_enable_checker, battery_charging_checker
  };

  std::string header = "E,ecu1_,326,";
  std::string body = ("0000000000,00000000000000,h00,h0000000000000000,"
                      "h00000000000000000000000000000000"
                      "00000000000000000000000000000000, 00000, 00000, 00000,"
                      " 00000, 000,h0000,00000,h0000,000,h00,h0000000000000000, "
                      "0000000000,-0000000000, 0000000000, 0000000000,-0000000000, "
                      "0000000000, 0000000000,-0000000000, 0000000000,-0000000000,"
                      "h00,");
  network_->UpdateBuffer(header + body);
  for (auto& publish_checker : publish_checkers) {publish_checker->Reset();}

  std::function<bool()> CheckUpdated = [&] {
    for (auto& publish_checker : publish_checkers) {
      if (!publish_checker->CheckUpdated()) return false;
    }
    return true;
  };
  EXPECT_NO_THROW(RunMainLoop(CheckUpdated));

  ASSERT_TRUE(runstop_checker->CheckExpected(true));
  ASSERT_TRUE(base_f_bumper_checker->CheckExpected(false));
  ASSERT_TRUE(base_b_bumper_checker->CheckExpected(false));
  ASSERT_TRUE(emergency_stop_button_checker->CheckExpected(true));
  ASSERT_TRUE(wireless_stop_button_checker->CheckExpected(true));
  ASSERT_TRUE(wireless_stop_enable_checker->CheckExpected(false));
  ASSERT_TRUE(battery_charging_checker->CheckExpected(false));
  auto value = battery_state_checker->GetValue();
  ASSERT_DOUBLE_EQ(value.capacity, 0.0);
  ASSERT_DOUBLE_EQ(value.charge, 0.0);
  ASSERT_DOUBLE_EQ(value.current, 0.0);
  ASSERT_DOUBLE_EQ(value.voltage, 0.0);
  ASSERT_DOUBLE_EQ(value.temperature, 0.0);

  // When changing the corresponding part, check if the issued is correctly changed.
  body = ("0000000000,00000000000000,h00,h0000000000000063,"
          "h00000000000000000000000000000000"
          "00000000000000000000000000000000, 01000, 01000, 01000,"
          " 01000, 001,h0000,00000,h0000,000,h60,h0000000000000000, "
          "0000000000,-0000000000, 0000000000, 0000000000,-0000000000, "
          "0000000000, 0000000000,-0000000000, 0000000000,-0000000000,"
          "h00,");
  network_->UpdateBuffer(header + body);
  for (auto& publish_checker : publish_checkers) {publish_checker->Reset();}
  EXPECT_NO_THROW(RunMainLoop(CheckUpdated));

  ASSERT_TRUE(runstop_checker->CheckExpected(false));
  ASSERT_TRUE(base_f_bumper_checker->CheckExpected(true));
  ASSERT_TRUE(base_b_bumper_checker->CheckExpected(true));
  ASSERT_TRUE(emergency_stop_button_checker->CheckExpected(false));
  ASSERT_TRUE(wireless_stop_button_checker->CheckExpected(false));
  ASSERT_TRUE(wireless_stop_enable_checker->CheckExpected(true));
  ASSERT_TRUE(battery_charging_checker->CheckExpected(true));
  value = battery_state_checker->GetValue();
  ASSERT_DOUBLE_EQ(value.capacity, 1.0);
  ASSERT_DOUBLE_EQ(value.charge, 1.0);
  ASSERT_DOUBLE_EQ(value.current, 1.0);
  ASSERT_DOUBLE_EQ(value.voltage, 1.0);
  ASSERT_DOUBLE_EQ(value.temperature, 1.0);
}
}  // namespace hsrb_power_ecu

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
