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
#include <gtest/gtest.h>

#include <tmc_utils/parameters.hpp>

namespace tmc_utils {

TEST(GetParameterTest, DeclaredParameter) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("test_param", 42)};
  auto node = rclcpp::Node::make_shared("test", options);

  EXPECT_EQ(GetParameter<int64_t>(node, "test_param", 108), 42);
}

TEST(GetParameterTest, DefaultParameter) {
  auto node = rclcpp::Node::make_shared("test");

  EXPECT_EQ(GetParameter<int64_t>(node, "test_param", 108), 108);
}

TEST(GetParameterTest, ParameterWithNamespace) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("test_ns.test_param", 42)};
  auto node = rclcpp::Node::make_shared("test", options);

  EXPECT_EQ(GetParameter<int64_t>(node, "test_ns.test_param", 108), 42);
}

TEST(GetParameterTest, ParameterTypes) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("test_bool", true),
      rclcpp::Parameter("test_int", 42),
      rclcpp::Parameter("test_double", 3.14),
      rclcpp::Parameter("test_string", "hoge"),
      rclcpp::Parameter("test_byte_array", std::vector<uint8_t>({1, 2, 3})),
      rclcpp::Parameter("test_bool_array", std::vector<bool>({true, false})),
      rclcpp::Parameter("test_int_array", std::vector<int64_t>({42, 108})),
      rclcpp::Parameter("test_double_array", std::vector<double>({3.14, 2.72})),
      rclcpp::Parameter("test_string_array", std::vector<std::string>({"hoge", "piyo"}))};
  auto node = rclcpp::Node::make_shared("test", options);

  EXPECT_EQ(GetParameter<bool>(node, "test_bool", false), true);
  EXPECT_EQ(GetParameter<int64_t>(node, "test_int", 108), 42);
  EXPECT_EQ(GetParameter<double>(node, "test_double", 2.72), 3.14);
  EXPECT_EQ(GetParameter<std::string>(node, "test_string", "piyo"), "hoge");
  EXPECT_EQ(GetParameter<std::vector<uint8_t>>(node, "test_byte_array", {}),
            std::vector<uint8_t>({1, 2, 3}));
  EXPECT_EQ(GetParameter<std::vector<bool>>(node, "test_bool_array", {}),
            std::vector<bool>({true, false}));
  EXPECT_EQ(GetParameter<std::vector<int64_t>>(node, "test_int_array", {}),
            std::vector<int64_t>({42, 108}));
  EXPECT_EQ(GetParameter<std::vector<double>>(node, "test_double_array", {}),
            std::vector<double>({3.14, 2.72}));
  EXPECT_EQ(GetParameter<std::vector<std::string>>(node, "test_string_array", {}),
            std::vector<std::string>({"hoge", "piyo"}));
}

TEST(GetParameterTest, InvalidType) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("test_param", 42)};
  auto node = rclcpp::Node::make_shared("test", options);

  EXPECT_THROW(GetParameter<double>(node, "test_param", 0.0), rclcpp::exceptions::InvalidParameterTypeException);
}

TEST(DynamicParameterTest, DeclaredParameter) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("test_param", 42)};
  auto node = rclcpp::Node::make_shared("test", options);

  auto dynamic_param = std::make_shared<DynamicParameter<int64_t>>(node, "test_param", 0);
  EXPECT_EQ(dynamic_param->value(), 42);

  node->set_parameter(rclcpp::Parameter("test_param", 108));
  EXPECT_EQ(dynamic_param->value(), 108);
}

TEST(DynamicParameterTest, DefaultParameter) {
  auto node = rclcpp::Node::make_shared("test");

  auto dynamic_param = std::make_shared<DynamicParameter<int64_t>>(node, "test_param", 42);
  EXPECT_EQ(dynamic_param->value(), 42);

  node->set_parameter(rclcpp::Parameter("test_param", 108));
  EXPECT_EQ(dynamic_param->value(), 108);
}

TEST(DynamicParameterTest, ParameterWithNamespace) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("test_ns.test_param", 42)};
  auto node = rclcpp::Node::make_shared("test", options);

  auto dynamic_param = std::make_shared<DynamicParameter<int64_t>>(node, "test_ns.test_param", 0);
  EXPECT_EQ(dynamic_param->value(), 42);

  node->set_parameter(rclcpp::Parameter("test_ns.test_param", 108));
  EXPECT_EQ(dynamic_param->value(), 108);
}

TEST(DynamicParameterTest, MultipleParameters) {
  auto node = rclcpp::Node::make_shared("test");

  auto dynamic_param_int = std::make_shared<DynamicParameter<int64_t>>(node, "test_int", 42);
  auto dynamic_param_double = std::make_shared<DynamicParameter<double>>(node, "test_double", 3.14);

  EXPECT_EQ(dynamic_param_int->value(), 42);
  EXPECT_EQ(dynamic_param_double->value(), 3.14);

  node->set_parameter(rclcpp::Parameter("test_int", 108));
  node->set_parameter(rclcpp::Parameter("test_double", 2.72));

  EXPECT_EQ(dynamic_param_int->value(), 108);
  EXPECT_EQ(dynamic_param_double->value(), 2.72);
}

TEST(DynamicParameterTest, InvalidType) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("test_param", 42)};
  auto node = rclcpp::Node::make_shared("test", options);

  EXPECT_THROW(std::make_shared<DynamicParameter<double>>(node, "test_param", 0.0),
               rclcpp::exceptions::InvalidParameterTypeException);
}

TEST(DynamicParameterTest, LawPointer) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("test_param", 42)};
  auto node = rclcpp::Node::make_shared("test", options);

  auto dynamic_param = std::make_shared<DynamicParameter<int64_t>>(node.get(), "test_param", 0);
  EXPECT_EQ(dynamic_param->value(), 42);

  node->set_parameter(rclcpp::Parameter("test_param", 108));
  EXPECT_EQ(dynamic_param->value(), 108);
}

}  // namespace tmc_utils

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
