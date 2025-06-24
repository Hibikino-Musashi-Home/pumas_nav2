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
#include <stdlib.h>

#include <gtest/gtest.h>

#include <boost/filesystem/operations.hpp>
#include <boost/range.hpp>

#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include <tmc_utils/msg_io.hpp>

const char* const kFileName = "/tmp/test.msg";

namespace {

std::vector<std::string> GetFileNames(const std::string& ouput_dir, const std::string& postfix) {
  std::vector<std::string> result;

  const boost::filesystem::path log_dir(ouput_dir);
  for (const auto& path : boost::make_iterator_range(boost::filesystem::directory_iterator(log_dir), {})) {
    if (!boost::filesystem::is_directory(path)) {
      const std::string filename = path.path().filename().native();
      if (std::equal(postfix.rbegin(), postfix.rend(), filename.rbegin())) {
        result.push_back(filename);
      }
    }
  }
  return result;
}

}  // namespace

namespace tmc_utils {

TEST(GetLogDirectoryTest, Get) {
  setenv("ROS_LOG_DIR", "/tmp/hoge", 1);
  EXPECT_EQ(GetLogDirectory(), "/tmp/hoge");
  unsetenv("ROS_LOG_DIR");

  setenv("ROS_HOME", "/tmp/piyo", 1);
  EXPECT_EQ(GetLogDirectory(), "/tmp/piyo/log");
  unsetenv("ROS_HOME");

  setenv("HOME", "/tmp", 1);
  EXPECT_EQ(GetLogDirectory(), "/tmp/.ros/log");
  unsetenv("HOME");
}

TEST(GetTimeString, Get) {
  setenv("TZ", "UTC", 1);
  EXPECT_EQ(GetTimeString(rclcpp::Time(0, 0)), "19700101T000000_000000000");
  EXPECT_EQ(GetTimeString(rclcpp::Time(0, 10)), "19700101T000000_000000010");
  EXPECT_EQ(GetTimeString(rclcpp::Time(1560000001, 0)), "20190608T132001_000000000");
}

TEST(MsgIOTest, SaveAndLoadMsg) {
  std_msgs::msg::String saved_msg;
  saved_msg.data = "hoge";

  EXPECT_TRUE(tmc_utils::SaveMsg(kFileName, saved_msg));

  std_msgs::msg::String loaded_msg;
  EXPECT_TRUE(tmc_utils::LoadMsg(kFileName, loaded_msg));
  EXPECT_EQ(loaded_msg.data, saved_msg.data);

  boost::filesystem::remove(kFileName);
}

TEST(MsgIOTest, SaveAndLoadSrv) {
  example_interfaces::srv::AddTwoInts::Request saved_req;
  saved_req.a = 42;
  saved_req.b = 108;

  EXPECT_TRUE(tmc_utils::SaveMsg(kFileName, saved_req));

  example_interfaces::srv::AddTwoInts::Request loaded_req;
  EXPECT_TRUE(tmc_utils::LoadMsg(kFileName, loaded_req));
  EXPECT_EQ(loaded_req.a, saved_req.a);
  EXPECT_EQ(loaded_req.b, saved_req.b);

  boost::filesystem::remove(kFileName);
}

TEST(MsgIOTest, SaveAndLoadAction) {
  action_tutorials_interfaces::action::Fibonacci::Goal saved_goal;
  saved_goal.order = 42;

  EXPECT_TRUE(tmc_utils::SaveMsg(kFileName, saved_goal));

  action_tutorials_interfaces::action::Fibonacci::Goal loaded_goal;
  EXPECT_TRUE(tmc_utils::LoadMsg(kFileName, loaded_goal));
  EXPECT_EQ(loaded_goal.order, saved_goal.order);

  boost::filesystem::remove(kFileName);
}

TEST(MsgIOTest, MismatchMsgType) {
  std_msgs::msg::String saved_msg;
  saved_msg.data = "hoge";

  EXPECT_TRUE(tmc_utils::SaveMsg(kFileName, saved_msg));

  std_msgs::msg::Header loaded_msg;
  EXPECT_FALSE(tmc_utils::LoadMsg(kFileName, loaded_msg));

  boost::filesystem::remove(kFileName);
}

TEST(MsgIOTest, CannotLoad) {
  std_msgs::msg::String loaded_msg;
  EXPECT_FALSE(tmc_utils::LoadMsg(kFileName, loaded_msg));
}

TEST(MessageLoggerTest, SaveMsg) {
  auto logger = std::make_shared<MessageLogger>("/tmp/test_");

  logger->UpdateStamp(rclcpp::Time(0, 0));

  std_msgs::msg::String saved_msg;
  saved_msg.data = "hoge";

  EXPECT_TRUE(logger->SaveMessage(".amsg", saved_msg));
  EXPECT_TRUE(logger->SaveMessage(".bmsg", saved_msg));

  const auto amsg_filenames = GetFileNames("/tmp", ".amsg");
  ASSERT_EQ(amsg_filenames.size(), 1u);
  EXPECT_EQ(amsg_filenames[0], "test_19700101T000000_000000000.amsg");

  const auto bmsg_filenames = GetFileNames("/tmp", ".bmsg");
  ASSERT_EQ(bmsg_filenames.size(), 1u);
  EXPECT_EQ(bmsg_filenames[0], "test_19700101T000000_000000000.bmsg");

  boost::filesystem::remove("/tmp/test_19700101T000000_000000000.amsg");
  boost::filesystem::remove("/tmp/test_19700101T000000_000000000.bmsg");
}

}  // namespace tmc_utils

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
