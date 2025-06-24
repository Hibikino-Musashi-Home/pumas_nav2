/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
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
#include <gtest/gtest.h>

#include <hsrb_servomotor_protocol/load_control_table.hpp>

// homeにcontrol_table.csvがる
// md5が一致する
TEST(LoadControlTableTest, SucceedByHomeTable) {
  char* str_result;
  std::string package_path;
  std::string selected_path;
  hsrb_servomotor_protocol::ControlTable hw_table;
  hsrb_servomotor_protocol::ControlTable table;
  hw_table.Load(
      "test_load_control_table/file_exist_home/"
      ".control_table/control_table.csv");
  putenv(const_cast<char*>("HOME=test_load_control_table/file_exist_home"));
  package_path = "test_load_control_table";

  EXPECT_TRUE(hsrb_servomotor_protocol::LoadControlTable(package_path, hw_table.GetMd5Sum(), table, selected_path));
  EXPECT_EQ(hw_table.GetMd5Sum(), table.GetMd5Sum());
  EXPECT_EQ(
      "test_load_control_table/file_exist_home/"
      ".control_table/control_table.csv",
      selected_path);
}

// homeにcontrol_table.csvがる
// md5が一致しない
TEST(LoadControlTableTest, FailByUnmatchHomeTable) {
  std::string package_path;
  std::string selected_path;
  hsrb_servomotor_protocol::ControlTable hw_table;
  hsrb_servomotor_protocol::ControlTable table;
  hw_table.Load("test_load_control_table/control_tables/v0.6.0/control_table.csv");
  putenv(const_cast<char*>("HOME=test_load_control_table/file_exist_home"));
  package_path = "test_load_control_table";

  EXPECT_FALSE(hsrb_servomotor_protocol::LoadControlTable(package_path, hw_table.GetMd5Sum(), table, selected_path));
}

// homeにcontrol_table.csvがない
// バージョンごとのディレクトリにcontrol_table.csvがある
// md5が一致する
TEST(LoadControlTableTest, SucceedByPackagePathTable1) {
  std::string package_path;
  std::string selected_path;
  hsrb_servomotor_protocol::ControlTable hw_table;
  hsrb_servomotor_protocol::ControlTable table;
  hw_table.Load("test_load_control_table/control_tables/v0.7.0/control_table.csv");
  putenv(const_cast<char*>("HOME=test_load_control_table/file_none_exist_home"));
  package_path = "test_load_control_table";

  EXPECT_TRUE(hsrb_servomotor_protocol::LoadControlTable(package_path, hw_table.GetMd5Sum(), table, selected_path));
  EXPECT_EQ(hw_table.GetMd5Sum(), table.GetMd5Sum());
  EXPECT_EQ("test_load_control_table/control_tables/v0.7.0/control_table.csv", selected_path);
}

// homeにcontrol_table.csvがない
// バージョンごとのディレクトリにcontrol_table.csvがある
// md5が一致する
TEST(LoadControlTableTest, SucceedByPackagePathTable2) {
  std::string package_path;
  std::string selected_path;
  hsrb_servomotor_protocol::ControlTable hw_table;
  hsrb_servomotor_protocol::ControlTable table;
  hw_table.Load("test_load_control_table/control_tables/v0.6.0/control_table.csv");
  putenv(const_cast<char*>("HOME=test_load_control_table/file_none_exist_home"));
  package_path = "test_load_control_table";

  EXPECT_TRUE(hsrb_servomotor_protocol::LoadControlTable(package_path, hw_table.GetMd5Sum(), table, selected_path));
  EXPECT_EQ(hw_table.GetMd5Sum(), table.GetMd5Sum());
  EXPECT_EQ("test_load_control_table/control_tables/v0.6.0/control_table.csv", selected_path);
}

// homeにcontrol_table.csvがない
// バージョンごとのディレクトリにcontrol_table.csvがある
// md5が一致しない
TEST(LoadControlTableTest, FailByNothingMatchedTable1) {
  std::string package_path;
  std::string selected_path;
  hsrb_servomotor_protocol::ControlTable hw_table;
  hsrb_servomotor_protocol::ControlTable table;
  hw_table.Load("test_load_control_table/v0.1.0/control_table.csv");
  putenv(const_cast<char*>("HOME=test_load_control_table/file_none_exist_home"));
  package_path = "test_load_control_table";

  EXPECT_FALSE(hsrb_servomotor_protocol::LoadControlTable(package_path, hw_table.GetMd5Sum(), table, selected_path));
}

// homeにcontrol_table.csvがない
// バージョンごとのディレクトリにcontrol_table.csvがある
// バージョンごとのディレクトリの親ディレクトリにcontrol_table.csvがある
// 親ディレクトリにあるcontrol_table.csvのmd5が一致する
TEST(LoadControlTableTest, FailByNothingMatchedTable2) {
  std::string package_path;
  std::string selected_path;
  hsrb_servomotor_protocol::ControlTable hw_table;
  hsrb_servomotor_protocol::ControlTable table;
  hw_table.Load("test_load_control_table/v0.1.0/control_table.csv");
  putenv(const_cast<char*>("HOME=test_load_control_table/file_none_exist_home"));
  package_path = "test_load_control_table";

  EXPECT_FALSE(hsrb_servomotor_protocol::LoadControlTable(package_path, hw_table.GetMd5Sum(), table, selected_path));
}

// homeにcontrol_table.csvがない
// バージョンごとのディレクトリがない
TEST(LoadControlTableTest, FailByNoneExistDir) {
  std::string package_path;
  std::string selected_path;
  hsrb_servomotor_protocol::ControlTable hw_table;
  hsrb_servomotor_protocol::ControlTable table;
  hw_table.Load("test_load_control_table/control_tables/v0.7.0/control_table.csv");
  putenv(const_cast<char*>("HOME=test_load_control_table/file_none_exist_home"));
  package_path = "hoge";

  EXPECT_FALSE(hsrb_servomotor_protocol::LoadControlTable(package_path, hw_table.GetMd5Sum(), table, selected_path));
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
