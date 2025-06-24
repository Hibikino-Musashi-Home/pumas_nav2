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
/// @file collision_detector_config-test.cpp
/// @brief Test of class that holds interference check settings
#include <fstream>
#include <string>
#include <gtest/gtest.h>
#include "tmc_robot_collision_detector/collision_detector_config.hpp"

namespace {
std::string LoadFile(const std::string& file_path) {
  std::string xml_string;
  std::fstream xml_file(file_path, std::fstream::in);
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  return xml_string;
}
}  // namespace

namespace tmc_robot_collision_detector {

const int32_t kObjectNum = 7;
const char* kGroupName[kObjectNum] =
    {"GROUP1", "GROUP1", "GROUP1", "GROUP2",
    "GROUP2", "GROUP3", "OUTER"};
const char* kObjectName[kObjectNum] =
    {"OBJECT1", "OBJECT2", "OBJECT3", "OBJECT4",
    "OBJECT5", "OBJECT6", "OUTER1"};
const uint16_t kGroupAnswer[kObjectNum] = {1, 1, 1, 2, 2, 4, 8};
const uint16_t kFilterAnswer[kObjectNum] =
    {0xFFFC, 0xFFFC, 0xFFFC, 0xFFFC, 0xFFFC, 0xFFFB, 0xFFF7};
const uint16_t kInnerAnswer[kObjectNum] =
    {0xFFF8, 0xFFF8, 0xFFF8, 0xFFFC, 0xFFFC, 0xFFFA, 0xFFF7};

class CollisionDetectorConfigTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_test_ = LoadFile("gtest/test_config.xml");
    config_ = std::make_shared<CollisionDetectorConfig>(config_test_);
  }

  std::string config_test_;
  CollisionDetectorConfig::Ptr config_;
};

TEST_F(CollisionDetectorConfigTest, Constructor) {
  // Normal system: Read the correct configuration file
  EXPECT_EQ(4, config_->GetGroupNameList().size());

  // Anomaly system: Specify a configuration file with a grammatical mistake
  EXPECT_ANY_THROW(std::make_shared<CollisionDetectorConfig>(LoadFile("gtest/incorrect_coldet_config.xml")));

  // Abnormal system: Read the configuration file with the number of groups exceeding the upper limit
  EXPECT_ANY_THROW(std::make_shared<CollisionDetectorConfig>(LoadFile("gtest/too_many_group_config.xml")));

  // Abnormal system: The non -existent group has been used as an exclusion pair of interference checks.
  // Read the configuration file
  EXPECT_ANY_THROW(std::make_shared<CollisionDetectorConfig>(LoadFile("gtest/non_exist_group_contact.xml")));

  // Abnormal system: The non -existent group has been used as an exclusion pair of interference checks.
  // Read the configuration file
  EXPECT_ANY_THROW(std::make_shared<CollisionDetectorConfig>(LoadFile("gtest/non_exist_group_inner.xml")));
}

TEST_F(CollisionDetectorConfigTest, GetBitByObject) {
  // Normal system: Acquire group BIT by specifying the object name
  for (int32_t i = 0; i < kObjectNum; i++) {
    EXPECT_EQ(kGroupAnswer[i],
              config_->GetGroupBitByObjectName(kObjectName[i]));
  }

  // Normal system: Acquire a filter bit by specifying the object name
  for (int32_t i = 0; i < kObjectNum; i++) {
    EXPECT_EQ(kFilterAnswer[i],
              config_->GetFilterBitByObjectName(kObjectName[i]));
  }

  // Normal system: Get Inner bit by specifying the object name
  for (int32_t i = 0; i < (kObjectNum-1); i++) {
    EXPECT_EQ(kInnerAnswer[i],
              config_->GetInnerFilterBit(kObjectName[i]));
  }

  // Abnormal system: Get Inner bit by specifying object names that are not robot sites
  EXPECT_ANY_THROW(config_->GetInnerFilterBit(kObjectName[kObjectNum - 1]));
}

TEST_F(CollisionDetectorConfigTest, GetBitByGroup) {
  // Normal system: Acquire group BIT by specifying the group name
  for (int32_t i = 0; i < kObjectNum; i++) {
    EXPECT_EQ(kGroupAnswer[i],
              config_->GetGroupBitByGroupName(kGroupName[i]));
  }

  // Normal system: Acquire a filter bit by specifying the group name
  for (int32_t i = 0; i < kObjectNum; i++) {
    EXPECT_EQ(kFilterAnswer[i],
              config_->GetFilterBitByGroupName(kGroupName[i]));
  }

  // Abnormal system: Acquire a group bit by specifying a group name that does not exist
  EXPECT_ANY_THROW(config_->GetGroupBitByGroupName("hoge"));

  // Abnormal system: Acquire a filter bit by specifying a group name that does not exist
  EXPECT_ANY_THROW(config_->GetFilterBitByGroupName("hoge"));
}

TEST_F(CollisionDetectorConfigTest, SetConfig) {
  // Normal system: Set and change the change
  uint16_t category = config_->GetGroupBitByGroupName(kGroupName[0]);
  uint16_t filter = config_->GetFilterBitByGroupName(kGroupName[0]);
  EXPECT_NO_THROW(config_->SetConfig(kGroupName[0], category - 1, filter - 1));
  EXPECT_EQ(category - 1, config_->GetGroupBitByGroupName(kGroupName[0]));
  EXPECT_EQ(filter - 1, config_->GetFilterBitByGroupName(kGroupName[0]));

  // Abnormal system: Set with specified group name that does not exist
  EXPECT_ANY_THROW(config_->SetConfig("hoge", 1, 2));
}


TEST_F(CollisionDetectorConfigTest, GetName) {
  // Normal system: Acquire group names from the object name
  for (int32_t i = 0; i < kObjectNum; i++) {
    EXPECT_EQ(kGroupName[i], config_->GetBelongedGroupName(kObjectName[i]));
  }

  // Normal system: Outer returns when you get a group of non -existent objects
  EXPECT_EQ(std::string("OUTER"), config_->GetBelongedGroupName("hoge"));

  // Normal system: Acquire a list of objects belonging to the group
  EXPECT_EQ(3, config_->GetObjectListInGroup(kGroupName[0]).size());
  EXPECT_EQ(2, config_->GetObjectListInGroup(kGroupName[3]).size());
  EXPECT_EQ(1, config_->GetObjectListInGroup(kGroupName[5]).size());
  EXPECT_EQ(0, config_->GetObjectListInGroup(kGroupName[6]).size());

  // Normal system: Obtain the group name list of robot parts
  EXPECT_EQ(3, config_->GetRobotPartsGroupNameList().size());

  // Normal system: Obtain a list of object pairs excluded from interference checks
  EXPECT_EQ(3, config_->GetDisableObjectPairList().size());

  // Abnormal system: Specify a group name that does not exist
  // Get the list of objects belonging to the group
  EXPECT_ANY_THROW(config_->GetObjectListInGroup("hoge"));
}
}  // namespace tmc_robot_collision_detector
int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

