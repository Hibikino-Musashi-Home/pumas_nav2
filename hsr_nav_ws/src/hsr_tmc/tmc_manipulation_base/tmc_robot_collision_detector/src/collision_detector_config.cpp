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
/// @brief Keep the interference check settings

#include <stdio.h>

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <tinyxml.h>

#include "tmc_robot_collision_detector/collision_detector_config.hpp"

namespace {
/// @brief Check if Name exists in name_list.
bool CheckNameList(const std::vector<std::string>& name_list,
                   const std::string& name) {
  if (std::find(name_list.begin(), name_list.end(), name) ==
      name_list.end()) {
    return false;
  } else {
    return true;
  }
}
}  // anonymous namespace

namespace tmc_robot_collision_detector {

// constructor.Read the settings from the configuration file.
CollisionDetectorConfig::CollisionDetectorConfig(
    const std::string &xml_string) {
  TiXmlDocument xml;
  xml.Parse(xml_string.c_str());
  if (xml.Error()) {
    throw std::domain_error("error: cannot parse robot_collision_pair");
  }

  // Pair of group names that do not check interference
  std::vector<PairString> non_contact_group_list;
  // List of objects/group names that do not check interference
  std::vector<PairString> non_contact_list;
  // Pair of nearby object names inside the robot
  std::vector<PairString> non_check_inner_distance_list;
  // Group reading
  TiXmlElement* root = xml.FirstChildElement();
  for (TiXmlElement* group = root->FirstChildElement("group");
       group != NULL;
       group = group->NextSiblingElement("group")) {
    std::string group_name = group->Attribute("name");
    group_name_.push_back(group_name);
    for (TiXmlElement* object = group->FirstChildElement("object");
         object != NULL;
         object = object->NextSiblingElement("object")) {
      std::string object_name = object->Attribute("name");
      belonged_group_list_.insert(PairString(object_name, group_name));
      group_member_list_.insert(PairString(group_name, object_name));
    }
  }

  // Get the group name of the robot site
  TiXmlElement* robot_parts_group = root->FirstChildElement("robot-parts-group");
  if (robot_parts_group != NULL) {
    for (TiXmlElement* group = robot_parts_group->FirstChildElement("group");
         group != NULL;
         group = group->NextSiblingElement("group")) {
      std::string group_name = group->Attribute("name");
      robot_parts_group_name_.push_back(group_name);
    }
  }

  // Read pair that does not interfere
  TiXmlElement* non_contact = root->FirstChildElement("non-contact");
  if (non_contact != NULL) {
    for (TiXmlElement* pair = non_contact->FirstChildElement("pair");
         pair != NULL;
         pair = pair->NextSiblingElement("pair")) {
      if ((pair->Attribute("group1") != NULL) &&
          (pair->Attribute("group2") != NULL)) {
        std::string group1_name = pair->Attribute("group1");
        std::string group2_name = pair->Attribute("group2");
        non_contact_group_list.push_back(PairString(group1_name,
                                                    group2_name));
      } else if ((pair->Attribute("name1") != NULL) &&
                 (pair->Attribute("name2") != NULL)) {
        std::string name1 = pair->Attribute("name1");
        std::string name2 = pair->Attribute("name2");
        non_contact_list.push_back(PairString(name1, name2));
      }
    }
  }

  // Reading pairs ignored by checking nearby parts inside
  TiXmlElement* non_check = root->FirstChildElement("non-check-inner-distance");
  if (non_check != NULL) {
    for (TiXmlElement* pair = non_check->FirstChildElement("pair");
         pair != NULL;
         pair = pair->NextSiblingElement("pair")) {
      if ((pair->Attribute("group1") != NULL) &&
          (pair->Attribute("group2") != NULL)) {
        std::string group1_name = pair->Attribute("group1");
        std::string group2_name = pair->Attribute("group2");
        non_check_inner_distance_list.push_back(
            PairString(group1_name, group2_name));
      }
    }
  }

  if (group_name_.size() > kMaxGroupNum) {
    throw std::domain_error("error: too many group in robot_collision_pair");
  }
  // Disassemble non_contact_list
  for (std::vector<PairString>::iterator it = non_contact_list.begin();
       it != non_contact_list.end(); ++it) {
    if (CheckNameList(group_name_, it->first) &&
        CheckNameList(group_name_, it->second)) {
      non_contact_group_list.push_back(*it);
      continue;
    }
    std::vector<std::string> name1_list;
    if (CheckNameList(group_name_, it->first)) {
      name1_list = GetObjectListInGroup(it->first);
    } else {
      name1_list.push_back(it->first);
    }
    std::vector<std::string> name2_list;
    if (CheckNameList(group_name_, it->second)) {
      name2_list = GetObjectListInGroup(it->second);
    } else {
      name2_list.push_back(it->second);
    }
    for (std::vector<std::string>::iterator name1 = name1_list.begin();
         name1 != name1_list.end(); ++name1) {
      for (std::vector<std::string>::iterator name2 = name2_list.begin();
           name2 != name2_list.end(); ++name2) {
        disable_collision_pair_list_.push_back(PairString(*name1, *name2));
      }
    }
  }

  // Create categories and filters
  for (uint32_t i = 0; i < group_name_.size(); i++) {
    uint16_t category = 1 << (i);
    group_bit_list_.insert(std::pair<std::string, uint16_t>(group_name_.at(i),
                                                            category));
    // The filter is set to interfere with anyone other than yourself
    filter_bit_list_.insert(
        std::pair<std::string, uint16_t>(group_name_.at(i), 0xFFFF - category));
  }
  // Adjust the filter using a list that does not check interference
  for (std::vector<PairString >::iterator it = non_contact_group_list.begin();
       it != non_contact_group_list.end(); ++it) {
    SetNonContactPairToFilter_(it->first, it->second);
    SetNonContactPairToFilter_(it->second, it->first);
  }
  // Adjust the internal filter using the nearby object name list inside the robot
  inner_filter_bit_list_ = filter_bit_list_;
  for (std::vector<PairString >::iterator it =
       non_check_inner_distance_list.begin();
       it != non_check_inner_distance_list.end(); ++it) {
    SetNotCheckInnerPairToFilter_(it->first, it->second);
    SetNotCheckInnerPairToFilter_(it->second, it->first);
  }
}

// Obtained an object group BIT
uint16_t CollisionDetectorConfig::GetGroupBitByObjectName(
    const std::string &object_name) const {
  std::map<std::string, std::string>::const_iterator it;
  it = belonged_group_list_.find(object_name);
  if (it == belonged_group_list_.end()) {
    return GetGroupBitByGroupName(std::string(kOuterGroupName));
  }
  return GetGroupBitByGroupName(it->second);
}

// Obtained an object filter bit
uint16_t CollisionDetectorConfig::GetFilterBitByObjectName(
    const std::string &object_name) const {
  std::map<std::string, std::string>::const_iterator it;
  it = belonged_group_list_.find(object_name);
  if (it == belonged_group_list_.end()) {
    return GetFilterBitByGroupName(std::string(kOuterGroupName));
  }
  return GetFilterBitByGroupName(it->second);
}

// Acquired a filter BIT for exploring nearby objects inside the robot
uint16_t CollisionDetectorConfig::GetInnerFilterBit(
    const std::string& object_name) const {
  std::map<std::string, std::string>::const_iterator group_name;
  group_name = belonged_group_list_.find(object_name);
  if (group_name == belonged_group_list_.end()) {
    throw NonExistName(object_name);
  }

  std::map<std::string, uint16_t>::const_iterator it;
  it = inner_filter_bit_list_.find(group_name->second);
  if (it == inner_filter_bit_list_.end()) {
    throw NonExistName(group_name->second);
  }
  return it->second;
}

// Acquired group group BIT
uint16_t CollisionDetectorConfig::GetGroupBitByGroupName(
    const std::string &group_name) const {
  std::map<std::string, uint16_t>::const_iterator it;
  it = group_bit_list_.find(group_name);
  if (it == group_bit_list_.end()) {
    throw NonExistName(group_name);
  }
  return it->second;
}

// Acquired a group filter bit
uint16_t CollisionDetectorConfig::GetFilterBitByGroupName(
    const std::string &group_name) const {
  std::map<std::string, uint16_t>::const_iterator it;
  it = filter_bit_list_.find(group_name);
  if (it == filter_bit_list_.end()) {
    throw NonExistName(group_name);
  }
  return it->second;
}

// Get the group name from the object name
std::string CollisionDetectorConfig::GetBelongedGroupName(
    const std::string& object_name) const {
  std::map<std::string, std::string>::const_iterator it;
  it = belonged_group_list_.find(object_name);
  if (it == belonged_group_list_.end()) {
    return kOuterGroupName;
  }
  return it->second;
}

// Obtained an object name to which you belong from the group name
std::vector<std::string> CollisionDetectorConfig::GetObjectListInGroup(
    const std::string& group_name) const {
  std::vector<std::string> object_list;
  std::pair<std::multimap<std::string, std::string>::const_iterator,
            std::multimap<std::string, std::string>::const_iterator> range;
  range = group_member_list_.equal_range(group_name);
  for (; range.first != range.second; range.first++) {
    object_list.push_back((range.first)->second);
  }
  if (object_list.empty() && (group_name != kOuterGroupName)) {
    throw std::domain_error("error: non exist group " + group_name);
  }
  return object_list;
}

// Set the group and filter
void CollisionDetectorConfig::SetConfig(
    const std::string& group_name, uint16_t category, uint16_t filter) {
  // Check if the group name exists
  if (std::find(group_name_.begin(), group_name_.end(), group_name) ==
      group_name_.end()) {
    throw NonExistName(group_name);
  }
  // Set
  group_bit_list_.find(group_name)->second = category;
  filter_bit_list_.find(group_name)->second = filter;
}

// Set the category/filter so that the interference check is not performed
void CollisionDetectorConfig::SetNonContactPairToFilter_(
    const std::string &groupA,
    const std::string &groupB) {
  std::map<std::string, uint16_t>::iterator category;
  std::map<std::string, uint16_t>::iterator filter;
  category = group_bit_list_.find(groupA);
  if (category == group_bit_list_.end()) {
    throw NonExistName(groupA);
  }
  filter = filter_bit_list_.find(groupB);
  if (filter == filter_bit_list_.end()) {
    throw NonExistName(groupB);
  }
  filter->second &= ~category->second;
}

// Set the category/filter so that the interference check is not performed
void CollisionDetectorConfig::SetNotCheckInnerPairToFilter_(
    const std::string& groupA,
    const std::string& groupB) {
  std::map<std::string, uint16_t>::iterator category;
  std::map<std::string, uint16_t>::iterator filter;
  category = group_bit_list_.find(groupA);
  if (category == group_bit_list_.end()) {
    throw NonExistName(groupA);
  }
  filter = inner_filter_bit_list_.find(groupB);
  if (filter == inner_filter_bit_list_.end()) {
    throw NonExistName(groupB);
  }
  filter->second &= ~category->second;
}
}  // end namespace tmc_robot_collision_detector

