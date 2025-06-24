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
#ifndef TMC_ROBOT_COLLISION_DETECTOR_COLLISION_DETECTOR_CONFIG_HPP_
#define TMC_ROBOT_COLLISION_DETECTOR_COLLISION_DETECTOR_CONFIG_HPP_

#include <stdint.h>

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>


namespace tmc_robot_collision_detector {

/// Exception when specifying an unusual object name
class NonExistName : public std::domain_error {
 public:
  explicit NonExistName(const std::string& error) :
      std::domain_error("error: not exist name " + error) {}
};

/// The pair and meaning of String vary depending on the function
using PairString = std::pair<std::string, std::string>;

/// External object group name
const char* const kOuterGroupName = "OUTER";

/// Collision Map's default group name
const char* const kCuboidGroupName = "CUBOID";

/// Maximum number of groups
const uint32_t kMaxGroupNum = 15;

/// Interference checker settings
class CollisionDetectorConfig {
 public:
  using Ptr = std::shared_ptr<CollisionDetectorConfig>;

  /// @brief constructor.Read the settings from the configuration file.
  explicit CollisionDetectorConfig(const std::string& coldet_config_file);
  ~CollisionDetectorConfig() {}

  /// @brief Obtained an object group BIT
  uint16_t GetGroupBitByObjectName(const std::string& object_name) const;

  /// @brief Obtained an object filter bit
  uint16_t GetFilterBitByObjectName(const std::string& object_name) const;

  /// @brief Acquired a filter BIT for exploring nearby objects inside the robot
  uint16_t GetInnerFilterBit(const std::string& object_name) const;

  /// @brief Acquired group group BIT
  uint16_t GetGroupBitByGroupName(const std::string& group_name) const;

  /// @brief Acquired a group filter bit
  uint16_t GetFilterBitByGroupName(const std::string& group_name) const;

  /// @brief Get the group name from the object name
  std::string GetBelongedGroupName(const std::string& object_name) const;

  /// @brief Obtained an object name to which you belong from the group name
  std::vector<std::string> GetObjectListInGroup(
      const std::string& group_name) const;

  /// @brief Set the group and filter
  void SetConfig(const std::string& group_name,
                 uint16_t category, uint16_t filter);

  /// @brief Acquisition of group name list
  std::vector<std::string> GetGroupNameList() const {return group_name_;}

  /// @brief Acquire the group name list of robot parts
  std::vector<std::string> GetRobotPartsGroupNameList() const {
    return robot_parts_group_name_;
  }

  /// @brief Get an object pair that does not check interference
  std::vector<PairString> GetDisableObjectPairList() const {
    return disable_collision_pair_list_;
  }

 private:
  /// @brief Set the category/filter so that the interference check is not performed
  void SetNonContactPairToFilter_(const std::string& groupA,
                                  const std::string& groupB);

  /// @brief Set the category/filter so that the interference check is not performed
  void SetNotCheckInnerPairToFilter_(const std::string& groupA,
                                     const std::string& groupB);

  /// Group name list
  std::vector<std::string> group_name_;
  /// Group name of robot site
  std::vector<std::string> robot_parts_group_name_;
  /// Support table for objects and group names
  std::map<std::string, std::string> belonged_group_list_;
  /// Correspondence table for group names and objects
  std::multimap<std::string, std::string> group_member_list_;
  /// Group name and the compatibility table of the group BIT that the group stands
  std::map<std::string, uint16_t> group_bit_list_;
  /// Correspondence table (general use) of the group name and the filter BIT that the group stands
  std::map<std::string, uint16_t> filter_bit_list_;
  /// Correspondence table (internal use) of the group name and the filter BIT that the group stands
  std::map<std::string, uint16_t> inner_filter_bit_list_;
  /// List of object names that removing interference checks by default
  std::vector<PairString> disable_collision_pair_list_;
};
}  // namespace tmc_robot_collision_detector
#endif  // TMC_ROBOT_COLLISION_DETECTOR_COLLISION_DETECTOR_CONFIG_HPP_

