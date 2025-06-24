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
#ifndef TMC_UTILS_MSG_IO_HPP_
#define TMC_UTILS_MSG_IO_HPP_

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>

namespace tmc_utils {

/// Obtained a ROS logging lectory
std::string GetLogDirectory();

/// Get the character string of time
/// @param [in] stamp time stamp
std::string GetTimeString(const rclcpp::Time& stamp);

/// Record any ROS MSG, SRV
/// @param [in] filename  file name
/// @param [in] msg  msg or srv.request or srv.response
/// @return true: Success, false: Failure
template <typename T>
bool SaveMsg(const std::string& filename, const T& msg) {
  std::ofstream ofs;
  ofs.open(filename.c_str(), std::ios::out | std::ios::binary);
  if (!ofs) {
    return false;
  }

  rclcpp::Serialization<T> serialization;
  rclcpp::SerializedMessage serialized_message;
  serialization.serialize_message(&msg, &serialized_message);
  const auto rcl_serialized_message = serialized_message.get_rcl_serialized_message();
  ofs.write(reinterpret_cast<char*>(rcl_serialized_message.buffer), rcl_serialized_message.buffer_length);
  ofs.close();

  return true;
}

/// Get any ROS MSG, SRV
/// @param [in] filename  file name
/// @param [out] msg  msg or srv.request or srv.response
/// @return true: Success, false: Failure
template <typename T>
bool LoadMsg(const std::string& filename, T& msg) {
  std::ifstream ifs;
  ifs.open(filename.c_str(), std::ios::in | std::ios::binary);
  if (!ifs) {
    return false;
  }

  std::vector<uint8_t> data;
  char buf[1];
  while (ifs.read(buf, 1)) {
    data.push_back(buf[0]);
  }
  ifs.close();

  rcl_serialized_message_t rcl_serialized_message;
  rcl_serialized_message.allocator = rcl_get_default_allocator();
  rcl_serialized_message.buffer = data.data();
  rcl_serialized_message.buffer_length = data.size();
  rcl_serialized_message.buffer_capacity = data.size();

  try {
    rclcpp::SerializedMessage serialized_message(rcl_serialized_message);
    rclcpp::Serialization<T> serialization;
    serialization.deserialize_message(&serialized_message, &msg);
    return true;
  } catch(...) {
    return false;
  }
}

/// Classes and time stamps to make SAVEMSG easier to use and save
class MessageLogger {
 public:
  using Ptr = std::shared_ptr<MessageLogger>;

  /// Keep the constructor and output destination
  /// @param [in] prefix  prefix of the output file, including the directory information at the output destination
  explicit MessageLogger(const std::string& prefix) : prefix_(prefix) {}

  /// Update the time stamp to attach to the output file
  /// @param [in] stamp  time stamp
  void UpdateStamp(const rclcpp::Time& stamp) { time_str_ = GetTimeString(stamp); }

  /// Save any ROS MSG, SRV
  /// @param [in] postfix  output file Postfix
  /// @param [in] msg  saving MSG, SRV
  template<typename TYPE>
  bool SaveMessage(const std::string& postfix, const TYPE& msg) const {
    return SaveMsg(prefix_ + time_str_ + postfix, msg);
  }

 private:
  std::string prefix_;
  std::string time_str_;
};

}  // namespace tmc_utils
#endif  // TMC_UTILS_MSG_IO_HPP_
