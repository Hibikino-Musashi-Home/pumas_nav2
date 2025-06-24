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

#include <tmc_utils/msg_io.hpp>

#include <iomanip>
#include <sstream>

#include <rcl_logging_interface/rcl_logging_interface.h>

namespace tmc_utils {

// Obtained a ROS logging lectory
std::string GetLogDirectory() {
  const auto allocator = rcutils_get_default_allocator();
  char* directory = nullptr;
  // When successful, RCL_LOGGING_RET_OK is returned, but it cannot be used because it is defined in CPP in Define.
  // Since it was defined as 0, compare it with 0
  if (rcl_logging_get_logging_directory(allocator, &directory) == 0) {
    return std::string(directory);
  } else {
    return std::string();
  }
}

// Get the character string of time
std::string GetTimeString(const rclcpp::Time& stamp) {
  const auto stamp_msg = builtin_interfaces::msg::Time(stamp);
  std::ostringstream ss;
  ss << std::setw(9) << std::setfill('0') << stamp_msg.nanosec;

  std::chrono::time_point<std::chrono::system_clock> time_point(std::chrono::seconds(stamp_msg.sec));
  const auto time_t = std::chrono::system_clock::to_time_t(time_point);
  const auto tm = std::localtime(&time_t);
  char date[17];
  strftime(date, sizeof(date), "%Y%m%dT%H%M%S_", tm);
  return std::string(date) + ss.str();
}

}  // namespace tmc_utils
