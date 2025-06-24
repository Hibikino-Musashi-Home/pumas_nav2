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

#include <string>

#include "imu_state_publisher.hpp"

namespace hsrb_power_ecu {

ImuStatePublisher::ImuStatePublisher(const rclcpp::Node::SharedPtr& node,
                                     const PowerEcuProtocol::Ptr protocol,
                                     const std::string frame_id)
    : StatePublisherBase(node, "imu/data"), clock_(node->get_clock()) {
  imu_frame = frame_id;
  // gyro_status = ExistParamPtr<std::string>(protocol, "gyro_status");
  imu_quaternions =
    ExistParamPtr<boost::array<double, 4> >(protocol, "imu_quaternions");
  imu_angular_velocities =
    ExistParamPtr<boost::array<double, 3> >(protocol, "imu_angular_velocities");
  imu_accelerations =
    ExistParamPtr<boost::array<double, 3> >(protocol, "imu_accelerations");
}

sensor_msgs::msg::Imu ImuStatePublisher::CreateMessage() {
  sensor_msgs::msg::Imu message;
  message.orientation.x = (*imu_quaternions)[0];
  message.orientation.y = (*imu_quaternions)[1];
  message.orientation.z = (*imu_quaternions)[2];
  message.orientation.w = (*imu_quaternions)[3];
  message.angular_velocity.x = (*imu_angular_velocities)[0];
  message.angular_velocity.y = (*imu_angular_velocities)[1];
  message.angular_velocity.z = (*imu_angular_velocities)[2];
  message.linear_acceleration.x = (*imu_accelerations)[0];
  message.linear_acceleration.y = (*imu_accelerations)[1];
  message.linear_acceleration.z = (*imu_accelerations)[2];
  message.header.frame_id = imu_frame;
  message.header.stamp = clock_->now();
  return message;
}

}  // namespace hsrb_power_ecu
