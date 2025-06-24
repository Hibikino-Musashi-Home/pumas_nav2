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
/// @brief Providing MPU9150 classes
#ifndef HSRB_IMU_SENSOR_PROTOCOLNODE_HPP
#define HSRB_IMU_SENSOR_PROTOCOLNODE_HPP

#include <memory>
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread.hpp>
#include <std_srvs/srv/empty.hpp>

#include <hsrb_imu_sensor_protocol/mpu9150_packet_parser.hpp>
#include <hsrb_imu_sensor_protocol/mpu9150_protocol.hpp>

class MPU9150Node {
 public:
  MPU9150Node();

  ~MPU9150Node();

  int Init(rclcpp::Node::SharedPtr node);

  boost::system::error_code EnableRealtimeProcess();

  int RunLoopImpl(const rclcpp::Duration& cycle_time, bool check_cycle, bool enable_realtime);

  int RunRealtimeLoop();

  int RunLoop(bool check_cycle);
  bool ResetImuCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                        std::shared_ptr<std_srvs::srv::Empty::Response> res);
  void Publish();
  void Write();
  void Read();
  bool GetIsRealTime();
  int Configure();
  int Spin();
  boost::system::error_code TryWithRetry(boost::function<boost::system::error_code()> func, uint32_t retry);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr imu_calibrate_service_;

  // MPU9150 communication
  std::shared_ptr<hsrb_imu_sensor_protocol::MPU9150Network> mpu9150_network_;
  std::shared_ptr<hsrb_imu_sensor_protocol::MPU9150Protocol> mpu9150_protocol_;
  boost::mutex reset_imu_mutex_;

  const int32_t kDefaultNetworkTimeout = 2000000;
  const int32_t kDefaultNetworkTick = 10000;
  const int32_t kDefaultImuNetworkTimeout = 500000;
  const int32_t kDefaultImuNetworkTick = 10000;
  const bool kDefaultUSB485 = false;
  const bool kDefaultIsRealtime = true;
  const double kDefaultCycleTime = 0.01;
  const double kDefaultCycleOverWarnTolerance = 0.1;
  const double kDefaultCycleOverErrorTolerance = 0.4;
  const double kDefaultCycleOverTimeTolerance = 1.0e-5;
  const char* const kDefaultImuDevice = "/dev/ttyCTI0";
  const char* const kDefaultImuFrameId = "base_imu_frame";
  const double kImuResetTimeout = 5.0;
  const uint32_t kRetryCount = 3;
  bool reset_imu_ = false;

  // Number of cycleover judgment results
  const int kCycleOverBufferNum = 100;
  // Cycleover check interval [SEC]
  const double kCycleOverCheckInterval = 0.1;

  int32_t network_timeout_;
  int32_t network_tick_;
  int32_t imu_network_timeout_;
  int32_t imu_network_tick_;
  bool use_usb_;
  bool is_realtime_;
  bool disable_imu_;
  double cycle_time_;
  double cycle_over_warn_tolerance_;
  double cycle_over_error_tolerance_;
  double cycle_over_time_tolerance_;
  std::string rs485_device_;
  std::string imu_device_;
  std::string imu_frame_id_;
  hsrb_imu_sensor_protocol::ImuState imu_output_;
};

#endif  // HSRB_IMU_SENSOR_PROTOCOLNODE_HPP
