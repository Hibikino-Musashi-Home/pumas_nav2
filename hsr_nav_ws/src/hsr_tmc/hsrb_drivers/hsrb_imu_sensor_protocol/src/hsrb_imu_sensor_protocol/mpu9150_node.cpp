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
#include <cstdlib>
#include <signal.h>
#include <time.h>
#include <limits>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/types.h>

#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>
#include <boost/type_traits.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "hsrb_imu_sensor_protocol/mpu9150_node.hpp"
#include "hsrb_imu_sensor_protocol/imu_protocol.hpp"
#include "hsrb_imu_sensor_protocol/mpu9150_network.hpp"
#include "hsrb_imu_sensor_protocol/mpu9150_protocol.hpp"

volatile bool g_running = true;

std::shared_ptr<MPU9150Node> g_mpu9150_node = std::make_shared<MPU9150Node>();

void QuitHandler(int sig);

MPU9150Node::MPU9150Node() {}

MPU9150Node::~MPU9150Node() {}

int MPU9150Node::Init(rclcpp::Node::SharedPtr node) {
  boost::system::error_code error;

  node_ = node;
  network_timeout_ = node_->declare_parameter<int32_t>("hw_config.network_timeout", kDefaultNetworkTimeout);
  network_tick_ = node_->declare_parameter<int32_t>("hw_config.network_tick", kDefaultNetworkTick);
  imu_network_timeout_ = node_->declare_parameter<int32_t>("hw_config.imu_network_timeout", kDefaultImuNetworkTimeout);
  imu_network_tick_ = node_->declare_parameter<int32_t>("hw_config.imu_network_tick", kDefaultImuNetworkTick);
  use_usb_ = node_->declare_parameter<bool>("hw_config.use_usb", kDefaultUSB485);
  is_realtime_ = node_->declare_parameter<bool>("hw_config.is_realtime", kDefaultIsRealtime);
  disable_imu_ = node_->declare_parameter<bool>("hw_config.disable_imu", false);
  cycle_time_ = node_->declare_parameter<double>("hw_config.cycle_time", kDefaultCycleTime);
  cycle_over_warn_tolerance_ =
      node_->declare_parameter<double>("hw_config.cycle_over_warn_tolerance", kDefaultCycleOverWarnTolerance);
  cycle_over_error_tolerance_ =
      node_->declare_parameter<double>("hw_config.cycle_over_error_tolerance", kDefaultCycleOverErrorTolerance);
  cycle_over_time_tolerance_ =
      node_->declare_parameter<double>("hw_config.cycle_over_time_tolerance", kDefaultCycleOverTimeTolerance);
  imu_device_ = node_->declare_parameter<std::string>("hw_config.imu_device", kDefaultImuDevice);
  imu_frame_id_ = node_->declare_parameter<std::string>("imu_frame_id", kDefaultImuFrameId);
  imu_publisher_ = node->create_publisher<sensor_msgs::msg::Imu>("base_imu", 1);

  if (!disable_imu_) {
    mpu9150_network_.reset(
        new hsrb_imu_sensor_protocol::MPU9150Network(imu_device_, error, imu_network_timeout_, imu_network_tick_));
    if (error) {
      RCLCPP_FATAL_STREAM(node_->get_logger(), "System cannot communicate with imu sensor. "
                                                   << "Error message: " << error.message());
      return EXIT_FAILURE;
    }

    mpu9150_protocol_.reset(new hsrb_imu_sensor_protocol::MPU9150Protocol(*mpu9150_network_));
    imu_calibrate_service_ = node_->create_service<std_srvs::srv::Empty>(
        "base_imu/calibrate",
        std::bind(&MPU9150Node::ResetImuCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
  return EXIT_SUCCESS;
}

boost::system::error_code MPU9150Node::TryWithRetry(boost::function<boost::system::error_code()> func, uint32_t retry) {
  boost::system::error_code error;
  for (uint32_t count = 0; count <= retry; ++count) {
    error = func();
    if (!error) {
      break;
    }
  }
  return error;
}
bool MPU9150Node::ResetImuCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> res) {
  boost::lock_guard<boost::mutex> lock(reset_imu_mutex_);
  reset_imu_ = true;
  return true;
}
int MPU9150Node::Configure() {
  if (mpu9150_protocol_) {
    boost::system::error_code error;
    error = TryWithRetry(
        boost::bind(&hsrb_imu_sensor_protocol::IImuProtocol::Reset, mpu9150_protocol_, kImuResetTimeout), kRetryCount);
    if (error) {
      RCLCPP_FATAL_STREAM(node_->get_logger(), "Failed to imu reset(" << error.message() << ")");
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}
void MPU9150Node::Write() {
  if (!disable_imu_) {
    if (reset_imu_mutex_.try_lock()) {
      if (reset_imu_) {
        hsrb_imu_sensor_protocol::IImuProtocol::ResetResult result;
        boost::system::error_code error = mpu9150_protocol_->TryReset(result);
        switch (result) {
          case hsrb_imu_sensor_protocol::IImuProtocol::kDone:
            reset_imu_ = false;
            break;
          case hsrb_imu_sensor_protocol::IImuProtocol::kError:
            reset_imu_ = false;
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to reset imu(" << error.message() << ")");
            break;
          default:
            break;
        }
      }
      reset_imu_mutex_.unlock();
    }
  }
}
void MPU9150Node::Read() {
  if (!disable_imu_) {
    boost::system::error_code error;
    error = mpu9150_protocol_->ReadState(imu_output_);
    if (error) {
      RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                   "Failed to read imu(" << error.message() << ")");
    }
  }
}
boost::system::error_code MPU9150Node::EnableRealtimeProcess() {
  boost::system::error_code error;
  struct rlimit limit = { 0 };

  if (getrlimit(RLIMIT_MEMLOCK, &limit) != 0) {
    error = boost::system::error_code(errno, boost::system::system_category());
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), error.message());
    std::cerr << "failed getrlimit" << std::endl;
    return error;
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "RLIMIT_MEMLOCK= " << limit.rlim_cur << " / " << limit.rlim_max);

  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "mlockall failed");
    error = boost::system::error_code(errno, boost::system::system_category());
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), error.message());
    return error;
  }

  if (getrlimit(RLIMIT_RTPRIO, &limit) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get resource limits.");
    error = boost::system::error_code(errno, boost::system::system_category());
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), error.message());
    return error;
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "RLIMIT_RTPRIO= " << limit.rlim_cur << " / " << limit.rlim_max);

  int max_priority = sched_get_priority_max(SCHED_FIFO);
  if (max_priority < 0) {
    error = boost::system::error_code(errno, boost::system::system_category());
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), error.message());
    return error;
  }

  sched_param param = { 0 };
  if (sched_getparam(0, &param) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get scheduler parameter.");
    error = boost::system::error_code(errno, boost::system::system_category());
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), error.message());
    return error;
  }
  param.sched_priority = 1;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set scheduler settings.");
    error = boost::system::error_code(errno, boost::system::system_category());
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), error.message());
    return error;
  }

  return boost::system::error_code(0, boost::system::system_category());
}
void MPU9150Node::Publish() {
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = node_->get_clock()->now();
  imu_msg.header.frame_id = imu_frame_id_;
  imu_msg.orientation.x = imu_output_.orientation[0];
  imu_msg.orientation.y = imu_output_.orientation[1];
  imu_msg.orientation.z = imu_output_.orientation[2];
  imu_msg.orientation.w = imu_output_.orientation[3];

  imu_msg.angular_velocity.x = imu_output_.angular_velocity[0];
  imu_msg.angular_velocity.y = imu_output_.angular_velocity[1];
  imu_msg.angular_velocity.z = imu_output_.angular_velocity[2];

  imu_msg.linear_acceleration.x = imu_output_.linear_acceleration[0];
  imu_msg.linear_acceleration.y = imu_output_.linear_acceleration[1];
  imu_msg.linear_acceleration.z = imu_output_.linear_acceleration[2];
  imu_publisher_->publish(imu_msg);

  return;
}
int MPU9150Node::RunLoopImpl(const rclcpp::Duration& cycle_time, bool check_cycle, bool enable_realtime) {
  struct sigaction action = { 0 };
  action.sa_handler = &QuitHandler;
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGINT, &action, NULL);
  sigaction(SIGHUP, &action, NULL);

  if (cycle_time.seconds() < std::numeric_limits<double>::min()) {
    RCLCPP_FATAL(node_->get_logger(), "cycle_time must be positive.");
    return EXIT_FAILURE;
  }
  if (cycle_time.seconds() <= 0) {
    RCLCPP_FATAL(node_->get_logger(), "cycle_time must be positive.");
    return EXIT_FAILURE;
  }
  if (cycle_over_warn_tolerance_ < std::numeric_limits<double>::min() || cycle_over_warn_tolerance_ >= 1.0) {
    RCLCPP_WARN(node_->get_logger(), "cycle_warn_tolerance must be in (0.0,1.0).");
  }
  if (cycle_over_error_tolerance_ < std::numeric_limits<double>::min() || cycle_over_error_tolerance_ >= 1.0) {
    RCLCPP_FATAL(node_->get_logger(), "cycle_error_tolerance must be in (0.0,1.0).");
    return EXIT_FAILURE;
  }
  if (cycle_over_time_tolerance_ < std::numeric_limits<double>::min()) {
    RCLCPP_FATAL(node_->get_logger(), "cycle_over_time_tolerance must be positive.");
    return EXIT_FAILURE;
  }

  if (enable_realtime) {
    if (EnableRealtimeProcess()) {
      return EXIT_FAILURE;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Started");
  rclcpp::Time last = node_->get_clock()->now();
  rclcpp::Time now = last;
  rclcpp::Time end;
  rclcpp::Rate rate(1.0 / cycle_time.seconds());
  rclcpp::Duration period = cycle_time;
  std::this_thread::sleep_for(std::chrono::nanoseconds(cycle_time.nanoseconds()));

  uint8_t cycle_count = 0;
  uint8_t cycle_over_count = 0;
  uint8_t buffer_position = 0;
  bool cycle_over_buffer[kCycleOverBufferNum] = { false };

  while (g_running) {
    now = node_->get_clock()->now();
    period = now - last;
    last = now;

    Read();
    Publish();
    Write();

    RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "time=" << now.seconds() << " period=" << period.seconds());
    end = node_->get_clock()->now();
    RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "elapsed[milliseconds]: " << (end - now).seconds() * 1e3);
    rclcpp::Rate cycleTime(1.0 / (end - now).seconds());

    if ((cycleTime.period().count() / 1e9) > ((rate.period().count() / 1e9) + cycle_over_time_tolerance_)) {
      if (!cycle_over_buffer[buffer_position]) {
        ++cycle_over_count;
      }
      cycle_over_buffer[buffer_position] = true;
    } else {
      if (cycle_over_buffer[buffer_position]) {
        --cycle_over_count;
      }
      cycle_over_buffer[buffer_position] = false;
    }
    ++cycle_count;
    ++buffer_position;
    if (buffer_position == kCycleOverBufferNum) {
      buffer_position = 0;
    }

    const double elapsed_time = cycle_count * (rate.period().count() / 1e9);
    if (elapsed_time > kCycleOverCheckInterval) {
      cycle_count = 0;
      const double cycle_over_rate = static_cast<double>(cycle_over_count) / kCycleOverBufferNum;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "cycle_over_rate = " << cycle_over_rate);
      if (check_cycle) {
        if (cycle_over_rate > cycle_over_error_tolerance_) {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "Cycle over rate exceeds " << cycle_over_error_tolerance_
                                                                              << ". Stopping realtime loop...");
          break;
        } else if (cycle_over_rate > cycle_over_warn_tolerance_) {
          RCLCPP_WARN_STREAM(node_->get_logger(), "Cycle over rate exceeds " << cycle_over_warn_tolerance_ << ".");
        }
      }
    }
    rclcpp::spin_some(node_);
    rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}

int MPU9150Node::RunRealtimeLoop() {
  return RunLoopImpl(rclcpp::Duration(std::chrono::nanoseconds(static_cast<int>(cycle_time_ * 1e9))), true, true);
}
int MPU9150Node::RunLoop(bool check_cycle) {
  return RunLoopImpl(rclcpp::Duration(std::chrono::nanoseconds(static_cast<int>(cycle_time_ * 1e9))), check_cycle,
                     false);
}
int MPU9150Node::Spin() {
  if (is_realtime_) {
    return RunRealtimeLoop();
  } else {
    return RunLoop(false);
  }
}
void QuitHandler(int sig) { g_running = false; }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("mpu9150_node");

  if (g_mpu9150_node->Init(node) == EXIT_FAILURE) {
    return 0;
  }
  if (g_mpu9150_node->Configure() == EXIT_FAILURE) {
    return 0;
  }
  g_mpu9150_node->Spin();
}
