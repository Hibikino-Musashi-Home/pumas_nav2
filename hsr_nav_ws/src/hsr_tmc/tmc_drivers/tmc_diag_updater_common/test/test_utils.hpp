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
#include <deque>
#include <limits>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace test_utils {

// Get a value corresponding to any key from the Values ​​property contained in Diag
double PickUpDiagValueOf(const std::string& key, const diagnostic_msgs::msg::DiagnosticStatus::_values_type& values) {
  for (const auto value : values) {
    if (key == value.key) {
      if (value.value == "nan") {
        return std::numeric_limits<double>::quiet_NaN();
      } else {
        return std::stod(value.value);
      }
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

// Wait until the given conditions formula is achieved
bool WaitUntil(rclcpp::Node::SharedPtr node, std::function<bool()>& condition_function, double timeout_sec = 5.0) {
  if (!condition_function) {
    throw std::runtime_error("Function for waiting is empty.");
  }

  const rclcpp::Time start_time = node->get_clock()->now();
  rclcpp::Rate rate(100.0);
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);
  while (node->get_clock()->now() - start_time < timeout) {
    rclcpp::spin_some(node);
    if (condition_function()) {
      return true;
    }
    rate.sleep();
  }
  return false;
}

// Wait until timeout
bool WaitUntilTimuout(rclcpp::Node::SharedPtr node, double timeout_sec = 5.0) {
  const rclcpp::Time start_time = node->get_clock()->now();
  rclcpp::Rate rate(100.0);
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);
  while (node->get_clock()->now() - start_time < timeout) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  return true;
}


class IDummySensorTime {
 public:
  using SharedPtr = std::shared_ptr<IDummySensorTime>;
  explicit IDummySensorTime(const rclcpp::Clock::SharedPtr clock) : clock_(clock) {
    base_time_ = clock_->now();
  }
  virtual ~IDummySensorTime() = default;

  virtual rclcpp::Time GetTime() const = 0;

 protected:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time base_time_;
};

class DummySensorTimeForward : public IDummySensorTime {
 public:
  explicit DummySensorTimeForward(const rclcpp::Clock::SharedPtr clock) : IDummySensorTime(clock) {}
  virtual ~DummySensorTimeForward() = default;

  rclcpp::Time GetTime() const override { return clock_->now(); }
};

class DummySensorTimeBackward : public IDummySensorTime {
 public:
  explicit DummySensorTimeBackward(const rclcpp::Clock::SharedPtr clock) : IDummySensorTime(clock) {}
  virtual ~DummySensorTimeBackward() = default;

  rclcpp::Time GetTime() const override { return base_time_ - (clock_->now() - base_time_); }
};

class DummySensorTimeFreezed : public IDummySensorTime {
 public:
  explicit DummySensorTimeFreezed(const rclcpp::Clock::SharedPtr clock) : IDummySensorTime(clock) {}
  virtual ~DummySensorTimeFreezed() = default;

  rclcpp::Time GetTime() const override { return base_time_; }
};


class CyclicImuPublisher {
 public:
  using SharedPtr = std::shared_ptr<CyclicImuPublisher>;

  explicit CyclicImuPublisher(rclcpp::Node::SharedPtr node) : node_(node), imu_msg_(sensor_msgs::msg::Imu()) {
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu", 1);
  }

  template <class DummySensorTime>
  void StartPublishing(const rclcpp::Clock::SharedPtr clock, const double rate_hz) {
    sensor_time_.reset(new DummySensorTime(clock));
    if (cyclic_publish_timer_ != nullptr) {
      cyclic_publish_timer_->cancel();
    }
    cyclic_publish_timer_ =
        node_->create_wall_timer(rclcpp::Duration::from_seconds(1.0 / rate_hz).to_chrono<std::chrono::nanoseconds>(),
                                 std::bind(&CyclicImuPublisher::PublishImu, this));
    cyclic_publish_timer_->reset();
  }

  void StopPublishing() { cyclic_publish_timer_->cancel(); }

  bool ImuIsSubscribed() const { return imu_pub_->get_subscription_count() != 0; }

  void set_imu_msg(const sensor_msgs::msg::Imu& imu_msg) { imu_msg_ = imu_msg; }

 private:
  void PublishImu() {
    imu_msg_.header.stamp = sensor_time_->GetTime();
    imu_pub_->publish(imu_msg_);
  }

  rclcpp::Node::SharedPtr node_;

  sensor_msgs::msg::Imu imu_msg_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr cyclic_publish_timer_;

  IDummySensorTime::SharedPtr sensor_time_;
};

// Message_filters :: Cache is not used because it cannot be reached to itchy.
class DiagCacheSubscriber {
 public:
  using SharedPtr = std::shared_ptr<DiagCacheSubscriber>;

  explicit DiagCacheSubscriber(rclcpp::Node::SharedPtr node) : node_(node) {}

  void StartCaching() {
    diag_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 1, std::bind(&DiagCacheSubscriber::CacheDiagMessage, this, std::placeholders::_1));
  }

  void StopCaching() { diag_sub_ = nullptr; }

  void ClearCache() { std::deque<diagnostic_msgs::msg::DiagnosticArray>().swap(cache_); }

  bool IsDiagMessageAvailable() const { return node_->get_publishers_info_by_topic("/diagnostics", false).size() != 0; }
  int GetQueueLength() const { return static_cast<int>(cache_.size()); }
  diagnostic_msgs::msg::DiagnosticArray GetLatestMessage() const { return cache_.back(); }

 private:
  void CacheDiagMessage(const diagnostic_msgs::msg::DiagnosticArray& msg) { cache_.push_back(msg); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  std::deque<diagnostic_msgs::msg::DiagnosticArray> cache_;
};

}  // namespace test_utils
