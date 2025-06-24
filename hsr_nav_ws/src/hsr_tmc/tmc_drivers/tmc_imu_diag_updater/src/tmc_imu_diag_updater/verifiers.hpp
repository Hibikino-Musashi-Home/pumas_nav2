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
#ifndef TMC_IMU_DIAG_UPDATER_VERIFIERS_HPP_
#define TMC_IMU_DIAG_UPDATER_VERIFIERS_HPP_

#include <cmath>
#include <string>

#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tmc_diag_updater_common/verifiers.hpp>

namespace tmc_imu_diag_updater {

// Vertical namespace
namespace verifier {

using tmc_diag_updater_common::verifier::Interface;

// Class to verify whether the angle speed / acceleration value is all 0
// It was needed because Adi_driver did not publish a very valid value
class ZeroVelocityAndAcceleration : public Interface<sensor_msgs::msg::Imu::Ptr> {
 public:
  static Interface<sensor_msgs::msg::Imu::Ptr>::SharedPtr Create(rclcpp::Node::SharedPtr node) {
    const double significant_threshold =
        node->declare_parameter<double>("verifiers.zero_velocity_and_acceleration.significant_threshold", 0.00001);
    return Interface<sensor_msgs::msg::Imu::Ptr>::SharedPtr(new ZeroVelocityAndAcceleration(significant_threshold));
  }

  explicit ZeroVelocityAndAcceleration(double significant_threshold) : significant_threshold_(significant_threshold) {
    if (significant_threshold_ <= 0.0) {
      throw std::invalid_argument("significant_threshold should be positive");
    }
  }

  virtual void UpdateSummary(const sensor_msgs::msg::Imu::Ptr& stamped_msg) {
    // If you can't find the value, judge that it's not the job of this class and do not update it.
    if (stamped_msg == nullptr) {
      return;
    }

    actual_angular_velocity_ = stamped_msg->angular_velocity;
    actual_linear_acceleration_ = stamped_msg->linear_acceleration;
    std::array<double, 6> iteratable{ actual_angular_velocity_.x,    actual_angular_velocity_.y,
                                      actual_angular_velocity_.z,    actual_linear_acceleration_.x,
                                      actual_linear_acceleration_.y, actual_linear_acceleration_.z };

    // Judgment is abnormal if all elements are within the specified value area
    for (auto value : iteratable) {
      if (value < -significant_threshold_ || significant_threshold_ < value) {
        this->SetAsOK();
        return;
      }
    }

    // At this point, judge that it does not contain a significant value
    this->SetAs(DiagnosticStatus::ERROR, "Velocities and accelerations are zero");
  }

  virtual void AddValues(diagnostic_updater::DiagnosticStatusWrapper& stat) const {
    stat.add("Actual Angular Velocity X [rad/s]", actual_angular_velocity_.x);
    stat.add("Actual Angular Velocity Y [rad/s]", actual_angular_velocity_.y);
    stat.add("Actual Angular Velocity Z [rad/s]", actual_angular_velocity_.z);
    stat.add("Actual Linear Acceleration X [m/s^2]", actual_linear_acceleration_.x);
    stat.add("Actual Linear Acceleration Y [m/s^2]", actual_linear_acceleration_.y);
    stat.add("Actual Linear Acceleration Z [m/s^2]", actual_linear_acceleration_.z);
  }

 private:
  const double significant_threshold_;

  geometry_msgs::msg::Vector3 actual_angular_velocity_;
  geometry_msgs::msg::Vector3 actual_linear_acceleration_;
};

// Class to verify that the same value is continuous
// It was needed because the value of Adi_driver hardens (when USB connection is cut)
// All property values ​​can harden, or in some cases
// (I don't know which property value will solidify, but there was such a phenomenon)
class ContiguousSameValue : public Interface<sensor_msgs::msg::Imu::Ptr> {
 public:
  static Interface<sensor_msgs::msg::Imu::Ptr>::SharedPtr Create(rclcpp::Node::SharedPtr node) {
    const int properties_num = node->declare_parameter<int>("verifiers.contiguous_same_value.properties_num", 3);
    const int contiguous_threshold =
        node->declare_parameter<int>("verifiers.contiguous_same_value.contiguous_threshold", 5);
    return Interface<sensor_msgs::msg::Imu::Ptr>::SharedPtr(
        new ContiguousSameValue(properties_num, contiguous_threshold));
  }

  // If the specified number of properties harden in a continuous specified number of times, the error should be made as an error.
  // -Secice of the number of properties: I don't know which property will solidify, so I decided to do it by number
  // -Secice of continuous number: The probability that the same value is continuous will not be 0 even if it is normal, so we can set it.
  //                       Set the parameter for that
  //                       Since the confirmation cycle is usually set to exceed the receiving cycle, there is a possibility that the same value will be confirmed multiple times in a row.
  //                       Therefore, it is recommended that you set this parameter sufficiently.
  ContiguousSameValue(const uint32_t property_num_threshold, const uint32_t contiguous_count_threshold)
      : property_num_threshold_(property_num_threshold),
        contiguous_count_threshold_(contiguous_count_threshold),
        contiguous_count_(0),
        same_property_counter_(SamePropertyCounter()) {
    if (property_num_threshold_ > SamePropertyCounter::kPropertyNum) {
      throw std::invalid_argument("significant_threshold must be larger than number of unique property of IMU");
    }
    if (contiguous_count_threshold_ <= 0) {
      throw std::invalid_argument("contiguous_threshold must be larger than 0");
    }
  }

  virtual void UpdateSummary(const sensor_msgs::msg::Imu::Ptr& stamped_msg) {
    if (stamped_msg == nullptr) {
      return;
    }

    auto same_property_num = same_property_counter_.CalcSamePropertyNum(*stamped_msg);
    if (same_property_num == std::nullopt) {
      return;
    }

    if (same_property_num.value() >= property_num_threshold_) {
      ++contiguous_count_;
    } else {
      contiguous_count_ = 0;
    }

    if (contiguous_count_ >= contiguous_count_threshold_) {
      this->SetAs(DiagnosticStatus::ERROR, "Contiguous same value");
      return;
    }

    this->SetAsOK();
  }

  virtual void AddValues(diagnostic_updater::DiagnosticStatusWrapper& stat) const {
    stat.add("Contiguous count", contiguous_count_);
  }

 private:
  class SamePropertyCounter {
   public:
    SamePropertyCounter() = default;
    virtual ~SamePropertyCounter() = default;

    static const int kPropertyNum = 6;

    std::optional<uint32_t> CalcSamePropertyNum(const sensor_msgs::msg::Imu& msg) {
      // If there is no last price, return the invalid value
      if (prev_msg_ == std::nullopt) {
        prev_msg_ = msg;
        return std::nullopt;
      }

      // Make an iterator to handle with for statement
      auto v = prev_msg_.value().angular_velocity;
      auto a = prev_msg_.value().linear_acceleration;
      const std::array<std::array<double, 2>, kPropertyNum> iteratable = { { { v.x, msg.angular_velocity.x },
                                                                             { v.y, msg.angular_velocity.y },
                                                                             { v.z, msg.angular_velocity.z },
                                                                             { a.x, msg.linear_acceleration.x },
                                                                             { a.y, msg.linear_acceleration.y },
                                                                             { a.z, msg.linear_acceleration.z } } };

      uint32_t same_property_num = 0;
      for (auto v : iteratable) {
        if (v[0] == v[1]) {
          ++same_property_num;
        }
      }
      prev_msg_ = msg;
      return same_property_num;
    }

   private:
    // Previous value for comparison
    std::optional<sensor_msgs::msg::Imu> prev_msg_;
  };

  const uint32_t property_num_threshold_;
  const uint32_t contiguous_count_threshold_;
  uint32_t contiguous_count_;
  SamePropertyCounter same_property_counter_;
};

// Class to verify whether the acceleration Norm at the time of start is within the threshold
// It was needed because Adi_driver did not publish a very valid value
class InitialAccelerationNorm : public Interface<sensor_msgs::msg::Imu::Ptr> {
 public:
  static Interface<sensor_msgs::msg::Imu::Ptr>::SharedPtr Create(rclcpp::Node::SharedPtr node) {
    const int sample_size = node->declare_parameter<int>("verifiers.initial_acceleration_norm.sample_size", 100);
    const double acc_norm_min =
      node->declare_parameter<double>("verifiers.initial_acceleration_norm.acc_norm_min", 9.5);
    const double acc_norm_max =
      node->declare_parameter<double>("verifiers.initial_acceleration_norm.acc_norm_max", 11.5);
    return Interface<sensor_msgs::msg::Imu::Ptr>::SharedPtr(
      new InitialAccelerationNorm(sample_size, acc_norm_min, acc_norm_max, node));
  }

  explicit InitialAccelerationNorm(int sample_size, double acc_norm_min, double acc_norm_max,
                                   rclcpp::Node::SharedPtr node)
    : sample_size_(sample_size),
      acc_norm_min_(acc_norm_min),
      acc_norm_max_(acc_norm_max),
      sample_count_(0),
      node_(node) {
    if (sample_size_ <= 0) {
      throw std::invalid_argument("sample size should be positive");
    }
    if (acc_norm_min_ <= 0.0 || acc_norm_max_ <= 0.0) {
      throw std::invalid_argument("norm should be positive");
    }

    reset_count_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "reset_sample_count", 1, std::bind(&InitialAccelerationNorm::ResetSampleCount, this, std::placeholders::_1));
  }

  void ResetSampleCount(std_msgs::msg::Empty::SharedPtr msg) { sample_count_ = 0; }

  virtual void UpdateSummary(const sensor_msgs::msg::Imu::Ptr& stamped_msg) {
    // If you can't find the value, judge that it's not the job of this class and do not update it.
    if (stamped_msg == nullptr) {
      return;
    }
    // If the number of verified topics reaches Sample Size, will not be updated
    if (sample_size_ <= sample_count_) {
      return;
    }
    sample_count_++;
    // If the acceleration Norm exceeds the threshold, it is judged as abnormal
    actual_linear_acceleration_ = stamped_msg->linear_acceleration;
    double acc_norm = sqrt(pow(actual_linear_acceleration_.x, 2.0) + pow(actual_linear_acceleration_.y, 2.0) +
                           pow(actual_linear_acceleration_.z, 2.0));
    if (acc_norm_min_ <= acc_norm && acc_norm <= acc_norm_max_) {
      this->SetAsOK();
    } else {
      this->SetAs(DiagnosticStatus::ERROR, "Initial Acceleration Norm is invalid");
    }
  }

  virtual void AddValues(diagnostic_updater::DiagnosticStatusWrapper& stat) const {
    stat.add("Actual Linear Acceleration X [m/s^2]", actual_linear_acceleration_.x);
    stat.add("Actual Linear Acceleration Y [m/s^2]", actual_linear_acceleration_.y);
    stat.add("Actual Linear Acceleration Z [m/s^2]", actual_linear_acceleration_.z);
  }

 private:
  const int sample_size_;
  const double acc_norm_min_;
  const double acc_norm_max_;

  int sample_count_;
  geometry_msgs::msg::Vector3 actual_linear_acceleration_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_count_sub_;
};

// Make an IMU verification
// If you create a new interface derivation, add a branch here
Interface<sensor_msgs::msg::Imu::Ptr>::SharedPtr Create(std::string type, rclcpp::Node::SharedPtr node) {
  typename Interface<sensor_msgs::msg::Imu::Ptr>::SharedPtr ptr;
  if (type == "zero_velocity_and_acceleration") {
    ptr = ZeroVelocityAndAcceleration::Create(node);
  } else if (type == "contiguous_same_value") {
    ptr = ContiguousSameValue::Create(node);
  } else if (type == "initial_acceleration_norm") {
    ptr = InitialAccelerationNorm::Create(node);
  } else {
    ptr = tmc_diag_updater_common::verifier::Create<sensor_msgs::msg::Imu::Ptr>(type, node);
  }
  return ptr;
}
}  // namespace verifier
}  // namespace tmc_imu_diag_updater
#endif  // TMC_IMU_DIAG_UPDATER_VERIFIERS_HPP_
