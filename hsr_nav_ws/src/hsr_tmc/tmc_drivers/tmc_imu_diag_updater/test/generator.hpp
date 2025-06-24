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
/**
 * @file generator.hpp
 * Providing a class to generate messages
 * @note If necessary, derive iGenerator
 *
 *
 */
#ifndef TEST_UTILS_GENERATOR_HPP_
#define TEST_UTILS_GENERATOR_HPP_
#include <memory>
#include <boost/optional.hpp>
#include <rclcpp/rclcpp.hpp>

namespace test_utils {

/**
 * Definition of interface for generating data
 *
 * @tparam T Some type
 */
template <class T>
class IGenerator {
 public:
  using SharedPtr = std::shared_ptr<IGenerator>;
  // Another name for Type_traits
  using MsgType = T;
  virtual ~IGenerator() = default;

  virtual T Generate() = 0;
};

/**
 * Blief generate fixed data
 *
 * The value was given at the time of initialization
 *
 * @tparam T Some type
 */
template <class T>
class FixedGenerator : public IGenerator<T> {
 public:
  /**
   * @bRIEF setting of initial values ​​(continued to be used)
   *
   * @Param Data initial value
   */
  explicit FixedGenerator(const T& data) : data_(data) {}

  virtual ~FixedGenerator() = default;

  /**
   * acquisition of @bRIEF data (fixed)
   *
   * Data set at the time of initialization
   */
  T Generate() override { return data_; }

 private:
  const T data_;
};

/**
 * generate STAMP according to the flow of time
 *
 * RCLCPP :: Time :: Now at the time of acquisition
 */
class ForwardStampGenerator : public IGenerator<rclcpp::Time> {
 public:
  ForwardStampGenerator() = default;
  virtual ~ForwardStampGenerator() = default;

  /**
   * acquisition of @brew time (direction)
   *
   * @return RCLCPP type time
   */
  rclcpp::Time Generate() override { return rclcpp::Clock(RCL_ROS_TIME).now(); }
};

/**
 * Birthburief generates a retrospective STAMP
 */
class BackwardStampGenerator : public IGenerator<rclcpp::Time> {
 public:
  BackwardStampGenerator() = default;
  virtual ~BackwardStampGenerator() = default;

  /**
   * acquisition of @brew time (reverse direction)
   *
   * @return RCLCPP type time
   */
  rclcpp::Time Generate() override {
    if (base_time_ == boost::none) {
      base_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
    }
    return base_time_.get() - (rclcpp::Clock(RCL_ROS_TIME).now() - base_time_.get());
  }

 private:
  boost::optional<rclcpp::Time> base_time_;
};


/**
 * Classes that generate and provide STAMP along with @bRIEF messages
 *
 * @tparam m rclcpp message type
 *
 * Must be in the following property structure
 *
 * msg.header.stamp
 */
template <class M>
class StampAndMessageGenerator : public IGenerator<M> {
 public:
  /**
   * @BRIEF STAMP Registration of generator
   *
   * RCLCPP message with @Param msg_gen header.stamp
   * @Param STAMP_GEN RCLCPP :: Time type Generator (default is forwardstampgenerator)
   */
  StampAndMessageGenerator(const typename IGenerator<M>::SharedPtr msg_gen,
                           const IGenerator<rclcpp::Time>::SharedPtr& stamp_gen = new ForwardStampGenerator())
      : msg_gen_(msg_gen), stamp_gen_(stamp_gen) {
    if (msg_gen_ == nullptr) {
      throw std::invalid_argument("Message generator is empty");
    }
    if (stamp_gen_ == nullptr) {
      throw std::invalid_argument("Stamp generator is empty");
    }
  }
  virtual ~StampAndMessageGenerator() = default;

  /**
   * Begenerate @brew messages and give STAMP to it
   *
   * @return RCLCPP message
   */
  M Generate() override {
    M msg = msg_gen_->Generate();
    msg.header.stamp = stamp_gen_->Generate();
    return msg;
  }

 protected:
  const typename IGenerator<M>::SharedPtr msg_gen_;
  const IGenerator<rclcpp::Time>::SharedPtr stamp_gen_;
};

}  // end of namespace test_utils
#endif  // TEST_UTILS_GENERATOR_HPP_
