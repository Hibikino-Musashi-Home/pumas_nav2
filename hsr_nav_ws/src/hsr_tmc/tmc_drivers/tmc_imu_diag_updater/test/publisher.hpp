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
 * @file publisher.hpp
 * ＠BRIEF Providing classes to publish a periodic / or single -shot message
 *
 *
 */
#ifndef TEST_UTILS_PUBLISHER_HPP_
#define TEST_UTILS_PUBLISHER_HPP_
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "util_function.hpp"

namespace test_utils {

/**
 * @BRIEF one -shot / periodic publishing class
 *
 * @TPARAM MSGGEN Message generator
 *
 */
template <class MsgGen>
class Publisher {
 public:
  using SharedPtr = std::shared_ptr<Publisher>;

  /**
   * @BRIEF constructor
   *
   * @param NH node handle
   * @Param Topic_name The name of the message to be published
   * @param queue_size publisher queue size
   * @Param msg_gen Message generator reference count with count
   */
  Publisher(rclcpp::Node::SharedPtr node, const std::string& topic_name, const uint32_t queue_size,
            const typename MsgGen::SharedPtr& msg_gen)
      : node_(node),
        pub_(node_->create_publisher<typename MsgGen::MsgType>(topic_name.c_str(), queue_size)),
        msg_gen_(msg_gen) {
    if (topic_name.empty()) throw std::invalid_argument("Topic name is empty");
    if (queue_size == 0) throw std::invalid_argument("Queue size of publisher must be larger than 1");
    if (msg_gen_ == nullptr) throw std::invalid_argument("Message generator is empty");
  }

  /**
   * @BRIEF Destrist
   */
  virtual ~Publisher() = default;

  /**
   * @BRIEF Publish only once
   */
  void PublishOnce() {
    typename MsgGen::MsgType msg = msg_gen_->Generate();
    pub_->publish(msg);
  }


  /**
   * Issuing a message periodically
   *
   * @Param Rate_hz frequency (Hz)
   */
  void PublishPeriodically(const double rate_hz) {
    if (rate_hz < std::numeric_limits<double>::epsilon()) {
      throw std::invalid_argument("Input Hz is too small");
    }
    cyclic_publish_timer_ =
        node_->create_wall_timer(rclcpp::Duration::from_seconds(1.0 / rate_hz).to_chrono<std::chrono::nanoseconds>(),
                                 std::bind(&Publisher::TimerDrivenPublish, this));
    cyclic_publish_timer_->reset();
  }

  /**
   * Ends publishing
   */
  void StopPublishing() {
    cyclic_publish_timer_->cancel();
    pub_.reset();
  }

  /**
   * Check if there is any subscriber at the @bRIEF topic and return it
   *
   * Whether there is even one subscriber to the topic topic
   */
  bool IsSubscribed() const { return pub_->get_subscription_count() != 0; }

  /**
   * Check if there is any subscriber at the @bRIEF topic and return it (specified time, maximum waiting)
   *
   * @param Wait_sec Standby Time (SEC)
   *
   * Whether there is even one subscriber to the topic topic
   */
  bool IsSubscribed(const double wait_sec) const {
    return WaitUntil(
        node_, [&]() { return pub_->get_subscription_count() != 0; }, wait_sec);
  }

 private:
  void TimerDrivenPublish() { PublishOnce(); }

  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Publisher<typename MsgGen::MsgType>::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr cyclic_publish_timer_;

  const typename MsgGen::SharedPtr msg_gen_;
};

}  // end of namespace test_utils
#endif  // TEST_UTILS_PUBLISHER_HPP_
