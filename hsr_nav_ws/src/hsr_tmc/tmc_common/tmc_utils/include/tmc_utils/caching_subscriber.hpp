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
#ifndef TMC_UTILS_CACHING_SUBSCRIBER_HPP_
#define TMC_UTILS_CACHING_SUBSCRIBER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace tmc_utils {

template<typename MsgType>
class CachingSubscriber {
 public:
  using Ptr = std::shared_ptr<CachingSubscriber>;

  CachingSubscriber(const rclcpp::Node::SharedPtr& node, const std::string& topic_name, const rclcpp::QoS& qos) {
    sub_ = node->create_subscription<MsgType>(
        topic_name, qos, std::bind(&CachingSubscriber::Callback, this, std::placeholders::_1));
  }
  CachingSubscriber(const rclcpp::Node::SharedPtr& node, const std::string& topic_name)
      : CachingSubscriber(node, topic_name, 1) {}
  virtual ~CachingSubscriber() = default;

  bool IsSubscribed() const { return msg_ != nullptr; }
  MsgType GetValue() const { return *msg_; }

 private:
  void Callback(const typename MsgType::SharedPtr msg) { msg_ = msg; }
  typename rclcpp::Subscription<MsgType>::SharedPtr sub_;
  typename MsgType::SharedPtr msg_;
};

}  // namespace tmc_utils
#endif  // TMC_UTILS_CACHING_SUBSCRIBER_HPP_
