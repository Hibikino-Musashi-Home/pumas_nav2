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
#ifndef TMC_DIAG_UPDATER_COMMON_TMC_DIAG_UPDATER_COMMON_DIAG_UPDATER_HPP_
#define TMC_DIAG_UPDATER_COMMON_TMC_DIAG_UPDATER_COMMON_DIAG_UPDATER_HPP_
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include "verifiers.hpp"

namespace tmc_diag_updater_common {

// Classes that make topics subscription, verification, and update diags
//
// Topic reception events and verification events are independent
// Because it was necessary to look down on the communication quality of the topic.
template <class MsgPtr, class MsgType>
class DiagUpdater {
 public:
  using SharedPtr = std::shared_ptr<DiagUpdater>;

  explicit DiagUpdater(rclcpp::Node::SharedPtr node) : DiagUpdater(node, verifier::Create<MsgPtr>) {}

  explicit DiagUpdater(
      rclcpp::Node::SharedPtr node,
      std::function<typename verifier::Interface<MsgPtr>::SharedPtr(std::string, rclcpp::Node::SharedPtr)> creator_func)
      : node_(node) {
    // Diagupdater settings
    std::string hardware_id;
    if (node_->has_parameter("hardware_id")) {
      hardware_id = node_->get_parameter("hardware_id").get_value<std::string>();
    } else {
      hardware_id = node_->declare_parameter("hardware_id", "data");
    }

    diag_updater_.reset(new diagnostic_updater::Updater(node_));
    diag_updater_->setHardwareID(hardware_id);
    diag_updater_->add(hardware_id + " topic status", this, &DiagUpdater::UpdateDiag);

    // Creation of verifications
    // Perth with Rosparam to generate multiple verification machines
    std::vector<std::string> verifiers_list =
        node_->declare_parameter<std::vector<std::string>>("verifiers_list", std::vector<std::string>({}));
    if (verifiers_list.size() == 0) {
      throw std::invalid_argument("Failed to get parameters for verifiers_list");
    }

    for (const auto& verifier : verifiers_list) {
      verifiers_.push_back(creator_func(verifier, node_));
    }
    verifier_summaries_.resize(verifiers_list.size());

    // It seems good to have the topic name and HardwareID together
    sub_ = node_->create_subscription<MsgType>(hardware_id, rclcpp::SensorDataQoS(),
                                               std::bind(&DiagUpdater::UpdateTopic, this, std::placeholders::_1));

    // Set a verification event for the timer
    const double sampling_hz = node_->declare_parameter<double>("sampling_hz", 200.0);
    if (sampling_hz <= std::numeric_limits<double>::epsilon()) {
      throw std::runtime_error("Specify positive value as sampling Hz");
    }

    cyclic_publish_timer_ = node_->create_wall_timer(
      rclcpp::Duration::from_seconds(1.0 / sampling_hz).to_chrono<std::chrono::nanoseconds>(),
      std::bind(&DiagUpdater::VerificationEvent, this));
    cyclic_publish_timer_->reset();

    RCLCPP_INFO(node_->get_logger(), "%s diag updater is on duty", hardware_id.c_str());
  }

 private:
  void UpdateTopic(const MsgPtr data) { data_ = data; }

  void UpdateDiag(diagnostic_updater::DiagnosticStatusWrapper& dst_stat) {
    Summary initial_summary;
    verifier::InitSummary(initial_summary);
    dst_stat.summary(initial_summary.first, initial_summary.second);

    for (auto verifier_summary : verifier_summaries_) {
      // ERROR> Warn> OK is the level priority, the message is in the registration order
      dst_stat.mergeSummary(verifier_summary.first, verifier_summary.second);
    }
    // Added verification result details, etc.
    for (auto verifier : verifiers_) {
      verifier->AddValues(dst_stat);
    }
  }

  void VerificationEvent() {
    for (int i = 0; i < verifier_summaries_.size(); ++i) {
      verifier_summaries_[i] = verifiers_[i]->Verify(data_);
    }
    data_.reset();
    diag_updater_->force_update();
  }

  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
  typename rclcpp::Subscription<MsgType>::SharedPtr sub_;

  // Verification data buffer
  MsgPtr data_;

  // Topic data validation and verification results
  std::vector<typename verifier::Interface<MsgPtr>::SharedPtr> verifiers_;
  Summaries verifier_summaries_;

  // DIAG update regular event issuance
  rclcpp::TimerBase::SharedPtr cyclic_publish_timer_;
  rclcpp::Node::SharedPtr node_;
};
}  // namespace tmc_diag_updater_common
#endif
