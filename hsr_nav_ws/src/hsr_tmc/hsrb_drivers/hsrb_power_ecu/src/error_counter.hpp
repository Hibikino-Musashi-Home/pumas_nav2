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
#ifndef HSRB_POWER_ECU_ERROR_COUNTER_HPP_
#define HSRB_POWER_ECU_ERROR_COUNTER_HPP_

#include <stdint.h>
#include <deque>

#include <boost/noncopyable.hpp>

namespace hsrb_power_ecu {
/**
 * Class to calculate the error rate
 */
class ErrorCounter : boost::noncopyable {
 public:
  /**
   * @brief Constructor
   *
   * @param [in] buffer_size Buffer size used to aggregate errate
   */
  explicit ErrorCounter(uint32_t buffer_size) : error_count_(0), buffer_size_(buffer_size) {
    buffer_.resize(buffer_size, true);
  }
  virtual ~ErrorCounter() {}
  /**
   * @brief Data registration
   *
   * @param [in] result Registered data Ture succeeded
   */
  void Register(bool result) {
    if (!result) {
      error_count_ += 1;
    }

    if (buffer_.size() == buffer_size_) {
      if (!buffer_.front()) {
        error_count_ -= 1;
      }
      buffer_.pop_front();
    }

    buffer_.push_back(result);
  }

  /**
   * @brief Obtain the total number of errors
   */
  uint32_t GetErrorCount() const { return error_count_; }

  /**
   * @brief Error rate
   */
  double GetErrorRate() const { return static_cast<double>(GetErrorCount()) / static_cast<double>(buffer_.size()); }

 private:
  std::deque<bool> buffer_;
  uint32_t error_count_;
  size_t buffer_size_;
};
}  // namespace hsrb_power_ecu
#endif  // HSRB_POWER_ECU_ERROR_COUNTER_HPP_
