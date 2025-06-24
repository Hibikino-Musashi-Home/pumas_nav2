/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
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
#include <limits>
#include <string>
#include <vector>
// <limits>はlong long int に対応していないのでboostを使う
// c++11になったら変更
#include <boost/limits.hpp>
// <round>はboostのものを使うがエラー時の挙動が面倒なので
// c++11になったら変更
#include <boost/math/special_functions/round.hpp>
#include <hsrb_servomotor_protocol/control_table_item_descriptor.hpp>

namespace {

// roundをエラーで返す
namespace policies = boost::math::policies;
typedef policies::policy<policies::rounding_error<policies::errno_on_error> > round_policy;

/// @attention バイトの長さチェックはしていないので，前処理でチェックすること
template <class T>
double DoubleCast(const uint8_t* bytes) {
  return static_cast<double>(*(reinterpret_cast<const T*>(&bytes[0])));
}

template <class T>
bool ByteCast(double value, std::vector<uint8_t>& bytes_out) {
  errno = 0;
  double round_value = boost::math::round(value, round_policy());
  if (errno == ERANGE) {
    return false;
  }

  if (round_value > std::numeric_limits<T>::max() ||
      round_value < -static_cast<double>(std::numeric_limits<T>::max())) {
    return false;
  } else {
    bytes_out.resize(sizeof(T));
    *(reinterpret_cast<T*>(&bytes_out[0])) = static_cast<double>(round_value);
    return true;
  }
}

template <>
bool ByteCast<float>(double value, std::vector<uint8_t>& bytes_out) {
  if (value > std::numeric_limits<float>::max() || value < -std::numeric_limits<float>::max()) {
    return false;
  } else {
    bytes_out.resize(sizeof(float));
    *(reinterpret_cast<float*>(&bytes_out[0])) = value;
    return true;
  }
}

template <>
bool ByteCast<double>(double value, std::vector<uint8_t>& bytes_out) {
  bytes_out.resize(sizeof(double));
  *(reinterpret_cast<double*>(&bytes_out[0])) = value;
  return true;
}
}  // anonymous namespace

namespace hsrb_servomotor_protocol {

ControlTableItemDescriptor::ControlTableItemDescriptor(ValueType type, uint16_t initial_address,
                                                       const std::string& attribute, double coefficient_mks)
    : type_(type), initial_address_(initial_address), attribute_(attribute), coefficient_mks_(coefficient_mks) {
  switch (type_) {
    case kUInt8:
    case kInt8:
      num_bytes_ = 1;
      break;
    case kUInt16:
    case kInt16:
      num_bytes_ = 2;
      break;
    case kUInt32:
    case kInt32:
    case kFloat:
      num_bytes_ = 4;
      break;
    case kUInt64:
    case kInt64:
    case kDouble:
      num_bytes_ = 8;
      break;
    default:
      assert(false);
      break;
  }
  final_address_ = initial_address_ + num_bytes_;
  if (coefficient_mks_ > std::numeric_limits<double>::min() || coefficient_mks_ < -std::numeric_limits<double>::min()) {
    coefficient_table_ = 1.0 / coefficient_mks_;
  } else {
    coefficient_table_ = 0.0;
  }
}

bool ControlTableItemDescriptor::ConvertToMKS(const std::vector<uint8_t>& bytes, double& mks_value_out) const {
  double non_mks_value = 0.0;
  if (bytes.size() != num_bytes_) {
    return false;
  }
  return ConvertToMKS(&bytes[0], mks_value_out);
}


bool ControlTableItemDescriptor::ConvertToMKS(const uint8_t* bytes_begin, double& mks_value_out) const {
  double non_mks_value = 0.0;
  switch (type_) {
    case kUInt8:
      non_mks_value = DoubleCast<uint8_t>(bytes_begin);
      break;
    case kInt8:
      non_mks_value = DoubleCast<int8_t>(bytes_begin);
      break;
    case kUInt16:
      non_mks_value = DoubleCast<uint16_t>(bytes_begin);
      break;
    case kInt16:
      non_mks_value = DoubleCast<int16_t>(bytes_begin);
      break;
    case kUInt32:
      non_mks_value = DoubleCast<uint32_t>(bytes_begin);
      break;
    case kInt32:
      non_mks_value = DoubleCast<int32_t>(bytes_begin);
      break;
    case kFloat:
      non_mks_value = DoubleCast<float>(bytes_begin);
      break;
    case kUInt64:
      non_mks_value = DoubleCast<uint64_t>(bytes_begin);
      break;
    case kInt64:
      non_mks_value = DoubleCast<int64_t>(bytes_begin);
      break;
    case kDouble:
      non_mks_value = DoubleCast<double>(bytes_begin);
      break;
    default:
      return false;
  }
  mks_value_out = non_mks_value / coefficient_mks_;
  return true;
}


bool ControlTableItemDescriptor::ConvertToBytes(double mks_value, std::vector<uint8_t>& bytes_out) const {
  double table_value = mks_value * coefficient_mks_;
  switch (type_) {
    case kUInt8:
      return ByteCast<uint8_t>(table_value, bytes_out);
    case kInt8:
      return ByteCast<int8_t>(table_value, bytes_out);
    case kUInt16:
      return ByteCast<uint16_t>(table_value, bytes_out);
    case kInt16:
      return ByteCast<int16_t>(table_value, bytes_out);
    case kUInt32:
      return ByteCast<uint32_t>(table_value, bytes_out);
    case kInt32:
      return ByteCast<int32_t>(table_value, bytes_out);
    case kFloat:
      return ByteCast<float>(table_value, bytes_out);
    case kUInt64:
      return ByteCast<uint64_t>(table_value, bytes_out);
    case kInt64:
      return ByteCast<int64_t>(table_value, bytes_out);
    case kDouble:
      return ByteCast<double>(table_value, bytes_out);
    default:
      assert(false);
  }
  return false;
}

}  // namespace hsrb_servomotor_protocol
