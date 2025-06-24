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
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <gtest/gtest.h>

#include "../src/power_ecu_com_element_encoder.hpp"
#include "common_methods.hpp"
namespace {
const size_t kTestValueLengs = 2;

size_t getDigit(const int32_t value, const int32_t base) { return static_cast<size_t>((log(value) / log(base)) + 1); }

template <typename T>
void HexUintEncoderTestHelper(const size_t digits, const boost::array<T, kTestValueLengs>& test_values,
                              const boost::array<std::string, kTestValueLengs>& test_results) {
  uint32_t base = 16;
  T value;
  hsrb_power_ecu::ElementHexUintEncoder<T> encoder(value, digits);
  hsrb_power_ecu::PacketBuffer buffer(1000);

  for (size_t i = 0; i < kTestValueLengs; ++i) {
    std::stringstream sst;
    sst << "nomal value = " << test_values[i] << ", result = " << test_results[i];
    SCOPED_TRACE(sst.str().c_str());
    buffer.clear();
    value = test_values[i];
    EXPECT_TRUE(encoder.Encode(buffer)) << "正常";
    EXPECT_STREQ(read_buffer(buffer).c_str(), test_results[i].c_str());
  }

  {
    uint64_t bad_value = std::pow(base, digits) + 1;
    if (bad_value > std::numeric_limits<T>::max()) {
      // Do not test if the range that can be expressed in the packet exceeds the original value
      return;
    }
    value = static_cast<T>(bad_value);
    std::stringstream sst;
    sst << "bad value = " << value << ", result = \"\"";
    SCOPED_TRACE(sst.str().c_str());

    buffer.clear();
    EXPECT_FALSE(encoder.Encode(buffer));
    EXPECT_STREQ(read_buffer(buffer).c_str(), "");
  }
}

void ElementHexUintBitsEncoderTestHelper(const size_t digits, const boost::array<std::string, 3>& results) {
  hsrb_power_ecu::ElementHexUintBitsEncoder encoder(digits);
  size_t max_digits = (digits * 4) - 1;
  hsrb_power_ecu::PacketBuffer buffer(1000);

  bool min_value;
  bool max_value;
  EXPECT_TRUE(encoder.RegisterBit(0, &min_value));
  EXPECT_TRUE(encoder.RegisterBit(max_digits, &max_value));

  {
    min_value = false;
    max_value = false;
    buffer.clear();
    EXPECT_TRUE(encoder.Encode(buffer));
    EXPECT_STREQ(read_buffer(buffer).c_str(), results[0].c_str());
  }
  {
    min_value = true;
    max_value = false;
    buffer.clear();
    EXPECT_TRUE(encoder.Encode(buffer));
    EXPECT_STREQ(read_buffer(buffer).c_str(), results[1].c_str());
  }
  {
    min_value = false;
    max_value = true;
    buffer.clear();
    EXPECT_TRUE(encoder.Encode(buffer));
    EXPECT_STREQ(read_buffer(buffer).c_str(), results[2].c_str());
  }
}

template <typename T>
void UintEncoderTestHelper(const size_t digits, const boost::array<T, kTestValueLengs>& test_values,
                           const boost::array<std::string, kTestValueLengs>& test_results) {
  uint32_t base = 16;
  T value;
  hsrb_power_ecu::ElementUintEncoder<T> encoder(value, digits);
  hsrb_power_ecu::PacketBuffer buffer(1000);

  for (size_t i = 0; i < kTestValueLengs; ++i) {
    std::stringstream sst;
    sst << "nomal value = " << test_values[i] << ", result = " << test_results[i];
    SCOPED_TRACE(sst.str().c_str());
    buffer.clear();
    value = test_values[i];
    EXPECT_TRUE(encoder.Encode(buffer)) << "正常";
    EXPECT_STREQ(read_buffer(buffer).c_str(), test_results[i].c_str());
  }

  {
    uint64_t bad_value = std::pow(base, digits) + 1;
    if (bad_value > std::numeric_limits<T>::max()) {
      // Do not test if the range that can be expressed in the packet exceeds the original value
      return;
    }
    value = static_cast<T>(bad_value);
    std::stringstream sst;
    sst << "bad value = " << value << ", result = \"\"";
    SCOPED_TRACE(sst.str().c_str());

    buffer.clear();
    EXPECT_FALSE(encoder.Encode(buffer));
    EXPECT_STREQ(read_buffer(buffer).c_str(), "");
  }
}

void StringEncoderTestHelper(const size_t digits, const boost::array<std::string, kTestValueLengs>& test_values) {
  std::string value;
  hsrb_power_ecu::ElementStringEncoder encoder(value, digits);
  hsrb_power_ecu::PacketBuffer buffer(1000);

  BOOST_FOREACH (const std::string& str, test_values) {
    SCOPED_TRACE("nomal vlaue=" + str);
    buffer.clear();
    value = str;
    EXPECT_TRUE(encoder.Encode(buffer));
    EXPECT_STREQ(read_buffer(buffer).c_str(), str.c_str());
  }
  {
    std::string bad_value = "";
    for (size_t i = 0; i < digits + 1; ++i) {
      bad_value += "a";
    }
    value = bad_value;
    buffer.clear();
    EXPECT_FALSE(encoder.Encode(buffer));
    EXPECT_STREQ(read_buffer(buffer).c_str(), "");
  }
  {
    std::string bad_value = "";
    for (size_t i = 0; i < digits - 1; ++i) {
      bad_value += "a";
    }
    value = bad_value;
    buffer.clear();
    EXPECT_FALSE(encoder.Encode(buffer));
    EXPECT_STREQ(read_buffer(buffer).c_str(), "");
  }
}

}  // anonymous namespace

/* Normal test */
TEST(ElementHexUintEncoderTest, NomalCase) {
  {
    SCOPED_TRACE("type=uint32_t digits=8");
    size_t digits = 8;
    boost::array<uint32_t, kTestValueLengs> test_values = {1, 0xFFFFFFFF};
    boost::array<std::string, kTestValueLengs> test_results = {"h00000001", "hFFFFFFFF"};
    HexUintEncoderTestHelper(digits, test_values, test_results);
  }
  {
    SCOPED_TRACE("type=uint8_t digits=8");
    size_t digits = 8;
    boost::array<uint8_t, kTestValueLengs> test_values = {1, 0xFF};
    boost::array<std::string, kTestValueLengs> test_results = {"h00000001", "h000000FF"};
    HexUintEncoderTestHelper(digits, test_values, test_results);
  }
  {
    SCOPED_TRACE("type=uint32_t digits=2");
    size_t digits = 2;
    boost::array<uint32_t, kTestValueLengs> test_values = {1, 0xFF};
    boost::array<std::string, kTestValueLengs> test_results = {"h01", "hFF"};
    HexUintEncoderTestHelper(digits, test_values, test_results);
  }
  {
    SCOPED_TRACE("type=uint8_t digits=2");
    size_t digits = 2;
    boost::array<uint8_t, kTestValueLengs> test_values = {1, 0xFF};
    boost::array<std::string, kTestValueLengs> test_results = {"h01", "hFF"};
    HexUintEncoderTestHelper(digits, test_values, test_results);
  }
  {
    SCOPED_TRACE("type=int32_t digits=2");
    size_t digits = 2;
    boost::array<int32_t, kTestValueLengs> test_values = {1, 0xFF};
    boost::array<std::string, kTestValueLengs> test_results = {"h01", "hFF"};
    HexUintEncoderTestHelper(digits, test_values, test_results);
  }
  {
    SCOPED_TRACE("type=int8_t digits=2");
    size_t digits = 2;
    boost::array<int8_t, kTestValueLengs> test_values = {1, 0x10};
    boost::array<std::string, kTestValueLengs> test_results = {"h01", "h10"};
    HexUintEncoderTestHelper(digits, test_values, test_results);
  }
}

TEST(ElementHexUintBitsEncoderTest, NomalCase) {
  {
    SCOPED_TRACE("digits=8");
    size_t digits = 8;
    boost::array<std::string, 3> results = {"h00000000", "h00000001", "h80000000"};
    ElementHexUintBitsEncoderTestHelper(digits, results);
  }
  {
    SCOPED_TRACE("digits=1");
    size_t digits = 1;
    boost::array<std::string, 3> results = {"h0", "h1", "h8"};
    ElementHexUintBitsEncoderTestHelper(digits, results);
  }
}

TEST(ElementUintEncoderTest, NomalCase) {
  {
    SCOPED_TRACE("type=uint32_t digits=9");
    size_t digits = 9;
    boost::array<uint32_t, kTestValueLengs> test_values = {1, 999999999};
    boost::array<std::string, kTestValueLengs> test_results = {"000000001", "999999999"};
    UintEncoderTestHelper(digits, test_values, test_results);
  }
  {
    SCOPED_TRACE("type=uint8_t digits=9");
    size_t digits = 9;
    boost::array<uint8_t, kTestValueLengs> test_values = {1, 255};
    boost::array<std::string, kTestValueLengs> test_results = {"000000001", "000000255"};
    UintEncoderTestHelper(digits, test_values, test_results);
  }
  {
    SCOPED_TRACE("type=uint32_t digits=2");
    size_t digits = 2;
    boost::array<uint32_t, kTestValueLengs> test_values = {1, 99};
    boost::array<std::string, kTestValueLengs> test_results = {"01", "99"};
    UintEncoderTestHelper(digits, test_values, test_results);
  }
  {
    SCOPED_TRACE("type=uint8_t digits=2");
    size_t digits = 2;
    boost::array<uint8_t, kTestValueLengs> test_values = {1, 99};
    boost::array<std::string, kTestValueLengs> test_results = {"01", "99"};
    UintEncoderTestHelper(digits, test_values, test_results);
  }
  {
    SCOPED_TRACE("type=int32_t digits=2");
    size_t digits = 2;
    boost::array<int32_t, kTestValueLengs> test_values = {1, 99};
    boost::array<std::string, kTestValueLengs> test_results = {"01", "99"};
    UintEncoderTestHelper(digits, test_values, test_results);
  }
  {
    SCOPED_TRACE("type=int8_t digits=2");
    size_t digits = 2;
    boost::array<int8_t, kTestValueLengs> test_values = {1, 99};
    boost::array<std::string, kTestValueLengs> test_results = {"01", "99"};
    UintEncoderTestHelper(digits, test_values, test_results);
  }
}

TEST(ElementStringEncoderTest, NomalCase) {
  {
    SCOPED_TRACE("digits=5");
    size_t digits = 5;
    boost::array<std::string, kTestValueLengs> test_values = {"tstst", "12345"};
    StringEncoderTestHelper(digits, test_values);
  }
  {
    SCOPED_TRACE("digits=2");
    size_t digits = 2;
    boost::array<std::string, kTestValueLengs> test_values = {"te", "12"};
    StringEncoderTestHelper(digits, test_values);
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
