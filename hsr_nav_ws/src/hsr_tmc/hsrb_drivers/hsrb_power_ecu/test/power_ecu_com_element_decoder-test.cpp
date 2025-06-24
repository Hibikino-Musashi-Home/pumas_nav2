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

#include <hsrb_power_ecu/i_network.hpp>

#include "../src/power_ecu_com_element_decoder.hpp"
#include "common_methods.hpp"
namespace {
const size_t kTestValueLengs = 3;
const size_t kBufferSize = 1000;

template <typename T>
void ElementUintDecoderTestHelper(const size_t digits, const boost::array<std::string, kTestValueLengs>& test_values,
                                  const boost::array<T, kTestValueLengs>& result_values) {
  T value = 0;
  hsrb_power_ecu::ElementUintDecoder<T> decoder(value, digits);
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);
  for (size_t i = 0; i < kTestValueLengs; ++i) {
    std::stringstream sst;
    sst << "nomal value = " << test_values[i] << ", result = " << result_values[i];
    SCOPED_TRACE(sst.str().c_str());
    buffer.clear();
    write_buffer(test_values[i], buffer);
    EXPECT_TRUE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_EQ(value, result_values[i]);
  }
  {
    SCOPED_TRACE("failue digits");
    std::string bad_string = "";
    // One digit
    for (size_t i = 0; i < digits + 1; ++i) {
      bad_string += "0";
    }
    buffer.clear();
    write_buffer(bad_string, buffer);

    value = 0;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 0);
    value = 1;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 1);
  }
  {
    SCOPED_TRACE("failue digits");
    std::string bad_string = "";
    // 1 digit little
    for (size_t i = 0; i < digits - 1; ++i) {
      bad_string += "0";
    }
    buffer.clear();
    write_buffer(bad_string, buffer);

    value = 0;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 0);
    value = 1;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 1);
  }
}

template <typename T>
void ElementHexUintDecoderTestHelper(const size_t digits, const boost::array<std::string, kTestValueLengs>& test_values,
                                     const boost::array<T, kTestValueLengs>& result_values) {
  T value = 0;
  hsrb_power_ecu::ElementHexUintDecoder<T> decoder(value, digits);
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);
  for (size_t i = 0; i < kTestValueLengs; ++i) {
    std::stringstream sst;
    sst << "nomal value = " << test_values[i] << ", result = " << result_values[i];
    SCOPED_TRACE(sst.str().c_str());
    buffer.clear();
    write_buffer(test_values[i], buffer);
    EXPECT_TRUE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_EQ(value, result_values[i]);
  }
  {
    SCOPED_TRACE("failue digits");
    std::string bad_string = "h";
    // One digit
    for (size_t i = 0; i < digits + 1; ++i) {
      bad_string += "0";
    }
    buffer.clear();
    write_buffer(bad_string, buffer);

    value = 0;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 0);
    value = 1;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 1);
  }
  {
    SCOPED_TRACE("failue digits");
    std::string bad_string = "h";
    // 1 digit little
    for (size_t i = 0; i < digits - 1; ++i) {
      bad_string += "0";
    }
    buffer.clear();
    write_buffer(bad_string, buffer);

    value = 0;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 0);
    value = 1;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 1);
  }
}

template <typename T>
void ElementIntDecoderTestHelper(const size_t digits, const boost::array<std::string, kTestValueLengs>& test_values,
                                 const boost::array<T, kTestValueLengs>& result_values) {
  T value = 0;
  hsrb_power_ecu::ElementIntDecoder<T> decoder(value, digits);
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);
  for (size_t i = 0; i < kTestValueLengs; ++i) {
    std::stringstream sst;
    sst << "nomal value = " << test_values[i] << ", result = " << result_values[i];
    SCOPED_TRACE(sst.str().c_str());
    buffer.clear();
    write_buffer(test_values[i], buffer);
    EXPECT_TRUE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_EQ(value, result_values[i]);
  }
  {
    SCOPED_TRACE("failue digits");
    std::string bad_string = " ";
    // One digit
    for (size_t i = 0; i < digits + 1; ++i) {
      bad_string += "0";
    }
    buffer.clear();
    write_buffer(bad_string, buffer);

    value = 0;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 0);
    value = 1;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 1);
  }
  {
    SCOPED_TRACE("failue digits");
    std::string bad_string = " ";
    // 1 digit little
    for (size_t i = 0; i < digits - 1; ++i) {
      bad_string += "0";
    }
    buffer.clear();
    write_buffer(bad_string, buffer);

    value = 0;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 0);
    value = 1;
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_EQ(value, 1);
  }
}

void ElementStringDecoderTestHelper(const size_t digits, const boost::array<std::string, kTestValueLengs>& test_values,
                                    const boost::array<std::string, kTestValueLengs>& result_values) {
  std::string value = "";
  hsrb_power_ecu::ElementStringDecoder decoder(value, digits);
  hsrb_power_ecu::PacketBuffer buffer(kBufferSize);
  for (size_t i = 0; i < kTestValueLengs; ++i) {
    std::stringstream sst;
    sst << "nomal value = " << test_values[i] << ", result = " << result_values[i];
    SCOPED_TRACE(sst.str().c_str());
    buffer.clear();
    write_buffer(test_values[i], buffer);
    EXPECT_TRUE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(value.c_str(), result_values[i].c_str());
  }
  {
    SCOPED_TRACE("failue digits");
    std::string bad_string = "";
    // One digit
    for (size_t i = 0; i < digits + 1; ++i) {
      bad_string += "0";
    }
    buffer.clear();
    write_buffer(bad_string, buffer);

    value = "a";
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_STREQ(value.c_str(), "a");
    value = "b";
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_STREQ(value.c_str(), "b");
  }
  {
    SCOPED_TRACE("failue digits");
    std::string bad_string = "";
    // 1 digit little
    for (size_t i = 0; i < digits - 1; ++i) {
      bad_string += "0";
    }
    buffer.clear();
    write_buffer(bad_string, buffer);

    value = "a";
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_STREQ(value.c_str(), "a");
    value = "b";
    EXPECT_FALSE(decoder.Decode(buffer.begin(), buffer.end()));
    EXPECT_STREQ(read_buffer(buffer).c_str(), bad_string.c_str());
    EXPECT_STREQ(value.c_str(), "b");
  }
}

}  // anonymous namespace

// ElementUintDecoder
TEST(ElementUintDecoderTest, NomalCase) {
  {
    SCOPED_TRACE("type=uint32_t, digits=9");
    size_t digits = 9;
    boost::array<std::string, kTestValueLengs> test_values = {"000000001", "000000000", "999999999"};
    boost::array<uint32_t, kTestValueLengs> result_values = {1, 0, 999999999};
    ElementUintDecoderTestHelper(digits, test_values, result_values);
  }
  {
    SCOPED_TRACE("type=uint8_t, digits=9");
    size_t digits = 9;
    boost::array<std::string, kTestValueLengs> test_values = {"000000001", "000000000", "999999999"};
    boost::array<uint8_t, kTestValueLengs> result_values = {1, 0, 255};
    ElementUintDecoderTestHelper(digits, test_values, result_values);
  }
  {
    SCOPED_TRACE("type=uint32_t, digits=2");
    size_t digits = 2;
    boost::array<std::string, kTestValueLengs> test_values = {"01", "00", "99"};
    boost::array<uint32_t, kTestValueLengs> result_values = {1, 0, 99};
    ElementUintDecoderTestHelper(digits, test_values, result_values);
  }
  {
    SCOPED_TRACE("type=uint8_t, digits=2");
    size_t digits = 2;
    boost::array<std::string, kTestValueLengs> test_values = {"01", "00", "99"};
    boost::array<uint8_t, kTestValueLengs> result_values = {1, 0, 99};
    ElementUintDecoderTestHelper(digits, test_values, result_values);
  }
}
// ElementHexUintDecoder
TEST(ElementHexUintDecoderTest, NomalCase) {
  {
    SCOPED_TRACE("type=uint32_t, digits=8");
    size_t digits = 8;
    boost::array<std::string, kTestValueLengs> test_values = {"h00000001", "h00000000", "hFFFFFFFF"};
    boost::array<uint32_t, kTestValueLengs> result_values = {0x1, 0x0, 0xffffffff};
    ElementHexUintDecoderTestHelper(digits, test_values, result_values);
  }
  {
    SCOPED_TRACE("type=uint8_t, digits=8");
    size_t digits = 8;
    boost::array<std::string, kTestValueLengs> test_values = {"h00000001", "h00000000", "hFFFFFFFF"};
    boost::array<uint8_t, kTestValueLengs> result_values = {0x1, 0x0, 0xFF};
    ElementHexUintDecoderTestHelper(digits, test_values, result_values);
  }
  {
    SCOPED_TRACE("type=uint32_t, digits=2");
    size_t digits = 2;
    boost::array<std::string, kTestValueLengs> test_values = {"h01", "h00", "hFF"};
    boost::array<uint32_t, kTestValueLengs> result_values = {0x1, 0x0, 0xFF};
    ElementHexUintDecoderTestHelper(digits, test_values, result_values);
  }
  {
    SCOPED_TRACE("type=uint8_t, digits=2");
    size_t digits = 2;
    boost::array<std::string, kTestValueLengs> test_values = {"h01", "h00", "hFF"};
    boost::array<uint8_t, kTestValueLengs> result_values = {0x1, 0x0, 0xFF};
    ElementHexUintDecoderTestHelper(digits, test_values, result_values);
  }
}

// ElementIntDecoder
TEST(ElementIntDecoderTest, NomalCase) {
  {
    SCOPED_TRACE("type=int32_t, digits=8");
    size_t digits = 9;
    boost::array<std::string, kTestValueLengs> test_values = {"-999999999", " 000000000", " 999999999"};
    boost::array<int32_t, kTestValueLengs> result_values = {-999999999, 0x0, 999999999};
    ElementIntDecoderTestHelper(digits, test_values, result_values);
  }
  {
    SCOPED_TRACE("type=int8_t, digits=8");
    size_t digits = 9;
    boost::array<std::string, kTestValueLengs> test_values = {"-999999999", " 000000000", " 999999999"};
    boost::array<int8_t, kTestValueLengs> result_values = {-128, 0, 127};
    ElementIntDecoderTestHelper(digits, test_values, result_values);
  }
  {
    SCOPED_TRACE("type=int32_t, digits=2");
    size_t digits = 2;
    boost::array<std::string, kTestValueLengs> test_values = {"-99", " 00", " 99"};
    boost::array<int32_t, kTestValueLengs> result_values = {-99, 0, 99};
    ElementIntDecoderTestHelper(digits, test_values, result_values);
  }
  {
    SCOPED_TRACE("type=int8_t, digits=2");
    size_t digits = 2;
    boost::array<std::string, kTestValueLengs> test_values = {"-99", " 00", " 99"};
    boost::array<int8_t, kTestValueLengs> result_values = {-99, 0, 99};
    ElementIntDecoderTestHelper(digits, test_values, result_values);
  }
}

// ElementStringDecoder
TEST(ElementStringDecoderTest, NomalCase) {
  {
    SCOPED_TRACE("digits=9");
    size_t digits = 9;
    boost::array<std::string, kTestValueLengs> test_values = {"999999999", "000000000", "999999999"};
    ElementStringDecoderTestHelper(digits, test_values, test_values);
  }
  {
    SCOPED_TRACE("digits=2");
    size_t digits = 2;
    boost::array<std::string, kTestValueLengs> test_values = {"01", "00", "99"};
    ElementStringDecoderTestHelper(digits, test_values, test_values);
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
