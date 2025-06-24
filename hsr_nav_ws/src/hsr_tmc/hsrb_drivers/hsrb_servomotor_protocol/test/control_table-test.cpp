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
#include <limits>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include <hsrb_servomotor_protocol/control_table.hpp>
#include <hsrb_servomotor_protocol/control_table_item_descriptor.hpp>

using hsrb_servomotor_protocol::ControlTable;
using hsrb_servomotor_protocol::ControlTableItemDescriptor;

// I can't open the file
TEST(LoadFailTest, NoExistFile) {
  ControlTable table;
  EXPECT_EQ(ControlTable::kFileOpenError, table.Load("file/non/exist"));
}

// Unusual file
TEST(LoadFailTest, InCorrectFile) {
  ControlTable table;
  EXPECT_EQ(ControlTable::kColumnSizeError, table.Load("test/test_csv_table/incorrect_table.csv"));
}

// A file containing the same entry
TEST(LoadFailTest, SameEntryFile) {
  ControlTable table;
  EXPECT_EQ(ControlTable::kAlreadyRecorded, table.Load("test/test_csv_table/same_entry.csv"));
}

// The name of the type is strange
TEST(LoadFailTest, BadTypeFile) {
  ControlTable table;
  EXPECT_EQ(ControlTable::kBadType, table.Load("test/test_csv_table/bad_type.csv"));
}

// Readable
TEST(LoadFailTest, CorrectFile) {
  ControlTable table;
  EXPECT_EQ(ControlTable::kSuccess, table.Load("test/test_csv_table/correct_table.csv"));
}

// Test after reading
class ControlTableTest : public ::testing::Test {
 public:
  ControlTableTest() { control_table_.Load("test/test_csv_table/correct_table.csv"); }
  virtual ~ControlTableTest() {}

 protected:
  ControlTable control_table_;
};

// Check if MD5 can be obtained correctly
TEST_F(ControlTableTest, CheckMd5) {
  const uint8_t expect_md5[16] = { 0x2c, 0xae, 0xa0, 0xeb, 0xf5, 0xd9, 0x4c, 0xa8,
                                   0xf7, 0x54, 0xc6, 0x5b, 0xf4, 0x30, 0x91, 0x0e };
  std::vector<uint8_t> result_md5 = control_table_.GetMd5Sum();
  ASSERT_EQ(16, result_md5.size());
  for (uint32_t i = 0; i < 16; ++i) {
    EXPECT_EQ(expect_md5[i], result_md5[i]);
  }
}

// Check if the specified descriptor can be taken properly
TEST_F(ControlTableTest, ReferItemDescriptor) {
  ControlTableItemDescriptor::Ptr value1 = control_table_.ReferItemDescriptor("value1");
  ASSERT_TRUE(value1);
  EXPECT_EQ(ControlTableItemDescriptor::kUInt8, value1->type());
  EXPECT_EQ(0, value1->initial_address());
  EXPECT_EQ(1, value1->num_bytes());
  EXPECT_EQ("static_param", value1->attribute());
  double mks_coeff;
  std::vector<uint8_t> table_bytes(value1->num_bytes());
  table_bytes[0] = 0x01;
  value1->ConvertToMKS(table_bytes, mks_coeff);
  EXPECT_EQ(1.0, mks_coeff);

  ControlTableItemDescriptor::Ptr value2 = control_table_.ReferItemDescriptor("value2");
  ASSERT_TRUE(value2);
  EXPECT_EQ(ControlTableItemDescriptor::kInt8, value2->type());
  EXPECT_EQ(1, value2->initial_address());
  EXPECT_EQ(1, value2->num_bytes());
  EXPECT_EQ("static_param", value2->attribute());

  ControlTableItemDescriptor::Ptr value3 = control_table_.ReferItemDescriptor("value3");
  ASSERT_TRUE(value3);
  EXPECT_EQ(ControlTableItemDescriptor::kUInt16, value3->type());
  EXPECT_EQ(2, value3->initial_address());
  EXPECT_EQ(2, value3->num_bytes());
  EXPECT_EQ("static_param", value3->attribute());

  ControlTableItemDescriptor::Ptr value4 = control_table_.ReferItemDescriptor("value4");
  ASSERT_TRUE(value4);
  EXPECT_EQ(ControlTableItemDescriptor::kInt16, value4->type());
  EXPECT_EQ(4, value4->initial_address());
  EXPECT_EQ(2, value4->num_bytes());
  EXPECT_EQ("static_param", value4->attribute());

  ControlTableItemDescriptor::Ptr value5 = control_table_.ReferItemDescriptor("value5");
  ASSERT_TRUE(value5);
  EXPECT_EQ(ControlTableItemDescriptor::kUInt32, value5->type());
  EXPECT_EQ(6, value5->initial_address());
  EXPECT_EQ(4, value5->num_bytes());
  EXPECT_EQ("order", value5->attribute());

  ControlTableItemDescriptor::Ptr value6 = control_table_.ReferItemDescriptor("value6");
  ASSERT_TRUE(value6);
  EXPECT_EQ(ControlTableItemDescriptor::kInt32, value6->type());
  EXPECT_EQ(10, value6->initial_address());
  EXPECT_EQ(4, value6->num_bytes());
  EXPECT_EQ("order", value6->attribute());

  ControlTableItemDescriptor::Ptr value7 = control_table_.ReferItemDescriptor("value7");
  ASSERT_TRUE(value7);
  EXPECT_EQ(ControlTableItemDescriptor::kUInt64, value7->type());
  EXPECT_EQ(14, value7->initial_address());
  EXPECT_EQ(8, value7->num_bytes());
  EXPECT_EQ("present", value7->attribute());

  ControlTableItemDescriptor::Ptr value8 = control_table_.ReferItemDescriptor("value8");
  ASSERT_TRUE(value8);
  EXPECT_EQ(ControlTableItemDescriptor::kInt64, value8->type());
  EXPECT_EQ(22, value8->initial_address());
  EXPECT_EQ(8, value8->num_bytes());
  EXPECT_EQ("present", value8->attribute());
  table_bytes.resize(value8->num_bytes());
  table_bytes[0] = 0x01;
  EXPECT_TRUE(value8->ConvertToMKS(table_bytes, mks_coeff));
  EXPECT_DOUBLE_EQ(0.001, mks_coeff);

  ControlTableItemDescriptor::Ptr value9 = control_table_.ReferItemDescriptor("value9");
  ASSERT_TRUE(value9);
  EXPECT_EQ(ControlTableItemDescriptor::kFloat, value9->type());
  EXPECT_EQ(30, value9->initial_address());
  EXPECT_EQ(4, value9->num_bytes());
  EXPECT_EQ("order", value9->attribute());
  table_bytes.resize(value9->num_bytes());
  *(reinterpret_cast<float*>(&table_bytes[0])) = 1.0;
  EXPECT_TRUE(value9->ConvertToMKS(table_bytes, mks_coeff));
  EXPECT_EQ(1e+6, mks_coeff);
}

// Is the non -existing descriptor empty?
TEST_F(ControlTableTest, NonExistItemDescriptor) {
  ControlTableItemDescriptor::Ptr value1 = control_table_.ReferItemDescriptor("non_exist");
  EXPECT_FALSE(value1);
}


// CONTROLTABLEITEMDEScriptor test
TEST(ControlTableItemDescriptorTest, Getter) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kUInt32, 10, "static_param", 0.0);
  EXPECT_EQ(ControlTableItemDescriptor::kUInt32, descriptor.type());
  EXPECT_EQ(10, descriptor.initial_address());
  EXPECT_EQ(std::string("static_param"), descriptor.attribute());
  EXPECT_EQ(4, descriptor.num_bytes());
  EXPECT_EQ(14, descriptor.final_address());
}

// MKS conversion test
TEST(ControlTableItemDescriptorTest, UnevenNumbytes) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kUInt8, 10, "static_param", 1.0e+3);
  std::vector<uint8_t> table_bytes(2);
  table_bytes[0] = 0x21;
  double mks_value = 0.0;
  EXPECT_FALSE(descriptor.ConvertToMKS(table_bytes, mks_value));
}

// MKS conversion test
TEST(ControlTableItemDescriptorTest, Overflow) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kUInt8, 10, "static_param", 1.0);
  std::vector<uint8_t> table_bytes_ret;
  EXPECT_FALSE(descriptor.ConvertToBytes(258.0, table_bytes_ret));
}

// MKS conversion test
TEST(ControlTableItemDescriptorTest, NaN) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kUInt8, 10, "static_param", 1.0);
  std::vector<uint8_t> table_bytes_ret;
  EXPECT_FALSE(descriptor.ConvertToBytes(std::numeric_limits<double>::quiet_NaN(), table_bytes_ret));
  EXPECT_TRUE(descriptor.ConvertToBytes(42.0, table_bytes_ret));
}

// MKS conversion test
TEST(ControlTableItemDescriptorTest, ConvertUint8) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kUInt8, 10, "static_param", 1.0e+3);
  std::vector<uint8_t> table_bytes(descriptor.num_bytes());
  table_bytes[0] = 0x21;
  double mks_value = 0.0;
  EXPECT_TRUE(descriptor.ConvertToMKS(table_bytes, mks_value));
  EXPECT_DOUBLE_EQ(0.033, mks_value);

  std::vector<uint8_t> table_bytes_ret(descriptor.num_bytes());
  EXPECT_TRUE(descriptor.ConvertToBytes(0.033, table_bytes_ret));
  EXPECT_EQ(0x21, table_bytes_ret[0]);

  EXPECT_TRUE(descriptor.ConvertToBytes(0.0, table_bytes_ret));
  EXPECT_EQ(0x00, table_bytes_ret[0]);
}

// MKS conversion test
TEST(ControlTableItemDescriptorTest, ConvertInt8) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kInt8, 10, "static_param", 1.0e+3);
  std::vector<uint8_t> table_bytes(descriptor.num_bytes());
  table_bytes[0] = 0xdc;
  double mks_value = 0.0;
  EXPECT_TRUE(descriptor.ConvertToMKS(table_bytes, mks_value));
  EXPECT_DOUBLE_EQ(-0.036, mks_value);

  std::vector<uint8_t> table_bytes_ret(descriptor.num_bytes());
  EXPECT_TRUE(descriptor.ConvertToBytes(-0.036, table_bytes_ret));
  EXPECT_EQ(0xdc, table_bytes_ret[0]);

  EXPECT_TRUE(descriptor.ConvertToBytes(0.0, table_bytes_ret));
  EXPECT_EQ(0x00, table_bytes_ret[0]);
}


// MKS conversion test
TEST(ControlTableItemDescriptorTest, ConvertUint16) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kUInt16, 10, "static_param", 1.0e-3);
  std::vector<uint8_t> table_bytes(descriptor.num_bytes());
  table_bytes[1] = 0x19;
  table_bytes[0] = 0x96;
  double mks_value = 0.0;
  EXPECT_TRUE(descriptor.ConvertToMKS(table_bytes, mks_value));
  EXPECT_DOUBLE_EQ(6550000, mks_value);

  std::vector<uint8_t> table_bytes_ret(descriptor.num_bytes());
  EXPECT_TRUE(descriptor.ConvertToBytes(6550000, table_bytes_ret));
  EXPECT_EQ(0x19, table_bytes_ret[1]);
  EXPECT_EQ(0x96, table_bytes_ret[0]);

  EXPECT_TRUE(descriptor.ConvertToBytes(0, table_bytes_ret));
  EXPECT_EQ(0x00, table_bytes_ret[1]);
  EXPECT_EQ(0x00, table_bytes_ret[0]);
}

// MKS conversion test
TEST(ControlTableItemDescriptorTest, ConvertInt16) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kInt16, 10, "static_param", 1.0e-1);
  std::vector<uint8_t> table_bytes(descriptor.num_bytes());
  table_bytes[1] = 0xdb;
  table_bytes[0] = 0x6f;
  double mks_value = 0.0;
  EXPECT_TRUE(descriptor.ConvertToMKS(table_bytes, mks_value));
  EXPECT_DOUBLE_EQ(-93610, mks_value);

  std::vector<uint8_t> table_bytes_ret(descriptor.num_bytes());
  EXPECT_TRUE(descriptor.ConvertToBytes(-93610, table_bytes_ret));
  EXPECT_EQ(0xdb, table_bytes_ret[1]);
  EXPECT_EQ(0x6f, table_bytes_ret[0]);

  EXPECT_TRUE(descriptor.ConvertToBytes(0, table_bytes_ret));
  EXPECT_EQ(0x00, table_bytes_ret[1]);
  EXPECT_EQ(0x00, table_bytes_ret[0]);
}

// MKS conversion test
TEST(ControlTableItemDescriptorTest, ConvertUInt32) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kUInt32, 10, "static_param", 1.0e-3);
  std::vector<uint8_t> table_bytes(descriptor.num_bytes());
  table_bytes[3] = 0x26;
  table_bytes[2] = 0xd1;
  table_bytes[1] = 0x87;
  table_bytes[0] = 0x03;
  double mks_value = 0.0;
  EXPECT_TRUE(descriptor.ConvertToMKS(table_bytes, mks_value));
  EXPECT_DOUBLE_EQ(651265795000, mks_value);

  std::vector<uint8_t> table_bytes_ret(descriptor.num_bytes());
  EXPECT_TRUE(descriptor.ConvertToBytes(651265795000, table_bytes_ret));
  EXPECT_EQ(0x26, table_bytes_ret[3]);
  EXPECT_EQ(0xd1, table_bytes_ret[2]);
  EXPECT_EQ(0x87, table_bytes_ret[1]);
  EXPECT_EQ(0x03, table_bytes_ret[0]);

  EXPECT_TRUE(descriptor.ConvertToBytes(0, table_bytes_ret));
  EXPECT_EQ(0x00, table_bytes_ret[3]);
  EXPECT_EQ(0x00, table_bytes_ret[2]);
  EXPECT_EQ(0x00, table_bytes_ret[1]);
  EXPECT_EQ(0x00, table_bytes_ret[0]);
}

// MKS conversion test
TEST(ControlTableItemDescriptorTest, ConvertInt32) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kInt32, 10, "static_param", 1.0e+6);
  std::vector<uint8_t> table_bytes(descriptor.num_bytes());
  table_bytes[3] = 0xed;
  table_bytes[2] = 0x9a;
  table_bytes[1] = 0xa0;
  table_bytes[0] = 0x56;
  double mks_value = 0.0;
  EXPECT_TRUE(descriptor.ConvertToMKS(table_bytes, mks_value));
  EXPECT_DOUBLE_EQ(-308.633514, mks_value);

  std::vector<uint8_t> table_bytes_ret(descriptor.num_bytes());
  EXPECT_TRUE(descriptor.ConvertToBytes(-308.633514, table_bytes_ret));
  EXPECT_EQ(0xed, table_bytes_ret[3]);
  EXPECT_EQ(0x9a, table_bytes_ret[2]);
  EXPECT_EQ(0xa0, table_bytes_ret[1]);
  EXPECT_EQ(0x56, table_bytes_ret[0]);

  EXPECT_TRUE(descriptor.ConvertToBytes(0, table_bytes_ret));
  EXPECT_EQ(0x00, table_bytes_ret[3]);
  EXPECT_EQ(0x00, table_bytes_ret[2]);
  EXPECT_EQ(0x00, table_bytes_ret[1]);
  EXPECT_EQ(0x00, table_bytes_ret[0]);
}

// MKS conversion test
TEST(ControlTableItemDescriptorTest, ConvertFloat) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kFloat, 10, "static_param", 1.0e-3);
  std::vector<uint8_t> table_bytes(descriptor.num_bytes());
  table_bytes[3] = 0x40;
  table_bytes[2] = 0x40;
  table_bytes[1] = 0x00;
  table_bytes[0] = 0x00;
  double mks_value = 0.0;
  EXPECT_TRUE(descriptor.ConvertToMKS(table_bytes, mks_value));
  EXPECT_DOUBLE_EQ(3000.0, mks_value);

  std::vector<uint8_t> table_bytes_ret(descriptor.num_bytes());
  EXPECT_TRUE(descriptor.ConvertToBytes(3000.0, table_bytes_ret));
  EXPECT_EQ(0x40, table_bytes_ret[3]);
  EXPECT_EQ(0x40, table_bytes_ret[2]);
  EXPECT_EQ(0x00, table_bytes_ret[1]);
  EXPECT_EQ(0x00, table_bytes_ret[0]);

  EXPECT_TRUE(descriptor.ConvertToBytes(3.0, table_bytes_ret));
  EXPECT_EQ(0x3b, table_bytes_ret[3]);
  EXPECT_EQ(0x44, table_bytes_ret[2]);
  EXPECT_EQ(0x9b, table_bytes_ret[1]);
  EXPECT_EQ(0xa6, table_bytes_ret[0]);

  EXPECT_FALSE(descriptor.ConvertToBytes(std::numeric_limits<double>::max(), table_bytes_ret));
}

// MKS conversion test
TEST(ControlTableItemDescriptorTest, ConvertUint64) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kUInt64, 10, "static_param", 1.0e+12);
  std::vector<uint8_t> table_bytes(descriptor.num_bytes());
  table_bytes[7] = 0xa5;
  table_bytes[6] = 0x14;
  table_bytes[5] = 0x51;
  table_bytes[4] = 0x56;
  table_bytes[3] = 0x9a;
  table_bytes[2] = 0x9b;
  table_bytes[1] = 0xe9;
  table_bytes[0] = 0xbe;

  double mks_value = 0.0;
  ASSERT_TRUE(descriptor.ConvertToMKS(table_bytes, mks_value));
  EXPECT_DOUBLE_EQ(11895221.948195267, mks_value);

  std::vector<uint8_t> table_bytes_ret(descriptor.num_bytes());
  ASSERT_TRUE(descriptor.ConvertToBytes(11895221.948195267, table_bytes_ret));
  EXPECT_EQ(0xa5, table_bytes_ret[7]);
  EXPECT_EQ(0x14, table_bytes_ret[6]);
  EXPECT_EQ(0x51, table_bytes_ret[5]);
  EXPECT_EQ(0x56, table_bytes_ret[4]);
  EXPECT_EQ(0x9a, table_bytes_ret[3]);
  EXPECT_EQ(0x9b, table_bytes_ret[2]);
  // Double accuracy limit
  // EXPECT_EQ(0xe9, table_bytes_ret[1]);
  // EXPECT_EQ(0xbe, table_bytes_ret[0]);

  ASSERT_TRUE(descriptor.ConvertToBytes(0.0, table_bytes_ret));
  EXPECT_EQ(0x00, table_bytes_ret[7]);
  EXPECT_EQ(0x00, table_bytes_ret[6]);
  EXPECT_EQ(0x00, table_bytes_ret[5]);
  EXPECT_EQ(0x00, table_bytes_ret[4]);
  EXPECT_EQ(0x00, table_bytes_ret[3]);
  EXPECT_EQ(0x00, table_bytes_ret[2]);
  EXPECT_EQ(0x00, table_bytes_ret[1]);
  EXPECT_EQ(0x00, table_bytes_ret[0]);
}

// MKS conversion test
TEST(ControlTableItemDescriptorTest, ConvertInt64) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kInt64, 10, "static_param", 1.0e+3);
  std::vector<uint8_t> table_bytes(descriptor.num_bytes());
  table_bytes[7] = 0xa5;
  table_bytes[6] = 0x14;
  table_bytes[5] = 0x51;
  table_bytes[4] = 0x56;
  table_bytes[3] = 0x9a;
  table_bytes[2] = 0x9b;
  table_bytes[1] = 0xe9;
  table_bytes[0] = 0xbe;

  double mks_value = 0.0;
  EXPECT_TRUE(descriptor.ConvertToMKS(table_bytes, mks_value));
  EXPECT_DOUBLE_EQ(-6551522125514282.562, mks_value);

  std::vector<uint8_t> table_bytes_ret(descriptor.num_bytes());
  EXPECT_TRUE(descriptor.ConvertToBytes(-6551522125514282.562, table_bytes_ret));
  EXPECT_EQ(0xa5, table_bytes_ret[7]);
  EXPECT_EQ(0x14, table_bytes_ret[6]);
  EXPECT_EQ(0x51, table_bytes_ret[5]);
  EXPECT_EQ(0x56, table_bytes_ret[4]);
  EXPECT_EQ(0x9a, table_bytes_ret[3]);
  EXPECT_EQ(0x9b, table_bytes_ret[2]);
  // Double accuracy limit
  // EXPECT_EQ(0xe9, table_bytes_ret[1]);
  // EXPECT_EQ(0xbe, table_bytes_ret[0]);

  ASSERT_TRUE(descriptor.ConvertToBytes(0.0, table_bytes_ret));
  EXPECT_EQ(0x00, table_bytes_ret[7]);
  EXPECT_EQ(0x00, table_bytes_ret[6]);
  EXPECT_EQ(0x00, table_bytes_ret[5]);
  EXPECT_EQ(0x00, table_bytes_ret[4]);
  EXPECT_EQ(0x00, table_bytes_ret[3]);
  EXPECT_EQ(0x00, table_bytes_ret[2]);
  EXPECT_EQ(0x00, table_bytes_ret[1]);
  EXPECT_EQ(0x00, table_bytes_ret[0]);
}


// MKS conversion test
TEST(ControlTableItemDescriptorTest, ConvertDouble) {
  ControlTableItemDescriptor descriptor(ControlTableItemDescriptor::kDouble, 10, "static_param", 1.0e-3);
  std::vector<uint8_t> table_bytes(descriptor.num_bytes());
  table_bytes[7] = 0x3f;
  table_bytes[6] = 0xf0;
  table_bytes[5] = 0x00;
  table_bytes[4] = 0x00;
  table_bytes[3] = 0x00;
  table_bytes[2] = 0x00;
  table_bytes[1] = 0x00;
  table_bytes[0] = 0x00;

  double mks_value = 0.0;
  EXPECT_TRUE(descriptor.ConvertToMKS(table_bytes, mks_value));
  EXPECT_DOUBLE_EQ(1000.0, mks_value);

  std::vector<uint8_t> table_bytes_ret(descriptor.num_bytes());
  EXPECT_TRUE(descriptor.ConvertToBytes(1000.0, table_bytes_ret));
  EXPECT_EQ(0x3f, table_bytes_ret[7]);
  EXPECT_EQ(0xf0, table_bytes_ret[6]);
  EXPECT_EQ(0x00, table_bytes_ret[5]);
  EXPECT_EQ(0x00, table_bytes_ret[4]);
  EXPECT_EQ(0x00, table_bytes_ret[3]);
  EXPECT_EQ(0x00, table_bytes_ret[2]);
  EXPECT_EQ(0x00, table_bytes_ret[1]);
  EXPECT_EQ(0x00, table_bytes_ret[0]);

  EXPECT_TRUE(descriptor.ConvertToBytes(42.0, table_bytes_ret));
  EXPECT_EQ(0x3f, table_bytes_ret[7]);
  EXPECT_EQ(0xa5, table_bytes_ret[6]);
  EXPECT_EQ(0x81, table_bytes_ret[5]);
  EXPECT_EQ(0x06, table_bytes_ret[4]);
  EXPECT_EQ(0x24, table_bytes_ret[3]);
  EXPECT_EQ(0xdd, table_bytes_ret[2]);
  EXPECT_EQ(0x2f, table_bytes_ret[1]);
  EXPECT_EQ(0x1b, table_bytes_ret[0]);
}


int main(int argc, char* argv[]) {
  // Initialization for getes
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
