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
#include <gtest/gtest.h>

#include "../src/error_counter.hpp"

TEST(ErrorCounterTest, NomalCase) {
  size_t buffer_size = 10;

  hsrb_power_ecu::ErrorCounter counter(buffer_size);
  EXPECT_DOUBLE_EQ(counter.GetErrorRate(), 0);
  EXPECT_EQ(counter.GetErrorCount(), 0);

  for (size_t i = 0; i < buffer_size; ++i) {
    counter.Register(false);
  }
  EXPECT_DOUBLE_EQ(counter.GetErrorRate(), 1);
  EXPECT_EQ(counter.GetErrorCount(), buffer_size);

  for (size_t i = 0; i < buffer_size; ++i) {
    counter.Register(true);
  }
  EXPECT_DOUBLE_EQ(counter.GetErrorRate(), 0);
  EXPECT_EQ(counter.GetErrorCount(), 0);

  size_t half = buffer_size / 2;
  for (size_t i = 0; i < half; ++i) {
    counter.Register(false);
  }
  EXPECT_DOUBLE_EQ(counter.GetErrorRate(), 0.5);
  EXPECT_EQ(counter.GetErrorCount(), half);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
