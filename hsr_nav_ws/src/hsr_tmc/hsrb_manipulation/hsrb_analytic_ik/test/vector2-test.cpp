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
/// @brief 2D vector class test
#include <gtest/gtest.h>

#include "vector2.hpp"

namespace opt {

TEST(Vector2_Test, test1) {
  // Constructor test
  Vector2 v(3, 4);
  EXPECT_EQ(3, v.v1);
  EXPECT_EQ(4, v.v2);

  // SET function test
  v.Set(9, 8);
  EXPECT_EQ(9, v.v1);
  EXPECT_EQ(8, v.v2);

  // ZERO function test
  v.Zero();
  EXPECT_EQ(0, v.v1);
  EXPECT_EQ(0, v.v2);

  // Norm function test
  v.Set(2, -1);
  EXPECT_EQ(std::sqrt(5.), v.Norm());

  // Norm2 function test
  EXPECT_EQ(5, v.Norm2());

  // Normalize function test
  v.Normalize();
  EXPECT_EQ(2 / std::sqrt(5.), v.v1);
  EXPECT_EQ(-1 / std::sqrt(5.), v.v2);

  // Norm Static function test
  {
    Vector2 a(10, 20);
    Vector2 b(10 + 2, 20 - 1);
    EXPECT_EQ(std::sqrt(5.), Vector2::Norm(a, b));
  }

  // MID STATIC function test
  {
    Vector2 a(10, 20);
    Vector2 b(10 + 2, 20 - 1);
    Vector2 c = Vector2::Mid(a, b);
    EXPECT_EQ(11.0, c.v1);
    EXPECT_EQ(19.5, c.v2);
  }

  // Dot Static function test
  {
    Vector2 a(1, 2);
    Vector2 b(-3, -4);
    EXPECT_EQ(-11, Vector2::Dot(a, b));
  }

  // + Operator test
  {
    Vector2 a(-1, 2);
    Vector2 b(3, -5);
    Vector2 c = a + b;
    EXPECT_EQ(2, c.v1);
    EXPECT_EQ(-3, c.v2);
  }

  // -The operator test
  {
    Vector2 a(-1, 2);
    Vector2 b(3, -5);
    Vector2 c = a - b;
    EXPECT_EQ(-4, c.v1);
    EXPECT_EQ(7, c.v2);
  }

  // Single -operator test
  {
    Vector2 a(-1, 2);
    Vector2 c = -a;
    EXPECT_EQ(1, c.v1);
    EXPECT_EQ(-2, c.v2);
  }

  // *Operator's test (scalar x vector)
  {
    Vector2 a(-1, 2);
    Vector2 c = 0.5 * a;
    EXPECT_EQ(-0.5, c.v1);
    EXPECT_EQ(1.0, c.v2);
  }

  // *Operator's test (vector x scalar)
  {
    Vector2 a(-1, 2);
    Vector2 c = a * 0.5;
    EXPECT_EQ(-0.5, c.v1);
    EXPECT_EQ(1.0, c.v2);
  }

  // / Operator's test (vector / scalar)
  {
    Vector2 a(-1, 2);
    Vector2 c = a / 2;
    EXPECT_EQ(-0.5, c.v1);
    EXPECT_EQ(1.0, c.v2);
  }
}

}  // namespace opt

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
