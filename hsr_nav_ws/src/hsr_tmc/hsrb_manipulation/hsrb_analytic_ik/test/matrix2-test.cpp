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
/// @brief 2x2 class test representing a queue
#include <gtest/gtest.h>

#include "matrix2.hpp"
#include "vector2.hpp"

namespace opt {

TEST(Matrix2_Test, test1) {
  // Constructor test
  {
    Matrix2 m(10, 20, 30, 40);
    EXPECT_EQ(10, m.m11);
    EXPECT_EQ(20, m.m12);
    EXPECT_EQ(30, m.m21);
    EXPECT_EQ(40, m.m22);
  }

  // ZERO function test
  {
    Matrix2 m(10, 20, 30, 40);
    m.Zero();
    EXPECT_EQ(0, m.m11);
    EXPECT_EQ(0, m.m12);
    EXPECT_EQ(0, m.m21);
    EXPECT_EQ(0, m.m22);
  }

  // Identity function test
  {
    Matrix2 m(10, 20, 30, 40);
    m.Identity();
    EXPECT_EQ(1, m.m11);
    EXPECT_EQ(0, m.m12);
    EXPECT_EQ(0, m.m21);
    EXPECT_EQ(1, m.m22);
  }

  // Inv Static function test
  {
    Matrix2 A(2, 5, 1, 3);
    Matrix2 B = Matrix2::Inv(A);
    EXPECT_EQ(3, B.m11);
    EXPECT_EQ(-5, B.m12);
    EXPECT_EQ(-1, B.m21);
    EXPECT_EQ(2, B.m22);
  }

  // Test of TOD STATIC function
  {
    Vector2 a(3, 7);
    Vector2 b(2, 4);
    Matrix2 M = Matrix2::Tod(a, b);
    EXPECT_EQ(6, M.m11);
    EXPECT_EQ(12, M.m12);
    EXPECT_EQ(14, M.m21);
    EXPECT_EQ(28, M.m22);
  }

  // + Operator test
  {
    Matrix2 A(2, 5, 1, 3);
    Matrix2 B(-1, 3, 2, -7);
    Matrix2 C = A + B;
    EXPECT_EQ(1, C.m11);
    EXPECT_EQ(8, C.m12);
    EXPECT_EQ(3, C.m21);
    EXPECT_EQ(-4, C.m22);
  }

  // -The operator test
  {
    Matrix2 A(2, 5, 1, 3);
    Matrix2 B(-1, 3, 2, -7);
    Matrix2 C = A - B;
    EXPECT_EQ(3, C.m11);
    EXPECT_EQ(2, C.m12);
    EXPECT_EQ(-1, C.m21);
    EXPECT_EQ(10, C.m22);
  }

  // *Test of operator (matrix x vector)
  {
    Matrix2 A(2, 5, 1, 3);
    Vector2 b(-1, 2);
    Vector2 x = A * b;
    EXPECT_EQ(8, x.v1);
    EXPECT_EQ(5, x.v2);
  }

  // *Test of operator (matrix x scalar)
  {
    Matrix2 A(2, 5, 1, 3);
    Matrix2 C = A * 0.5;
    EXPECT_EQ(1.0, C.m11);
    EXPECT_EQ(2.5, C.m12);
    EXPECT_EQ(0.5, C.m21);
    EXPECT_EQ(1.5, C.m22);
  }

  // *Test of operator (scalar x matrix)
  {
    Matrix2 A(2, 5, 1, 3);
    Matrix2 C = 0.5 * A;
    EXPECT_EQ(1.0, C.m11);
    EXPECT_EQ(2.5, C.m12);
    EXPECT_EQ(0.5, C.m21);
    EXPECT_EQ(1.5, C.m22);
  }

  // / Test of operator (matrix / scalar)
  {
    Matrix2 A(2, 5, 1, 3);
    Matrix2 C = A / 2.0;
    EXPECT_EQ(1.0, C.m11);
    EXPECT_EQ(2.5, C.m12);
    EXPECT_EQ(0.5, C.m21);
    EXPECT_EQ(1.5, C.m22);
  }
}

}  // namespace opt

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
