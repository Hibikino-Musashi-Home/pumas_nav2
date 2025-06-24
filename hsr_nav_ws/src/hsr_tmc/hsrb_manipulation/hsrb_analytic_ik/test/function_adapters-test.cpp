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
/// @brief Testing adapter groups that operate the function
#include <gtest/gtest.h>

#include "function_adapters.hpp"
#include "functions_for_testing.hpp"

namespace opt {
////////////////////////////////////////////////////////////////////////////////
//// Shift adapter test

// f(x)=(x-1)^4  (x∈[0,2])
class TestFunctionA {
 public:
  bool IsFeasible(double x) {
    return (0 <= x && x <= 2);
  }
  double Value(double x) {
    return (x - 1) * (x - 1) * (x - 1) * (x - 1);
  }
  double Gradient(double x) {
    return 4 * (x - 1) * (x - 1) * (x - 1);
  }
  double Hessian(double x) {
    return 12 * (x - 1) * (x - 1);
  }
};

// A function that moved TestFunctiona only 1 to the right.In other words:
// f(x)=(x-2)^4  (x∈[1,3])
class TestFunctionB {
 public:
  bool IsFeasible(double x) {
    return (1 <= x && x <= 3);
  }
  double Value(double x) {
    return (x - 2) * (x - 2) * (x - 2) * (x - 2);
  }
  double Gradient(double x) {
    return 4 * (x - 2) * (x - 2) * (x - 2);
  }
  double Hessian(double x) {
    return 12 * (x - 2) * (x - 2);
  }
};

TEST(FunctionAdapters_Test, ShiftAdapterFnuction1_Test) {
  TestFunctionA funcA;
  TestFunctionB funcB;
  ShiftAdapterFunction1<TestFunctionA> funcC(funcA, 1);

  // Confirm that FUNCB == FUNCC.
  EXPECT_EQ(funcB.IsFeasible(-1), funcC.IsFeasible(-1));
  EXPECT_EQ(funcB.IsFeasible(0), funcC.IsFeasible(0));
  EXPECT_EQ(funcB.IsFeasible(1), funcC.IsFeasible(1));
  EXPECT_EQ(funcB.IsFeasible(2), funcC.IsFeasible(2));
  EXPECT_EQ(funcB.IsFeasible(3), funcC.IsFeasible(3));
  EXPECT_EQ(funcB.IsFeasible(4), funcC.IsFeasible(4));

  EXPECT_EQ(funcB.Value(-1), funcC.Value(-1));
  EXPECT_EQ(funcB.Value(0), funcC.Value(0));
  EXPECT_EQ(funcB.Value(1), funcC.Value(1));
  EXPECT_EQ(funcB.Value(2), funcC.Value(2));
  EXPECT_EQ(funcB.Value(3), funcC.Value(3));
  EXPECT_EQ(funcB.Value(4), funcC.Value(4));

  EXPECT_EQ(funcB.Gradient(-1), funcC.Gradient(-1));
  EXPECT_EQ(funcB.Gradient(0), funcC.Gradient(0));
  EXPECT_EQ(funcB.Gradient(1), funcC.Gradient(1));
  EXPECT_EQ(funcB.Gradient(2), funcC.Gradient(2));
  EXPECT_EQ(funcB.Gradient(3), funcC.Gradient(3));
  EXPECT_EQ(funcB.Gradient(4), funcC.Gradient(4));

  EXPECT_EQ(funcB.Hessian(-1), funcC.Hessian(-1));
  EXPECT_EQ(funcB.Hessian(0), funcC.Hessian(0));
  EXPECT_EQ(funcB.Hessian(1), funcC.Hessian(1));
  EXPECT_EQ(funcB.Hessian(2), funcC.Hessian(2));
  EXPECT_EQ(funcB.Hessian(3), funcC.Hessian(3));
  EXPECT_EQ(funcB.Hessian(4), funcC.Hessian(4));
}

////////////////////////////////////////////////////////////////////////////////
//// Reverse adapter test

// f(x)=(x-1)^3  (x∈[1,3])
class TestFunctionD {
 public:
  bool IsFeasible(double x) {
    return (1 <= x && x <= 3);
  }
  double Value(double x) {
    return (x - 1) * (x - 1) * (x - 1);
  }
  double Gradient(double x) {
    return 3 * (x - 1) * (x - 1);
  }
  double Hessian(double x) {
    return 6 * (x - 1);
  }
};

// A function in which TestFunctiona is inverted around the X axis, that is,:
// f(x)=(-x-1)^3  (x∈[-3,-1])
class TestFunctionE {
 public:
  bool IsFeasible(double x) {
    return (-3 <= x && x <= -1);
  }
  double Value(double x) {
    return (-x - 1) * (-x - 1) * (-x - 1);
  }
  double Gradient(double x) {
    return 3 * (-x - 1) * (-x - 1);
  }
  double Hessian(double x) {
    return 6 * (-x - 1);
  }
};

TEST(FunctionAdapters_Test, ReverseAdapterFnuction1_Test) {
  TestFunctionD funcD;
  TestFunctionE funcE;
  ReverseAdapterFunction1<TestFunctionD> funcF(funcD);

  // Check that FUNCE == FUNCF.
  EXPECT_EQ(funcE.IsFeasible(-3), funcF.IsFeasible(-3));
  EXPECT_EQ(funcE.IsFeasible(-2), funcF.IsFeasible(-2));
  EXPECT_EQ(funcE.IsFeasible(-1), funcF.IsFeasible(-1));
  EXPECT_EQ(funcE.IsFeasible(0), funcF.IsFeasible(0));
  EXPECT_EQ(funcE.IsFeasible(1), funcF.IsFeasible(1));
  EXPECT_EQ(funcE.IsFeasible(2), funcF.IsFeasible(2));
  EXPECT_EQ(funcE.IsFeasible(3), funcF.IsFeasible(3));

  EXPECT_EQ(funcE.Value(-3), funcF.Value(-3));
  EXPECT_EQ(funcE.Value(-2), funcF.Value(-2));
  EXPECT_EQ(funcE.Value(-1), funcF.Value(-1));
  EXPECT_EQ(funcE.Value(0), funcF.Value(0));
  EXPECT_EQ(funcE.Value(1), funcF.Value(1));
  EXPECT_EQ(funcE.Value(2), funcF.Value(2));
  EXPECT_EQ(funcE.Value(3), funcF.Value(3));

  EXPECT_EQ(funcE.Gradient(-3), funcF.Gradient(-3));
  EXPECT_EQ(funcE.Gradient(-2), funcF.Gradient(-2));
  EXPECT_EQ(funcE.Gradient(-1), funcF.Gradient(-1));
  EXPECT_EQ(funcE.Gradient(0), funcF.Gradient(0));
  EXPECT_EQ(funcE.Gradient(1), funcF.Gradient(1));
  EXPECT_EQ(funcE.Gradient(2), funcF.Gradient(2));
  EXPECT_EQ(funcE.Gradient(3), funcF.Gradient(3));

  EXPECT_EQ(funcE.Hessian(-3), funcF.Hessian(-3));
  EXPECT_EQ(funcE.Hessian(-2), funcF.Hessian(-2));
  EXPECT_EQ(funcE.Hessian(-1), funcF.Hessian(-1));
  EXPECT_EQ(funcE.Hessian(0), funcF.Hessian(0));
  EXPECT_EQ(funcE.Hessian(1), funcF.Hessian(1));
  EXPECT_EQ(funcE.Hessian(2), funcF.Hessian(2));
  EXPECT_EQ(funcE.Hessian(3), funcF.Hessian(3));
}

////////////////////////////////////////////////////////////////////////////////
//// Test of direction adapter

TEST(FunctionAdapters_Test, DirectionAdapterFnuction1_Test) {
  {
    // Create a primary function from point (0,3) to direction (1,0) for two variables quadruple numbers.
    QuarticFunction2A func2;
    Vector2 x(0, 3);
    Vector2 d(1, 0);
    DirectionAdapterFunction2<QuarticFunction2A> func1(func2, x, d);

    // It should be Func1 (5) == FUNC2 (5,3).
    EXPECT_EQ(func2.Value(Vector2(5, 3)), func1.Value(5));

    // It should be Func1 (-7) == Func2 (-7,3).
    EXPECT_EQ(func2.Value(Vector2(-7, 3)), func1.Value(-7));
  }

  {
    // Create a primary function from point (0,3) to direction (1, -1) for two variables quadruples quarticFunction2a.
    QuarticFunction2A func2;
    Vector2 x(0, 3);
    Vector2 d(1, -1);
    DirectionAdapterFunction2<QuarticFunction2A> func1(func2, x, d);

    // It should be Func1 (2) == FUNC2 (2,1).
    EXPECT_EQ(func2.Value(Vector2(2, 1)), func1.Value(2));

    // FUNC1 (-7) == Func2 (-7,10) should be.
    EXPECT_EQ(func2.Value(Vector2(-7, 10)), func1.Value(-7));
  }
}
}  // namespace opt

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
