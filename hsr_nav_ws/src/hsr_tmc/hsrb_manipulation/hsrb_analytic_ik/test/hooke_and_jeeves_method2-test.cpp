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
/// @brief Testing classes that optimize by the Hook-and-Jeeven method
#include <gtest/gtest.h>

#include "bi_golden_section_line_search.hpp"
#include "functions_for_testing.hpp"
#include "hooke_and_jeeves_method2.hpp"

namespace opt {

TEST(HookeAndJeevesMethod2_Test, QuadraticFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-6;
  BiGoldenSectionLineSearch lineSearch(maxItor, epsilon * 0.0001);
  HookeAndJeevesMethod2 search(maxItor, epsilon);
  QuarticFunction2A func;

  // Search in step 1.0.
  {
    double step = 1.0;
    Vector2 x0(0, 3);
    Vector2 expected(2, 1);

    // I will search.
    OptResult result = search.Search(func, lineSearch, x0, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    Vector2 solution = search.solution();
    EXPECT_NEAR(expected.v1, solution.v1, epsilon * 10);
    EXPECT_NEAR(expected.v2, solution.v2, epsilon * 10);
  }

  // Search by step 0.1.
  {
    double step = 0.1;
    Vector2 x0(0, 3);
    Vector2 expected(2, 1);

    // I will search.
    OptResult result = search.Search(func, lineSearch, x0, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    Vector2 solution = search.solution();
    EXPECT_NEAR(expected.v1, solution.v1, epsilon * 10);
    EXPECT_NEAR(expected.v2, solution.v2, epsilon * 10);
  }

  // Search by step 0.01.
  {
    double step = 0.01;
    Vector2 x0(0, 3);
    Vector2 expected(2, 1);

    // I will search.
    OptResult result = search.Search(func, lineSearch, x0, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    Vector2 solution = search.solution();
    EXPECT_NEAR(expected.v1, solution.v1, epsilon * 10);
    EXPECT_NEAR(expected.v2, solution.v2, epsilon * 10);
  }

  // Search by step 10.
  {
    double step = 10;
    Vector2 x0(0, 3);
    Vector2 expected(2, 1);

    // I will search.
    OptResult result = search.Search(func, lineSearch, x0, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    Vector2 solution = search.solution();
    EXPECT_NEAR(expected.v1, solution.v1, epsilon * 10);
    EXPECT_NEAR(expected.v2, solution.v2, epsilon * 10);
  }

  // Search by step 100.
  {
    double step = 100;
    Vector2 x0(0, 3);
    Vector2 expected(2, 1);

    // I will search.
    OptResult result = search.Search(func, lineSearch, x0, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    Vector2 solution = search.solution();
    EXPECT_NEAR(expected.v1, solution.v1, epsilon * 10);
    EXPECT_NEAR(expected.v2, solution.v2, epsilon * 10);
  }
}

}  // namespace opt

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

