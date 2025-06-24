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
/// @brief Test of a class that performs a golden split linear search method
#include <gtest/gtest.h>

#include "functions_for_testing.hpp"
#include "golden_section_line_search.hpp"

namespace opt {
////////////////////////////////////////////////////////////////////////////////
//// Test by primary function A

TEST(GoldenSectionLineSearch_Test, LinearFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // Explore in the section [0,2].X = 0 is solved.
  {
    LinearFunction1A func;
    GoldenSectionLineSearch search(maxItor, epsilon);
    double a = 0.0;
    double b = 2.0;
    double expected = 0.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// Test by secondary function A

TEST(GoldenSectionLineSearch_Test, QuadraticFunction1A_Test) {
  QuadraticFunction1A func;

  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // Explore in the section [0,2].X = 1 is solved.
  {
    double a = 0.0;
    double b = 2.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    // [Semo] An error from the true value due to the calculation error of the function value
    //  It does not necessarily mean that it will be less than the error of the uncertain section, so
    //  Here, the error is verified by 10 times.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Explore in the section [0,1].X = 1 is solved.
  {
    double a = 0.0;
    double b = 1.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Explore in the section [1,2].X = 1 is solved.
  {
    double a = 1.0;
    double b = 2.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Explore in the section [-4, -1].X = -1 is solved.
  {
    double a = -4.0;
    double b = -1.0;
    double expected = -1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Explore in the section [2, 8].X = 2 is solved.
  {
    double a = 2.0;
    double b = 8.0;
    double expected = 2.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Explore in the section [0,2].X = 1 is solved.
  // Verify that OptMaxitor is returned with a limited number of repetitions.
  {
    double a = 0.0;
    double b = 2.0;
    double expected = 1.0;

    // Explore and get the number of repetitions.
    OptResult result = search.Search(func, a, b);
    int iteration = search.iteration();

    // Explore only one repetitive number.
    search.set_max_iteration(iteration - 1);
    result = search.Search(func, a, b);
    EXPECT_EQ(OptMaxItor, result);
    EXPECT_EQ(OptMaxItor, search.result());

    // Make sure the number of repetitions is the maximum.
    EXPECT_EQ(iteration - 1, search.iteration());
  }
}

////////////////////////////////////////////////////////////////////////////////
//// Tests with unauthorized functions

TEST(GoldenSectionLineSearch_Test, NonDiffenrentialFunction1A_Test) {
  NonDiffenrentialFunction1A func;

  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // Explore in the section [-2, 2].X = -1 is solved.
  {
    double a = -2.0;
    double b = 2.0;
    double expected = -1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // Explore in the section [-2, -1].X = -1 is solved.
  {
    double a = -2.0;
    double b = -1.0;
    double expected = -1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // Explore in the section [-1, 2].X = -1 is solved.
  {
    double a = -1.0;
    double b = 2.0;
    double expected = -1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // Explore in the section [-4, -3].X = -3 is the solution.
  {
    double a = -4.0;
    double b = -3.0;
    double expected = -3.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // Explore in the section [3, 4].X = 3 is solved.
  {
    double a = 3.0;
    double b = 4.0;
    double expected = 3.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// Test by inverted function function

TEST(GoldenSectionLineSearch_Test, InvertedTrapeziumFunction1A_Test) {
  InvertedTrapeziumFunction1A func;

  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // Explore in the section [-5, 5].x∈ [-1,2] is the solution.
  {
    double a = -5.0;
    double b = 5.0;
    double expected1 = -1.0;
    double expected2 = 2.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }

  // Explore in the section [-1/2, 1/2].x∈ [-1/2,1/2] is solved.
  {
    double a = -0.5;
    double b = 0.5;
    double expected1 = -0.5;
    double expected2 = 0.5;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }

  // Explore in the section [-5, 0].x∈ [-1,0] is the solution.
  {
    double a = -5.0;
    double b = 0.0;
    double expected1 = -1.0;
    double expected2 = 0.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }

  // Explore in the section [1, 3].X∈ [1, 2] is the solution.
  {
    double a = 1.0;
    double b = 3.0;
    double expected1 = 1.0;
    double expected2 = 2.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// Test by unexpected function A

TEST(GoldenSectionLineSearch_Test, DiscontinuousFunction1A_Test) {
  DiscontinuousFunction1A func;

  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // Explore in the section [-5, 5].X = 1 is solved.
  {
    double a = -5.0;
    double b = 5.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // Explore in the section [0, 3].X = 1 is solved.
  {
    double a = 0.0;
    double b = 3.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // Search by section [0, 1].X = 1 is solved.
  {
    double a = 0.0;
    double b = 1.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // Explore in the section [0, 2].X = 1 is solved.
  {
    double a = 0.0;
    double b = 1.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // Explore in the section [1, 3].X = 1 is solved.
  {
    double a = 1.0;
    double b = 3.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // Explore in the section [1.5, 3].X = 1.5 is solved.
  {
    double a = 1.5;
    double b = 3.0;
    double expected = 1.5;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// Test by unexpected function B

TEST(GoldenSectionLineSearch_Test, DiscontinuousFunction1B_Test) {
  double penalty = 1e8;
  DiscontinuousFnction1B func(penalty);
  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // Search by section [-100, 100].X = 0 is solved.
  {
    double a = -20;
    double b = 20;
    double expected = 0.0;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// Announced tests due to a non -convex function

TEST(GoldenSectionLineSearch_Test, NonConvexFunction1A_Test) {
  NonConvexFunction1A func;

  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // Explore in the section [0, 4].
  // The search results should be 1.5.
  {
    double a = 0;
    double b = 4;
    double expected = 1.5;

    // I will search.
    OptResult result = search.Search(func, a, b);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }
}

}  // namespace opt

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
