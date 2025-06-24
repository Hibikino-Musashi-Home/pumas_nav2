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
/// @brief Test of a class that performs a two -way golden split linear search method
#include <gtest/gtest.h>

#include "bi_golden_section_line_search.hpp"
#include "functions_for_testing.hpp"

namespace opt {
////////////////////////////////////////////////////////////////////////////////
//// Test by secondary function A

/**
 * Test if the secondary function is the same result as a single direction to the QuadratricFunction1a.
 */
TEST(BiGoldenSectionLineSearch_Test, QuadraticFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // Explore with the initial step length of 0.1.X = 1 is solved.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length of 0.5.X = 1 is solved.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.5;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length 1.0.X = 1 is solved.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 1.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length 2.0.X = 1 is solved.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 2.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length of 100.0.X = 1 is solved.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 100.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length of 0.01.X = 1 is solved.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.01;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Test with the secondary function that takes the minimum value in x = -1.
  // Explore with the initial step length of 0.1.
  {
    ShiftAdapterFunction1<QuadraticFunction1A> func(QuadraticFunction1A(), -2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = -1;

    // I will search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// Test by secondary function B

/**
 * Test if the secondary function is the same result as a single direction to the QuadratricFunction1b.
 */
TEST(BiGoldenSectionLineSearch_Test, QuadraticFunction1B_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // Explore with the initial step length of 0.1.X = -1 is solved.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = -1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length of 0.5.X = -1 is solved.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.5;
    double expected = -1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length 1.0.X = -1 is solved.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 1.0;
    double expected = -1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length 2.0.X = -1 is solved.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 2.0;
    double expected = -1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length of 100.0.X = -1 is solved.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 100.0;
    double expected = -1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length of 0.01.X = -1 is solved.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.01;
    double expected = -1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Test with the secondary function that takes the minimum value with x = 2.
  // Explore with the initial step length of 0.1.
  {
    ShiftAdapterFunction1<QuadraticFunction1B> func(QuadraticFunction1B(), 3.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = 2.0;

    // I will search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// Tests with unauthorized functions

TEST(BiGoldenSectionLineSearch_Test, NonDiffenrentialFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // Explore with the initial step length of 0.1.X = 1 is solved.
  {
    // Use a function that shifts only +2 nondiffenRENTIALFUNCTION1A.
    // It has the minimum value in x = 1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(NonDiffenrentialFunction1A(), +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length of 0.5.X = 1 is solved.
  {
    // Use a function that shifts only +2 nondiffenRENTIALFUNCTION1A.
    // It has the minimum value in x = 1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.5;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length 1.0.X = 1 is solved.
  {
    // Use a function that shifts only +2 nondiffenRENTIALFUNCTION1A.
    // It has the minimum value in x = 1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 1.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length 2.0.X = 1 is solved.
  {
    // Use a function that shifts only +2 nondiffenRENTIALFUNCTION1A.
    // It has the minimum value in x = 1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 2.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length of 100.0.X = 1 is solved.
  {
    // Use a function that shifts only +2 nondiffenRENTIALFUNCTION1A.
    // It has the minimum value in x = 1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 100.0;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length of 0.01.X = 1 is solved.
  {
    // Use a function that shifts only +2 nondiffenRENTIALFUNCTION1A.
    // It has the minimum value in x = 1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.01;
    double expected = 1.0;

    // I will search.
    OptResult result = search.Search(func, step);

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
//// Test by inverted function function

TEST(BiGoldenSectionLineSearch_Test, InvertedTrapeziumFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // Explore with the initial step length of 0.1.x∈ [-1,2] is the solution.
  {
    InvertedTrapeziumFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);

    double step = 0.1;
    double expected1 = -1.0;
    double expected2 = 2.0;

    // I will search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of repetitions.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }

  // Explore with the initial step length of 0.1.x∈ [1,4] is the solution.
  {
    // Shift only +2 the inverted function function.
    // X∈ [1,4] takes the minimum value.
    ShiftAdapterFunction1<InvertedTrapeziumFunction1A> func(InvertedTrapeziumFunction1A(), +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);

    double step = 0.1;
    double expected1 = 1.0;
    double expected2 = 4.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

  // Explore with the initial step length 100.x∈ [1,4] is the solution.
  {
    // Shift only +2 the inverted function function.
    // X∈ [1,4] takes the minimum value.
    ShiftAdapterFunction1<InvertedTrapeziumFunction1A> func(
                                                            InvertedTrapeziumFunction1A(),
                                                            +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);

    double step = 100.0;
    double expected1 = 1.0;
    double expected2 = 4.0;

    // I will search.
    OptResult result = search.Search(func, step);

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

}  // namespace opt

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
