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
/// @brief Class that performs a single -directional golden split linear search method
#ifndef HSRB_ANALYTIC_IK_UNI_GOLDEN_SECTION_LINE_SEARCH_HPP_
#define HSRB_ANALYTIC_IK_UNI_GOLDEN_SECTION_LINE_SEARCH_HPP_

#include "common.hpp"
#include "golden_section_line_search.hpp"

namespace opt {

/**
 * This is a class that performs a single -direction golden split linear search method.
 */
class UniGoldenSectionLineSearch {
 public:
  /**
   * Constructor
   */
  explicit UniGoldenSectionLineSearch(int max_iteration = 100, double epsilon = 1e-4)
      : max_iteration_(max_iteration),
        epsilon_(epsilon),
        result_(OptFail),
        iteration_(0),
        solution_(0),
        value_(0) {}

  /**
   * Explore within the section (0,+∞).
   *
   * @param f one variable function
   * The initial value of the @param STEP search section [0, STEP]
   * @return
   */
  template<class Function1>
  OptResult Search(Function1& f, double step) {
    // Initialize the search results.
    result_ = OptFail;
    iteration_ = 0;
    solution_ = 0;
    value_ = 0;

    // Create a golden split straight line search.
    GoldenSectionLineSearch search(max_iteration_, epsilon_);

    // Initialize the search section.
    const double a = 0;
    double b = step;

    // Do the first search for the first time.
    OptResult result = search.Search(f, a, b);

    // If you succeed in searching, record the solution.
    iteration_ = search.iteration();
    solution_ = search.solution();
    value_ = search.value();
    if (result == OptSuccess) {
      result_ = OptSuccess;
      // Continue to expand the section and explore the solution.
    } else if (result == OptMaxItor) {
      result_ = OptMaxItor;
      return result_;
    }

    // If the uniform convex is detected in the first golden split search,
    // Reduce the step width so that it becomes a semi -convex in the section.
    if (result == OptFail) {
      // Repeat K repeated.
      for (int k = 2; k <= max_iteration_; k++) {
        // Reduce the section.
        b *= 0.5;

        // I will search.
        OptResult result = search.Search(f, a, b);

        if (result == OptSuccess) {
          result_ = OptSuccess;
          iteration_ = search.iteration();
          solution_ = search.solution();
          value_ = search.value();
          return result_;
        }
      }

      result_ = OptMaxItor;
      return result_;
    } else {
      // Repeat K repeated.
      for (int k = 2; k <= max_iteration_; k++) {
        // Enlarge the section.
        b *= 2;

        // I will search.
        OptResult result = search.Search(f, a, b);

        if (result == OptSuccess) {
          double comparativeSolution = search.solution();
          double comparativeValue = search.value();

          // If the previous solution and this solution are close enough, of the two
          // Solve the smaller the target function value.
          // If you do not do this, if you judge only by the size of the target function value,
          // It will be a wasted repetition due to a small calculation error.
          if (std::abs(solution_ - comparativeSolution) <= epsilon_) {
            if (comparativeValue < value_) {
              iteration_ = search.iteration();
              solution_ = comparativeSolution;
              value_ = comparativeValue;
            }
            return result_;
          } else if (comparativeValue < value_) {
            // When this solution and the previous solution are far away,
            // If the target function value is smaller, we will continue to search.
            iteration_ = search.iteration();
            solution_ = comparativeSolution;
            value_ = comparativeValue;
            continue;
          } else {
            // Return the successful solution last time.
            return result_;
          }
        } else {
          // For OptMaxitor or OptFail
          // Return the successful solution last time.
          return result_;
        }
      }

      // At least once, you have succeeded in searching, so we will solve it.
      return result_;
    }
  }

  /**
   * Set the maximum repetition number.
   */
  void set_max_iteration(int max_iteration) {
    max_iteration_ = max_iteration;
  }

  /**
   * Set the convergence conditions.
   */
  void set_epsilon(double epsilon) {
    epsilon_ = epsilon;
  }

  /**
   * Get the search results.
   */
  OptResult result() const {
    return result_;
  }

  /**
   * Acquires the repeated number of exploration.
   */
  int iteration() const {
    return iteration_;
  }

  /**
   * Acquires the solution after the search.
   */
  double solution() const {
    return solution_;
  }

  /**
   * Get the target function value after search.
   */
  double value() const {
    return value_;
  }

 private:
  OPT_CLASS_UNCOPYABLE(UniGoldenSectionLineSearch)

  /**
   * Maximum number of repeats
   */
  int max_iteration_;

  /**
   * Convergence conditions
   * (If the width of the uncertain section is below this value, it shall be converged).
   */
  double epsilon_;

  /**
   * Holds search results.
   */
  OptResult result_;

  /**
   * Holds the repeated number of repetitions.
   */
  int iteration_;

  /**
   * Holds the solution after search.
   */
  double solution_;

  /**
   * Holds the purpose function value of the solution.
   */
  double value_;
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_UNI_GOLDEN_SECTION_LINE_SEARCH_HPP_
