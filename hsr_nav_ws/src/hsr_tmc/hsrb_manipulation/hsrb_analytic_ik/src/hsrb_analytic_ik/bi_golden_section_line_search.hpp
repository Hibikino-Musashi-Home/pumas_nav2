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
/// @brief Class that performs a two -way golden split linear search method
#ifndef HSRB_ANALYTIC_IK_BI_GOLDEN_SECTION_LINE_SEARCH_HPP_
#define HSRB_ANALYTIC_IK_BI_GOLDEN_SECTION_LINE_SEARCH_HPP_

#include "common.hpp"
#include "function_adapters.hpp"
#include "uni_golden_section_line_search.hpp"

namespace opt {

/**
 * This is a class that performs a two -way golden split linear search method.
 */
class BiGoldenSectionLineSearch {
 public:
  /**
   * Constructor
   */
  explicit BiGoldenSectionLineSearch(int max_iteration = 100, double epsilon = 1e-4) {
    max_iteration_ = max_iteration;
    epsilon_ = epsilon;
    result_ = OptFail;
    iteration_ = 0;
    solution_ = 0;
    value_ = 0;
  }

  /**
   * Explore within the section (-∞,+∞).
   *
   * @param f one variable function
   * The initial value of the @param STEP search section [-Step, STEP]
   * @return
   */
  template<class Function1>
  OptResult Search(Function1& f, double step) {
    // Do a single -directional search in the positive direction.
    {
      UniGoldenSectionLineSearch search(max_iteration_, epsilon_);
      OptResult result = search.Search(f, step);

      // If the search fails
      if (result == OptFail) {
        result_ = OptFail;
        // Continue searching in the negative direction.
      } else {
        // If you succeed in searching or reach the maximum number of repetitions
        // Record a solution and compare it with the negative search results.
        result_ = result;
        iteration_ = search.iteration();
        solution_ = search.solution();
        value_ = search.value();
      }
    }

    // Do a single direction in the negative direction.
    {
      // Create an inverted function.
      ReverseAdapterFunction1<Function1> revserseFunc(f);

      UniGoldenSectionLineSearch search(max_iteration_, epsilon_);
      OptResult result = search.Search(revserseFunc, step);

      // If the search fails, return the positive direction.
      if (result == OptFail) {
        return result_;
      } else {
        // If you succeed in searching or reach the maximum number of repetitions
        // Failure or successful in the orientation,
        // If the negative direction is better, it will be the final solution.
        if ((result_ == OptFail) || (search.value() < value_)) {
          result_ = result;
          iteration_ = search.iteration();
          solution_ = -search.solution(); // Please note that it is negative
          value_ = search.value();
        }
        return result_;
      }
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
  OPT_CLASS_UNCOPYABLE(BiGoldenSectionLineSearch)

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
#endif  // HSRB_ANALYTIC_IK_BI_GOLDEN_SECTION_LINE_SEARCH_HPP_
