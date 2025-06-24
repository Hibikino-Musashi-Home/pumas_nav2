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
/// @brief Class to perform a golden split straight line search method
#ifndef HSRB_ANALYTIC_IK_GOLDEN_SECTION_LINE_SEARCH_HPP_
#define HSRB_ANALYTIC_IK_GOLDEN_SECTION_LINE_SEARCH_HPP_

#include "common.hpp"

namespace opt {

/**
 * This is a class that performs a golden split linear search method.
 */
class GoldenSectionLineSearch {
 public:
  /**
   * Constructor
   */
  explicit GoldenSectionLineSearch(int max_iteration = 100, double epsilon = 1e-4)
      : max_iteration_(max_iteration),
        epsilon_(epsilon),
        result_(OptFail),
        iteration_(0),
        solution_(0),
        value_(0) {}

  /**
   * Do the search.
   *
   * If one variable function F is narrowly convex in the search section [a, b],
   * Converge to extremely small by search.
   *
   * If it is detected that it is not narrowly convex while searching, the search fails and returns OptFail.
   * However, if it is not necessarily detected and is not detected, you will find a local solution.
   *
   * @param f one variable function
   * @Param A Search section [a, b] lower limit value
   * @Param B In search section [a, b] upper limit value
   * @Param maxitor maximum number of repetitions
   * @Param EPSILON Convergence conditions.If the width of the uncertain section is below this value, it will be converged.
   * @return
   */
  template<class Function1>
  OptResult Search(Function1& f, double a, double b) {
    // Initialize the search results.
    result_ = OptFail;
    iteration_ = 0;
    solution_ = 0;
    value_ = 0;

    // Initialize the uncertain section.
    double a_k = a;
    double b_k = b;

    // Calculate the function value f_a_k, f_b_k in the end point a_k and b_k of the uncertain section.
    double f_a_k = f.Value(a_k);
    double f_b_k = f.Value(b_k);

    // Set the golden ratio.
    const double _alpha = 0.61803398874989484820458683436563811772;

    // Select the evaluation point S_K, T_k in the uncertain section [a_k, b_k].
    double s_k = a_k + (1 - _alpha) * (b_k - a_k);
    double t_k = a_k + _alpha * (b_k - a_k);

    // Calculate the function value f_s_k, f_t_k in the evaluation point S_K and t_k.
    double f_s_k = f.Value(s_k);
    double f_t_k = f.Value(t_k);

    // Repeat K repeated.
    for (int k = 1; k <= max_iteration_; k++) {
      // When selecting [a_k, t_k] as the next uncertain section, True,
      // When selecting [s_k, b_k], it is a flag of False.
      bool left = (f_a_k <= f_s_k && f_a_k <= f_t_k && f_a_k <= f_b_k)
               || (f_s_k <= f_a_k && f_s_k <= f_t_k && f_s_k <= f_b_k);

      // The next uncertain section is determined.
      if (left) {
        // a_k is the same
        b_k = t_k;
        f_b_k = f_t_k;
        t_k = s_k;
        f_t_k = f_s_k;
        s_k = a_k + (1 - _alpha) * (b_k - a_k);
        f_s_k = f.Value(s_k);
      } else {
        // B_k is the same
        a_k = s_k;
        f_a_k = f_s_k;
        s_k = t_k;
        f_s_k = f_t_k;
        t_k = a_k + _alpha * (b_k - a_k);
        f_t_k = f.Value(t_k);
      }

      // Judge the convergence conditions.
      if (b_k - a_k <= epsilon_) {
        result_ = OptSuccess;
        iteration_ = k;
        if (f_a_k <= f_b_k) {
          solution_ = a_k;
          value_ = f_a_k;
        } else {
          solution_ = b_k;
          value_ = f_b_k;
        }
        return result_;
      }
    }

    // Returns OptMaxitor as the maximum number of repetitions have reached.
    result_ = OptMaxItor;
    iteration_ = max_iteration_;
    if (f_a_k <= f_b_k) {
      solution_ = a_k;
      value_ = f_a_k;
    } else {
      solution_ = b_k;
      value_ = f_b_k;
    }
    return result_;
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
  OPT_CLASS_UNCOPYABLE(GoldenSectionLineSearch)

  /**
   * Maximum number of repeats
   */
  int max_iteration_;

  /**
   * Convergence conditions
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
   * Holds the target function value after searching.
   */
  double value_;
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_GOLDEN_SECTION_LINE_SEARCH_HPP_
