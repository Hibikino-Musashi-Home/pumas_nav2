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
/// @brief Classes that optimize by the Hook-and-Jeeven method
#ifndef HSRB_ANALYTIC_IK_HOOKE_AND_JEEVES_METHOD_2_HPP_
#define HSRB_ANALYTIC_IK_HOOKE_AND_JEEVES_METHOD_2_HPP_

#include "common.hpp"
#include "function_adapters.hpp"

namespace opt {

/**
 * This is a class that optimizes by the Hook-and-Jeeven method.
 * It is an optimization method that does not use the gradient.
 */
class HookeAndJeevesMethod2 {
 public:
  /**
   * Constructor
   */
  explicit HookeAndJeevesMethod2(int max_iteration = 100,
                                 double epsilon = 1e-4)
      : max_iteration_(max_iteration), epsilon_(epsilon) {
  }

  /**
   * Do the search.
   *
   * @Param Func purpose function
   * @Param x0 Initial value
   * @Param STEP step width (used for straight line search)
   * @Param maxitor's maximum repetition
   * @Param Epsilon convergence condition (It is deemed that it has converged when the distance moving due to repetition is below this distance)
   */
  template<class Function2, class BiLineSearch>
  OptResult Search(Function2& func, BiLineSearch& lineSearch, const Vector2& x0,
                   double step) {
    // Initialize the search results.
    result_ = OptFail;
    iteration_ = 0;
    solution_.Zero();

    // Initialize the column.
    Vector2 x = x0;

    // Initialize the previous position Z_PREV.
    Vector2 z_prev = x0;

    // Repeat K repeated.
    for (int k = 1; k <= max_iteration_; k++) {
      // Determine the point Y by exploring a straight line from the point x (1,0).
      Vector2 y;
      {
        DirectionAdapterFunction2<Function2> func_x1(func, x, Vector2(1, 0));
        OptResult result_x1 = lineSearch.Search(func_x1, step);
        if (result_x1 == OptFail) {
          result_ = OptFail;
          return result_;
        }
        double solution_x1 = lineSearch.solution();
        y.v1 = x.v1 + solution_x1;
        y.v2 = x.v2;
      }

      // Find a straight line in the direction (0, 1) from the point Y and determine the point Z.
      Vector2 z;
      {
        DirectionAdapterFunction2<Function2> func_x2(func, y, Vector2(0, 1));
        OptResult result_x2 = lineSearch.Search(func_x2, step);
        if (result_x2 == OptFail) {
          result_ = OptFail;
          return result_;
        }
        double solution_x2 = lineSearch.solution();
        z.v1 = y.v1;
        z.v2 = y.v2 + solution_x2;
      }

      // Under the Hooke-and-Jeeven method, you will be searching for patterns.
      // It is not a search for direction D = Z-X based on X before exploring the X1 and X2 direction,
      // Explore the direction D = Z-Z_PREV based on the previous position Z_PREV.
      Vector2 d = z - z_prev;

      // If the point z and the point z_prev are close enough, the repetition ends.
      if (d.Norm() <= epsilon_) {
        result_ = OptSuccess;
        iteration_ = k;
        solution_ = (func.Value(z) <= func.Value(z_prev)) ? z : z_prev;
        return result_;
      }

      // From the point Z, search the direction D in the direction D and determine the point X_next.
      Vector2 x_next;
      {
        d.Normalize();
        DirectionAdapterFunction2<Function2> func_d(func, z, d);
        OptResult result_d = lineSearch.Search(func_d, step);
        if (result_d == OptFail) {
          result_ = OptFail;
          return result_;
        }
        double solution_d = lineSearch.solution();
        x_next = z + solution_d * d;
      }

      // Calculate the distance between point x and point x_next.
      double dx = Vector2::Norm(x, x_next);

      // If the point x and point x_next are close enough, the repetition ends.
      if (dx <= epsilon_) {
        result_ = OptSuccess;
        iteration_ = k;
        solution_ = (func.Value(x_next) <= func.Value(x)) ? x_next : x;
        return result_;
      }

      // Record the previous point Z.
      z_prev = z;

      // Update the point X.
      x = x_next;
    }

    // Returns OptMaxitor as the maximum number of repetitions have reached.
    result_ = OptMaxItor;
    iteration_ = max_iteration_;
    solution_ = x;
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
  Vector2 solution() const {
    return solution_;
  }

 private:
  OPT_CLASS_UNCOPYABLE(HookeAndJeevesMethod2)

  /**
   * Maximum number of repeats
   */
  int max_iteration_;

  /**
   * Convergence conditions
   * (It is deemed that it converged when the distance moving by repetition is below this distance)
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
  Vector2 solution_;
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_HOOKE_AND_JEEVES_METHOD_2_HPP_
