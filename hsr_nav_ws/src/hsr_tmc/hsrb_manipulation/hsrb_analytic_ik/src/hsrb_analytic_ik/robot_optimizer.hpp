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
/// @file     RobotOptimizer.hpp
#ifndef HSRB_ANALYTIC_IK_ROBOT_OPTIMIZER_HPP_
#define HSRB_ANALYTIC_IK_ROBOT_OPTIMIZER_HPP_

#include <iostream>
#include <limits>

#include "bi_golden_section_line_search.hpp"
#include "hooke_and_jeeves_method2.hpp"
#include "robot_function2.hpp"

namespace opt {

/**
 * 2 Make optimization in various methods for variable function Robotfunction2
 * Provides functions.
 */
class RobotOptimizer {
 public:
  static const int maxItor = 500;

 public:
  /**
   * An optimization entry point.
   */
  static OptResult Optimize(RobotFunction2& f) {
    // Parameters that were the best results in the evaluation
    return OptimizeByHookeAndJeevesMethod(f, 1e-3, 1e-4, 1.0, 1e7);
  }

 private:
  /**
   * The initial point of optimization is determined.
   */
  static Vector2 FindInitPoint(RobotFunction2& f) {
    Vector2 x0(0, -1);
    double t2_lower = -M_PI;
    double t2_upper = +M_PI;
    double t4_lower;
    double t4_upper;

    // Use the lower limit and upper limit of θ4 from the limit range of θ3.
    if (!f.GetTheta4Boundary(t4_lower, t4_upper)) {
      // If neither θ4 can be executed, it is meaningless to search for the initial value.
      // Returns the default value of the initial value.
      return x0;
    }

    Vector2 x0_feasible;
    Vector2 x0_infeasible;
    double min_value_feasible = std::numeric_limits<double>::max();
    double min_value_infeasible = std::numeric_limits<double>::max();
    bool feasible_found = false;

    for (int grid = 10; grid <= 50; grid += 10) {
      // Find the grid width.
      double t2_grid_width = (t2_upper - t2_lower) / (grid + 2);
      double t4_grid_width = (t4_upper - t4_lower) / (grid + 2);

      // List the T2 grid.
      for (int t2_index = 1; t2_index < grid + 2; t2_index++) {
        // Calculate the grid coordinates of T2.
        double t2 = t2_lower + t2_grid_width * t2_index;

        // List the T4 grid.
        for (int t4_index = 1; t4_index < grid + 2; t4_index++) {
          // Calculate the grid coordinates of T4.
          double t4 = t4_lower + t4_grid_width * t4_index;

          // Get the target function value and feasibility of points (T2, T4).
          Vector2 x(t2, t4);
          double value = f.Value(x);
          bool feasible = f.IsFeasibleFromMembers();

          if (feasible) {
            if (value < min_value_feasible) {
              min_value_feasible = value;
              x0_feasible = x;
              feasible_found = true;
            }
          } else {
            if (value < min_value_infeasible) {
              min_value_infeasible = value;
              x0_infeasible = x;
            }
          }
        }
      }

      if (feasible_found) {
        x0 = x0_feasible;
        break;
      } else {
        x0 = x0_infeasible;
        continue;
      }
    }

    return x0;
  }

  /**
   * Hooke-and-Jeeves 法
   */
  static OptResult OptimizeByHookeAndJeevesMethod(RobotFunction2& f,
                                                  double epsilon,
                                                  double lineEpsilon,
                                                  double step,
                                                  double penaltyCoeff) {
    BiGoldenSectionLineSearch lineSearch(maxItor, lineEpsilon);
    HookeAndJeevesMethod2 method(maxItor, epsilon);
    f.set_penalty_coeff(penaltyCoeff);

    // We will optimize.
    Vector2 x0 = FindInitPoint(f);
    OptResult result = method.Search(f, lineSearch, x0, step);

    // Re -calculate F with the optimal solution.
    f.Value(method.solution());

    return result;
  }
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_ROBOT_OPTIMIZER_HPP_
