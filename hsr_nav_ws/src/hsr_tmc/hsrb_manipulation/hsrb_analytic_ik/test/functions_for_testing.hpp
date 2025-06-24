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
/// @brief Test function group
#ifndef HSRB_ANALYTIC_IK_FUNCTION_FOR_TESTING_HPP_
#define HSRB_ANALYTIC_IK_FUNCTION_FOR_TESTING_HPP_

#include <limits>

#include "vector2.hpp"

namespace opt {

/**
 * Primary function A for testing.
 * f(x) = 2*x + 1
 * There is no minimum price.
 */
class LinearFunction1A {
 public:
  double Value(double x) {
    return (2.0 * x + 1.0);
  }
  double Gradient(double x) {
    return 2.0;
  }
  double Hessian(double x) {
    return 0;
  }
};

/**
 * Secondary function A for testing.
 * f(x) = (x-1)^2 + 2
 * In x = 1, take the minimum value f (x) = 2.
 */
class QuadraticFunction1A {
 public:
  double Value(double x) {
    return (x * x - 2.0 * x + 3.0);
  }
  double Gradient(double x) {
    return 2.0 * (x - 1.0);
  }
  double Hessian(double x) {
    return 2.0;
  }
};

/**
 * This is a secondary function B for testing.
 * f(x) = 1/2*(x+1)^2 - 1/2
 *In x = -1, the minimum value f (x) = 1/2 is taken.
 */
class QuadraticFunction1B {
 public:
  double Value(double x) {
    return (0.5 * x * (x + 2.0));
  }
  double Gradient(double x) {
    return (x + 1.0);
  }
  double Hessian(double x) {
    return 1.0;
  }
};

/**
 * It is an unauthorized function for testing.
 *
 * f(x) = -x/2        (if x <= -1)
 * f(x) = 2*x + 5/2   (if x > -1)
 *
 * X = -1 has the minimum value F (x) = 0.5.
 */
class NonDiffenrentialFunction1A {
 public:
  double Value(double x) {
    if (x <= -1)
      return -0.5 * x;
    else
      return 2.0 * x + 2.5;
  }
  double Gradient(double x) {
    if (x <= -1)
      return -0.5;
    else
      return 2.0;
  }
  double Hessian(double x) {
    return 0;
  }
};

/**
 * It is a reverse -type function for testing.
 *
 * f(x) = -x + 2 (if x < -1)
 * f(x) = 3      (if  -1 <= x <= 2);
 * f(x) = x+1    (if 2 < x)
 *
 * X = -1 has the minimum value F (x) = 0.5.
 */
class InvertedTrapeziumFunction1A {
 public:
  double Value(double x) {
    if (x < -1.0)
      return 2.0 - x;
    if (x < 2.0)
      return 3.0;
    return x + 1.0;
  }
  double Gradient(double x) {
    if (x < -1.0)
      return -1.0;
    if (x < 2.0)
      return 0.0;
    return 1.0;
  }
  double Hessian(double x) {
    return 0;
  }
};


/**
 * Discontinued function A for testing.
 *
 * f(x) = -2*x + 1 (if x < 1)
 * f(x) = x-1      (if 1 <= x < 2);
 * f(x) = x        (if 2 <= x)
 *
 * X = -1 has the minimum value F (x) = 0.5.
 */
class DiscontinuousFunction1A {
 public:
  double Value(double x) {
    if (x < 1.0)
      return -2.0 * x + 1.0;
    if (x < 2.0)
      return x - 1;
    return x;
  }
  double Gradient(double x) {
    if (x < 1.0)
      return -2.0;
    if (x < 2.0)
      return 1.0;
    return 1.0;
  }
  double Hessian(double x) {
    return 0;
  }
};

/**
 * It is a non -continuous function B for testing.
 * Penalties similar to the penalty function of Robotofunction2.
 *
 * f(x) = x^2        (if -1 <= x <= 1)
 * f(x) = +x + BIG   (if 1 < x)
 * f(x) = -x + BIG   (if x < -1)
 *
 * X = -1 has the minimum value F (x) = 0.5.
 */
class DiscontinuousFnction1B {
 public:
  explicit DiscontinuousFnction1B(double penalty = std::numeric_limits<double>::max())
      : penalty_(penalty) {}

  double Value(double x) {
    if (x < -1) {
      return -x + penalty_;
    } else if (x < 1) {
      return x * x;
    } else {
      return +x + penalty_;
    }
  }
  double Gradient(double x) {
    if (x < -1) {
      return -1;
    } else if (x < 1) {
      return 2 * x;
    } else {
      return +1;
    }
  }
  double Hessian(double x) {
    if (x < -1) {
      return 0;
    } else if (x < 1) {
      return 2;
    } else {
      return 0;
    }
  }

 private:
  double penalty_;
};

/**
 * It is a non -convex function for testing.
 * Sin function is connected at point x = 2n, and the amplitude is ABS (n).
 * X = 2n is impossible to differentiate, but it is easy to understand the minimum point in the section.
 *
 * f(x) = sin (PI * x) * abs(n)   (if x∈[2n-2,2n]);
 *
 * In the section [0, 1], the minimum value is set at x = 0,1.
 *In the section [0, 2], the minimum price is taken at x = 3/2.
 *In the section [0, 4], the minimum price is taken at x = 7/2.
 *
 */
class NonConvexFunction1A {
 public:
  double Value(double x) {
    int n = 1 + std::abs(static_cast<int>(std::floor(x / 2)));
    return sin(M_PI * x) * n;
  }
  double Gradient(double x) {
    int n = 1 + std::abs(static_cast<int>(std::floor(x / 2)));
    return M_PI * cos(M_PI * x) * n;
  }
  double Hessian(double x) {
    int n = 1 + std::abs(static_cast<int>(std::floor(x / 2)));
    return -M_PI * M_PI * sin(M_PI * x) * n;
  }
};

/**
 * Forty -fourth function A for testing.
 *
 * f(x) = (x1-2)^4 + (x1-2*x2)^2
 *
 * Take a minimum of 0 at points (2,1).
 *
 */
class QuarticFunction2A {
 public:
  bool IsFeasible(Vector2 x) {
    return true;
  }
  double Value(Vector2 x) {
    double a = x.v1 - 2;
    double b = x.v1 - 2 * x.v2;
    return (a*a*a*a + b*b);
  }
  Vector2 Gradient(Vector2 x) {
    double a = x.v1 - 2;
    double b = x.v1 - 2 * x.v2;
    return Vector2(4*a*a*a + 2*b, -4*b);
  }
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_FUNCTION_FOR_TESTING_HPP_
