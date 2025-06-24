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
/// @brief A group of adapters that operate the function
#ifndef HSRB_ANALYTIC_IK_FUNCTION_ADAPTERS_HPP_
#define HSRB_ANALYTIC_IK_FUNCTION_ADAPTERS_HPP_

#include "common.hpp"
#include "vector2.hpp"

namespace opt {

/**
 * This is an adapter that performs an X -axis direction to a variable function.
 */
template<class Function1>
class ShiftAdapterFunction1 {
 public:
  ShiftAdapterFunction1(Function1 func, double shift)
      : function_(func), shift_(shift) {
  }

  bool IsFeasible(double x) {
    return function_.IsFeasible(x - shift_);
  }
  double Value(double x) {
    return function_.Value(x - shift_);
  }
  double Gradient(double x) {
    return function_.Gradient(x - shift_);
  }
  double Hessian(double x) {
    return function_.Hessian(x - shift_);
  }

 private:
  Function1& function_;
  double shift_;
};

/**
 * An adapter that reverses the X -axis direction for one variable function.
 */
template<class Function1>
class ReverseAdapterFunction1 {
 public:
  explicit ReverseAdapterFunction1(Function1& func)
      : function_(func) {
  }

  bool IsFeasible(double x) {
    return function_.IsFeasible(-x);
  }
  double Value(double x) {
    return function_.Value(-x);
  }
  double Gradient(double x) {
    return function_.Gradient(-x);
  }
  double Hessian(double x) {
    return function_.Hessian(-x);
  }

 private:
  Function1& function_;
};

/**
 * 2 This adapter is converted to one variable function G (u) = f (x + u * d), which is straightened in a specific direction.
 */
template<class Function2>
class DirectionAdapterFunction2 {
 public:
  DirectionAdapterFunction2(Function2& func, const Vector2& x, const Vector2& d) :
  function_(func), x_(x), d_(d) {
  }
  bool IsFeasible(double u) {
    return function_.IsFeasible(x_ + u * d_);
  }
  double Value(double u) {
    return function_.Value(x_ + u * d_);
  }
  double Gradient(double u) {
    Vector2 g = function_.Gradient(x_ + u * d_);
    return Vector2::Dot(g, d_);
  }

 private:
  OPT_CLASS_UNCOPYABLE(DirectionAdapterFunction2<Function2>)
  Function2& function_;
  Vector2 x_;
  Vector2 d_;
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_FUNCTION_ADAPTERS_HPP_
