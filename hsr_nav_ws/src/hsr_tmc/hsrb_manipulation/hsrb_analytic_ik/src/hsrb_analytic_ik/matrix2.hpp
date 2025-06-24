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
/// @brief Class that represents 2x2 matrix
#ifndef HSRB_ANALYTIC_IK_MATRIX_2_HPP_
#define HSRB_ANALYTIC_IK_MATRIX_2_HPP_

#include "vector2.hpp"

namespace opt {

/**
 * A class that represents a 2x2 matrix.
 */
struct Matrix2 {
  double m11;
  double m12;
  double m21;
  double m22;

  Matrix2() {
  }

  Matrix2(double a11, double a12, double a21, double a22)
      : m11(a11), m12(a12), m21(a21), m22(a22) {
  }

  void Zero() {
    m11 = m12 = m21 = m22 = 0;
  }

  void Identity() {
    m11 = 1;
    m12 = 0;
    m21 = 0;
    m22 = 1;
  }

  static Matrix2 Inv(Matrix2 M) {
    double a = 1 / (M.m11 * M.m22 - M.m12 * M.m21);
    return Matrix2(a * M.m22, -a * M.m12, -a * M.m21, a * M.m11);
  }

  /**
   * Calculate the matrix x* y '.
   */
  static Matrix2 Tod(const Vector2& x, const Vector2& y) {
    return Matrix2(x.v1 * y.v1, x.v1 * y.v2, x.v2 * y.v1, x.v2 * y.v2);
  }
};

inline Matrix2 operator+(const Matrix2& A, const Matrix2& B) {
  return Matrix2(A.m11 + B.m11, A.m12 + B.m12, A.m21 + B.m21, A.m22 + B.m22);
}

inline Matrix2 operator-(const Matrix2& A, const Matrix2& B) {
  return Matrix2(A.m11 - B.m11, A.m12 - B.m12, A.m21 - B.m21, A.m22 - B.m22);
}

inline Vector2 operator*(const Matrix2& A, const Vector2& x) {
  return Vector2(A.m11 * x.v1 + A.m12 * x.v2, A.m21 * x.v1 + A.m22 * x.v2);
}

inline Matrix2 operator*(const Matrix2& A, double a) {
  return Matrix2(A.m11 * a, A.m12 * a, A.m21 * a, A.m22 * a);
}

inline Matrix2 operator*(double a, const Matrix2& A) {
  return Matrix2(a * A.m11, a * A.m12, a * A.m21, a * A.m22);
}

inline Matrix2 operator/(const Matrix2& A, double a) {
  double b = 1 / a;
  return Matrix2(A.m11 * b, A.m12 * b, A.m21 * b, A.m22 * b);
}

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_MATRIX_2_HPP_
