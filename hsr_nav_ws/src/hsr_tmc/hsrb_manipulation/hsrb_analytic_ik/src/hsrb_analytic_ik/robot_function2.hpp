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
/// @brief Purpose function for IK optimization
#ifndef HSRB_ANALYTIC_IK_ROBOT_FUNCTION_2_HPP_
#define HSRB_ANALYTIC_IK_ROBOT_FUNCTION_2_HPP_

#include <algorithm>
#include <utility>

#include <hsrb_analytic_ik/ik_solver_base.hpp>

#include "common.hpp"
#include "vector2.hpp"

namespace opt {

/**
 * Enter the purpose function for IK optimization
 * Current joint angle, etc.
 */
struct RobotFunction2Request {
  /**
   * Simultaneous conversion matrix reference value (T_ref)
   */
  double R11, R12, R13, px;
  double R21, R22, R23, py;
  double R31, R32, R33, pz;

  /**
   * The diagonal component W_i (i = 0..7) of the weight matrix (diagonal matrix)
   */
  double w0, w1, w2, w3, w4, w5, w6, w7;

  /**
   * Reference value of parameters θ^Ref_i (i = 0..7)
   */
  double r0, r1, r2, r3, r4, r5, r6, r7;
};

/**
 * Output of the purpose function for optimizing IK
 * IK results, etc.
 */
struct RobotFunction2Response {
  /**
   * Parameter θ_i (i = 0..7)
   */
  double t0, t1, t2, t3, t4, t5, t6, t7;
};


/**
 * It is a purpose function for IK optimization.
 *
 When you do x = (θ_2, θ_4),
 * f(x) = || W * ( θ^ref(x) - θ(x) ) ||^2
 * is.
 */
class RobotFunction2 {
 public:
  /**
   * It is the type of penalty to be added.
   */
  enum PenaltyType {
    /**
     * No penalty.
     */
    PenaltyNone = 0,

    /**
     * It will be a huge value outside the reality area,
     * Make sure that the value comes out of the area.
     */
    PenaltyBigDiscontinuous = 1,

    /**
     * To the realized area,
     * The value will be greater as it comes out of the area,
     * Continuity is maintained.
     */
    PenaltyBigProportional = 2,
  };

  /**
   * Constructor
   */
  RobotFunction2(const RobotFunction2Request& request,
                 const hsrb_analytic_ik::RobotParameter& parameter)
      : request_(request),
        parameter_(parameter),
        penalty_type_(PenaltyBigProportional), penalty_coeff_(1000) {}

  RobotFunction2Response response() const {
    return response_;
  }

  void set_penalty_coeff(double penalty_coeff) {
    penalty_coeff_ = penalty_coeff;
  }

  void set_penalty_type(PenaltyType penalty_type) {
    penalty_type_ = penalty_type;
  }

  bool t6_use_plus() const {
    return t6_use_plus_;
  }

  /**
   * Judge whether it is a reality.
   */
  bool IsFeasible(const Vector2& x) {
    CalculateTheta_(x);
    return IsFeasibleFromMembers();
  }

  /**
   * Based on the current member variable, it is determined whether it is a reality.
   */
  bool IsFeasibleFromMembers() {
    return ((parameter_.t3_min <= response_.t3 && response_.t3 <= parameter_.t3_max) &&
            (parameter_.t4_min <= response_.t4 && response_.t4 <= parameter_.t4_max) &&
            (parameter_.t5_min <= response_.t5 && response_.t5 <= parameter_.t5_max) &&
            (parameter_.t6_min <= response_.t6 && response_.t6 <= parameter_.t6_max) &&
            (parameter_.t7_min <= response_.t7 && response_.t7 <= parameter_.t7_max));
  }

  /**
   * Forcibly pull back in the realized area.
   */
  void ForceFeasible() {
    if (response_.t4 < parameter_.t4_min) {
      CalculateTheta_(Vector2(response_.t2, parameter_.t4_min));
    } else if (response_.t4 > parameter_.t4_max) {
      CalculateTheta_(Vector2(response_.t2, parameter_.t4_max));
    }

    if (response_.t3 < parameter_.t3_min)
      response_.t3 = parameter_.t3_min;
    else if (response_.t3 > parameter_.t3_max)
      response_.t3 = parameter_.t3_max;

    if (response_.t5 < parameter_.t5_min)
      response_.t5 = parameter_.t5_min;
    else if (response_.t5 > parameter_.t5_max)
      response_.t5 = parameter_.t5_max;

    if (response_.t6 < parameter_.t6_min)
      response_.t6 = parameter_.t6_min;
    else if (response_.t6 > parameter_.t6_max)
      response_.t6 = parameter_.t6_max;

    if (response_.t7 < parameter_.t7_min)
      response_.t7 = parameter_.t7_min;
    else if (response_.t7 > parameter_.t7_max)
      response_.t7 = parameter_.t7_max;
  }

  /**
   * Get function value.
   */
  double Value(const Vector2& x) {
    // Calculate other θ_i from θ_2, θ_4.
    CalculateTheta_(x);

    return ValueFromMembers();
  }

  /**
   * Get the function value from the currently set T_I (I = 0, .., 7).
   */
  double ValueFromMembers() {
    if (penalty_type_ == PenaltyBigDiscontinuous) {
      // If it is in the realized area
      if (IsFeasibleFromMembers()) {
        // Calculate the function value.
        return ValueWithoutPenaltyFromMembers();
      } else {
        // In the case of a reality area, a discontinuous barrier is set up according to the protruding condition.
        // If the size of the penalty is too large, it will not fit in the accuracy range of DOUBLE.
        // Please note that the optimization will not work properly.
        return penalty_coeff_ + GetPenaltyGrade();
      }
    } else if (penalty_type_ == PenaltyBigProportional) {
      // In the case of a reality area, a continuous barrier is set up according to the protruding condition.
      return ValueWithoutPenaltyFromMembers() + penalty_coeff_ * GetPenaltyGrade();
    } else {
      // Calculate the function value.
      return ValueWithoutPenaltyFromMembers();
    }
  }

  /**
   * Calculate the degree of execution from the executable area by weight.
   * This value is used for penalty calculation.
   */
  double GetPenaltyGrade() {
    return request_.w3 * Plus_(parameter_.t3_min - response_.t3) +
           request_.w3 * Plus_(response_.t3 - parameter_.t3_max) +
           request_.w4 * Plus_(parameter_.t4_min - response_.t4) +
           request_.w4 * Plus_(response_.t4 - parameter_.t4_max) +
           request_.w5 * Plus_(parameter_.t5_min - response_.t5) +
           request_.w5 * Plus_(response_.t5 - parameter_.t5_max) +
           request_.w6 * Plus_(parameter_.t6_min - response_.t6) +
           request_.w6 * Plus_(response_.t6 - parameter_.t6_max) +
           request_.w7 * Plus_(parameter_.t7_min - response_.t7) +
           request_.w7 * Plus_(response_.t7 - parameter_.t7_max);
  }

  /**
   * Calculate the outstanding condition from the executable area without weight.
   * This value is a straight line in the executable area,
   * When the calculation error is converged to the outside point,
   * It can be used to determine whether to force a force back to the point.
   */
  double GetOuterGrade() {
    return Plus_(parameter_.t3_min - response_.t3) +
           Plus_(response_.t3 - parameter_.t3_max) +
           Plus_(parameter_.t4_min - response_.t4) +
           Plus_(response_.t4 - parameter_.t4_max) +
           Plus_(parameter_.t5_min - response_.t5) +
           Plus_(response_.t5 - parameter_.t5_max) +
           Plus_(parameter_.t6_min - response_.t6) +
           Plus_(response_.t6 - parameter_.t6_max) +
           Plus_(parameter_.t7_min - response_.t7) +
           Plus_(response_.t7 - parameter_.t7_max);
  }

  /**
   * Calculate the purpose function value without penalty based on the current member variable.
   */
  double ValueWithoutPenaltyFromMembers() {
    double V = 0;
    double a;
    a = request_.w0 * (request_.r0 - response_.t0);
    V += a * a;
    a = request_.w1 * (request_.r1 - response_.t1);
    V += a * a;
    a = request_.w2 * (request_.r2 - response_.t2);
    V += a * a;
    a = request_.w3 * (request_.r3 - response_.t3);
    V += a * a;
    a = request_.w4 * (request_.r4 - response_.t4);
    V += a * a;
    a = request_.w5 * (request_.r5 - response_.t5);
    V += a * a;
    a = request_.w6 * (request_.r6 - response_.t6);
    V += a * a;
    a = request_.w7 * (request_.r7 - response_.t7);
    V += a * a;
    return V;
  }

  /**
   * Get the gradient value.
   */
  Vector2 Gradient(const Vector2& x) {
    response_.t2 = x.v1;
    response_.t4 = x.v2;

    // Calculate other θ_i from θ_2, θ_4.
    CalculateTheta_(x);

    const double S2 = sin(response_.t2);
    const double C2 = cos(response_.t2);
    const double S4 = sin(response_.t4);
    const double C4 = cos(response_.t4);

    // Calculate the T0's uneven differential value.
    double d0_2 = - parameter_.L52 * S2 * S4 + parameter_.L51 * S2 * C4
                  + parameter_.L42 * C2 + parameter_.L41 * S2;
    double d0_4 = parameter_.L52 * C2 * C4 + parameter_.L51 * C2 * S4;

    // Calculate the dyseral value of T1.
    double d1_2 = parameter_.L52 * C2 * S4 - parameter_.L51 * C2 * C4
                + parameter_.L42 * S2 - parameter_.L41 * C2;
    double d1_4 = parameter_.L52 * S2 * C4 + parameter_.L51 * S2 * S4;

    // Calculate the T2 uneven differential value.
    double d2_2 = 1;
    double d2_4 = 0;

    // Calculate the T3 uneven differential value.
    double d3_2 = 0;
    double d3_4 = parameter_.L52 * S4 - parameter_.L51 * C4;

    // Calculate the T4 unnecessary value.
    double d4_2 = 0;
    double d4_4 = 1;

    // Calculate the T6 uneven differential value.
    double d6_2;
    double d6_4;
    {
      double a = request_.R23 * S2 * S4 + request_.R13 * C2 * S4 - request_.R33 * C4;
      a *= a;
      double b = request_.R13 * S2 * S4 - request_.R23 * C2 * S4;
      double c = -(request_.R33 * S4 + request_.R23 * S2 * C4 + request_.R13 * C2 * C4);

      double d = 1 / std::sqrt(1 - a);
      d6_2 = b * d;
      d6_4 = c * d;

      // Inverts ± when T6 is calculated to select a sign.
      if (t6_use_plus_) {
        d6_2 = -d6_2;
        d6_4 = -d6_4;
      }
    }

    // Calculate the T5 unnecessary value.
    double d5_2;
    double d5_4;
    {
      double a1 = (request_.R23 * S2 + request_.R13 * C2);
      double P1 = a1 * request_.R33 * S4
                + (request_.R23 * request_.R23 + request_.R13 * request_.R13) * C4;
      double Q1 = 2 * request_.R13 * request_.R23 * C2 * S2
                + (request_.R13 * request_.R13 - request_.R23 * request_.R23) * C2 * C2
                - request_.R33 * request_.R33 + request_.R23 * request_.R23;
      double R1 = -2 * a1 * request_.R33 * C4 * S4
                - (request_.R23 * request_.R23 + request_.R13 * request_.R13);
      d5_2 = P1 / (Q1 * S4 * S4 + R1);

      double b1 = request_.R13 * S2 - request_.R23 * C2;
      double P2 = a1 * b1 * S4
                + (request_.R23 * request_.R33 * C2 - request_.R13 * request_.R33 * S2) * C4;
      double Q2 = request_.R33 * request_.R33 * S4 * S4
                + 2 * a1 * request_.R33 * C4 * S4;
      double R2 = a1 * a1 * C4 * C4 + b1 * b1;
      d5_4 = -P2 / (Q2 + R2);
    }

    // Calculate the T7 uneven differential value.
    double d7_2 = 0;
    double d7_4 = 0;
    {
      double a2 = request_.R12 * request_.R31 - request_.R11 * request_.R32;
      double b2 = request_.R21 * request_.R32 - request_.R22 * request_.R31;
      double c2 = 2 * (request_.R12 * request_.R22 + request_.R11 * request_.R21) * C2 * S2;

      double Z = -2 * ((request_.R22 * request_.R32 + request_.R21 * request_.R31) * S2
               + (request_.R12 * request_.R32 + request_.R11 * request_.R31) * C2) * C4 * S4;

      double P3 = (request_.R11 * request_.R22 - request_.R12 * request_.R21) * S4 * S4
                + (a2 * S2 + b2 * C2) * C4 * S4;
      double Q3 = (c2 + (- (request_.R22 * request_.R22 + request_.R21 * request_.R21)
                         + (request_.R12 * request_.R12 + request_.R11 * request_.R11)) * C2 * C2
                   + request_.R22 * request_.R22 + request_.R21 * request_.R21) * S4 * S4;
      double U3 = Z + (request_.R32 * request_.R32 + request_.R31 * request_.R31) * C4 * C4;

      d7_2 = -P3 / (Q3 + U3);

      double P4 = b2 * S2 - a2 * C2;
      double Q4 = ((request_.R22 * request_.R22 + request_.R21 * request_.R21) * S2 * S2 + c2
                   + (request_.R12 * request_.R12 + request_.R11 * request_.R11) * C2 * C2
                   - (request_.R32 * request_.R32 + request_.R31 * request_.R31)) * S4 * S4;
      double U4 = Z + request_.R32 * request_.R32 + request_.R31 * request_.R31;

      d7_4 = -P4 / (Q4 + U4);
    }

    // Calculate the gradient value.
    double a0 = request_.w0 * request_.w0 * (request_.r0 - response_.t0);
    double a1 = request_.w1 * request_.w1 * (request_.r1 - response_.t1);
    double a2 = request_.w2 * request_.w2 * (request_.r2 - response_.t2);
    double a3 = request_.w3 * request_.w3 * (request_.r3 - response_.t3);
    double a4 = request_.w4 * request_.w4 * (request_.r4 - response_.t4);
    double a5 = request_.w5 * request_.w5 * (request_.r5 - response_.t5);
    double a6 = request_.w6 * request_.w6 * (request_.r6 - response_.t6);
    double a7 = request_.w7 * request_.w7 * (request_.r7 - response_.t7);

    Vector2 g;
    g.v1 = d0_2 * a0 + d1_2 * a1 + d2_2 * a2 + d3_2 * a3 + d4_2 * a4 + d5_2 * a5
    + d6_2 * a6 + d7_2 * a7;
    g.v2 = d0_4 * a0 + d1_4 * a1 + d2_4 * a2 + d3_4 * a3 + d4_4 * a4 + d5_4 * a5
    + d6_4 * a6 + d7_4 * a7;

    g = -2.0 * g;

    // If there is no penalty, return the calculated gradient.
    if (penalty_type_ == PenaltyNone) {
      return g;
    }

    // Calculate the gradient according to the extent that it has protruded.
    double h2 = 0;
    double h4 = 0;

    if (response_.t3 > parameter_.t3_max) {
      h2 += request_.w3 * d3_2;
      h4 += request_.w3 * d3_4;
    } else if (response_.t3 < parameter_.t3_min) {
      h2 -= request_.w3 * d3_2;
      h4 -= request_.w3 * d3_4;
    }

    if (response_.t4 > parameter_.t4_max) {
      h2 += request_.w4 * d4_2;
      h4 += request_.w4 * d4_4;
    } else if (response_.t4 < parameter_.t4_min) {
      h2 -= request_.w4 * d4_2;
      h4 -= request_.w4 * d4_4;
    }

    if (response_.t5 > parameter_.t5_max) {
      h2 += request_.w5 * d5_2;
      h4 += request_.w5 * d5_4;
    } else if (response_.t5 < parameter_.t5_min) {
      h2 -= request_.w5 * d5_2;
      h4 -= request_.w5 * d5_4;
    }

    if (response_.t6 > parameter_.t6_max) {
      h2 += request_.w6 * d6_2;
      h4 += request_.w6 * d6_4;
    } else if (response_.t6 < parameter_.t6_min) {
      h2 -= request_.w6 * d6_2;
      h4 -= request_.w6 * d6_4;
    }

    if (response_.t7 > parameter_.t7_max) {
      h2 += request_.w7 * d7_2;
      h4 += request_.w7 * d7_4;
    } else if (response_.t7 < parameter_.t7_min) {
      h2 -= request_.w7 * d7_2;
      h4 -= request_.w7 * d7_4;
    }

    Vector2 h(h2, h4);

    // In the case of unexpected penalty
    if (penalty_type_ == PenaltyBigDiscontinuous) {
      return h;
    } else if (penalty_type_ == PenaltyBigProportional) {
    // In the case of continuous penalty
      return g + penalty_coeff_ * h;
    } else {
      return g;
    }
  }

  /**
   * Narrow down the range that θ4 is available from the limit range of θ3.
   *
   * Returns the lower limit of @param Lower θ4.The value is more than t4_min.
   * Returns the upper limit of @Param Upper θ4.T4_max is less than the value.
   * If there is no possible range in @return θ4, return False.
   * In this case, LOWER, UPPER is undefined.
   */
  bool GetTheta4Boundary(double& lower, double& upper) {
    const double A = parameter_.L52;
    const double B = parameter_.L51;
    const double C0 = -request_.R33 * parameter_.L82 + request_.R31 * parameter_.L81 - parameter_.L3 + request_.pz;
    const double C1 = C0 - parameter_.t3_max;
    const double C2 = C0 - parameter_.t3_min;

    const double AA_BB = A * A + B * B;
    const double SQRT_AA_BB = std::sqrt(AA_BB);

    const double Cmax = +SQRT_AA_BB;
    const double Cmin = -SQRT_AA_BB;

    const double X_Cmax = +A / SQRT_AA_BB;
    const double X_Cmin = -A / SQRT_AA_BB;

    double Xmin;
    double Xmax;

    // CMIN, A, CMAX and C1, C2 positions XMIN and XMAX.
    if (C1 < Cmin) {
      if (C2 < Cmin) {
        return false;
      } else if (C2 <= A) {
        Xmin = X_Cmin;
        Xmax = X_C_(C2);
      } else {
        Xmin = X_Cmin;
        Xmax = 1.0;
      }
    } else if (C1 <= A) {
      if (C2 <= A) {
        Xmin = X_C_(C1);
        Xmax = X_C_(C2);
      } else if (C2 <= Cmax) {
        Xmin = std::min(X_C_(C1), X_C_(C2));
        Xmax = 1.0;
      } else {
        Xmin = std::min(X_C_(C1), X_Cmax);
        Xmax = 1.0;
      }
    } else if (C1 <= Cmax) {
      if (C2 <= Cmax) {
        Xmin = X_C_(C2);
        Xmax = X_C_(C1);
      } else {
        Xmin = X_Cmax;
        Xmax = X_C_(C1);
      }
    } else {
      return false;
    }

    // If XMIN> XMAX due to calculation error, replace it.
    if (Xmin > Xmax) {
      std::swap(Xmin, Xmax);
    }

    // Xmin and Xmax exceed the range of [-1,+1] due to calculation errors,
    // Make sure that NAN is not generated in the subsequent ACOS.
    Xmin = std::max(-1.0, Xmin);
    Xmax = std::min(+1.0, Xmax);

    // Use the lower and upper limit of θ4 from Xmin and Xmax.
    lower = std::max(-acos(Xmin), parameter_.t4_min);
    upper = std::min(-acos(Xmax), parameter_.t4_max);

    if (lower > upper) {
      std::swap(lower, upper);
    }

    return true;
  }


 private:
  OPT_CLASS_UNCOPYABLE(RobotFunction2)

  RobotFunction2Request request_;
  hsrb_analytic_ik::RobotParameter parameter_;
  RobotFunction2Response response_;

  /**
   * When calculating T2 and T4 from T6, when the code + is adopted, TRUE,
   If you adopt * of the sign, record False.
   * This member variable is updated by the CalculateTheta function,
   * Used in the Gradient method.
   */
  bool t6_use_plus_;

  /**
   * Penalty type.
   * The default is PENALTYBIGPROPRTIONAL.
   */
  PenaltyType penalty_type_;

  /**
   * Penalty coefficient.
   * Executable areas only in the protruding condition, only the value that multiplies this coefficient,
   * Penalty is applied.
   * The default is 1000.
   */
  double penalty_coeff_;

  /**
   * Calculate t_i (i = 0, ..., 7) from the value passed (θ_2, θ_4) passed to the argument.
   */
  void CalculateTheta_(const Vector2& x) {
    response_.t2 = x.v1;
    response_.t4 = x.v2;

    const double S2 = sin(response_.t2);
    const double C2 = cos(response_.t2);
    const double S4 = sin(response_.t4);
    const double C4 = cos(response_.t4);

    response_.t0 = - request_.R13 * parameter_.L82
                   + request_.R11 * parameter_.L81
                   + parameter_.L52 * C2 * S4
                   - parameter_.L51 * C2 * C4
                   + parameter_.L42 * S2
                   - parameter_.L41 * C2
                   + request_.px;

    response_.t1 = - request_.R23 * parameter_.L82
                   + request_.R21 * parameter_.L81
                   + parameter_.L52 * S2 * S4
                   - parameter_.L51 * S2 * C4
                   - parameter_.L42 * C2
                   - parameter_.L41 * S2
                   + request_.py;

    response_.t3 = - request_.R33 * parameter_.L82
                   + request_.R31 * parameter_.L81
                   - parameter_.L52 * C4
                   - parameter_.L51 * S4
                   - parameter_.L3
                   + request_.pz;

    // Calculate T6.
    {
      double b = -request_.R23 * S2 * S4 - request_.R13 * C2 * S4 + request_.R33 * C4;
      double a = std::sqrt(1 - b * b);
      response_.t6 = std::atan2(a, b);

      // Here, T6 has ± uncertainty,
      // Select the target function value so that it is smaller.
      // Since the purpose function value depends on T5 and T7, which depends on T6.
      // After calculation of T5, T7, select a sign.
    }

    const double S6 = sin(response_.t6);

    if (S6 != 0) {
      // Since the division is expensive, the reverse number is multiplied.
      // const double S6_inv = 1.0 / S6;

      // Sin (T6) divisions are considered to be unstable the numerical calculation.
      // Only the sign is multiplied by the argument of ATAN2.
      const double S6_inv = (S6 > 0) ? 1.0 : -1.0;

      // Calculate T5.
      double t5_plus, t5_minus;
      {
        double a = (request_.R23 * C2 - request_.R13 * S2) * S6_inv;
        double b = -(request_.R33 * S4 + request_.R23 * S2 * C4 + request_.R13 * C2 * C4) * S6_inv;
        t5_plus = -std::atan2(a, b);
        t5_minus = -std::atan2(-a, -b);

        // It is normalized so that T5 is within the range of [t5_min, t5_max].
        // Since T5 is given as a result of ATAN2, it is within [-pi,+pi].
        // Therefore, it is enough to add 2*pi at most.
        if (t5_plus < parameter_.t5_min) {
          t5_plus += 2 * M_PI;
        }
        if (t5_minus < parameter_.t5_min) {
          t5_minus += 2 * M_PI;
        }
      }

      // Calculate T7.
      double t7_plus, t7_minus;
      {
        double a = (-request_.R22 * S2 * S4 - request_.R12 * C2 * S4 + request_.R32 * C4) * S6_inv;
        double b = -(-request_.R21 * S2 * S4 - request_.R11 * C2 * S4 + request_.R31 * C4) * S6_inv;
        t7_plus = std::atan2(a, b);
        t7_minus = std::atan2(-a, -b);

        // It is normalized so that T7 is within the range of [t7_min, t7_max].
        // Same as T5.
        if (t7_plus < parameter_.t7_min) {
          t7_plus += 2 * M_PI;
        }
        if (t7_minus < parameter_.t7_min) {
          t7_minus += 2 * M_PI;
        }
      }

      // Calculate the target function value in the case of the T6's normal mark and negative marks,
      // Select a sign.
      {
        double t6_plus = response_.t6;
        double t6_minus = -response_.t6;

        response_.t5 = t5_plus;
        response_.t6 = t6_plus;
        response_.t7 = t7_plus;
        double value_plus = ValueFromMembers();

        response_.t5 = t5_minus;
        response_.t6 = t6_minus;
        response_.t7 = t7_minus;
        double value_minus = ValueFromMembers();

        if (value_plus < value_minus) {
          t6_use_plus_ = true;
          response_.t5 = t5_plus;
          response_.t6 = t6_plus;
          response_.t7 = t7_plus;
        } else {
          t6_use_plus_ = false;
        }
      }
    } else {
      // In the case of T6 = 0, pi.
      // Since T6 = Pi is out of the movable range of T6, it is not considered here.
      double a = request_.R11 * S2 - request_.R21 * C2;
      double b = request_.R12 * S2 - request_.R22 * C2;
      double A = std::atan2(a, b);

      // Calculate T5.
      response_.t5 = (request_.w7 * request_.w7 * (A - request_.r7)
                      + request_.r5 * request_.w5 * request_.w5) /
                     (request_.w7 * request_.w7 + request_.w5 * request_.w5);

      if (parameter_.t5_max < response_.t5) {
        response_.t5 = std::min(parameter_.t5_max, A - parameter_.t7_min);
      } else if (response_.t5 < parameter_.t5_min) {
        response_.t5 = std::max(parameter_.t5_min, A - parameter_.t7_max);
      }

      // Calculate T7.
      response_.t7 = -response_.t5 + A;

      // What is the normalization of T5 and T7?
    }
  }

  /**
   * Plus function.
   */
  inline static double Plus_(double x) {
    return (x > 0) ? x : 0;
  }

  inline double X_C_(double C) {
    const double A = parameter_.L52;
    const double B = parameter_.L51;
    const double AA_BB = A * A + B * B;

    double AA_BB_CC = AA_BB - C * C;

    // A*a+b*b-c*c may be a negative number depending on the calculation error, so
    // Clip properly.
    if (AA_BB_CC < 0)
      AA_BB_CC = 0;

    return (A * C + B * std::sqrt(AA_BB_CC)) / AA_BB;
  }
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_ROBOT_FUNCTION_2_HPP_
