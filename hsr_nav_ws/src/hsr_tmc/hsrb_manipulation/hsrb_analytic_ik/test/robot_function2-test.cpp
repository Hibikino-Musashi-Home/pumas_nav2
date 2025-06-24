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
/// @brief Test of purpose function for IK optimization
#include <gtest/gtest.h>

#include "robot_function2.hpp"

namespace opt {
////////////////////////////////////////////////////////////////////////////////
//// Comparison test with calculation results by maxima

/*
 Test case 1 (when + is selected by calculation of θ_6)

 W = (0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9)
 θref = ( 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9)
 θ2= 1.7
 θ4= 1.8

 R[1,1]=-0.91009974788523
 R[2,1]=0.41187596402249
 R[3,1]=-0.045570156460034

 R[1,2]=-0.34238207861077
 R[2,2]=-0.68544490869513
 R[3,2]=0.64260391330126

 R[1,3]=0.23343727454161
 R[2,3]=0.60043606437694
 R[3,3]=0.76484218728449

 px=0.2
 py=0.5
 pz=0.7

 At the time

 Parameter value:
 θ0	0.208362584
 θ1	0.625109915
 θ3	0.325508315
 θ6(+)	2.380876721
 θ5(+)	2.677089633
 θ7(+)	0.756052367

 Function value:
 E(+)	2.197715882

 Divisional value:
 θ1
 /∂θ2	0.052081718
 /∂θ4	-0.072902719
 θ3
 /∂θ2	0
 /∂θ4	0.337113443
 θ6(+)
 /∂θ2	-0.436262835
 /∂θ4	0.894044312
 θ5(+)
 /∂θ2	-0.687537509
 /∂θ4	-0.470657117
 θ7(+)
 /∂θ2	-1.262854896
 /∂θ4	-0.649771413

 Slope value:
 E(+)
 /∂θ2	1.463493993
 /∂θ4	1.372354674
 */

TEST(RobotFunction2_Test, case1_t6_plus) {
  opt::RobotFunction2Request function_req;
  function_req.w0 = 0.2;
  function_req.w1 = 0.3;
  function_req.w2 = 0.4;
  function_req.w3 = 0.5;
  function_req.w4 = 0.6;
  function_req.w5 = 0.7;
  function_req.w6 = 0.8;
  function_req.w7 = 0.9;

  function_req.r0 = 1.2;
  function_req.r1 = 1.3;
  function_req.r2 = 1.4;
  function_req.r3 = 1.5;
  function_req.r4 = 1.6;
  function_req.r5 = 1.7;
  function_req.r6 = 1.8;
  function_req.r7 = 1.9;

  function_req.R11 = -0.91009974788523;
  function_req.R21 = 0.41187596402249;
  function_req.R31 = -0.045570156460034;

  function_req.R12 = -0.34238207861077;
  function_req.R22 = -0.68544490869513;
  function_req.R32 = 0.64260391330126;

  function_req.R13 = 0.23343727454161;
  function_req.R23 = 0.60043606437694;
  function_req.R33 = 0.76484218728449;

  function_req.px = 0.2;
  function_req.py = 0.5;
  function_req.pz = 0.7;

  Vector2 x(1.7, 1.8);

  double value_expected = 2.19771588208185;
  double grad1_expected = 1.46349399279955;
  double grad2_expected = 1.37235467372499;
  double epsilon = 1e-8;

  opt::RobotFunction2 f(function_req, hsrb_analytic_ik::RobotParameter());
  f.set_penalty_type(RobotFunction2::PenaltyNone);
  // Verify the function value.
  double value = f.Value(x);
  EXPECT_TRUE(f.t6_use_plus());
  EXPECT_NEAR(value_expected, value, epsilon);

  // Verify the gradient.
  Vector2 grad = f.Gradient(x);
  EXPECT_NEAR(grad1_expected, grad.v1, epsilon);
  EXPECT_NEAR(grad2_expected, grad.v2, epsilon);
}

/*
 Test case 4 (when the calculation of θ_6 is selected)

 W = (0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9)
 θref = ( -1.2, -1.3, -1.4, -1.5, -1.6, -1.7, -1.8, -1.9)
 θ2= 2.5
 θ4= -2.7

 R[1,1]=-0.91009974788523
 R[2,1]=0.41187596402249
 R[3,1]=-0.045570156460034

 R[1,2]=-0.34238207861077
 R[2,2]=-0.68544490869513
 R[3,2]=0.64260391330126

 R[1,3]=0.23343727454161
 R[2,3]=0.60043606437694
 R[3,3]=0.76484218728449

 px=0.2
 py=0.5
 pz=0.7

 At the time

 Parameter value:
 θ0		0.43042695341123
 θ1		0.31314882530530
 θ3		0.56603461920606
 θ6(-)	-2.23676768325620
 θ5(-)	4.051470388272410
 θ7(-)	0.948794949289840

 Function value:
 E(-)	27.181523266162000

 Divisional value:
 θ0
 /∂θ2	0.10743241921801
 /∂θ4	0.25159257378924
 θ1
 /∂θ2	0.27414608745895
 /∂θ4	-0.18794526244023
 θ3
 /∂θ2	0.00000000000000
 /∂θ4	-0.14292569797059
 θ6(-)
 /∂θ2	-0.33738594068214
 /∂θ4	0.61384227382649
 θ5(-)
 /∂θ2	0.69794394201724
 /∂θ4	-0.62026810801162
 θ7(-)
 /∂θ2	-0.33363621490161
 /∂θ4	-1.00395726436927

 Slope value:
 E(-)
 /∂θ2	3.92440840017665
 /∂θ4	-9.43399316103728

 */

TEST(RobotFunction2_Test, case3_t6_minus) {
  opt::RobotFunction2Request function_req;
  function_req.w0 = 0.2;
  function_req.w1 = 0.3;
  function_req.w2 = 0.4;
  function_req.w3 = 0.5;
  function_req.w4 = 0.6;
  function_req.w5 = 0.7;
  function_req.w6 = 0.8;
  function_req.w7 = 0.9;

  function_req.r0 = -1.2;
  function_req.r1 = -1.3;
  function_req.r2 = -1.4;
  function_req.r3 = -1.5;
  function_req.r4 = -1.6;
  function_req.r5 = -1.7;
  function_req.r6 = -1.8;
  function_req.r7 = -1.9;

  function_req.R11 = -0.91009974788523;
  function_req.R21 = 0.41187596402249;
  function_req.R31 = -0.045570156460034;

  function_req.R12 = -0.34238207861077;
  function_req.R22 = -0.68544490869513;
  function_req.R32 = 0.64260391330126;

  function_req.R13 = 0.23343727454161;
  function_req.R23 = 0.60043606437694;
  function_req.R33 = 0.76484218728449;

  function_req.px = 0.2;
  function_req.py = 0.5;
  function_req.pz = 0.7;

  Vector2 x(2.5, -2.7);

  double value_expected = 27.181523266162000;
  double grad1_expected = 3.92440840017665;
  double grad2_expected = -9.43399316103728;
  double epsilon = 1e-8;

  opt::RobotFunction2 f(function_req, hsrb_analytic_ik::RobotParameter());
  f.set_penalty_type(RobotFunction2::PenaltyNone);

  // Verify the function value.
  double value = f.Value(x);
  EXPECT_FALSE(f.t6_use_plus());
  EXPECT_NEAR(value_expected, value, epsilon);

  // Verify the gradient.
  Vector2 grad = f.Gradient(x);
  EXPECT_NEAR(grad1_expected, grad.v1, epsilon);
  EXPECT_NEAR(grad2_expected, grad.v2, epsilon);
}

////////////////////////////////////////////////////////////////////////////////
//// Gettheta4Boundary function test

TEST(RobotFunction2_Test, GetTheta4Boundary_Test) {
  // Set the parameters provided by the test driver RandomTest.
  opt::RobotFunction2Request function_req;
  function_req.r0 = -0.93168;
  function_req.r1 = 0.87079;
  function_req.r2 = -2.73357;
  function_req.r3 = 0.489344;
  function_req.r4 = -2.6188;
  function_req.r5 = 3.07458;
  function_req.r6 = -0.0421796;
  function_req.r7 = -0.808225;

  function_req.w0 = 10;
  function_req.w1 = 10;
  function_req.w2 = 1;
  function_req.w3 = 10;
  function_req.w4 = 1;
  function_req.w5 = 1;
  function_req.w6 = 1;
  function_req.w7 = 1;

  function_req.R11 = 0.250973665183036;
  function_req.R12 = -0.938863588891737;
  function_req.R13 = 0.235684918562729;
  function_req.R21 = 0.480183765799543;
  function_req.R22 = -0.0906571186691201;
  function_req.R23 = -0.872470536979434;
  function_req.R31 = 0.840497335181476;
  function_req.R32 = 0.33213920016755;
  function_req.R33 = 0.428074504338786;
  function_req.px = 0.72868365535673;
  function_req.py = -0.809962838009126;
  function_req.pz = 0.630257865249447;

  opt::RobotFunction2 f(function_req, hsrb_analytic_ik::RobotParameter());

  double t4_lower, t4_upper;
  bool feasible_exists = f.GetTheta4Boundary(t4_lower, t4_upper);
  EXPECT_TRUE(feasible_exists);

  // Make sure the value is visually consistent with the graph.
  // It is better to write a test code compared to the value calculated separately.
}

}  // namespace opt

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
