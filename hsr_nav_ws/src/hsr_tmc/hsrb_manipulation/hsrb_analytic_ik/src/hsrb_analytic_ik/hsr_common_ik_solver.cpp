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
/// @brief High-speed IK for HSR-B

#include "hsr_common_ik_solver.hpp"

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "robot_optimizer.hpp"

namespace {
Eigen::Vector3d ConvertPose2d(const Eigen::Affine3d& pose) {
  return Eigen::Vector3d(pose.translation().x(), pose.translation().y(), pose.linear().eulerAngles(0, 1, 2)[2]);
}
}  // namespace

namespace hsrb_analytic_ik {

/// Check if Basemovement is a parallel of x, y and Z
bool SuitBaseMovement(const tmc_robot_kinematics_model::IKRequest& request) {
  if (request.linear_base_movements.size() != 2) {
    return false;
  }
  if (request.rotational_base_movements.size() != 1) {
    return false;
  }
  if (request.linear_base_movements[0] != Eigen::Vector3d::UnitX()) {
    return false;
  }
  if (request.linear_base_movements[1] != Eigen::Vector3d::UnitY()) {
    return false;
  }
  if (request.rotational_base_movements[0] != Eigen::Vector3d::UnitZ()) {
    return false;
  }
  return true;
}

/// Check if Basemovement is the rotation of Z
bool SuitBaseRotationZ(const tmc_robot_kinematics_model::IKRequest& request) {
  if (request.linear_base_movements.size() != 0) {
    return false;
  }
  if (request.rotational_base_movements.size() != 1) {
    return false;
  }
  if (request.rotational_base_movements[0] != Eigen::Vector3d::UnitZ()) {
    return false;
  }
  return true;
}

/// Check if Joint contains all the axes of the arm
bool SuitUseJoint(const tmc_robot_kinematics_model::IKRequest& request) {
  std::string arm_joints[] =
      {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

  for (const auto& arm_joint : arm_joints) {
    if (std::find(request.use_joints.begin(),
                  request.use_joints.end(),
                  arm_joint) == request.use_joints.end()) {
      return false;
    }
  }
  return true;
}

/// Check if Frame_name is Hand_palm_link
bool SuitFrame(const tmc_robot_kinematics_model::IKRequest& request) {
  return ((request.frame_name == "hand_palm_link") ||
          (request.frame_name == "hand_palm_joint"));
}

/// Make a joint name and ID map
bool MapJointAndId(const std::vector<std::string>& use_joints,
                   std::map<std::string, uint32_t>& map_out) {
  std::string arm_joints[] =
      {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
  map_out.clear();
  for (const auto& arm_joint : arm_joints) {
    std::vector<std::string>::const_iterator it =
        std::find(use_joints.begin(), use_joints.end(), arm_joint);
    if (it == use_joints.end()) {
      return false;
    } else {
      map_out[arm_joint] = std::distance(use_joints.begin(), it);
    }
  }
  return true;
}

/// Output Joint_names values ​​from Joint_state.If not found, return default_pos.
double ExtractJointPosition(
    const tmc_manipulation_types::JointState& joint_state,
    const std::string& joint_name,
    double default_pos) {
  if (joint_state.name.size() != joint_state.position.size()) {
    return default_pos;
  }
  for (uint32_t i = 0; i < joint_state.name.size(); ++i) {
    if (joint_state.name[i] == joint_name) {
      return joint_state.position(i);
    }
  }
  return default_pos;
}

/// Solve IK, but also allow BASE movement
tmc_robot_kinematics_model::IKResult SolveIK(
    const tmc_robot_kinematics_model::IKRequest& request,
    const RobotParameter& function_param,
    tmc_manipulation_types::JointState& solution_angle_out,
    Eigen::Affine3d& origin_to_base_out,
    Eigen::Affine3d& origin_to_end_out) {

  std::map<std::string, uint32_t> joint_map;
  MapJointAndId(request.use_joints, joint_map);

  solution_angle_out.name = request.use_joints;
  solution_angle_out.position.resize(request.use_joints.size());
  // Copy Initial for joint angles outside the chain
  for (uint32_t i = 0; i < request.use_joints.size(); ++i) {
    solution_angle_out.position[i] =
        ExtractJointPosition(request.initial_angle,
                             request.use_joints[i], 0.0);
  }

  origin_to_base_out = Eigen::Affine3d::Identity();
  origin_to_end_out = Eigen::Affine3d::Identity();

  // Copy the input.
  opt::RobotFunction2Request function_req;
  // Set the reference value (T_ref) in the simultaneous conversion matrix.
  function_req.R11 = request.ref_origin_to_end(0, 0);
  function_req.R21 = request.ref_origin_to_end(1, 0);
  function_req.R31 = request.ref_origin_to_end(2, 0);

  function_req.R12 = request.ref_origin_to_end(0, 1);
  function_req.R22 = request.ref_origin_to_end(1, 1);
  function_req.R32 = request.ref_origin_to_end(2, 1);

  function_req.R13 = request.ref_origin_to_end(0, 2);
  function_req.R23 = request.ref_origin_to_end(1, 2);
  function_req.R33 = request.ref_origin_to_end(2, 2);

  function_req.px = request.ref_origin_to_end(0, 3);
  function_req.py = request.ref_origin_to_end(1, 3);
  function_req.pz = request.ref_origin_to_end(2, 3);

  // Set the diagonal component W_i (i = 0..7) in the weight matrix (diagonal matrix).
  function_req.w0 = request.weight(request.use_joints.size());
  function_req.w1 = request.weight(request.use_joints.size() + 1);
  function_req.w2 = request.weight(request.use_joints.size() + 2);
  function_req.w3 = request.weight(joint_map["arm_lift_joint"]);
  function_req.w4 = request.weight(joint_map["arm_flex_joint"]);
  function_req.w5 = request.weight(joint_map["arm_roll_joint"]);
  function_req.w6 = request.weight(joint_map["wrist_flex_joint"]);
  function_req.w7 = request.weight(joint_map["wrist_roll_joint"]);

  // Set the reference value of the parameter θ^ref_i (i = 0..7).
  function_req.r0 = request.origin_to_base(0, 3);
  function_req.r1 = request.origin_to_base(1, 3);
  function_req.r2 = std::atan2(request.origin_to_base(1, 0), request.origin_to_base(0, 0));
  function_req.r3 = ExtractJointPosition(request.initial_angle, "arm_lift_joint", 0.0);
  function_req.r4 = ExtractJointPosition(request.initial_angle, "arm_flex_joint", 0.0);
  function_req.r5 = ExtractJointPosition(request.initial_angle, "arm_roll_joint", 0.0);
  function_req.r6 = ExtractJointPosition(request.initial_angle, "wrist_flex_joint", 0.0);
  function_req.r7 = ExtractJointPosition(request.initial_angle, "wrist_roll_joint", 0.0);

  // Create an optimization purpose function.
  opt::RobotFunction2 f(function_req, function_param);

  // Execute optimization.
  opt::OptResult result = opt::RobotOptimizer::Optimize(f);

  // If the optimization fails for some reason, it will be unusual.
  // (Depending on the optimization method used, OptFail may not be returned.)
  if (result == opt::OptFail) {
    return tmc_robot_kinematics_model::kFail;
  }

  // Optimization is treated as a successful one, even in the maximum repetition.
  if (result == opt::OptMaxItor) {
    // return kMaxItr;
  }

  // If the optimal solution is outside the executable area, if the degree of outing is small,
  // It is forcibly pulled back to the executable area and solved, and treated as a successful optimization.
  // In cases where the executable area is on a straight line, it is not expected that it will converge to the inner point due to the calculation error, so
  // This process is required.
  // There is such a case in the test driver Limittest.
  double outerGrade = f.GetOuterGrade();
  if (outerGrade > 0) {
    if (outerGrade <= 1e-6) {
      f.ForceFeasible();
    } else {
      return tmc_robot_kinematics_model::kFail;
    }
  }

  // Copy the output.
  opt::RobotFunction2Response function_res(f.response());

  // Create Solution_angle_out.position.
  solution_angle_out.position(joint_map["arm_lift_joint"]) = function_res.t3;
  solution_angle_out.position(joint_map["arm_flex_joint"]) = function_res.t4;
  solution_angle_out.position(joint_map["arm_roll_joint"]) = function_res.t5;
  solution_angle_out.position(joint_map["wrist_flex_joint"]) = function_res.t6;
  solution_angle_out.position(joint_map["wrist_roll_joint"]) = function_res.t7;

  // Create a simultaneous conversion matrix between links corresponding to parameter θ_i.
  Eigen::Translation3d   T0 = Eigen::Translation3d(function_res.t0, 0, 0);
  Eigen::Translation3d   T1 = Eigen::Translation3d(0, function_res.t1, 0);
  Eigen::AngleAxisd      T2 = Eigen::AngleAxisd(function_res.t2, Eigen::Vector3d(0, 0, 1));
  Eigen::Translation3d   T3 = Eigen::Translation3d(0, 0, function_res.t3 + function_param.L3);
  Eigen::Affine3d        T4 = Eigen::Translation3d(function_param.L41, function_param.L42, 0)
                            * Eigen::AngleAxisd(-function_res.t4, Eigen::Vector3d(0, 1, 0));
  Eigen::Affine3d        T5 = Eigen::Translation3d(function_param.L51, 0, function_param.L52)
                            * Eigen::AngleAxisd(function_res.t5, Eigen::Vector3d(0, 0, 1));
  Eigen::AngleAxisd      T6 = Eigen::AngleAxisd(-function_res.t6, Eigen::Vector3d(0, 1, 0));
  Eigen::AngleAxisd      T7 = Eigen::AngleAxisd(function_res.t7, Eigen::Vector3d(0, 0, 1));
  Eigen::Affine3d        T8 = Eigen::Translation3d(function_param.L81, 0, function_param.L82)
                            * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1));

  // Create Origin_to_base_out, Origin_to_end_out.
  origin_to_base_out = T0 * T1 * T2;
  origin_to_end_out = origin_to_base_out * T3 * T4 * T5 * T6 * T7 * T8;

  return tmc_robot_kinematics_model::kSuccess;
}

// The input can take the expression of -2π ~ 2π, but the expression of the joint angle is changed to -π ~ π Created to use it for the wheel fixing 6-axis IK.
double ThetaReperesentationChange(const double theta) {
  if (theta > M_PI) {
    return (-2*M_PI + theta);
  } else if (theta < -M_PI) {
    return (theta + 2*M_PI);
  }
  return theta;
}

// A function that solves the equation of a triangular function synthesis ASINθ+bcosθ = c d = √ (a^2+b^2) There is a 2O attached letter, but it is meaningless.
// Created to use it for wheel fixed 6 -axis IK T.Yamamoto
bool TrigonometricCompositionFormula(const double A2o, const double B2o, const double C2o, const double D2o,
                                     double& theta21, double& theta22) {
  if (fabs(C2o) > D2o) {
    return false;
  }
  const double alpha2o = atan2(B2o, A2o);
  const double AS1 = asin(C2o/D2o);
  double AS2;
  if (AS1 >= 0) {
    AS2 = M_PI - AS1;
  } else {
    AS2 = -M_PI - AS1;
  }
  theta21 = ThetaReperesentationChange(AS1-alpha2o);
  theta22 = ThetaReperesentationChange(AS2-alpha2o);
  return true;
}


// Use only in judgethetapi_pi.
// Created to use it for wheel fixed 6 -axis IK T.Yamamoto
void ThetaWithinLimit(const double max_in, const double min_in, const double epsilon, double& theta) {
  if (theta > max_in && theta <= (max_in + epsilon)) {
    theta = max_in;
  } else if (theta < min_in && theta >= (min_in - epsilon)) {
    theta = min_in;
  }
}


// Check the upper limit and lower limit of the joint angle, and if it is established, the maximum value or the maximum value is higher, or if the minimum value is smaller than -pi.
// Change the joint angle (-pi to pi) required by IK to change the value beyond the range (because it does not pass by test)
// Created to use it for wheel fixed 6 -axis IK T.Yamamoto
bool JudgeThetaPi_Pi(const double max_in, const double min_in, const double epsilon, double& theta) {
  const double max = max_in + epsilon;
  const double min = min_in - epsilon;

  if (theta <= max && theta >= min) {
    ThetaWithinLimit(max_in, min_in, epsilon, theta);
    return true;
  }

  if (max > M_PI) {
    if (theta >= -M_PI && theta <= -2*M_PI + max) {
      theta = theta + 2*M_PI;
      ThetaWithinLimit(max_in, min_in, epsilon, theta);
      return true;
    }
  } else if (min < -M_PI) {
    if (theta >= 2*M_PI+min && theta <= M_PI) {
       theta = theta-2*M_PI;
       ThetaWithinLimit(max_in, min_in, epsilon, theta);
       return true;
    }
  }
  return false;
}

// Wheel fixed 6 -axis IK 2022.8. T.yamamoto
// Solve the IK on the arm (THETA3-7) + the bogie turning axis (THETA2)
tmc_robot_kinematics_model::IKResult SolveBaseYawIK(
    const tmc_robot_kinematics_model::IKRequest& request,
    const RobotParameter& function_param,
    std::vector<tmc_robot_kinematics_model::IKResponse>& responses_out) {
  std::map<std::string, uint32_t> joint_map;
  MapJointAndId(request.use_joints, joint_map);

  // THETA0 THETA1 is not changed from the target value (fixed)
  const double theta0 = request.origin_to_base(0, 3);
  const double theta1 = request.origin_to_base(1, 3);

  Eigen::Matrix4d T78;
  T78  << -1, 0, 0, function_param.L81,  0, -1, 0, 0, 0, 0, 1, function_param.L82,  0, 0, 0, 1;
  Eigen::Matrix4d UB8o = request.ref_origin_to_end.matrix();
  Eigen::Matrix4d UB7o = UB8o*T78.inverse();

  // double xwo, ywo, zwo
  const double xwo = UB7o(0, 3);
  const double ywo = UB7o(1, 3);
  const double zwo = UB7o(2, 3);

  // Introduction of the coefficient of triangular function synthesis.
  const double A2o = -theta0+xwo;
  const double B2o = (theta1-ywo);
  const double C2o = -function_param.L42;
  const double D2o = sqrt(pow(A2o, 2.0) + pow(B2o, 2.0));

  // Solve the triangular function synthesis and find THETA2.Two solutions
  double theta21, theta22;
  const double eps_theta = 1e-3;  // A range that can be exceeded the upper and lower lower limit.Output as a joint lower or lower limit (T0 to T7)
  std::vector<std::vector<double>> theta01234;  // The first argument is a solution number 0,1,2,3 (maximum 4) The second argument is THETA0-4

  if (TrigonometricCompositionFormula(A2o, B2o, C2o, D2o, theta21, theta22)) {
    const double A4o = -function_param.L52;
    const double B4o = function_param.L51;
    const double D4o = sqrt(pow(A4o, 2.0) + pow(B4o, 2.0));

    for (auto theta2 : {theta21, theta22}) {
      const double C4o = (-cos(theta2)*(theta0-xwo)-sin(theta2)*(theta1-ywo)-function_param.L41);
      // Find the solution of THETA4.Two for THETA2
      double theta41, theta42;
      if (TrigonometricCompositionFormula(A4o, B4o, C4o, D4o, theta41, theta42)) {
        for (auto theta4 : {theta41, theta42}) {
          if (JudgeThetaPi_Pi(function_param.t4_max, function_param.t4_min, eps_theta, theta4)) {
            // Find THETA3.
            double theta3 = zwo-function_param.L3-function_param.L52*cos(theta4)-function_param.L51*sin(theta4);
            if (JudgeThetaPi_Pi(function_param.t3_max, function_param.t3_min, eps_theta, theta3)) {
              theta01234.push_back({theta0, theta1, theta2, theta3, theta4});
            }
          }
        }
      }
    }
  }  // The end of the IF triangular function synthesis and finds THETA2

  if (theta01234.empty()) {  // 解が１つもなかったらfailで終了
    return tmc_robot_kinematics_model::kFail;
  }

  std::vector<std::vector<double>> theta_all;  // The first argument is the second argument of the IK solution is the joint number.
  for (const auto& theta_parts : theta01234) {
    double theta2o = theta_parts.at(2);
    double theta3o = theta_parts.at(3);
    double theta4o = theta_parts.at(4);

    Eigen::Matrix4d TBm1, Tm10, T01, T12, T23, T34;
    TBm1 << 0, 0, 1, 0,   1, 0,  0, 0,   0, 1, 0,  0,    0, 0, 0, 1;
    Tm10 << 0, -1, 0, 0,  1, 0,  0, 0,   0, 0, 1, theta0,  0, 0, 0, 1;
    T01  << 0, -1, 0, 0,  0, 0, -1, -theta1,   1, 0, 0,  0,   0, 0, 0, 1;
    T12 << -sin(theta2o), -cos(theta2o), 0, 0,  0, 0, -1, 0,  cos(theta2o), -sin(theta2o), 0, 0,  0, 0, 0, 1;
    T23 << 0, 1, 0, 0,  -1, 0, 0, 0,  0, 0, 1, theta3o+function_param.L3,  0, 0, 0, 1;
    T34 << cos(theta4o), -sin(theta4o), 0, function_param.L41,  0, 0, -1, function_param.L42,
           sin(theta4o), cos(theta4o), 0, 0,  0, 0, 0, 1;

    Eigen::Matrix4d UB4o, U47o;
    UB4o = TBm1*Tm10*T01*T12*T23*T34;
    U47o = (UB4o.inverse())*UB7o;

    // THETA6 2 is solved, so two solutions
    const double theta61 = atan2(-sqrt(pow(U47o(1, 0), 2.0)+pow(U47o(1, 1), 2.0)), U47o(1, 2));
    const double theta62 = atan2(sqrt(pow(U47o(1, 0), 2.0)+pow(U47o(1, 1), 2.0)), U47o(1, 2));
    for (auto theta6 : {theta61, theta62}) {
      // When THETA6 is near 0, the hands are extended with a unique posture.It is good if theTa5+THETA7 is constant.
      if (fabs(theta6) < 1e-9) {
        const double theta57a = atan2(-U47o(0, 1), U47o(0, 0));
        const double theta57b = atan2(-U47o(2, 0), -U47o(2, 1));
        if (fabs(theta57a-theta57b) < 1e-6) {
          const double theta5 = theta57a/2.0;  // simple calculation
          const double theta7 = theta57a/2.0;
          theta_all.push_back({theta_parts.at(0), theta_parts.at(1), theta_parts.at(2), theta_parts.at(3),
                               theta_parts.at(4), theta5, theta6, theta7});
          continue;
        }
      }
      if (!JudgeThetaPi_Pi(function_param.t6_max, function_param.t6_min, eps_theta, theta6)) {
        continue;
      }
      int sign6;
      // the hellen (the 1,1)=-sin6sin (thetata7)
      // Therefore, the value of ATAN2 changes depending on the Sin (THETA6) code.Use Sign instead of sin to prevent 0 %
      if (theta6 >= 0) {sign6 = 1.0;} else {sign6 = -1.0;}
      double theta7 = atan2(-U47o(1, 1)*sign6, U47o(1, 0)*sign6);
      if (!JudgeThetaPi_Pi(function_param.t7_max, function_param.t7_min, eps_theta, theta7)) {
        continue;
      }
      double theta5 = atan2(U47o(2, 2)*sign6, -U47o(0, 2)*sign6);
      if (!JudgeThetaPi_Pi(function_param.t5_max, function_param.t5_min, eps_theta, theta5)) {
        continue;
      }
      theta_all.push_back({theta_parts.at(0), theta_parts.at(1), theta_parts.at(2), theta_parts.at(3),
                           theta_parts.at(4), theta5, theta6, theta7});
    }  // for theta6
  }    // for i01234

  if (theta_all.empty()) {  // 5,6,7 If there is no solution to 5,6,7
    return tmc_robot_kinematics_model::kFail;
  }

  for (const auto& joint_positions : theta_all) {
    const double t0 = joint_positions.at(0);
    const double t1 = joint_positions.at(1);
    const double t2 = joint_positions.at(2);
    const double t3 = joint_positions.at(3);
    const double t4 = joint_positions.at(4);
    const double t5 = joint_positions.at(5);
    const double t6 = joint_positions.at(6);
    const double t7 = joint_positions.at(7);

    // Create Solution_angle.position.
    tmc_robot_kinematics_model::IKResponse response;
    response.solution_angle.name = request.use_joints;
    response.solution_angle.position.resize(request.use_joints.size());
    response.solution_angle.position(joint_map["arm_lift_joint"]) = t3;
    response.solution_angle.position(joint_map["arm_flex_joint"]) = t4;
    response.solution_angle.position(joint_map["arm_roll_joint"]) = t5;
    response.solution_angle.position(joint_map["wrist_flex_joint"]) = t6;
    response.solution_angle.position(joint_map["wrist_roll_joint"]) = t7;

    // Create a simultaneous conversion matrix between links corresponding to parameter θ_i.
    Eigen::Translation3d   T0 = Eigen::Translation3d(t0, 0, 0);
    Eigen::Translation3d   T1 = Eigen::Translation3d(0, t1, 0);
    Eigen::AngleAxisd      T2 = Eigen::AngleAxisd(t2, Eigen::Vector3d(0, 0, 1));

    Eigen::Translation3d   T3 = Eigen::Translation3d(0, 0, t3 + function_param.L3);
    Eigen::Affine3d        T4 = Eigen::Translation3d(function_param.L41, function_param.L42, 0)
                              * Eigen::AngleAxisd(-t4, Eigen::Vector3d(0, 1, 0));
    Eigen::Affine3d        T5 = Eigen::Translation3d(function_param.L51, 0, function_param.L52)
                              * Eigen::AngleAxisd(t5, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd      T6 = Eigen::AngleAxisd(-t6, Eigen::Vector3d(0, 1, 0));
    Eigen::AngleAxisd      T7 = Eigen::AngleAxisd(t7, Eigen::Vector3d(0, 0, 1));
    Eigen::Affine3d        T8 = Eigen::Translation3d(function_param.L81, 0, function_param.L82)
                              * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1));

    // Create Origin_to_base, Origin_to_end.
    response.origin_to_base = T0 * T1 * T2;
    response.origin_to_end = response.origin_to_base * T3 * T4 * T5 * T6 * T7 * T8;
    responses_out.push_back(response);
  }
  return tmc_robot_kinematics_model::kSuccess;
}

uint32_t SelectClosestSolution(
    const tmc_robot_kinematics_model::IKRequest& request,
    const std::vector<tmc_robot_kinematics_model::IKResponse>& responses) {
  // The assumption that the joint order of Request.use_Joints and Responses matches, as in Solvebaseyawik solutions
  Eigen::VectorXd current_arm_positions = Eigen::VectorXd::Zero(request.use_joints.size());
  for (auto i = 0; i < request.use_joints.size(); ++i) {
    current_arm_positions[i] = ExtractJointPosition(request.initial_angle, request.use_joints[i], 0.0);
  }
  const auto current_base_positions = ConvertPose2d(request.origin_to_base);

  double min_distance = std::numeric_limits<double>::max();
  uint32_t index_out = -1;

  for (uint32_t i = 0; i < responses.size(); ++i) {
    const auto response_base_positions = ConvertPose2d(responses[i].origin_to_base);
    const auto arm_diff = (current_arm_positions - responses[i].solution_angle.position).array().abs();
    const auto base_diff = (current_base_positions - response_base_positions).array().abs();
    const double distance = (arm_diff * request.weight.head(request.use_joints.size()).array()).sum()
                          + (base_diff * request.weight.tail(3).array()).sum();
    if (distance < min_distance) {
      min_distance = distance;
      index_out = i;
    }
  }
  return index_out;
}

}  // namespace hsrb_analytic_ik
