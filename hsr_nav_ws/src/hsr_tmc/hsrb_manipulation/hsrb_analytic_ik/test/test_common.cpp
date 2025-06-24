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
#include "test_common.hpp"

#include <limits>

#include <gtest/gtest.h>

#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>

namespace {

// Numerical IK parameters
constexpr uint32_t kMaxItr = 1000;
constexpr double kEpsilon = 0.001;
constexpr double kConvergeThreshold = 1e-6;
const char* const kHandName = "hand_palm_link";

// Tolerance error parallel [M]
constexpr double kTransThreshold = 1e-3;
// Telegral mistake difference angle [RAD]
constexpr double kRotThreshold = 2*M_PI*1e-2;
// Robot freedom
constexpr uint32_t kDOF = 8;
constexpr uint32_t kArmDOF = 5;

// 10% of the tolerance limit of optimality compared to the case of numerical calculations
constexpr double kOptimizeThreshold = 1.1;
// High speed compared to the case of numerical solutions
constexpr double kAnalyticExpectTimes = 2.0;

// Convert Configuration into Angle and Base posture
void ConfigurationToAngleAndBase(const Eigen::VectorXd& config,
                                 Eigen::VectorXd& angle_out,
                                 Eigen::Affine3d& origin_to_base_out) {
  origin_to_base_out = Eigen::Translation3d(config(0), config(1), 0.0);
  origin_to_base_out.linear() = Eigen::AngleAxisd(config(2), Eigen::Vector3d::UnitZ()).matrix();
  angle_out = config.tail(kArmDOF);
}

// Convert Angle and Base to Configuration
void AngleAndBaseToConfiguration(const Eigen::VectorXd& angle,
                                 const Eigen::Affine3d& origin_to_base,
                                 Eigen::VectorXd& config_out) {
  config_out.resize(kDOF);
  config_out(0) = origin_to_base.translation().x();
  config_out(1) = origin_to_base.translation().y();
  config_out(2) = Eigen::AngleAxisd(origin_to_base.rotation()).axis().z()
      * Eigen::AngleAxisd(origin_to_base.rotation()).angle();
  config_out.tail(kArmDOF) = angle;
}

// Judgment whether the two postures are close enough
// Judgment is close to Ktransthreshold [M] rotation within Krotthreshold [RAD]
bool IsNearAffine(const Eigen::Affine3d& ref, const Eigen::Affine3d& cur) {
  const auto trans_diff = (ref.translation() - cur.translation()).norm();
  const auto rot_diff = fabs(Eigen::AngleAxisd(ref.linear() * cur.linear().transpose()).angle());
  return ((trans_diff < kTransThreshold) && (rot_diff < kRotThreshold));
}

// Time measurement GetTimeOfday is not very appropriate, so please correct it if possible.
double GetTime(void) {
  //
  // Measure the thread -specific CPU time.
  // This allows you to perform performance tests that are difficult to depend on the load status of the execution environment.
  // (This is not to be completely affected by the execution environment)
  //
  // [Change] Intelligence Information System, 2015-02-13
  // [Note] In order to use the clock_gettime function on Ubuntu 12, insurden <sys/time.h> and
  // You need to link the libbrt.so library.
  // Fixed src/hsrb_analytic_ik/cmakeLists.txt,
  // The library to link is added by the Target_link_libraries command of Cmake.
  //
  struct timespec t;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
  double seconds = static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_nsec) * 1e-9;
  return seconds;
}

/**
 * This is a class for collecting statistical information.
 * [Added] Intelligence Information System, 2015-02-13
 */
class Stats {
 public:
  Stats();

  void Reset();
  void Add(double value);
  double Min() const { return min_; }
  double Max() const { return max_; }
  double Mean() const { return sum1_ / count_; }
  double Var() const {
    double m = Mean();
    return (sum2_ - count_ * m * m) / count_;
  }
  double Sd() const { return std::sqrt(Var());}

 private:
  double count_;
  double min_;
  double max_;
  double sum1_;
  double sum2_;
};

Stats::Stats() {
  Reset();
}

void Stats::Reset() {
  count_ = 0;
  min_ = +std::numeric_limits<double>::infinity();
  max_ = -std::numeric_limits<double>::infinity();
  sum1_ = 0;
  sum2_ = 0;
}

void Stats::Add(double value) {
  count_++;
  if (value < min_) min_ = value;
  if (max_ < value) max_ = value;
  sum1_ += value;
  sum2_ += value * value;
}

}  // namespace

namespace hsrb_analytic_ik {

// Read 1x8 configuration from the file.
void LoadDataFile(const std::string& file_name, std::vector<Eigen::VectorXd>& data_out) {
  data_out.clear();
  std::string buf;
  std::ifstream ifs(file_name.c_str());
  while (ifs && getline(ifs, buf)) {
    Eigen::VectorXd config(kDOF);
    std::stringstream input_line(buf);
    for (uint32_t i = 0; i < kDOF; ++i) {
      input_line >> config(i);
    }
    data_out.push_back(config);
  }
}

IKTestDriver::IKTestDriver(const std::string& robot_description,
                           tmc_manipulation_types::BaseMovementType base_movement_type)
    : base_movement_type_(base_movement_type) {
  robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(robot_description);

  use_name_ = {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
  weight_vector_.resize(8);
  weight_vector_ << 10.0, 10.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0;
  robot_->GetMinMax(use_name_, joint_min_, joint_max_);

  numeric_solver_ = std::make_shared<tmc_robot_kinematics_model::NumericIKSolver>(
      tmc_robot_kinematics_model::IKSolver::Ptr(), robot_, kMaxItr, kEpsilon, kConvergeThreshold);
}

// Solve FK
void IKTestDriver::SolveFK(const Eigen::VectorXd& config, Eigen::Affine3d& origin_to_end_out) {
  tmc_manipulation_types::JointState joint_state;
  joint_state.name = use_name_;
  Eigen::Affine3d origin_to_base;
  ConfigurationToAngleAndBase(config, joint_state.position, origin_to_base);
  robot_->SetRobotTransform(origin_to_base);
  robot_->SetNamedAngle(joint_state);
  origin_to_end_out = robot_->GetObjectTransform(kHandName);
}

// Solve it with the given IKSOLVER
bool IKTestDriver::SolveIK(const tmc_robot_kinematics_model::IKSolver::Ptr& ik_solver,
                           const Eigen::Affine3d& ref_origin_to_end,
                           const Eigen::VectorXd& initial_config,
                           Eigen::VectorXd& solution_config_out) {
  tmc_manipulation_types::JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base_solution;
  const auto result =  ik_solver->Solve(GenerateRequest(ref_origin_to_end, initial_config),
                                        solution, origin_to_base_solution, origin_to_hand_result);
  if (result == tmc_robot_kinematics_model::kSuccess) {
    AngleAndBaseToConfiguration(solution.position, origin_to_base_solution, solution_config_out);
    return true;
  } else {
    return false;
  }
}

// Solve it with the given IKSOLVER
bool IKTestDriver::SolveIK(const tmc_robot_kinematics_model::IKSolver::Ptr& ik_solver,
                           const Eigen::Affine3d& ref_origin_to_end,
                           const Eigen::VectorXd& initial_config,
                           std::vector<Eigen::VectorXd>& solution_configs_out) {
  std::vector<tmc_robot_kinematics_model::IKResponse> responses;
  const auto result =  ik_solver->Solve(GenerateRequest(ref_origin_to_end, initial_config), responses);
  if (result == tmc_robot_kinematics_model::kSuccess) {
    for (const auto& res : responses) {
      Eigen::VectorXd solution;
      AngleAndBaseToConfiguration(res.solution_angle.position, res.origin_to_base, solution);
      solution_configs_out.push_back(solution);
    }
    return true;
  } else {
    return false;
  }
}

// Solve it with the numerical value IKSOLVER that you have
bool IKTestDriver::SolveNumericIK(const Eigen::Affine3d& ref_origin_to_end,
                                  const Eigen::VectorXd& initial_config,
                                  Eigen::VectorXd& solution_config_out) {
  return SolveIK(numeric_solver_, ref_origin_to_end, initial_config, solution_config_out);
}

double IKTestDriver::CalcWeightedNorm(const Eigen::VectorXd& config1, const Eigen::VectorXd& config2) const {
  Eigen::VectorXd diff = config1 - config2;
  // Diff (2) is infinite rotation, so normalization
  if (fabs(diff(2)) > M_PI) diff(2) = M_PI - fabs(diff(2));
  return (weight_vector_.cwiseProduct(diff)).norm();
}

// Check the maximum of the joints
void IKTestDriver::CheckMinMax(const Eigen::VectorXd& joint) const {
  for (uint32_t i = 0; i < kArmDOF; ++i) {
    EXPECT_GE(joint(i), joint_min_(i)) << "[" << use_name_[i] << "] : lower limit";
    EXPECT_LE(joint(i), joint_max_(i)) << "[" << use_name_[i] << "] : higher limit";
  }
}
// A function that demands the minimum value of the joint angle range of the arm joint (0 to 4 3) (inevitably installed for test data of HSR-C)
bool IKTestDriver::ArmJointCheckMinMax(const Eigen::VectorXd& joint) const {
  for (uint32_t i = 0; i < kArmDOF; ++i) {
    if (joint(i) < joint_min_(i) || joint(i) > joint_max_(i)) {
      return false;
    }
  }
  return true;
}

tmc_robot_kinematics_model::IKRequest IKTestDriver::GenerateRequest(const Eigen::Affine3d& ref_origin_to_end,
                                                                    const Eigen::VectorXd& initial_config) const {
  tmc_robot_kinematics_model::IKRequest req(base_movement_type_);
  req.frame_name = kHandName;
  req.frame_to_end = Eigen::Affine3d::Identity();
  req.initial_angle.name = use_name_;
  req.use_joints = use_name_;
  req.weight = weight_vector_;
  req.ref_origin_to_end = ref_origin_to_end;
  ConfigurationToAngleAndBase(initial_config, req.initial_angle.position, req.origin_to_base);
  return req;
}

void RunTest(const std::vector<Eigen::VectorXd>& ref_configs,
             const std::vector<Eigen::VectorXd>& init_configs,
             const std::shared_ptr<IKTestDriver>& ik_test_driver,
             const tmc_robot_kinematics_model::IKSolver::Ptr& analytic_ik_solver,
             const std::vector<uint32_t>& mask_indices) {
  // [Added] (Intelligence Information System Co., Ltd., 2015-02-13
  // Create an object to statistically aggregate optimality and calculation time.
  Stats norm_stats;
  Stats time_stats;

  int32_t numeric_ik_solve = 0;

  // Solution pattern loop
  for (const auto& ref_config : ref_configs) {
    // If the file data for the joint angle is out of the range, the processing is performed to omit the data.
    if (!ik_test_driver->ArmJointCheckMinMax(ref_config.tail(kArmDOF))) {
      continue;
    }
    for (const auto& init_config : init_configs) {
      if (!ik_test_driver->ArmJointCheckMinMax(init_config.tail(kArmDOF))) {
        continue;
      }
      // For the axis that is not used for IK, put the same value as INIT
      auto masked_ref_config = ref_config;
      for (const auto i : mask_indices) {
        masked_ref_config[i] = init_config[i];
      }

      // Create the target value with FK
      Eigen::Affine3d ref_origin_to_hand;
      ik_test_driver->SolveFK(masked_ref_config, ref_origin_to_hand);

      // Analysis IK+optimization confirms that it can be solved first
      const auto start = GetTime();
      Eigen::VectorXd analytic_solution(8);
      auto solved = ik_test_driver->SolveIK(analytic_ik_solver, ref_origin_to_hand, init_config, analytic_solution);
      EXPECT_TRUE(solved) << "[test case ref " << masked_ref_config.transpose() << " ] : IK cannot solve.";
      const auto end = GetTime();
      const auto analytic_elapsed = end - start;

      // If you can't solve it, it's meaningless
      if (!solved) {
        continue;
      }

      // Confirm that it is solved properly
      Eigen::Affine3d ik_origin_to_hand;
      ik_test_driver->SolveFK(analytic_solution, ik_origin_to_hand);

      // Check if it matches the target position
      EXPECT_TRUE(IsNearAffine(ref_origin_to_hand, ik_origin_to_hand))
          << "[test case ref " << masked_ref_config.transpose()
          << " ] : IK solution is too far"
          << "\nref_origin_to_hand=\n" << ref_origin_to_hand.matrix()
          << "\nik_origin_to_hand=\n" << ik_origin_to_hand.matrix();  // CJS added.

      // Check whether to enter the joint angle
      ik_test_driver->CheckMinMax(analytic_solution.tail(kArmDOF));

      const auto start_n = GetTime();
      Eigen::VectorXd numeric_solution(8);
      solved = ik_test_driver->SolveNumericIK(ref_origin_to_hand, init_config, numeric_solution);
      const auto end_n = GetTime();
      const auto numeric_elapsed = end_n - start_n;

      if (solved) {
        ++numeric_ik_solve;

        const auto analytic_weighted_norm = ik_test_driver->CalcWeightedNorm(init_config, analytic_solution);
        const auto numeric_weighted_norm = ik_test_driver->CalcWeightedNorm(init_config, numeric_solution);

        /*
         // Confirm that the numerical solution and optimality are compared to (koptimizethreshold) that will fit within double
         EXPECT_LE(analytic_weighted_norm,
         numeric_weighted_norm * kOptimizeThreshold);
         // Compare the numerical solution and the calculation time (KanalyticeXpectTimes) that is twice as faster.
         // If this is difficult, compare it on average in INIT_CONFIGS.
         EXPECT_LE(analytic_elapsed * kAnalyticExpectTimes,
         numeric_elapsed);
       */

        // [Modification] (Intelligence Information System Co., Ltd., 2015-02-13
        // Change from individual judgments to statistical judgments.
        // (Exclude cases where statistical data becomes infinite)
        if (numeric_weighted_norm > 0 && analytic_elapsed > 0) {
          norm_stats.Add(analytic_weighted_norm / numeric_weighted_norm);
          time_stats.Add(numeric_elapsed / analytic_elapsed);
        }
      }
    }
  }

/* Commented by TY
  std::cout << "numeric_ik_solves : " << numeric_ik_solve << std::endl;

  // [Added] (Intelligence Information System Co., Ltd., 2015-02-13
  // Perform tests based on the statistical values ​​of optimality and calculation time.

  // Confirm that the numerical solution and optimality are compared to (koptimizethreshold) that will fit within double
  EXPECT_LE(norm_stats.Mean(), kOptimizeThreshold);

  // Compare the numerical solution and the calculation time (KanalyticeXpectTimes) that is twice as faster.
  EXPECT_LE(kAnalyticExpectTimes, time_stats.Mean());


  // For reference, output how the statistical value is.
  std::cout << "Norm Stats: " << "   mean=" << norm_stats.Mean() << " , sd="
            << norm_stats.Sd() << " , min=" << norm_stats.Min() << " , max="
            << norm_stats.Max() << std::endl;
  std::cout << "Time Stats: " << "   mean=" << time_stats.Mean() << " , sd="
            << time_stats.Sd() << " , min=" << time_stats.Min() << " , max="
            << time_stats.Max() << std::endl;
Commented by TY */
}

void MultiSolutionTest(const Eigen::Affine3d& origin_to_end,
                       uint32_t expected_result_num,
                       const IKTestDriver::Ptr& ik_test_driver,
                       const tmc_robot_kinematics_model::IKSolver::Ptr& analytic_ik_solver) {
  std::vector<Eigen::VectorXd> solution_configs;
  ASSERT_TRUE(ik_test_driver->SolveIK(analytic_ik_solver, origin_to_end, Eigen::VectorXd::Zero(8), solution_configs));

  EXPECT_EQ(solution_configs.size(), expected_result_num);
  for (const auto& solution : solution_configs) {
    Eigen::Affine3d ik_origin_to_hand;
    ik_test_driver->SolveFK(solution, ik_origin_to_hand);
    EXPECT_TRUE(IsNearAffine(origin_to_end, ik_origin_to_hand));
    ik_test_driver->CheckMinMax(solution.tail(kArmDOF));
  }
}

}  // namespace hsrb_analytic_ik
