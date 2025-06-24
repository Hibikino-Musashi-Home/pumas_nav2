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
/// @brief    Eigen_utils test
#include <stdlib.h>
#include <gtest/gtest.h>
#include <tmc_eigen_utils/eigen_utils.hpp>

using tmc_eigen_utils::RPYToQuaternion;
using tmc_eigen_utils::QuaternionToRPY;

namespace {
// A threshold that is regarded as the value of DOUBLE is near
double kDoubleEps = 1.0e-10;
// A threshold that is considered that the value of Float is near
float kFloatEps = 1.0e-4;
// Number of random values
int32_t kRandomItMax = 1000000;
}

TEST(RPYToQuaternionTest, Convert) {
  Eigen::Vector3d rpy;
  Eigen::Vector3f rpyf;
  Eigen::Quaterniond q;
  Eigen::Quaternionf qf;

  rpy << 0, 0, 0;
  q = RPYToQuaternion(rpy);
  EXPECT_NEAR(q.w(), 1.0, kDoubleEps);
  EXPECT_NEAR(q.x(), 0.0, kDoubleEps);
  EXPECT_NEAR(q.y(), 0.0, kDoubleEps);
  EXPECT_NEAR(q.z(), 0.0, kDoubleEps);

  rpyf << 0, 0, 0;
  qf = RPYToQuaternion(rpyf);
  EXPECT_NEAR(qf.w(), 1.0, kFloatEps);
  EXPECT_NEAR(qf.x(), 0.0, kFloatEps);
  EXPECT_NEAR(qf.y(), 0.0, kFloatEps);
  EXPECT_NEAR(qf.z(), 0.0, kFloatEps);


  rpy << M_PI, 0, 0;
  q = RPYToQuaternion(rpy);
  EXPECT_NEAR(q.w(), 0.0, kDoubleEps);
  EXPECT_NEAR(q.x(), 1.0, kDoubleEps);
  EXPECT_NEAR(q.y(), 0.0, kDoubleEps);
  EXPECT_NEAR(q.z(), 0.0, kDoubleEps);

  rpy << 0, M_PI, 0;
  q = RPYToQuaternion(rpy);
  EXPECT_NEAR(q.w(), 0.0, kDoubleEps);
  EXPECT_NEAR(q.x(), 0.0, kDoubleEps);
  EXPECT_NEAR(q.y(), 1.0, kDoubleEps);
  EXPECT_NEAR(q.z(), 0.0, kDoubleEps);

  rpy << 0, 0, M_PI;
  q = RPYToQuaternion(rpy);
  EXPECT_NEAR(q.w(), 0.0, kDoubleEps);
  EXPECT_NEAR(q.x(), 0.0, kDoubleEps);
  EXPECT_NEAR(q.y(), 0.0, kDoubleEps);
  EXPECT_NEAR(q.z(), 1.0, kDoubleEps);

  rpyf << M_PI, 0, 0;
  qf = RPYToQuaternion(rpyf);
  EXPECT_NEAR(qf.w(), 0.0, kFloatEps);
  EXPECT_NEAR(qf.x(), 1.0, kFloatEps);
  EXPECT_NEAR(qf.y(), 0.0, kFloatEps);
  EXPECT_NEAR(qf.z(), 0.0, kFloatEps);

  rpyf << 0, M_PI, 0;
  qf = RPYToQuaternion(rpyf);
  EXPECT_NEAR(qf.w(), 0.0, kFloatEps);
  EXPECT_NEAR(qf.x(), 0.0, kFloatEps);
  EXPECT_NEAR(qf.y(), 1.0, kFloatEps);
  EXPECT_NEAR(qf.z(), 0.0, kFloatEps);

  rpyf << 0, 0, M_PI;
  qf = RPYToQuaternion(rpyf);
  EXPECT_NEAR(qf.w(), 0.0, kFloatEps);
  EXPECT_NEAR(qf.x(), 0.0, kFloatEps);
  EXPECT_NEAR(qf.y(), 0.0, kFloatEps);
  EXPECT_NEAR(qf.z(), 1.0, kFloatEps);
}

TEST(QuaternionToRPYTest, Convert) {
  Eigen::Vector3d rpy;
  Eigen::Vector3f rpyf;
  Eigen::Quaterniond q;
  Eigen::Quaternionf qf;

  q.w() = 1.0;
  q.x() = 0.0;
  q.y() = 0.0;
  q.z() = 0.0;
  rpy = QuaternionToRPY(q);
  EXPECT_NEAR(rpy(0), 0.0, kDoubleEps);
  EXPECT_NEAR(rpy(1), 0.0, kDoubleEps);
  EXPECT_NEAR(rpy(2), 0.0, kDoubleEps);

  q.w() = 0.0;
  q.x() = 1.0;
  q.y() = 0.0;
  q.z() = 0.0;
  rpy = QuaternionToRPY(q);
  EXPECT_NEAR(rpy(0), M_PI, kDoubleEps);
  EXPECT_NEAR(rpy(1), 0.0, kDoubleEps);
  EXPECT_NEAR(rpy(2), 0.0, kDoubleEps);
}

// Will Quaternion convert to RPY and return?
TEST(RandomTest, test) {
  Eigen::Vector3d rpy;
  Eigen::Quaterniond q;
  Eigen::Quaterniond q2;
  Eigen::Vector3f rpyf;
  Eigen::Quaternionf qf;
  Eigen::Quaternionf qf2;


  for (int32_t i = 0; i < kRandomItMax; ++i) {
    q = Eigen::Vector4d::Random(4);
    q.normalize();
    rpy = QuaternionToRPY(q);
    q2 = RPYToQuaternion(rpy);
    if (q.w() * q2.w() > 0.0) {
      EXPECT_NEAR(q.w(), q2.w(), kDoubleEps);
      EXPECT_NEAR(q.x(), q2.x(), kDoubleEps);
      EXPECT_NEAR(q.y(), q2.y(), kDoubleEps);
      EXPECT_NEAR(q.z(), q2.z(), kDoubleEps);
    } else {
      EXPECT_NEAR(q.w(), -q2.w(), kDoubleEps);
      EXPECT_NEAR(q.x(), -q2.x(), kDoubleEps);
      EXPECT_NEAR(q.y(), -q2.y(), kDoubleEps);
      EXPECT_NEAR(q.z(), -q2.z(), kDoubleEps);
    }

    qf = Eigen::Vector4f::Random(4);
    qf.normalize();
    rpyf = QuaternionToRPY(qf);
    qf2 = RPYToQuaternion(rpyf);
    if (qf.w() * qf2.w() > 0.0) {
      EXPECT_NEAR(qf.w(), qf2.w(), kFloatEps);
      EXPECT_NEAR(qf.x(), qf2.x(), kFloatEps);
      EXPECT_NEAR(qf.y(), qf2.y(), kFloatEps);
      EXPECT_NEAR(qf.z(), qf2.z(), kFloatEps);
    } else {
      EXPECT_NEAR(qf.w(), -qf2.w(), kFloatEps);
      EXPECT_NEAR(qf.x(), -qf2.x(), kFloatEps);
      EXPECT_NEAR(qf.y(), -qf2.y(), kFloatEps);
      EXPECT_NEAR(qf.z(), -qf2.z(), kFloatEps);
    }
  }
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
