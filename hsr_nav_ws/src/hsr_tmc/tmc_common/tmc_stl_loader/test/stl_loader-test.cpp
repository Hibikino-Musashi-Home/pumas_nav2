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
/// @file     stl_loader-test.cpp
/// @brief    Test to read STL files
/// @version  0.1.1
/// @author   Takao Yasuda
/// @note     Applied for Partner-Robot Coding Rule(Ver:x.xx)
#include <string>
#include <gtest/gtest.h>
#include "stl_loader-test.hpp"

namespace tmc_stl_loader {
/// File mode (loading)
const char* const kFineModeRead = "r";

/// [Normal system] Kinect STL file reading test
/// @note Accurate number can be confirmed by Meshlab etc.
TEST_F(STLLoaderTest, GiveExistingFileOfKinect) {
  // Absolute path
  stl_loader_->Load("kinect.stl", mesh_);
  // judgement
  EXPECT_EQ(mesh_.vertices.size(), 3815u);
  EXPECT_EQ(mesh_.normals.size(), 7590u);
  EXPECT_EQ(mesh_.indices.size(), 22770u);

  // package
  stl_loader_->Load("package://tmc_stl_loader/test/kinect.stl", mesh_);
  // judgement
  EXPECT_EQ(mesh_.vertices.size(), 3815u);
  EXPECT_EQ(mesh_.normals.size(), 7590u);
  EXPECT_EQ(mesh_.indices.size(), 22770u);
}
/// [Unusual system] Read test of files that do not exist
TEST_F(STLLoaderTest, GiveNonExistFile) {
  // File reading (empty file)
  stl_loader_->Load("", mesh_);

  // Judgment (all are empty)
  EXPECT_EQ(mesh_.vertices.size(), 0u);
  EXPECT_EQ(mesh_.normals.size(), 0u);
  EXPECT_EQ(mesh_.indices.size(), 0u);
}
/// [Unusual system] Read test of different forms
TEST_F(STLLoaderTest, GiveDifferentFormFile) {
  // File reading (polygon file)
  stl_loader_->Load("kinect.ply", mesh_);

  // Judgment (all are empty)
  EXPECT_EQ(mesh_.vertices.size(), 0u);
  EXPECT_EQ(mesh_.normals.size(), 0u);
  EXPECT_EQ(mesh_.indices.size(), 0u);
}
///// [Abnormal system] Read test of different files in binary
TEST_F(STLLoaderTest, GiveDifferentContentsFile) {
  // File reading (ASCII format file)
  stl_loader_->Load("cube-ascii.stl", mesh_);

  // Judgment (all are empty)
  EXPECT_EQ(mesh_.vertices.size(), 0u);
  EXPECT_EQ(mesh_.normals.size(), 0u);
  EXPECT_EQ(mesh_.indices.size(), 0u);
}
}  // namespace tmc_stl_loader
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
