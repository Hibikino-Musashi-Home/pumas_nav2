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
/// @file     stl_loader-test.hpp
/// @brief    Test to read STL files
/// @version  0.1.1
/// @author   Takao Yasuda
/// @note     Applied for Partner-Robot Coding Rule(Ver:x.xx)

#ifndef TMC_STL_LOADER_STL_LOADER_TEST_HPP_
#define TMC_STL_LOADER_STL_LOADER_TEST_HPP_
#include <memory>
#include <gtest/gtest.h>
#include <tmc_stl_loader/stl_loader.hpp>

class STLLoaderTest : public ::testing::Test {
 public:
  STLLoaderTest() : stl_loader_(new tmc_stl_loader::STLLoader()) {}
  virtual ~STLLoaderTest() {}

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  /// STL file reading library
  std::shared_ptr<tmc_stl_loader::STLLoader> stl_loader_;
  /// mesh
  tmc_stl_loader::Mesh mesh_;

 private:
  STLLoaderTest(const STLLoaderTest&);
  STLLoaderTest& operator=(const STLLoaderTest&);
};

#endif  // TMC_STL_LOADER_STL_LOADER_TEST_HPP_
