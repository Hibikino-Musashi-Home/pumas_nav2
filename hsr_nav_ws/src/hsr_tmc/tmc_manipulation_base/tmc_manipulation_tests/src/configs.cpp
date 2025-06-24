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

#include <tmc_manipulation_tests/configs.hpp>

#include <fstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace {
std::string GetPackagePath() {
  return ament_index_cpp::get_package_share_directory("tmc_manipulation_tests");
}

std::string LoadFile(const std::string& file_path) {
  std::string file_string;
  std::fstream file_stream(file_path, std::fstream::in);
  while (file_stream.good()) {
    std::string line;
    std::getline(file_stream, line);
    file_string += (line + "\n");
  }
  file_stream.close();
  return file_string;
}

}  // namespace

namespace tmc_manipulation_tests {

namespace hsra {
std::string GetUrdf() { return LoadFile(GetPackagePath() + "/urdf/hsra/hsra.urdf"); }
std::string GetCollisionConfig() { return LoadFile(GetPackagePath() + "/robots/hsra/collision_pair_hsra.xml"); }
}  // namespace hsra

namespace hsrb {
std::string GetUrdf() { return LoadFile(GetPackagePath() + "/urdf/hsrb/hsrb4s.urdf"); }
std::string GetCollisionConfig() { return LoadFile(GetPackagePath() + "/robots/hsrb/collision_pair_hsrb.xml"); }
}  // namespace hsrb

namespace hsrc {
std::string GetUrdf() { return LoadFile(GetPackagePath() + "/urdf/hsrc/hsrc1s.urdf"); }
}  // namespace hsrc

namespace stanford_manipulator {
std::string GetUrdf() { return LoadFile(GetPackagePath() + "/urdf/stanford_manipulator/stanford.urdf"); }
std::string GetCollisionConfig() {
  return LoadFile(GetPackagePath() + "/robots/stanford_manipulator/collision_detect_stanford.xml");
}
}  // namespace stanford_manipulator

}  // namespace tmc_manipulation_tests
