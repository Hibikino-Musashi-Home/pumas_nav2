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
/// @brief    Read the STL file
#ifndef TMC_STL_LOADER_STL_LOADER_HPP_
#define TMC_STL_LOADER_STL_LOADER_HPP_
#include <stdint.h>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace tmc_stl_loader {
// Mesh information
struct Mesh {
  Mesh() {
    vertices.clear();
    normals.clear();
    indices.clear();
  }
  /// vertex
  std::vector<Eigen::Vector3f> vertices;
  /// Lines
  std::vector<Eigen::Vector3f> normals;
  /// Number of aspects
  std::vector<uint32_t> indices;
};
// Search for the target vector from 3D vector groups
void GetVertexIndex(const std::vector<Eigen::Vector3f> vertices, const Eigen::Vector3f& vector3, uint32_t& vector_id);
// STL file reading class
class STLLoader {
 public:
  using Ptr = std::shared_ptr<STLLoader>;
  STLLoader() {}
  virtual ~STLLoader() {}
  /// Read the STL file
  void Load(const std::string& file_name, Mesh& mesh);

 private:
  STLLoader(const STLLoader&);
  STLLoader& operator=(const STLLoader&);
  /// Read FLOAT type 1 byte
  void ReadFloat_(uint8_t* data, float& byte_out);
  /// Read FLOAT type 1 byte
  void ReadFloat_(FILE* file, float& byte_out);
  /// Read the UNSIGNED-Integer type 1 byte
  void ReadLongInt_(uint8_t* data, uint32_t& byte_out);
  /// Read the UNSIGNED-Integer type 1 byte
  void ReadLongInt_(FILE* file, uint32_t& byte_out);
  /// Read STL binary according to the format
  void ReadBinary_(FILE* file, tmc_stl_loader::Mesh& mesh);
};  // class STLLoader
}  // namespace tmc_stl_loader
#endif  // TMC_STL_LOADER_STL_LOADER_HPP_
