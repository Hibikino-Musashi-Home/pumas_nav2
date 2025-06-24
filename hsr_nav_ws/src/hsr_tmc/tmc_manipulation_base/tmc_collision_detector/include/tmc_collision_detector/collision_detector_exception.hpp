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
#ifndef TMC_COLLISION_DETECTOR_COLLISION_DETECTOR_EXCEPTION_HPP_
#define TMC_COLLISION_DETECTOR_COLLISION_DETECTOR_EXCEPTION_HPP_

#include  <exception>
#include  <stdexcept>
#include  <string>

namespace tmc_collision_detector {
/// I tried to manipulate an object that I didn't Create
class NonCreateError : public std::domain_error {
 public:
  explicit NonCreateError(const std::string& error) :
    std::domain_error("error: operation to non created object " + error) {}
};

/// There is no type
class NonExistTypeError : public std::domain_error {
 public:
  explicit NonExistTypeError(const std::string& error) :
  std::domain_error("error: not exist primitive type " + error) {}
};

/// Shape parameters are illegal (length is negative, there is no STL file, etc.)
class InvalidShapeParamError : public std::domain_error {
 public:
  explicit InvalidShapeParamError(const std::string& error) :
  std::domain_error("error: " + error) {}
};

/// Unauthorized objects interfere
class InvalidObjectContactError : public std::domain_error {
 public:
  explicit InvalidObjectContactError(const std::string& error) :
  std::domain_error("error: invalid collision objects contacted" + error) {}
};

/// Called an unsupported method
class UnsupportMethodError : public std::domain_error {
 public:
  explicit UnsupportMethodError(const std::string& error) :
  std::domain_error("error: this method is not supported. " + error) {}
};
}  // namespace tmc_collision_detector
#endif  // TMC_COLLISION_DETECTOR_COLLISION_DETECTOR_EXCEPTION_HPP_

