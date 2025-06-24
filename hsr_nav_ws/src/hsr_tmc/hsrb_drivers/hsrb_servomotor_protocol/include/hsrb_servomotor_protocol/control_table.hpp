/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
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
#ifndef HSRB_SERVOMOTOR_PROTOCOL_CONTROL_TABLE_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_CONTROL_TABLE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <boost/cstdint.hpp>
#include <boost/utility.hpp>

namespace hsrb_servomotor_protocol {

class ControlTableItemDescriptor;

class ControlTable : private boost::noncopyable {
 public:
  enum ErrorCode {
    kSuccess = 0,      /// Success
    kFileOpenError,    /// Failed to open the file
    kColumnSizeError,  /// Elementary error in control table
    kAlreadyRecorded,  /// There are two or more of the same entry
    kBadType,          /// The wrong type specification was made
  };

  ControlTable();

  ~ControlTable();

  /// Pass the definition file and calculate MD5Sum
  /// @return Successful with the success or failure of reading the file
  ErrorCode CalculateMd5Sum(const std::string& definition_file);

  /// Pass the definition file and initialize
  /// @return Successful with the success or failure of reading the file
  ErrorCode Load(const std::string& definition_file);

  /// Get MD5Sum of this control table
  std::vector<uint8_t> GetMd5Sum();

  /// Acquired property from the control table entry name
  /// If you give an unusual name, return the Shared_ptr sky
  std::shared_ptr<ControlTableItemDescriptor> ReferItemDescriptor(const std::string& entry) const;

 private:
  /// implementation
  class ControlTableImpl;

  /// pimpl
  std::unique_ptr<ControlTableImpl> pimpl_;
};

}  // namespace hsrb_servomotor_protocol
#endif  // HSRB_SERVOMOTOR_PROTOCOL_CONTROL_TABLE_HPP_
