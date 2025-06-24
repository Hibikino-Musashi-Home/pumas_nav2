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
/// @file control_table.cpp
/// @brief Implementation of classes that acquire control tables from CSV and use for communication
#define OPENSSL_API_COMPAT 10101

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <openssl/md5.h>

#include <hsrb_servomotor_protocol/control_table.hpp>
#include <hsrb_servomotor_protocol/control_table_item_descriptor.hpp>

typedef boost::tokenizer<boost::escaped_list_separator<char> > Tokenizer;

namespace hsrb_servomotor_protocol {

class ControlTable::ControlTableImpl {
 public:
  ControlTableImpl() : md5sum_(16) {}

  ~ControlTableImpl() {}

  ControlTable::ErrorCode CalculateMd5Sum(const std::string& definition_file) {
    std::ifstream ifs;
    try {
      ifs.open(definition_file.c_str());
    } catch (std::ifstream::failure e) {
      return kFileOpenError;
    } catch (...) {
      return kFileOpenError;
    }
    if (!ifs.is_open()) {
      return kFileOpenError;
    }

    std::ostringstream oss;
    std::string message;
    oss << ifs.rdbuf();
    message = oss.str();

    // Calculate MD5
    MD5(reinterpret_cast<const unsigned char*>(message.c_str()), message.length(), &md5sum_[0]);
    ifs.close();
    return kSuccess;
  }

  ControlTable::ErrorCode Load(const std::string& definition_file) {
    std::ifstream ifs;
    try {
      ifs.open(definition_file.c_str());
    } catch (std::ifstream::failure e) {
      return kFileOpenError;
    } catch (...) {
      return kFileOpenError;
    }
    if (!ifs.is_open()) {
      return kFileOpenError;
    }

    std::ostringstream oss;
    std::string message;
    oss << ifs.rdbuf();
    message = oss.str();

    // Calculate MD5
    MD5(reinterpret_cast<const unsigned char*>(message.c_str()), message.length(), &md5sum_[0]);

    std::string line;
    // From the beginning
    ifs.seekg(0, std::ios_base::beg);
    // The first line is skipped as a comment
    std::getline(ifs, line);
    uint16_t last_address = 0;
    uint32_t num_line = 0;
    while (std::getline(ifs, line)) {
      ErrorCode error = ProcessLine(line, last_address);
      if (error != kSuccess) {
        std::string error_string;
        switch (error) {
          case kColumnSizeError:
            error_string = "(column size error)";
            break;
          case kAlreadyRecorded:
            error_string = "(duplicate entry)";
            break;
          case kBadType:
            error_string = "(invalid value type)";
            break;
          default:
            error_string = "(invalid error)";
        }
        std::cerr << "Parse Error " << error_string << " at line " << num_line << std::endl;
        return error;
      }
      ++num_line;
    }
    ifs.close();
    return kSuccess;
  }

  std::vector<uint8_t> GetMd5Sum() { return md5sum_; }

  ControlTableItemDescriptor::Ptr ReferItemDescriptor(const std::string& entry) {
    ControlTableItemDescriptor::Ptr value;
    if (descriptors_.find(entry) != descriptors_.end()) {
      value = descriptors_[entry];
    }
    return value;
  }

 private:
  std::vector<uint8_t> md5sum_;

  std::map<std::string, ControlTableItemDescriptor::Ptr> descriptors_;

  /// One line process and return the final address
  ControlTable::ErrorCode ProcessLine(const std::string& line, uint16_t& last_address) {
    Tokenizer tokens(line);
    if (std::distance(tokens.begin(), tokens.end()) != 12) {
      return kColumnSizeError;
    }
    Tokenizer::iterator it = tokens.begin();
    // Read Type
    ++it;
    std::string type_str = *it;
    // Read Name
    ++it;
    std::string name = *it;
    // Read MKS units
    std::advance(it, 3);
    std::string mks_str = *it;
    // Read the type
    ++it;
    std::string attribute_str = *it;

    // If you have the same name, fail immediately
    if (descriptors_.find(name) != descriptors_.end()) {
      return kAlreadyRecorded;
    }
    ControlTableItemDescriptor::ValueType type;
    uint8_t num_bytes = 0;
    if (type_str == "uint8_t") {
      type = ControlTableItemDescriptor::kUInt8;
      num_bytes = 1;
    } else if (type_str == "int8_t") {
      type = ControlTableItemDescriptor::kInt8;
      num_bytes = 1;
    } else if (type_str == "uint16_t") {
      type = ControlTableItemDescriptor::kUInt16;
      num_bytes = 2;
    } else if (type_str == "int16_t") {
      type = ControlTableItemDescriptor::kInt16;
      num_bytes = 2;
    } else if (type_str == "uint32_t") {
      type = ControlTableItemDescriptor::kUInt32;
      num_bytes = 4;
    } else if (type_str == "int32_t") {
      type = ControlTableItemDescriptor::kInt32;
      num_bytes = 4;
    } else if (type_str == "uint64_t") {
      type = ControlTableItemDescriptor::kUInt64;
      num_bytes = 8;
    } else if (type_str == "int64_t") {
      type = ControlTableItemDescriptor::kInt64;
      num_bytes = 8;
    } else if (type_str == "float") {
      type = ControlTableItemDescriptor::kFloat;
      num_bytes = 4;
    } else if (type_str == "double") {
      type = ControlTableItemDescriptor::kDouble;
      num_bytes = 8;
    } else {
      return kBadType;
    }
    double coeff_mks = 1.0;
    try {
      coeff_mks = boost::lexical_cast<double>(mks_str);
    } catch (boost::bad_lexical_cast& e) {
      coeff_mks = 1.0;
    }

    ControlTableItemDescriptor::Ptr descriptor(
        new ControlTableItemDescriptor(type, last_address, attribute_str, coeff_mks));
    descriptors_.insert(std::make_pair(name, descriptor));

    last_address += num_bytes;
    return kSuccess;
  }
};

ControlTable::ControlTable() : pimpl_(new ControlTable::ControlTableImpl()) {}

ControlTable::~ControlTable() {}

ControlTable::ErrorCode ControlTable::CalculateMd5Sum(const std::string& definition_file) {
  return pimpl_->CalculateMd5Sum(definition_file);
}

ControlTable::ErrorCode ControlTable::Load(const std::string& definition_file) { return pimpl_->Load(definition_file); }

std::vector<uint8_t> ControlTable::GetMd5Sum() { return pimpl_->GetMd5Sum(); }

std::shared_ptr<ControlTableItemDescriptor> ControlTable::ReferItemDescriptor(const std::string& entry) const {
  return pimpl_->ReferItemDescriptor(entry);
}

}  // namespace hsrb_servomotor_protocol
