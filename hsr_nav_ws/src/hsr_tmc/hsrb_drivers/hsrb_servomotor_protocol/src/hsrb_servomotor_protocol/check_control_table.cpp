/*
Copyright (c) 2014 TOYOTA MOTOR CORPORATION
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
#include <iostream>
#include <string>
#include <vector>

#include <hsrb_servomotor_protocol/control_table.hpp>
#include <hsrb_servomotor_protocol/control_table_item_descriptor.hpp>

using hsrb_servomotor_protocol::ControlTable;
using hsrb_servomotor_protocol::ControlTableItemDescriptor;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "check_control_table [TABLE_FILE]" << std::endl;
    return EXIT_FAILURE;
  }
  ControlTable control_table;
  ControlTable::ErrorCode error = control_table.Load(argv[1]);
  if (error == ControlTable::kSuccess) {
    std::cerr << "Parse Success!!" << std::endl;
    return EXIT_SUCCESS;
  } else {
    std::cerr << "Parse Failed!!" << std::endl;
    return EXIT_FAILURE;
  }
}
