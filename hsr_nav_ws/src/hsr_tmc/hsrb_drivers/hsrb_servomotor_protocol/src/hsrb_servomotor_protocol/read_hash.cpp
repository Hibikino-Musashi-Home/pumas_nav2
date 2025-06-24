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
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <hsrb_servomotor_protocol/exxx_network.hpp>
#include <hsrb_servomotor_protocol/exxx_protocol.hpp>
#include <hsrb_servomotor_protocol/network.hpp>

namespace {
const int32_t kNetworkTimeout = 100000000;
const int32_t kNetworkTick = 10000;
}

int main(int argc, char** argv) {
  using hsrb_servomotor_protocol::INetwork;
  using hsrb_servomotor_protocol::ExxxNetwork;
  using hsrb_servomotor_protocol::ExxxProtocol;
  if ((argc < 2) || (argc > 4)) {
    std::cout << "USAGE: exxx_hash DEV ID [OPTION]\n"
              << "\n"
              << "OPTION: \n"
              << "  -u, --usb use usb485." << std::endl;
    return EXIT_SUCCESS;
  }

  boost::system::error_code error;
  uint8_t id = boost::lexical_cast<int>(argv[2]);
  bool is_signed = false;
  bool is_usb = false;
  if (argc > 2) {
    for (int i = 3; i < argc; ++i) {
      if (std::string("-u") == std::string(argv[i]) || std::string("--usb") == std::string(argv[i])) {
        is_usb = true;
      }
    }
  }
  INetwork::Ptr network(new ExxxNetwork(argv[1], error, is_usb, kNetworkTimeout, kNetworkTick));
  if (error) {
    std::cerr << error.message() << std::endl;
    return EXIT_FAILURE;
  }

  ExxxProtocol protocol(network);
  int8_t value = 0;
  std::vector<uint8_t> control_table_hash;
  std::vector<uint8_t> firmware_hash;

  error = protocol.ReadHash(id, control_table_hash, firmware_hash);
  if (error) {
    std::cerr << error.message() << std::endl;
    return EXIT_FAILURE;
  }
  {
    std::stringstream buf;
    for (size_t i = 0; i < control_table_hash.size(); ++i) {
      buf << " " << std::setw(2) << std::setfill('0') << std::hex << static_cast<uint32_t>(control_table_hash[i]);
    }
    std::cout << "control_table_hash = " << buf.str() << std::endl;
  }
  {
    std::stringstream buf;
    for (size_t i = 0; i < firmware_hash.size(); ++i) {
      buf << " " << std::setw(2) << std::setfill('0') << std::hex << static_cast<uint32_t>(firmware_hash[i]);
    }
    std::cout << "firmware_hash = " << buf.str() << std::endl;
  }

  return EXIT_SUCCESS;
}
