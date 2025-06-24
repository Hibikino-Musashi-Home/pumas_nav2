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
#include <boost/lexical_cast.hpp>
#include <hsrb_servomotor_protocol/exxx_network.hpp>
#include <hsrb_servomotor_protocol/exxx_protocol.hpp>

namespace {
const int32_t kNetworkTimeout = 100000000;
const int32_t kNetworkTick = 10000;
}


int main(int argc, char** argv) {
  using hsrb_servomotor_protocol::INetwork;
  using hsrb_servomotor_protocol::ExxxNetwork;
  using hsrb_servomotor_protocol::ExxxProtocol;
  using hsrb_servomotor_protocol::ExxxWarningCategory;
  if ((argc != 5) && (argc != 6) && (argc != 7)) {
    std::cout << "USAGE: exxx_read_data DEV ID ADDRESS BYTE [OPTION]\n"
              << "\n"
              << "OPTION: \n"
              << "  -s, --signed  符号つき整数として表示する．"
              << "  -u, --usb usb485を利用．" << std::endl;
    return EXIT_SUCCESS;
  }

  boost::system::error_code error;
  try {
    uint8_t id = boost::lexical_cast<int>(argv[2]);
    uint16_t addr = boost::lexical_cast<int>(argv[3]);
    uint8_t byte = boost::lexical_cast<int>(argv[4]);
    bool is_signed = false;
    bool is_usb = false;
    if (argc > 5) {
      for (int i = 5; i < argc; ++i) {
        if (std::string("-s") == std::string(argv[i]) || std::string("--signed") == std::string(argv[i])) {
          is_signed = true;
        }
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

    switch (byte) {
      case 1:
        if (is_signed) {
          int8_t value = 0;
          error = protocol.ReadData(id, addr, value);
          std::cout << static_cast<int>(value) << std::endl;
        } else {
          uint8_t value = 0;
          error = protocol.ReadData(id, addr, value);
          std::cout << static_cast<int>(value) << std::endl;
        }
        break;
      case 2:
        if (is_signed) {
          int16_t value = 0;
          error = protocol.ReadData(id, addr, value);
          std::cout << value << std::endl;
        } else {
          uint16_t value = 0;
          error = protocol.ReadData(id, addr, value);
          std::cout << value << std::endl;
        }
        break;
      case 4: {
        if (is_signed) {
          int32_t value = 0;
          error = protocol.ReadData(id, addr, value);
          std::cout << value << std::endl;
        } else {
          uint32_t value = 0;
          error = protocol.ReadData(id, addr, value);
          std::cout << value << std::endl;
        }
        break;
      }
      case 8: {
        if (is_signed) {
          int64_t value = 0;
          error = protocol.ReadData(id, addr, value);
          std::cout << value << std::endl;
        } else {
          uint64_t value = 0;
          error = protocol.ReadData(id, addr, value);
          std::cout << value << std::endl;
        }
        break;
      }
      default:
        std::cerr << "byte must be 1 or 2 or 4 or 8." << std::endl;
        return EXIT_FAILURE;
    }
  } catch (const boost::bad_lexical_cast& ex) {
    std::cerr << ex.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Unexpected catch!" << std::endl;
    return EXIT_FAILURE;
  }

  if (error && error.category() != ExxxWarningCategory()) {
    std::cerr << "Failed to read control table: " << error.message() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
