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
#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/lexical_cast.hpp>
#include <rclcpp/rclcpp.hpp>
#include <hsrb_servomotor_protocol/control_table.hpp>
#include <hsrb_servomotor_protocol/control_table_item_descriptor.hpp>
#include <hsrb_servomotor_protocol/exxx_network.hpp>
#include <hsrb_servomotor_protocol/exxx_protocol.hpp>
#include <hsrb_servomotor_protocol/exxx_warning_category.hpp>
#include <hsrb_servomotor_protocol/load_control_table.hpp>
#include <hsrb_servomotor_protocol/network.hpp>

namespace {
const int32_t kNetworkTimeout = 100000000;
const int32_t kNetworkTick = 10000;
}

using hsrb_servomotor_protocol::ControlTable;
using hsrb_servomotor_protocol::ControlTableItemDescriptor;

int main(int argc, char** argv) {
  using hsrb_servomotor_protocol::INetwork;
  using hsrb_servomotor_protocol::ExxxNetwork;
  using hsrb_servomotor_protocol::ExxxProtocol;
  using hsrb_servomotor_protocol::ExxxWarningCategory;
  if ((argc != 4) && (argc != 5)) {
    std::cout << "USAGE: exxx_read DEV ID ITEM [OPTION]\n"
              << "\n"
              << "OPTION: \n"
              << "  -u, --usb use usb485." << std::endl;
    return EXIT_SUCCESS;
  }

  boost::system::error_code error;

  uint8_t id = boost::lexical_cast<int32_t>(argv[2]);
  uint16_t addr;
  bool is_usb = false;
  if (argc == 5) {
    if (std::string("-u") == std::string(argv[4]) || std::string("--usb") == std::string(argv[4])) {
      is_usb = true;
    }
  }
  INetwork::Ptr network(new ExxxNetwork(argv[1], error, is_usb, kNetworkTimeout, kNetworkTick));
  if (error) {
    std::cerr << error.message() << std::endl;
    return EXIT_FAILURE;
  }
  ExxxProtocol protocol(network);
  ControlTable table;

  std::vector<uint8_t> control_table_hash;
  std::vector<uint8_t> firmware_hash;
  error = protocol.ReadHash(id, control_table_hash, firmware_hash);
  if (error && error.category() == boost::system::system_category()) {
    std::cerr << "Failed to read control table: " << error.message() << std::endl;
    return EXIT_FAILURE;
  }

  auto logger = rclcpp::get_logger("read_data_table");
  std::string package_path = ament_index_cpp::get_package_share_directory("exxx_control_table");
  if (package_path.empty()) {
    RCLCPP_FATAL(logger, "exxx_control_table package is not found");
    return EXIT_FAILURE;
  }

  std::string control_table_path;
  if (hsrb_servomotor_protocol::LoadControlTable(package_path, control_table_hash, table, control_table_path)) {
    RCLCPP_INFO_STREAM(logger, "control_table_path : " << control_table_path);
  } else {
    RCLCPP_FATAL(logger, "Cannot find right control table");
    return EXIT_FAILURE;
  }

  ControlTableItemDescriptor::Ptr item = table.ReferItemDescriptor(argv[3]);
  if (!item) {
    std::cerr << "Control table item " << argv[3] << " is not found!" << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<uint8_t> data(item->num_bytes());
  error = protocol.ReadBlock(id, item->initial_address(), item->num_bytes(), &data[0]);
  if (error && error.category() == boost::system::system_category()) {
    std::cerr << "Failed to read control table: " << error.message() << std::endl;
    return EXIT_FAILURE;
  }

  double dvalue;
  if (item->ConvertToMKS(data, dvalue)) {
    std::cout << argv[3] << " of node " << static_cast<int32_t>(id) << " : " << dvalue << std::endl;
  } else {
    std::cerr << "Control table item " << argv[3] << " cannot convert" << std::endl;
  }
  return EXIT_SUCCESS;
}
