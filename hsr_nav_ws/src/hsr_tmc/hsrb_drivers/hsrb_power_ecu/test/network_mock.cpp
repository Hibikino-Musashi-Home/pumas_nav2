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

#include "network_mock.hpp"

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>

#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/xpressive/xpressive.hpp>

#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termio.h>
#include <unistd.h>

#include <hsrb_power_ecu/i_network.hpp>
#include "common_methods.hpp"

namespace {
//! <Packet list to return ACK
const char kNeedAckPackets[20][9] = {"H,time_,", "H,pdown,", "H,start,", "H,stop_,", "H,heart,",
                                     "H,pump_,", "H,pbmsw,", "H,ledc_,", "H,g_res,", "H,solsw,",
                                     "H,pdcmd,", "H,mute_,", "H,rpros,", "H,rprod,", "H,rproe,",
                                     "H,undck,", "H,12vu_,", "H,5vd3_,", "H,5vd4_,", "H,5vd5_,"};
const char kNeedVerPacket[] = "H,getv_,";

const char kAckPacketString[] = "E,rxack,015,h00,h83693205,\n";
const char kVerPacketString[] =
    "E,ver__,095,h0040A85DD7B13048AEA3B4D1DC5AA120A314797D,"
    "hB7335B767D0FA2E6925BC8E965E443291A16A26A,";
}  // anonymous namespace

namespace hsrb_power_ecu {
NetworkMock::NetworkMock()
    : receive_buffer_(),
      send_buffer_(),
      timeout_ns_(0),
      port_name_(),
      is_need_ack_(false) {
}

NetworkMock::~NetworkMock() {}

boost::system::error_code NetworkMock::Open() {
  return boost::system::errc::make_error_code(boost::system::errc::success);
}

boost::system::error_code NetworkMock::Close() {
  return boost::system::errc::make_error_code(boost::system::errc::success);
}

boost::system::error_code NetworkMock::Configure(const std::string &param, const int32_t value) {
  return Configure(param, boost::lexical_cast<std::string>(value));
}

boost::system::error_code NetworkMock::Configure(const std::string &param, const double value) {
  return Configure(param, boost::lexical_cast<std::string>(value));
}

boost::system::error_code NetworkMock::Configure(const std::string &param, const std::string &value) {
  if (param == "receive_timeout_ms") {
    // timeout
    double const timeout = boost::lexical_cast<double>(value);
    if (timeout < 0) {
      return boost::system::errc::make_error_code(boost::system::errc::invalid_argument);
    }
    timeout_ns_ = static_cast<uint32_t>(timeout * 1000000.0);
  } else if (param == "device_name") {
    // Device name
    port_name_ = value;
  } else {
    return boost::system::errc::make_error_code(boost::system::errc::invalid_argument);
  }
  return boost::system::errc::make_error_code(boost::system::errc::success);
}

boost::system::error_code NetworkMock::Send(const PacketBuffer &data) {
  // Copy data to the transmission buffer
  send_buffer_ = "";
  std::copy(data.begin(), data.end(), std::back_inserter(send_buffer_));

  if (!is_need_ack_) {
    BOOST_FOREACH (const std::string &s, kNeedAckPackets) {
      if (send_buffer_.find(s) != std::string::npos) {
        is_need_ack_ = true;
        break;
      }
    }
  }
  if (send_buffer_.find(kNeedVerPacket) != std::string::npos) {
    is_need_ver_ = true;
  }

  return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
}

boost::system::error_code NetworkMock::Receive(PacketBuffer &data) {
  std::string packet = "";
  if (is_need_ack_) {
    packet += kAckPacketString;
    is_need_ack_ = false;
  }
  if (is_need_ver_) {
    std::stringstream sst;
    sst << kVerPacketString;
    add_footer(sst);
    packet += sst.str();
    is_need_ver_ = false;
  }
  {
    boost::unique_lock<boost::mutex> is_lock(receive_buffer_mutex_, boost::try_to_lock);
    if (is_lock) {
      if (is_update_receive_buffer_) {
        packet += receive_buffer_;
        receive_buffer_ = "";
      }
    }
  }

  if (packet != "") {
    // Copy the contents of the receipt buffer
    std::copy(packet.begin(), packet.end(), std::back_inserter(data));
  }
  return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
}

std::string NetworkMock::GetSendBuffer() const {return send_buffer_;}
void NetworkMock::ResetSendBuffer() {send_buffer_ = "";}

void NetworkMock::UpdateBuffer(const std::string& buffer_data) {
  std::stringstream sst;
  sst << buffer_data;
  add_footer(sst);
  boost::lock_guard<boost::mutex> lock(receive_buffer_mutex_);
  receive_buffer_ = sst.str();
  is_update_receive_buffer_ = true;
}
}  // namespace hsrb_power_ecu
