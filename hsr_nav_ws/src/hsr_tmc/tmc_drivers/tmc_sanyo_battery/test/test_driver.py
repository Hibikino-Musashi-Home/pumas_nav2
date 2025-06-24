#!/usr/bin/env python
'''
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
'''

import copy

from nose.tools import eq_, ok_, raises
from tmc_sanyo_battery.driver import driver

valid_packet = bytearray([
    0xFF, 0xFF, 0x0E, 0xD0,
    0x01, 0x02, 0x03, 0x04,
    0x05, 0x06, 0x07, 0x08,
    0x09, 0x00, 0x00, 0x00,
    0x00, 0x5A, 0x9B,
])

valid_result = {
    'full_charge_capacity': float(0x0201) / 1000.0,
    'remaining_charge': float(0x0403) / 1000.0,
    'battery_level': (float(0x0403) / 1000.0) / (float(0x0201) / 1000.0) * 100.0,
    'electric_current': float(0x0605) / 1000.0 * 2.0,
    'voltage': float(0x0807) / 1000.0,
    'temperature': float(0x09) - 128.0,
    'zero_percent_detected': False,
    'discharge_enabled': True,
    'charge_enabled': False,
    'over_discharge': True,
    'full_charge': True,
    'learning_enabled': False,
    'triple_parallel': True,
    'over_charge': False,
}


def test_checksum():
    cs = driver.checksum(valid_packet[2:-1])
    ok_(isinstance(cs, int))
    eq_(cs, valid_packet[-1])


@raises(driver.ProtocolError)
def test_read_packet_header():
    invalid_packet = copy.copy(valid_packet)
    invalid_packet[1] = 0xFE
    driver.read_packet(bytes(invalid_packet))


@raises(driver.ProtocolError)
def test_read_packet_checksum():
    invalid_packet = copy.copy(valid_packet)
    invalid_packet[-1] = 0x9A
    driver.read_packet(bytes(invalid_packet))


def test_read_packet():
    result = driver.read_packet(bytes(valid_packet))
    eq_(result, valid_result)


def test_connection():
    class PortMock(object):

        def read(self, num_bytes):
            return bytes(valid_packet)

        def write(self, data):
            self.data = data

    port_mock = PortMock()
    conn = driver.Connection(port_mock)
    result = conn.read()
    eq_(len(port_mock.data), 5)
    eq_(port_mock.data, bytes(bytearray([0xFF, 0xFF, 0x00, 0xB0, 0x50])))
    eq_(result, valid_result)


@raises(driver.TimeoutError)
def test_connection_timeout():
    class PortMock(object):

        def read(self, num_bytes):
            return []

        def write(self, data):
            self.data = data

    port_mock = PortMock()
    conn = driver.Connection(port_mock)
    conn.read()
