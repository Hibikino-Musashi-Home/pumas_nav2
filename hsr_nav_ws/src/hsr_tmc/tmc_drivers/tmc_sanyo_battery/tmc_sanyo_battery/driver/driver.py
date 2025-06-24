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
# vim: fileencoding=utf-8 :
u"""Sanyo Electric Battery Communication Driver

Based on the communication specification [1] _, the battery and the communication are performed to get the information.

.. [1] For bicycles 4 average 5th communication specification ver.1.0

"""

import contextlib
import struct
import sys
import time

import serial


class BatteryError(Exception):
    pass


class TimeoutError(BatteryError):
    pass


class ProtocolError(BatteryError):
    pass


class ValidationError(BatteryError):
    pass


class DeviceOpenError(BatteryError):
    pass


def checksum(seq):
    u"""Checksum calculation

    Calculate the 2nd of 2 bytes in the Japanese sum of the handed array

    Args:
        SEQ (bytes): Checksam calculated bytes
    """
    if sys.version_info.major == 3:
        return struct.pack("<q", -sum(seq))[0]
    else:
        return struct.unpack("B", (struct.pack("<q", -sum(seq))[0]))[0]


@contextlib.contextmanager
def connect(device_name, baudrate, timeout=0.1):
    u"""Open the device and return the Connection object that holds it.

    It is premised that it is used in with statement.

    Args:
        Device_name (str): Device file name
    """
    port = serial.Serial(port=device_name,
                         baudrate=baudrate,
                         parity=serial.PARITY_NONE,
                         bytesize=serial.EIGHTBITS,
                         stopbits=serial.STOPBITS_ONE,
                         timeout=timeout)
    try:
        yield Connection(port)
    finally:
        port.close()


def default_status():
    u"""Return the DICT with the initial value of the batterystore"""
    status = {}
    status['battery_level'] = 0.0
    status['full_charge_capacity'] = 0.0
    status['remaining_charge'] = 0.0
    status['electric_current'] = 0.0
    status['voltage'] = 0.0
    status['temperature'] = 0
    status['zero_percent_detected'] = False
    status['discharge_enabled'] = False
    status['charge_enabled'] = False
    status['over_discharge'] = False
    status['full_charge'] = False
    status['learning_enabled'] = False
    status['triple_parallel'] = False
    status['over_charge'] = False
    return status


def read_packet(data):
    u"""Analyze the receiving packet and return the DICT with a battery station

    Args:
        Data (Bytes): Byte column of inbox
    """
    fields = struct.unpack("<BBBBHHhHBBBBBBB", data)
    if fields[0:4] != (0xFF, 0xFF, 0x0E, 0xD0):
        raise ProtocolError(
            "Invalid header: {0:02X} {1:02X} {2:02X} {3:02X}".format(
                *fields[0:4]))
    cs = checksum(bytearray(data[2:-1]))
    if cs != fields[14]:
        raise ProtocolError(
            "Expected checksum is {0}, actually {1}.".format(cs, fields[14]))
    status = default_status()
    status['full_charge_capacity'] = fields[4] / 1000.0
    if status['full_charge_capacity'] == 0.0:
        raise ValidationError("Zero full_charge_capacity is invalid.")
    status['remaining_charge'] = fields[5] / 1000.0
    status['battery_level'] = status['remaining_charge'] / \
        status['full_charge_capacity'] * 100.0
    status['electric_current'] = fields[6] / 1000.0 * 2.0
    status['voltage'] = fields[7] / 1000.
    status['temperature'] = fields[8] - 128
    flags = fields[13]
    status['zero_percent_detected'] = bool(flags & 0x01)
    status['discharge_enabled'] = bool(flags & 0x02)
    status['charge_enabled'] = bool(flags & 0x04)
    status['over_discharge'] = bool(flags & 0x08)
    status['full_charge'] = bool(flags & 0x10)
    status['learning_enabled'] = bool(flags & 0x20)
    status['triple_parallel'] = bool(flags & 0x40)
    status['over_charge'] = bool(flags & 0x80)
    return status


class Connection(object):
    u"""Serial communication connection with the battery pack

    Args:
        PORT (FileLike): File Like Budget connected to the battery pack
    """

    def __init__(self, filelike):
        self._filelike = filelike
        if sys.version_info.major == 3:
            self._last_updated = time.process_time()
        else:
            self._last_updated = time.clock()

    def read(self):
        u"""Communicate with the battery pack and get the current state.

        TODO: プロトコル仕様を書いておく
        """
        # It is necessary to open 0.1Sec or more between transmission and reception (see specifications)
        if sys.version_info.major == 3:
            now = time.process_time()
        else:
            now = time.clock()
        elapsed = now - self._last_updated
        if elapsed < 0.1:
            time.sleep(0.1 - elapsed)
        self._last_updated = now

        request = bytearray([0xFF, 0xFF, 0x00, 0xB0, 0x50])
        self._filelike.write(request)
        # Discard more for garbage data measures
        data = self._filelike.read(4096)
        if len(data) < 19:
            raise TimeoutError(
                "Timed out before receiving expected data. Expected 19 bytes, "
                "actually received {0} bytes.".format(len(data)))
        return read_packet(data[:19])
