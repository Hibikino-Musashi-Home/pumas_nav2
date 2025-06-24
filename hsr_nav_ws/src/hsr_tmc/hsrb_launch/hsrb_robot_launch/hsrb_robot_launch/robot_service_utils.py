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
# -*- coding: utf-8 -*-
import subprocess
import sys

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ros2run.api import get_executable_path

MOTOR_DEVICE = '/dev/ttyCTI2'
MOTOR_IDS = [11, 12, 13, 21, 22, 23, 24, 25, 31, 32, 41]


def exxx_read_hash(motor_id):
    u"""EXXX_Read_hash trumpet

    If you succeed in communication, return the following DICT
    {'firmware_hash': '0bd8ae8035a365c72dddf85e712117156c824372',
     'control_table_hash': '6fe89466421b08f50b5bb06e59f1fb09'}
    Return None if you fail
    """
    try:
        exxx_read_hash_commnad = get_executable_path(
            package_name='hsrb_servomotor_protocol', executable_name='exxx_read_hash')
        output = subprocess.check_output([exxx_read_hash_commnad,
                                          MOTOR_DEVICE,
                                          str(motor_id)],
                                         stderr=subprocess.PIPE)
        if sys.version_info.major == 3:
            output = output.decode()
        output = {line.split('=')[0]: line.split('=')[1]
                  for line in output.replace(' ', '').split('\n')[:-1]}
        return output
    except subprocess.CalledProcessError:
        return None


def exxx_read_data_table(param, motor_id):
    u"""EXXX_Read_data_table trumpet

    Return None if you fail
    """
    try:
        exxx_read_data_table_command = get_executable_path(
            package_name='hsrb_servomotor_protocol', executable_name='exxx_read_data_table')
        output = subprocess.check_output([exxx_read_data_table_command,
                                          MOTOR_DEVICE,
                                          str(motor_id),
                                          param],
                                         stderr=subprocess.PIPE)
        if sys.version_info.major == 3:
            output = output.decode()
        for line in output.split('\n'):
            if line.startswith(param):
                return line.split(':')[1].strip()
        return None
    except subprocess.CalledProcessError:
        return None


class RobotServiceDiagPublisher(object):
    u"""ROBOT_SERVICE diager class"""

    def __init__(self, node):
        self._node = node
        self._diag_pub = self._node.create_publisher(
            DiagnosticArray, 'diagnostics', 10)

    def _get_lsusb_diag(self):
        u"""Set the LSUSB result to DiagnostixTatus"""
        status = DiagnosticStatus()
        status.name = "hsrb_robot_service: lsusb"
        status.hardware_id = 'USB'
        try:
            result = subprocess.check_output(["lsusb"])
            if sys.version_info.major == 3:
                result = result.decode()
            status.level = DiagnosticStatus.OK
            status.message = "lsusb succeeded"
        except subprocess.CalledProcessError as err:
            status.level = DiagnosticStatus.ERROR
            status.message = "lsusb failed"
            result = err.output
        status.values.append(KeyValue(
            key="result",
            value=result))
        return status

    def _get_motor_status_diag(self, param):
        u"""Read the param for the specified axis and make it Diagnosticstatus"""
        status = DiagnosticStatus()
        status.name = "hsrb_robot_service: {0}".format(param)
        status.level = DiagnosticStatus.OK
        status.hardware_id = 'motor {0}'.format(param)
        status.message = "exxx_read {0} succeeded".format(param)
        for motor_id in MOTOR_IDS:
            value = exxx_read_data_table(param, motor_id)
            if value is None:
                status.level = DiagnosticStatus.ERROR
                status.message = "exxx_read {0} failed".format(param)
                value = "Unknown"
            status.values.append(KeyValue(
                key=str(motor_id),
                value=value))
        return status

    def get_subscription_count(self):
        return self._diag_pub.get_subscription_count()

    def publish(self):
        u"""Published the necessary logs before starting

        1. lsusb
        2. present_log_power_cycle
        3. present_log_motor_rev_total
        """
        diag = DiagnosticArray()
        diag.header.stamp = self._node.get_clock().now().to_msg()
        status = self._get_lsusb_diag()
        diag.status.append(status)
        status = self._get_motor_status_diag('present_log_power_cycle')
        diag.status.append(status)
        status = self._get_motor_status_diag('present_log_motor_rev_total')
        diag.status.append(status)
        self._diag_pub.publish(diag)
