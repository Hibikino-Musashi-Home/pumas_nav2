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
#! /usr/bin/env python
# -*- coding: utf-8 -*-
import unittest

from diagnostic_msgs.msg import (
    DiagnosticArray,
    DiagnosticStatus,
)
import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import ColorRGBA


@pytest.mark.launch_test
def generate_test_description():
    status_led_node = launch_ros.actions.Node(
        package='hsrb_status_led',
        executable='status_led_node',
        parameters=[{'publish_rate': 10.0,
                     'update_timeout': 0.5,
                     'status_boot_timeout': 2.0}],
        output='screen'
    )
    return launch.LaunchDescription(
        [status_led_node, launch_testing.actions.ReadyToTest()])


class TestStatusLedNodeCommon(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._node = rclpy.create_node("test_node")

        self._command_color = None
        self._command_sub = self._node.create_subscription(
            ColorRGBA, 'command_status_led_rgb', self._command_callback, 1)

        status = DiagnosticStatus()
        status.name = 'test_diag'
        self._diag_agg = DiagnosticArray()
        self._diag_agg.status.append(status)

        # When I Destroy Timer other than TEARDOWN, the exception flew in spin_once.
        # So, always run and manage the Publish Timer.
        self._diag_pub = self._node.create_publisher(DiagnosticArray, '/diagnostics_agg', 1)
        self._do_publish_diag = False
        self._diag_pub_timer = self._node.create_timer(0.2, self._publish_diag_thread)

    def tearDown(self):
        self._diag_pub_timer.destroy()

    def _start_diag_publish(self, level):
        self._diag_agg.status[0].level = level
        self._do_publish_diag = True

    def _stop_diag_publish(self):
        self._do_publish_diag = False

    def _command_callback(self, msg):
        self._command_color = msg

    def _wait_for_command(self, command):
        timeout = self._node.get_clock().now() + Duration(seconds=3.0)
        while self._node.get_clock().now() < timeout:
            if self._command_color == command:
                return True
            rclpy.spin_once(self._node)
        return False

    def _publish_diag_thread(self):
        if self._do_publish_diag:
            self._diag_pub.publish(self._diag_agg)
