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
import threading

from diagnostic_msgs.msg import DiagnosticStatus
from hsrb_status_led.diagnostics import Diagnostics
from hsrb_status_led.led_pattern import (
    DiagnosticMonitoringPattern,
    FixedColorPattern,
)
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import ColorRGBA


class StatusLedControlMain(Node):
    """State display LED operation class"""

    _DEFAULT_OK_COLOR = ColorRGBA(g=1.0, b=1.0)    # CYAN
    _DEFAULT_BOOT_COLOR = ColorRGBA(r=1.0, g=1.0)  # YELLOW
    _DEFAULT_ERROR_COLOR = ColorRGBA(r=1.0)        # RED

    def __init__(self):
        """constructor"""
        super().__init__('status_led')

        self._active_pattern = None
        self._next_pattern = None
        self._lock = threading.Lock()

        ok_color = self._get_color_param('ok_color', self._DEFAULT_OK_COLOR)
        boot_color = self._get_color_param('boot_color', self._DEFAULT_BOOT_COLOR)
        error_color = self._get_color_param('error_color', self._DEFAULT_ERROR_COLOR)
        error_blinking_period = self._get_param('error_blinking_period', 0.0)
        publish_rate = self._get_param('publish_rate', 1.0)
        diag_timeout = self._get_param('update_timeout', 5.0)
        boot_timeout = self._get_param('status_boot_timeout', 60.0)
        ignored_diags = self._get_param('ignored_diagnostics', [])

        self._diagnostics = Diagnostics(self, diag_timeout, ignored_diags)
        self._monitoring_pattern = DiagnosticMonitoringPattern(
            publish_rate, self.get_clock(), self._diagnostics, ok_color, error_color, error_blinking_period)

        self._booting_pattern = FixedColorPattern(publish_rate, self.get_clock(), boot_color)
        self.notify_pattern(self._booting_pattern)

        self._boot_timeout = self.get_clock().now() + Duration(seconds=boot_timeout)
        self._on_booting_timer = self.create_timer(1.0 / publish_rate, self._on_booting_callback)

        self._status_led_pub = self.create_publisher(ColorRGBA, 'command_status_led_rgb', 1)
        self._run_timer = self.create_timer(0.01, self._run_timer_callback)

    def _get_param(self, name, default_value):
        self.declare_parameter(name, default_value)
        return self.get_parameter(name).value

    def _get_color_param(self, param_name, default_color):
        """Get color information from the parameter server

        Args:
            param_name (String): Parameter name
            default_color (std_msgs.msg.colorrgba): Value when parameters do not exist

        Returns:
            std_msgs.msg.colorrgba: Acquired colored information
        """
        color = ColorRGBA()
        color.r = self._get_param(param_name + '.r', -1.0)
        color.g = self._get_param(param_name + '.g', -1.0)
        color.b = self._get_param(param_name + '.b', -1.0)
        if all([0.0 <= x <= 1.0 for x in [color.r, color.g, color.b]]):
            return color
        else:
            return default_color

    def _run_timer_callback(self):
        """Main loop"""
        with self._lock:
            if self._next_pattern is not None:
                self._active_pattern = self._next_pattern
                self._active_pattern.reset()
                self._next_pattern = None
        if self._active_pattern is None:
            return

        if self._active_pattern.do_publish:
            with self._lock:
                last_color = self._active_pattern.color
            self._status_led_pub.publish(last_color)

    def notify_pattern(self, pattern=None):
        """Notify the LED light emission pattern you want to change

        Args:
            pattern (led_patternLedPatternBase):
                LED light emitting pattern, none means diager monitoring, default value is None
        """
        with self._lock:
            if pattern is None:
                self._next_pattern = self._monitoring_pattern
            else:
                self._next_pattern = pattern

    def _on_booting_callback(self):
        """Promotion at the time of node startup

        Diag information becomes normal or diag monitoring is started during timeout
        However, if the light emitting pattern has been changed before the start, it will not do anything.
        """
        if (self.get_clock().now() > self._boot_timeout
                or self._diagnostics.last_level in [DiagnosticStatus.OK, DiagnosticStatus.WARN]):
            self._on_booting_timer.destroy()
            if self._active_pattern == self._booting_pattern:
                self.notify_pattern()
