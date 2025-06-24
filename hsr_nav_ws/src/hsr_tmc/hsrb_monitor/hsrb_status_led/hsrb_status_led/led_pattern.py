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
import sys

from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.duration import Duration
from std_msgs.msg import ColorRGBA


class LedPatternBase:
    """State display LED basics class class"""

    def __init__(self, rate, clock):
        """constructor

        Args:
            rate (Float): Discount issuance cycle [Hz]
            clock (Rclpy.clock.clock): Clock instance
        """
        self._clock = clock
        self._sleep_duration = Duration(seconds=1.0 / rate)
        self.reset()

    @property
    def color(self):
        """Return the color you want to shine

        Returns:
            std_msgs.msg.colorrgba: All elements 0
        """
        return ColorRGBA(r=0.0, g=0.0, b=0.0)

    @property
    def do_publish(self):
        """Whether to update the status display LED value"""
        if self._clock.now() > self._next_publish_time:
            self._next_publish_time += self._sleep_duration
            return True
        else:
            return False

    def reset(self):
        """Time reset for Sleep"""
        self._next_publish_time = self._clock.now() + self._sleep_duration


class FixedColorPattern(LedPatternBase):
    """Light of the specified color"""

    def __init__(self, rate, clock, color):
        """constructor

        Args:
            rate (Float): Discount issuance cycle [Hz]
            clock (Rclpy.clock.clock): Clock instance
            color (std_msgs.msg.colorrgba): Designated color
        """
        super().__init__(rate, clock)
        self._color = color

    @property
    def color(self):
        """Return the color you want to shine

        Returns:
            std_msgs.msg.colorrgba: Color specified by constructor
        """
        return self._color


class DiagnosticMonitoringPattern(LedPatternBase):
    """Monitor the diag and notify the status of the robot"""

    def __init__(self, rate, clock, diagnostics, ok_color, error_color,
                 error_blinking_period=0.0):
        """constructor

        Args:
            rate (Float): Discount issuance cycle [Hz]
            clock (Rclpy.clock.clock): Clock instance
            diagnostics (Diagnostics.diagnostics): Diag information
            ok_color (Std_msgs.msg.colorrgba): OK, Color at Warn
            error_color (Std_msgs.msg.colorrgba): Error, Stale color
            error_blinking_period (Float): Blinking cycle at the time of error, stale, 0 or negative time does not blink.
        """
        super().__init__(rate, clock)
        self._diagnostics = diagnostics
        self._ok_color = ok_color
        self._error_color = error_color
        self._error_period = error_blinking_period

    @property
    def color(self):
        """Return the color you want to shine

        Returns:
            std_msgs.msg.colorrgba: Color according to the diag
        """
        level = self._diagnostics.last_level
        if level in [DiagnosticStatus.STALE, DiagnosticStatus.ERROR]:
            if self._error_period < sys.float_info.min:
                return self._error_color
            else:
                if int(self._clock.now().to_sec() / self._error_period) % 2:
                    return self._error_color
                else:
                    return ColorRGBA()
        elif level in [DiagnosticStatus.WARN, DiagnosticStatus.OK]:
            return self._ok_color
        else:
            return self._error_color
