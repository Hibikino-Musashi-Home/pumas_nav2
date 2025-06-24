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
import copy
import threading

from diagnostic_msgs.msg import (
    DiagnosticArray,
    DiagnosticStatus,
)
from rclpy.duration import Duration


class Diagnostics:
    """Subscribe and store diagram information"""

    def __init__(self, node, timeout, ignored_diags=[]):
        """constructor

        Args:
            node (rclpy.node.Node): ros2 node
            timeout (Float): Time to judge that the diagram information has not subscribed [SEC]
            ignored_diags (List of String): Diag name ignored from the level judgment of the diagram
        """
        self._lock = threading.Lock()
        self._diag_agg_sub = node.create_subscription(DiagnosticArray, '/diagnostics_agg', self._diag_agg_callback, 1)
        self._last_level = DiagnosticStatus.ERROR
        self._clock = node.get_clock()
        self._update_time = self._clock.now()
        self._timeout = Duration(seconds=timeout)
        self._ignored_diags = []
        for ignored_diag in ignored_diags:
            self._ignored_diags.append(ignored_diag)
            components = ignored_diag.split('/')
            for i in range(1, len(components) - 1):
                self._ignored_diags.append('/'.join(components[:-i]))

    def _diag_agg_callback(self, data):
        """Diag information callback

        Args:
            data (Diagnostic_msg.msg.diagnosticarray): Diag information
        """
        levels = [status.level for status in data.status if status.name not in self._ignored_diags]
        if len(levels) == 0:
            max_level = DiagnosticStatus.STALE
        else:
            max_level = max(levels)
        with self._lock:
            self._last_level = max_level
            self._update_time = self._clock.now()

    @property
    def last_level(self):
        """Return the level of the latest diagram information

        Returns:
            int: Return the latest diag information level
                 If you have not been subscribed, return DiagnostixStatus.stalee
        """
        with self._lock:
            if self._clock.now() > self._update_time + self._timeout:
                return DiagnosticStatus.STALE
            else:
                return copy.deepcopy(self._last_level)
