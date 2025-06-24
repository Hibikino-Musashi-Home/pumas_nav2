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

from std_msgs.msg import (
    Bool,
    ColorRGBA,
)


class BoolPublisher:

    def __init__(self, node, topic_name, get_value_func, publish_inverval):
        self._pub = node.create_publisher(Bool, topic_name, 1)
        self._value = get_value_func
        self._timer = node.create_timer(publish_inverval, self._callback)

    def _callback(self):
        msg = Bool()
        msg.data = self._value()
        self._pub.publish(msg)


class BoolSubscriber:

    def __init__(self, node, topic_name, set_value_func):
        self._sub = node.create_subscription(Bool, topic_name, self._callback, 1)
        self._set_value_func = set_value_func

    def _callback(self, msg):
        self._set_value_func(msg.data)


class ColorSubscriber:

    def __init__(self, node, set_color_func):
        self._sub = node.create_subscription(ColorRGBA, 'command_status_led_rgb', self._callback, 1)
        self._set_color_func = set_color_func

    def _callback(self, msg):
        self._set_color_func(msg.r, msg.g, msg.b)
