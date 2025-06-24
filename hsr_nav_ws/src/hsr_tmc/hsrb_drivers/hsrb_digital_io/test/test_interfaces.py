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
from unittest.mock import MagicMock

from hsrb_digital_io.interfaces import (
    BoolPublisher,
    BoolSubscriber,
    ColorSubscriber,
)

import pytest

import rclpy
from std_msgs.msg import (
    Bool,
    ColorRGBA,
)


@pytest.fixture
def node():
    rclpy.init()
    node = rclpy.create_node('test_node')

    yield node

    node.destroy_node()
    rclpy.shutdown()


def test_bool_publisher(node):
    get_value_mock = MagicMock()
    get_value_mock.side_effect = [True, False] * 2

    values = []

    def callback(msg):
        values.append(msg.data)
    node.create_subscription(Bool, 'test_topic', callback, 1)

    start = node.get_clock().now()
    BoolPublisher(node, 'test_topic', get_value_mock, 0.1)

    while len(values) < 3:
        rclpy.spin_once(node)
    end = node.get_clock().now()

    # First topic may be dropped
    assert 300000000 < (end - start).nanoseconds < 410000000
    assert values[0]
    assert not values[1]
    assert values[2]


def test_bool_subscriber(node):
    set_value_mock = MagicMock()
    BoolSubscriber(node, 'test_topic', set_value_mock)

    pub = node.create_publisher(Bool, 'test_topic', 1)
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node)

    msg = Bool()
    msg.data = True
    pub.publish(msg)

    while set_value_mock.call_count == 0:
        rclpy.spin_once(node)
    set_value_mock.assert_called_with(True)


def test_color_subscriber(node):
    set_color_mock = MagicMock()
    ColorSubscriber(node, set_color_mock)

    pub = node.create_publisher(ColorRGBA, 'command_status_led_rgb', 1)
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node)

    msg = ColorRGBA()
    msg.r = -0.1
    msg.g = 0.2
    msg.b = 1.3
    pub.publish(msg)

    while set_color_mock.call_count == 0:
        rclpy.spin_once(node)
    # The following assert is inappropriate due to precision issues
    # set_color_mock.assert_called_with(-0.1, 0.2, 1.3)
    assert pytest.approx(set_color_mock.mock_calls[0][1][0]) == -0.1
    assert pytest.approx(set_color_mock.mock_calls[0][1][1]) == 0.2
    assert pytest.approx(set_color_mock.mock_calls[0][1][2]) == 1.3
