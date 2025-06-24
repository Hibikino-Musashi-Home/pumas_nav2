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
import threading

from hsrb_digital_io.node import main

import pytest

import rclpy
from std_msgs.msg import (
    Bool,
    ColorRGBA,
)


@pytest.fixture
def setup(mocker):
    connect_mock = mocker.patch('hsrb_digital_io.node.SerialConnection')
    connect_mock.return_value.magnetic_sensor_1.return_value = True
    connect_mock.return_value.magnetic_sensor_2.return_value = True
    connect_mock.return_value.emergency_button.return_value = True
    connect_mock.return_value.pressure_sensor.return_value = True
    connect_mock.return_value.front_bumper_sensor.return_value = True
    connect_mock.return_value.rear_bumper_sensor.return_value = True

    rclpy.init()
    test_node = rclpy.create_node('test_node')
    mocker.patch('rclpy.init')

    exit_flag = False

    def spin_func(node):
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(test_node)
        while rclpy.ok():
            executor.spin_once()
            if exit_flag:
                break

    spin_mock = mocker.patch('rclpy.spin')
    spin_mock.side_effect = spin_func

    main_thread = threading.Thread(target=main)
    main_thread.start()

    yield test_node, connect_mock

    exit_flag = True
    main_thread.join()

    test_node.destroy_node()
    # rclpy.shutdown is called in main


def _test_bool_publisher(node, topic_name):
    values = []

    def callback(msg):
        values.append(msg.data)

    node.create_subscription(Bool, topic_name, callback, 1)
    while len(values) == 0:
        pass

    assert values[0]


def test_publishers(setup):
    test_node = setup[0]
    _test_bool_publisher(test_node, 'base_magnetic_sensor_1')
    _test_bool_publisher(test_node, 'base_magnetic_sensor_2')
    _test_bool_publisher(test_node, 'runstop_button')
    _test_bool_publisher(test_node, 'pressure_sensor')
    _test_bool_publisher(test_node, 'base_f_bumper_sensor')
    _test_bool_publisher(test_node, 'base_b_bumper_sensor')


def test_led_subscriber(setup):
    test_node = setup[0]
    pub = test_node.create_publisher(ColorRGBA, 'command_status_led_rgb', 1)
    while pub.get_subscription_count() == 0:
        pass

    msg = ColorRGBA()
    msg.r = -0.1
    msg.g = 0.2
    msg.b = 1.3
    pub.publish(msg)

    connect_mock = setup[1]
    set_color_mock = connect_mock.return_value.set_color

    while set_color_mock.call_count == 0:
        pass
    assert pytest.approx(set_color_mock.mock_calls[0][1][0]) == -0.1
    assert pytest.approx(set_color_mock.mock_calls[0][1][1]) == 0.2
    assert pytest.approx(set_color_mock.mock_calls[0][1][2]) == 1.3


def _test_bool_subscriber(node, topic_name, set_value_mock):
    pub = node.create_publisher(Bool, topic_name, 1)
    while pub.get_subscription_count() == 0:
        pass

    msg = Bool()
    msg.data = True
    pub.publish(msg)

    while set_value_mock.call_count == 0:
        pass
    set_value_mock.assert_called_with(True)


def test_subscribers(setup):
    test_node, connect_mock = setup
    _test_bool_subscriber(test_node, 'command_head_led', connect_mock.return_value.set_head_led)
    _test_bool_subscriber(test_node, 'command_hand_led', connect_mock.return_value.set_hand_led)
    _test_bool_subscriber(test_node, 'command_drive_power', connect_mock.return_value.set_drive_power)
    _test_bool_subscriber(test_node, 'command_suction', connect_mock.return_value.set_suction)
