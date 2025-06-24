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
import rclpy

from .interfaces import (
    BoolPublisher,
    BoolSubscriber,
    ColorSubscriber,
)
from .serial_connection import SerialConnection


def main():
    connect = SerialConnection('/dev/ttyUSB_IO', 57600)
    connect.run(0.02)

    rclpy.init()
    node = rclpy.create_node('usb_io_node')

    BoolPublisher(node, 'base_magnetic_sensor_1', connect.magnetic_sensor_1, 0.1)
    BoolPublisher(node, 'base_magnetic_sensor_2', connect.magnetic_sensor_2, 0.1)
    BoolPublisher(node, 'runstop_button', connect.emergency_button, 0.1)
    BoolPublisher(node, 'pressure_sensor', connect.pressure_sensor, 0.1)
    BoolPublisher(node, 'base_f_bumper_sensor', connect.front_bumper_sensor, 0.1)
    BoolPublisher(node, 'base_b_bumper_sensor', connect.rear_bumper_sensor, 0.1)
    BoolSubscriber(node, 'command_head_led', connect.set_head_led)
    BoolSubscriber(node, 'command_hand_led', connect.set_hand_led)
    BoolSubscriber(node, 'command_drive_power', connect.set_drive_power)
    BoolSubscriber(node, 'command_suction', connect.set_suction)
    ColorSubscriber(node, connect.set_color)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
