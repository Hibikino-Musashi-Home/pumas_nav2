#!/usr/bin/env python3
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
import os

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node


def generate_launch_description():
    # select robot version
    robot_version = os.environ.get("ROBOT_VERSION")
    robot_name = robot_version.replace('"', '').split('-')[0].lower()

    talk_hoya_node = Node(package='tmc_talk_hoya_py', executable='text_to_speech')

    if robot_name == 'hsrb':
        battery_state_node = Node(package='tmc_sanyo_battery', executable='sanyo_battery_node')
        digital_io_node = Node(package='hsrb_digital_io', executable='digital_io_node')
        imu_node = Node(package='hsrb_imu_sensor_protocol', executable='mpu9150_node',
                        remappings=[('base_imu', '/imu/data')])
        return LaunchDescription([battery_state_node, digital_io_node, imu_node, talk_hoya_node])
    elif robot_name == 'hsrc' or robot_name == 'hsrd':
        power_ecu_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/power_ecu.py']),
            launch_arguments={'parameter_file': 'power_ecu.yaml'}.items())
        return LaunchDescription([power_ecu_node, talk_hoya_node])
    else:
        raise Exception("Invalid ROBOT_VERSION:{0}".format(robot_version))
