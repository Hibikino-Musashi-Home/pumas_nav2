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
import os

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('runtime_config_package',
                              default_value='hsrb_bringup',
                              description='Package with the dynpick sensor\'s configuration in "config" folder.'))
    declared_arguments.append(
        DeclareLaunchArgument('parameter_file',
                              default_value='dynpick.yaml',
                              description='YAML file with the dynpick sensor configuration.'))
    return declared_arguments


def generate_launch_description():
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    parameter_file = LaunchConfiguration('parameter_file')
    dynpick_driver_parameter = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', parameter_file])

    dynpick_node = Node(package='tmc_dynpick_driver',
                        executable='dynpick_driver_node',
                        name='dynpick_driver_node',
                        parameters=[dynpick_driver_parameter,
                                    {'device': os.environ.get('DYNPICK_DEVICE', '/dev/dynpick')}],
                        remappings=[('force', 'wrist_wrench/raw')])

    return LaunchDescription(declare_arguments() + [dynpick_node])
