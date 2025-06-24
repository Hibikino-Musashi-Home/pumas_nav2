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

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def get_robot_version():
    robot_version = os.environ.get("ROBOT_VERSION")
    if robot_version is None:
        print("Environment variable not found : ROBOT_VERSION")
        exit()
    # ex. ROBOT_VERSION="HSRE-PROTO2-1"
    robot_version_list = robot_version.lower().replace('"', '').split('-')
    return robot_version_list


def load_robot_description(description_package_arg_name='description_package',
                           description_file_arg_name='description_file', sim_condition='False', xacro_arg=''):
    description_package = LaunchConfiguration(description_package_arg_name)
    description_file = LaunchConfiguration(description_file_arg_name)
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
         PathJoinSubstitution([FindPackageShare(description_package), 'robots', description_file]), ' ',
         'rviz_sim:=', sim_condition, ' ', xacro_arg])
    return {'robot_description': robot_description_content}


def load_collision_description(description_package_arg_name='description_package',
                               collision_file_arg_name='collision_file', xacro_arg=''):
    description_package = LaunchConfiguration(description_package_arg_name)
    collision_file = LaunchConfiguration(collision_file_arg_name)

    collision_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
         PathJoinSubstitution([FindPackageShare(description_package), 'robots', collision_file]), ' ',
         xacro_arg])
    return {'robot_collision_pair': collision_description_content}
