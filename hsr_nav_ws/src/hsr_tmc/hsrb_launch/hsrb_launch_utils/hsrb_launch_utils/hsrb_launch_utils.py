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

from launch.actions import (
    DeclareLaunchArgument
)


def declare_launch_arguments():
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument('namespace', default_value='hsrb'))
    declared_arguments.append(DeclareLaunchArgument('debug', default_value="false"))
    declared_arguments.append(DeclareLaunchArgument('gui', default_value="true"))
    declared_arguments.append(DeclareLaunchArgument('rviz', default_value="true"))
    declared_arguments.append(DeclareLaunchArgument('gazebo_visualization', default_value="false"))
    declared_arguments.append(DeclareLaunchArgument('use_odom_ground_truth', default_value="false"))
    declared_arguments.append(DeclareLaunchArgument('use_manipulation', default_value="true"))
    declared_arguments.append(DeclareLaunchArgument('use_navigation', default_value="true"))
    declared_arguments.append(DeclareLaunchArgument('fast_physics', default_value="false"))
    declared_arguments.append(DeclareLaunchArgument('robot_name', default_value=os.getenv("ROBOT_NAME", "hsrb")))
    declared_arguments.append(DeclareLaunchArgument('set_initial_pose', default_value="True"))
    declared_arguments.append(DeclareLaunchArgument(
        'description_package', default_value='hsrb_description',
        description='Description package with robot URDF/oo files.'))
    declared_arguments.append(DeclareLaunchArgument(
        'description_file', default_value='hsrb4s.urdf.xacro',
        description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(DeclareLaunchArgument(
        'collision_file', default_value='collision_pair_hsrb.xml',
        description='collision_pair_hsrb file with the robot.'))
    return declared_arguments
