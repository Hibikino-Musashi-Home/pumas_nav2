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
from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from tmc_launch_ros_utils.tmc_launch_ros_utils import load_robot_description


def declare_arguments():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument('description_package',
                              default_value='hsrb_description',
                              description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(
        DeclareLaunchArgument('description_file',
                              default_value='hsrb4s.urdf.xacro',
                              description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument('teleop_runtime_config_package',
                              default_value='hsrb_bringup',
                              description='Package with the controller\'s configuration in "config" folder.'))

    return declared_arguments


def generate_launch_description():
    robot_description = load_robot_description()

    joy_node = Node(package='joy_linux', executable='joy_linux_node', output='screen')

    joystick_teleop_launch = PathJoinSubstitution(
        [FindPackageShare('hsrb_joystick_teleop'), 'launch', 'hsrb_joystick_control.launch.py'])
    joystick_teleop_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_teleop_launch))

    runtime_config_package = LaunchConfiguration('teleop_runtime_config_package')
    pseudo_ee_controller_config = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', 'pseudo_endeffector_controller_config.yaml'])
    pseudo_ee_controller_node = Node(package='hsrb_pseudo_endeffector_controller',
                                     executable='hsrb_pseudo_endeffector_controller',
                                     name='pseudo_endeffector_controller',
                                     parameters=[robot_description, pseudo_ee_controller_config],
                                     remappings=[('odom', 'omni_base_controller/wheel_odom')])

    pseudo_velocity_controller_config = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', 'pseudo_velocity_controller_config.yaml'])
    pseudo_velocity_controller_node = Node(package='tmc_pseudo_velocity_controller',
                                           executable='pseudo_velocity_controller',
                                           name='pseudo_velocity_controller',
                                           parameters=[robot_description, pseudo_velocity_controller_config])

    return LaunchDescription(declare_arguments()
                             + [joy_node,
                                joystick_teleop_node,
                                pseudo_ee_controller_node,
                                pseudo_velocity_controller_node])
