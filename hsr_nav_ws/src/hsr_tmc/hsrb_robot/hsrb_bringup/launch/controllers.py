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
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml


_JOINT_POSITION_OFFSET_FILE = '/etc/opt/tmc/robot/conf.d/calib_results/joint_position_offset.yaml'


def load_robot_description():
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
         PathJoinSubstitution([FindPackageShare(description_package), 'robots', description_file])])
    return {'robot_description': robot_description_content}


def load_joint_offset():
    with open(_JOINT_POSITION_OFFSET_FILE, 'r') as fp:
        offset_input = yaml.safe_load(fp)
    offset_output = {}
    for joint_name, value in offset_input.items():
        offset_output[joint_name] = value['position_offset']
    return {'position_offset': offset_output}


def create_spanwer_node(controller_name, manager_name='/controller_manager'):
    return Node(package='controller_manager',
                executable='spawner',
                arguments=[controller_name, '--controller-manager', manager_name])


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('runtime_config_package',
                              default_value='hsrb_bringup',
                              description='Package with the controller\'s configuration in "config" folder.'))
    declared_arguments.append(
        DeclareLaunchArgument('controllers_file',
                              default_value='controllers.yaml',
                              description='YAML file with the controllers configuration.'))

    declared_arguments.append(
        DeclareLaunchArgument('description_package',
                              default_value='hsrb_description',
                              description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(
        DeclareLaunchArgument('description_file',
                              default_value='hsrb4s.urdf.xacro',
                              description='URDF/XACRO description file with the robot.'))
    return declared_arguments


def generate_launch_description():
    robot_description = load_robot_description()

    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    robot_controllers = PathJoinSubstitution([FindPackageShare(runtime_config_package), 'config', controllers_file])
    control_node = Node(package='controller_manager',
                        executable='ros2_control_node',
                        parameters=[robot_description, robot_controllers, load_joint_offset()],
                        remappings=[('odom', '~/wheel_odom')])

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 parameters=[{'source_list': ['/joint_states']}],
                                 namespace='whole_body',
                                 remappings=[('robot_description', '/robot_description')])
    robot_state_pub_node = Node(package='robot_state_publisher',
                                executable='robot_state_publisher',
                                parameters=[robot_description],
                                namespace='whole_body',
                                output={'both': 'log'},
                                remappings=[('robot_description', '/robot_description')])
    wheel_odom_connector_tf = Node(package='tf2_ros',
                                   executable='static_transform_publisher',
                                   name='static_transform_publisher',
                                   output='log',
                                   arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                                              'base_footprint_wheel', 'base_footprint'])

    nodes = [control_node,
             joint_state_publisher,
             robot_state_pub_node,
             wheel_odom_connector_tf,
             create_spanwer_node('joint_state_broadcaster'),
             create_spanwer_node('head_trajectory_controller'),
             create_spanwer_node('arm_trajectory_controller'),
             create_spanwer_node('gripper_controller'),
             create_spanwer_node('omni_base_controller'),
             create_spanwer_node('servo_diagnostic_broadcaster')]

    return LaunchDescription(declare_arguments() + nodes)
