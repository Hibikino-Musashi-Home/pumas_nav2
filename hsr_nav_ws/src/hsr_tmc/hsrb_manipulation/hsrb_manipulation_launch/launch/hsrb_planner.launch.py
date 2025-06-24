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
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_robot_description():
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
         PathJoinSubstitution([FindPackageShare(description_package), 'robots', description_file])])
    return {'robot_description': robot_description_content}


def load_robot_collision_config():
    description_package = LaunchConfiguration('description_package')
    robot_collision_file = LaunchConfiguration('robot_collision_config')
    robot_collision_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
         PathJoinSubstitution([FindPackageShare(description_package), 'robots', robot_collision_file])])
    return {'robot_collision_pair': robot_collision_content}


def declare_arguments():
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument('description_package', default_value='hsrb_description',
                                                    description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(DeclareLaunchArgument('description_file', default_value='hsrb4s.urdf.xacro',
                                                    description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(DeclareLaunchArgument('robot_collision_config', default_value='collision_pair_hsrb.xml',
                                                    description='Robot collision config xml file.'))

    declared_arguments.append(DeclareLaunchArgument('ik_plugin', default_value='hsrb_analytic_ik/HsrbIKSolver',
                                                    description='Ik plugins for the robot.'))

    declared_arguments.append(
        DeclareLaunchArgument('runtime_config_package',
                              default_value='hsrb_manipulation_launch',
                              description='Configuration package with rrt planner node.'))
    declared_arguments.append(
        DeclareLaunchArgument('configuration_file',
                              default_value='rrt_planner_config.yaml',
                              description='Configuration file with rrt planner node.'))

    return declared_arguments


def create_planner_node(context, *args, **kwargs):
    robot_description = load_robot_description()
    robot_collision_config = load_robot_collision_config()

    runtime_config_package = LaunchConfiguration('runtime_config_package')
    configuration_file = LaunchConfiguration('configuration_file')
    rrt_planner_config = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', configuration_file])
    ik_plugin = LaunchConfiguration('ik_plugin').perform(context)

    debug_param = {'publish_debug_info': False,
                   'save_request': True,
                   'step_mode': False}

    planner_node = Node(
        package='tmc_robot_rrt_planner_node',
        executable='robot_rrt_planner_node',
        parameters=[robot_description, robot_collision_config, rrt_planner_config, debug_param,
                    {'ik_plugins': [ik_plugin]}],
        output='screen'
    )
    return [planner_node]


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [OpaqueFunction(function=create_planner_node)])
