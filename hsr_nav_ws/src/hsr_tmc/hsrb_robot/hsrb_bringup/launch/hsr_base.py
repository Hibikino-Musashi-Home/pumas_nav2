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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    ThisLaunchFileDir,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('description_package', default_value='hsrb_description',
                                                    description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(DeclareLaunchArgument('description_file', default_value='hsrb4s.urdf.xacro',
                                                    description='URDF/XACRO description file with the robot.'))

    declared_arguments.append(DeclareLaunchArgument('use_head_center_camera', default_value='True',
                                                    description='If true, the head center camara is active.'))

    declared_arguments.append(
        DeclareLaunchArgument('timeopt_ros_launch_file',
                              default_value='launch/hsrb_timeopt_filter.launch.py',
                              description='timeopt_filter launch file'))
    declared_arguments.append(
        DeclareLaunchArgument('planner_launch_file',
                              default_value='launch/hsrb_planner.launch.py',
                              description='planner launch file'))
    return declared_arguments


def generate_launch_description():
    # If the node has parameter_file, it will be overwritten unless it is specified in Launch_arguments.
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/controllers.py']),
        launch_arguments={'description_package': LaunchConfiguration('description_package'),
                          'description_file': LaunchConfiguration('description_file')}.items())

    teleop_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/teleop.py']))

    sensor_launch_dir = PathJoinSubstitution([FindPackageShare('hsrb_bringup'), 'launch', 'sensors'])
    urg_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sensor_launch_dir, '/urg.py']),
        launch_arguments={'parameter_file': 'urg.yaml'}.items())
    head_center_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sensor_launch_dir, '/usb_camera.py']),
        launch_arguments={'parameter_file': 'head_center_camera.yaml',
                          'camera_name': 'head_center_camera'}.items(),
        condition=IfCondition(LaunchConfiguration('use_head_center_camera')))
    hand_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sensor_launch_dir, '/usb_camera.py']),
        launch_arguments={'parameter_file': 'hand_camera.yaml',
                          'camera_name': 'hand_camera'}.items())
    xtion_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sensor_launch_dir, '/xtion.py']),
        launch_arguments={'camera_name': 'head_rgbd_sensor'}.items())
    dynpick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sensor_launch_dir, '/dynpick.py']),
        launch_arguments={'parameter_file': 'dynpick.yaml'}.items())

    frame_publishser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sensor_launch_dir, '/frames.py']))

    base_bumper_node = Node(package='hsrb_bumper', executable='hsrb_bumper', name='base_bumper_node')

    hsrb_manipulation_launch_dir = get_package_share_directory(
        'hsrb_manipulation_launch')

    # timeopt_ros_node
    # Switching is required with B/C
    timeopt_ros_launch_file = PathJoinSubstitution([
        hsrb_manipulation_launch_dir,
        LaunchConfiguration('timeopt_ros_launch_file')])
    planner_launch_file = PathJoinSubstitution([
        hsrb_manipulation_launch_dir,
        LaunchConfiguration('planner_launch_file')])
    timeopt_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(timeopt_ros_launch_file))
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(planner_launch_file))

    # safe_pose_changer_node
    safe_pose_changer_launch = os.path.join(
        hsrb_manipulation_launch_dir,
        'launch/safe_pose_changer.launch.py')
    safe_pose_changer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(safe_pose_changer_launch),
        launch_arguments={'runtime_config_package': 'hsrb_manipulation_launch'}.items())

    nodes = [controllers, teleop_node, urg_node, head_center_camera_node, hand_camera_node,
             xtion_node, dynpick_node, frame_publishser_node, base_bumper_node,
             timeopt_ros_launch, planner_launch, safe_pose_changer]

    return LaunchDescription(declare_arguments() + nodes)
