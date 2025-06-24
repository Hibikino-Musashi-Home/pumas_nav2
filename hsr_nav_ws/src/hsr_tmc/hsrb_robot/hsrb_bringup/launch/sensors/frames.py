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

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import yaml


_FILES = ['head_l_stereo_camera_frame_to_head_rgbd_sensor_rgb_frame.yaml',
          'head_l_stereo_camera_link_to_head_l_stereo_camera_frame.yaml',
          'head_rgbd_sensor_rgb_frame_to_head_rgbd_sensor_depth_frame.yaml']

_PREFIX = 'file://'


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('calibration_file_directory',
                              default_value='file:///etc/opt/tmc/robot/conf.d/calib_results',
                              description='Directory containing calibration files.'))
    return declared_arguments


def generate_frame_publisher_node(calib_result_yaml):
    with open(calib_result_yaml, 'r') as fp:
        calib_result = yaml.safe_load(fp)
    translation = [str(x) for x in calib_result['translation']]
    rotation = [str(x) for x in calib_result['rotation']]
    parent_frame_id = calib_result['parent_frame_id']
    child_frame_id = calib_result['child_frame_id']
    arguments = translation + rotation + [parent_frame_id, child_frame_id]
    return Node(package='tf2_ros',
                executable='static_transform_publisher',
                name=os.path.splitext(os.path.basename(calib_result_yaml))[0] + '_publisher',
                output='log',
                arguments=arguments)


def launch_setup(context, calibration_file_directory):
    calib_dir = context.perform_substitution(calibration_file_directory)
    if calib_dir.find(_PREFIX) >= 0:
        calib_dir = calib_dir[len(_PREFIX):]
    return [generate_frame_publisher_node(os.path.join(calib_dir, calib_result)) for calib_result in _FILES]


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [
        OpaqueFunction(function=launch_setup,
                       args=[LaunchConfiguration('calibration_file_directory')])])
