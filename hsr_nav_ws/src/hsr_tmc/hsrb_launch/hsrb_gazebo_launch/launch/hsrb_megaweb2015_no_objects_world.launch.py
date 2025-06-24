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
from hsrb_launch_utils.hsrb_launch_utils import declare_launch_arguments
from launch import (
    LaunchDescription
)
from launch.actions import (
    IncludeLaunchDescription
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration
)


def generate_launch_description():
    declared_arguments = declare_launch_arguments()
    condition = LaunchConfiguration('fast_physics')

    hsrb_gazebo_common_path = os.path.join(
        get_package_share_directory('hsrb_gazebo_launch'),
        'launch/include/hsrb_gazebo_common.launch.py')

    tmc_gazebo_worlds_dir = get_package_share_directory('tmc_gazebo_worlds')

    # Launch_arguments is fixed here.
    launch_arg_info = {
        "map": os.path.join(
            get_package_share_directory('tmc_potential_maps'),
            'maps/mega-web/map.yaml'),
        "initial_orientation_xyzw": "0.0,0.0,0.0,1.0",
        "robot_pos": "-1.0,-7.5,0.32,1.57",
        "ground_truth_xyz": "1.0\\ 7.5\\ -0.304866",
        "ground_truth_rpy": "0.0\\ 0.0\\ -1.57",
    }

    hsrb_gazebo_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_gazebo_common_path),
        launch_arguments={**launch_arg_info,
                          **{"world_name": os.path.join(tmc_gazebo_worlds_dir,
                                                        'worlds/megaweb2015_no_objects.world')}
                          }.items(),
        condition=UnlessCondition(condition))

    hsrb_gazebo_common_fast = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_gazebo_common_path),
        launch_arguments={**launch_arg_info,
                          **{"world_name": os.path.join(tmc_gazebo_worlds_dir,
                                                        'worlds/megaweb2015_no_objects_fast.world')}
                          }.items(),
        condition=IfCondition(condition))

    return LaunchDescription(declared_arguments + [
        hsrb_gazebo_common,
        hsrb_gazebo_common_fast])
