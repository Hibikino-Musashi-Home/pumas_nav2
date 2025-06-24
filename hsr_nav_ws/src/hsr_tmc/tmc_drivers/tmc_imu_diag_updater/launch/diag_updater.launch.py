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
from launch import (
    LaunchDescription
)
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_name = DeclareLaunchArgument("input_topic_name", default_value="imu/data_raw")
    sampling_hz = DeclareLaunchArgument("sampling_hz", default_value="200.0")
    com_timeout_sec = DeclareLaunchArgument("com_timeout_sec", default_value="50.0")
    expected_frame_id = DeclareLaunchArgument("expected_frame_id", default_value="imu")
    imu_significant_threshold = DeclareLaunchArgument("imu_significant_threshold", default_value="0.00001")
    warn_hz = DeclareLaunchArgument("warn_hz", default_value="80.0")
    error_hz = DeclareLaunchArgument("error_hz", default_value="50.0")
    window_size_for_computing_rate = DeclareLaunchArgument("window_size_for_computing_rate", default_value="20")
    properties_num = DeclareLaunchArgument("properties_num", default_value="6")
    contiguous_threshold = DeclareLaunchArgument("contiguous_threshold", default_value="1")
    sample_size = DeclareLaunchArgument("sample_size", default_value="100")
    acc_norm_min = DeclareLaunchArgument("acc_norm_min", default_value="9.0")
    acc_norm_max = DeclareLaunchArgument("acc_norm_max", default_value="10.5")

    tmc_imu_diag_updater_node = Node(
        package='tmc_imu_diag_updater',
        executable='tmc_imu_diag_updater_node',
        name='imu_diag_updater',
        parameters=[{
            'hardware_id': LaunchConfiguration('input_topic_name'),
            'sampling_hz': LaunchConfiguration('sampling_hz'),
            'verifiers_list': [
                'disconnection',
                'unexpected_rate',
                'unexpected_frame_id',
                'zero_velocity_and_acceleration',
                'contiguous_same_value',
                'initial_acceleration_norm'],
            'verifiers': {
                'disconnection': {
                    'timeout_sec': LaunchConfiguration('com_timeout_sec')
                },
                'unexpected_rate': {
                    'warn_hz': LaunchConfiguration('warn_hz'),
                    'error_hz': LaunchConfiguration('error_hz')
                },
                'unexpected_frame_id': {
                    'expected_frame_id': LaunchConfiguration('expected_frame_id')
                },
                'zero_velocity_and_acceleration': {
                    'significant_threshold': LaunchConfiguration('imu_significant_threshold')
                },
                'contiguous_same_value': {
                    'properties_num': LaunchConfiguration('properties_num'),
                    'contiguous_threshold': LaunchConfiguration('contiguous_threshold')
                },
                'initial_acceleration_norm': {
                    'sample_size': LaunchConfiguration('sample_size'),
                    'acc_norm_min': LaunchConfiguration('acc_norm_min'),
                    'acc_norm_max': LaunchConfiguration('acc_norm_max')
                },
            }
        }],
        output='screen')

    ld = LaunchDescription([
        input_topic_name,
        sampling_hz,
        com_timeout_sec,
        expected_frame_id,
        imu_significant_threshold,
        warn_hz,
        error_hz,
        window_size_for_computing_rate,
        properties_num,
        contiguous_threshold,
        sample_size,
        acc_norm_min,
        acc_norm_max,
        tmc_imu_diag_updater_node,
    ])
    return ld
