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
from hsrb_timeopt_ros.hsr_target import HsrKinematicsTarget
import rclpy
from tmc_timeopt_ros.timeopt_filter import TimeoptFilterNode


class HsrbTimeoptFilter(TimeoptFilterNode):
    u"""Node that converts the spatial command trajectory of HSR (bogie+arm) into time orbit."""

    _DEFAULT_JOINTS = ['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_roll_joint',
                       'wrist_flex_joint',
                       'hand_motor_joint',
                       'head_pan_joint',
                       'head_tilt_joint']

    _BASE_JOINTS = ['caster',
                    'left_wheel',
                    'right_wheel']

    def __init__(self):
        u"""Initialization"""
        # The default value is HSR-B parameter
        use_joint = self._DEFAULT_JOINTS + self._BASE_JOINTS
        super(HsrbTimeoptFilter, self).__init__(srv_name='~/filter_trajectory', default_joint=use_joint)

        omni_base_param = {'caster_offset': self._get_param('caster_offset', 0.11),
                           'tread_length': self._get_param('tread', 0.266),
                           'wheel_radius': self._get_param('wheel_radius', 0.040)}
        joint_names = list(set(self._use_joint) - set(self._BASE_JOINTS))
        self._target = HsrKinematicsTarget(param=omni_base_param, joint_names=joint_names)

    def _timeopt_trajectory_from_ros(self, start_state, trajectory):
        u"""Convert ROS orbit to Timeopt orbit"""
        traj_dict = super(HsrbTimeoptFilter, self)._timeopt_trajectory_from_ros(start_state, trajectory)
        if traj_dict is None:
            return None
        # Use the caster axis in the Start_state
        try:
            yaw_index = start_state.name.index("base_roll_joint")
        except ValueError:
            raise ValueError("base_roll_joint not found in start_state")
        # The casta shaft is a base standard, so it is negative
        self._caster_position = -start_state.position[yaw_index]
        traj_dict['caster'][0] = (self._caster_position, 0, 0)
        return traj_dict

    def _set_trajectory(self, timeopt_trajectory):
        u"""Set the orbit to the time optimization instance"""
        self._target._ac.generate_base_trajectory(
            timeopt_trajectory,
            self._caster_position,
            self._timeopt_resolution)
        super(HsrbTimeoptFilter, self)._set_trajectory(timeopt_trajectory)


def main():
    rclpy.init()
    node = HsrbTimeoptFilter()

    rclpy.spin(node)

    rclpy.try_shutdown()
    node.destroy_node()
