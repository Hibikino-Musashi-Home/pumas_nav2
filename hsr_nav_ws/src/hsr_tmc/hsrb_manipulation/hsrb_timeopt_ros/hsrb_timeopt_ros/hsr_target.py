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

from hsrb_timeopt_ros.active_caster_kinematics import ActiveCasterKinematicsTarget
from tmc_timeopt.target import Target
from tmc_timeopt_ros.simple_joint_target import SimpleJointTarget


class HsrKinematicsTarget(Target):
    u"""Target consisting of an Active Caster and joints."""

    _DEFAULT_JOINTS = ['arm_lift_joint',
                       'arm_flex_joint',
                       'arm_roll_joint',
                       'wrist_roll_joint',
                       'wrist_flex_joint',
                       'head_pan_joint',
                       'head_tilt_joint']

    def __init__(self, param={},
                 joint_names=_DEFAULT_JOINTS,
                 use_base=True):
        u"""Pass parameter and initialize."""
        self._ac = ActiveCasterKinematicsTarget(param)
        self._joints = SimpleJointTarget(joint_names)
        self._use_base = use_base
        self.names = self._joints.names + self._ac.names

    def update_kinematics(self, point):
        u"""Update TARGET athletic.

        Args:
            Point: DICT of state amount
        """
        self._ac.update_kinematics(point)
        self._joints.update_kinematics(point)

    def update_point(self, point):
        u"""Update Target kinematics using the calculated point.

        Args:
            Point: DICT of state amount
        """
        self._ac.point = point
        self._joints.point = point

    def update_dynamics(self):
        u"""I don't do anything because it is an acceleration level restraint."""
        pass

    def get_dynamics(self):
        u"""Return the dynamic spalameter (A, B, C, D).

        Returns:
            (a, b, c, d): ('variables' name', 'restriction type') is key and the value is value.
        """
        if self._use_base:
            (a, b, c, d) = self._ac.get_dynamics()
        else:
            (a, b, c, d) = ({}, {}, {}, {})
        (ja, jb, jc, jd) = self._joints.get_dynamics()
        a.update(ja)
        b.update(jb)
        c.update(jc)
        d.update(jd)
        return a, b, c, d
