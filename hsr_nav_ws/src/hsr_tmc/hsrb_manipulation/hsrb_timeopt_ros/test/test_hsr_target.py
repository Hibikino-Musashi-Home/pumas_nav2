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
u"""HSR model single test."""

import unittest

from hsrb_timeopt_ros.hsr_target import HsrKinematicsTarget
from nose.tools import eq_


class HsrTestCase(unittest.TestCase):
    def test_init(self):
        u"""Default."""
        hsr = HsrKinematicsTarget()
        EXPECT_LIST = [
            'arm_lift_joint',
            'arm_flex_joint',
            'arm_roll_joint',
            'wrist_roll_joint',
            'wrist_flex_joint',
            'head_pan_joint',
            'head_tilt_joint',
            'x02',
            'y02',
            'a02',
            'x01',
            'y01',
            'a01',
            'a12',
            'x11',
            'caster',
            'left_wheel',
            'right_wheel',
            'odom_x',
            'odom_y',
            'odom_t'
        ]
        eq_(set(EXPECT_LIST), set(hsr.names))

        # When the joint name is different
        hsr_mod = HsrKinematicsTarget(joint_names=[
            'joint1', 'joint2', 'joint3'])
        EXPECT_LIST2 = [
            'joint1',
            'joint2',
            'joint3',
            'x02',
            'y02',
            'a02',
            'x01',
            'y01',
            'a01',
            'a12',
            'x11',
            'caster',
            'left_wheel',
            'right_wheel',
            'odom_x',
            'odom_y',
            'odom_t']
        eq_(set(EXPECT_LIST2), set(hsr_mod.names))
