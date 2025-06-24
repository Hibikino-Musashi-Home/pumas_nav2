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
u"""Joint model single test."""

import math
import unittest

from hsrb_timeopt_ros.active_caster_kinematics import ActiveCasterKinematicsTarget
from nose.tools import assert_almost_equal
from nose.tools import eq_
from tmc_timeopt.trajectory import NaturalCubicSplineTrajectory
from tmc_timeopt.trajectory import TrajectoryDict


def assert_almost_equal_tuple(val1, val2):
    u"""Assert on Tuple."""
    for index in range(len(val1)):
        assert_almost_equal(val1[index], val2[index])


class ActiveCasterTestCase(unittest.TestCase):
    def setUp(self):
        self.ac = ActiveCasterKinematicsTarget()

    def test_set_param_default(self):
        u"""Initialize with the default value."""
        ac = ActiveCasterKinematicsTarget()
        eq_(0.11, ac.xd12)
        eq_(0.266, ac.d)
        eq_(0.04, ac.r)

    def test_set_param_custom(self):
        u"""Initialize with the specified value."""
        custom_param = {'caster_offset': 1,
                        'tread_length': 2,
                        'wheel_radius': 3}
        ac = ActiveCasterKinematicsTarget(custom_param)
        eq_(1, ac.xd12)
        eq_(2, ac.d)
        eq_(3, ac.r)

    def test_update_kinematics(self):
        u"""ACTIVECASTER geometric calculation test.

        Left_wheel from the state of x02, y02, a02, a12
        Right_wheel, get the condition of the Caster correct
        """
        # still
        point1 = {}
        point1['x02'] = (0, 0, 0)
        point1['y02'] = (0, 0, 0)
        point1['a02'] = (0, 0, 0)
        point1['a12'] = (0, 0, 0)

        self.ac.update_kinematics(point1)
        assert_almost_equal_tuple((0, 0, 0), self.ac.point['left_wheel'])
        assert_almost_equal_tuple((0, 0, 0), self.ac.point['right_wheel'])
        assert_almost_equal_tuple((0, 0, 0), self.ac.point['caster'])

        # Go straight Caster = 0
        point2 = {}
        point2['x02'] = (0, 1, 0.1)
        point2['y02'] = (0, 0, 0)
        point2['a02'] = (0, 0, 0)
        point2['a12'] = (0, 0, 0)

        self.ac.update_kinematics(point2)
        assert_almost_equal_tuple((0, 25.0, 2.5), self.ac.point['left_wheel'])
        assert_almost_equal_tuple((0, 25.0, 2.5), self.ac.point['right_wheel'])
        assert_almost_equal_tuple((0, 0, 0), self.ac.point['caster'])

        # Go straight Caster = pi/2
        point2 = {}
        point2['x02'] = (0, 1, 0.1)
        point2['y02'] = (0, 0, 0)
        point2['a02'] = (0, 0, 0)
        point2['a12'] = (math.pi / 2.0, 0, 0)

        self.ac.update_kinematics(point2)
        assert_almost_equal_tuple((0, -30.2272727272, 224.25),
                                  self.ac.point['left_wheel'])
        assert_almost_equal_tuple((0, 30.2272727272, 230.29545454545456),
                                  self.ac.point['right_wheel'])
        assert_almost_equal_tuple((math.pi / 2.0, -9.09090909, -0.9090909090),
                                  self.ac.point['caster'])

    def test_generate_base_trajectory(self):
        u"""ACTIVECASTER Casta Trade Test."""
        points = 11
        traj = TrajectoryDict(points)
        traj.append('odom_x', NaturalCubicSplineTrajectory)
        traj.append('odom_y', NaturalCubicSplineTrajectory)
        traj.append('odom_t', NaturalCubicSplineTrajectory)

        for i in range(points):
            traj['odom_x'][i] = (0.1 * i, 0.0, 0.0)
            traj['odom_y'][i] = (0.0, 0.0, 0.0)
            traj['odom_t'][i] = (0.0, 0.0, 0)

        self.ac.generate_base_trajectory(traj, math.pi / 2.0, 0.1)
        eq_(101, len(traj['x01']))
        eq_(101, len(traj['y01']))
        eq_(101, len(traj['a01']))
        eq_(101, len(traj['x11']))
        eq_(101, len(traj['a12']))
        eq_(101, len(traj['left_wheel']))
        eq_(101, len(traj['right_wheel']))
        eq_(101, len(traj['caster']))

        # Casta converges to 0 when Active_caster goes straight
        assert_almost_equal(math.pi / 2.0, traj['caster'][0][0])
        assert_almost_equal(0.0, traj['caster'][10.0][0], places=3)

        traj = TrajectoryDict(points)
        traj.append('odom_x', NaturalCubicSplineTrajectory)
        traj.append('odom_y', NaturalCubicSplineTrajectory)
        traj.append('odom_t', NaturalCubicSplineTrajectory)

        for i in range(points):
            traj['odom_x'][i] = (0.0, 0.0, 0.0)
            traj['odom_y'][i] = (0.1 * i, 0.0, 0.0)
            traj['odom_t'][i] = (0.0, 0.0, 0)

        self.ac.generate_base_trajectory(traj, 0.0, 0.1)

        # When Active_caster goes to the right, Casta converges to -pi/2
        assert_almost_equal(0.0, traj['caster'][0][0])
        assert_almost_equal(-math.pi / 2.0, traj['caster'][10.0][0], places=3)

    def test_calc_dynamics(self):
        u"""Dynamics update.

        Easy because it only gets speed and acceleration
        """
        self.ac.point = {}
        self.ac.point['left_wheel'] = (0, 1.0, 2.0)
        self.ac.point['right_wheel'] = (0, 3.0, 4.0)
        self.ac.point['caster'] = (0, 5.0, 6.0)
        (a, b, c, d) = self.ac.get_dynamics()

        eq_(1.0, a['left_wheel', 'acceleration'])
        eq_(2.0, b['left_wheel', 'acceleration'])
        eq_(0.0, c['left_wheel', 'acceleration'])
        eq_(0.0, d['left_wheel', 'acceleration'])
        eq_(3.0, a['right_wheel', 'acceleration'])
        eq_(4.0, b['right_wheel', 'acceleration'])
        eq_(0.0, c['right_wheel', 'acceleration'])
        eq_(0.0, d['right_wheel', 'acceleration'])
        eq_(5.0, a['caster', 'acceleration'])
        eq_(6.0, b['caster', 'acceleration'])
        eq_(0.0, c['caster', 'acceleration'])
        eq_(0.0, d['caster', 'acceleration'])
