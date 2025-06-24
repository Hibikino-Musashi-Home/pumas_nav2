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
u"""HSRB_Timeopt_filter binding test."""
import threading
import unittest

import launch
import launch_ros.actions
import launch_testing
from nose.tools import assert_almost_equal
from nose.tools import assert_false
from nose.tools import assert_less_equal
from nose.tools import assert_true
from nose.tools import eq_
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from tmc_manipulation_msgs.srv import FilterJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


_LIMITS = {'arm_lift_joint': {'velocity': 0.15, 'acceleration': 0.15},
           'arm_flex_joint': {'velocity': 1.0, 'acceleration': 1.0},
           'arm_roll_joint': {'velocity': 1.0, 'acceleration': 1.0},
           'wrist_flex_joint': {'velocity': 1.0, 'acceleration': 1.0},
           'wrist_roll_joint': {'velocity': 1.0, 'acceleration': 1.0},
           'hand_motor_joint': {'velocity': 3.0, 'acceleration': 20.0},
           'head_pan_joint': {'velocity': 1.0, 'acceleration': 1.0},
           'head_tilt_joint': {'velocity': 1.0, 'acceleration': 1.0},
           'dummy_joint': {'velocity': 0.2, 'acceleration': 0.1},
           'left_wheel': {'velocity': 8.5, 'acceleration': 5.0},
           'right_wheel': {'velocity': 8.5, 'acceleration': 5.0},
           'caster': {'velocity': 1.8, 'acceleration': 1.8}}


@pytest.mark.launch_test
def generate_test_description():
    timeopt_filter_node = launch_ros.actions.Node(
        package='hsrb_timeopt_ros',
        executable='timeopt_filter_node',
        parameters=[_LIMITS,
                    {'use_joint': ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
                                   'wrist_flex_joint', 'wrist_roll_joint', 'hand_motor_joint',
                                   'head_pan_joint', 'head_tilt_joint', 'dummy_joint',
                                   'left_wheel', 'right_wheel', 'caster']}],
        output='screen'
    )
    return launch.LaunchDescription(
        [timeopt_filter_node, launch_testing.actions.ReadyToTest()])


class HsrbTimeoptTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # In order to call the service, it is necessary to make a separate thread of SPIN
        self._node = rclpy.create_node("test_node")
        self._executor = MultiThreadedExecutor()
        self._thread = threading.Thread(target=rclpy.spin, args=(self._node, self._executor), daemon=True)
        self._thread.start()

        self._filter_srv = self._node.create_client(
            FilterJointTrajectory, 'timeopt_filter_node/filter_trajectory')
        self.assertTrue(self._filter_srv.wait_for_service(timeout_sec=10.0))

        self._joint_names = [
            'arm_lift_joint',
            'arm_flex_joint',
            'arm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint',
            'hand_motor_joint',
            'head_pan_joint',
            'head_tilt_joint',
            'odom_x',
            'odom_y',
            'odom_t']

    def test_normal_case(self):
        req = FilterJointTrajectory.Request()
        req.start_state.joint_state.name = self._joint_names + ['base_roll_joint']
        req.start_state.joint_state.position = [
            0.2,
            -0.5,
            0.0,
            0.0,
            0.0,
            1.0,

            0.0,
            0.0,

            0.0,
            0.0,
            0.0,

            0.0]
        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        point1 = JointTrajectoryPoint()
        point1.positions = [
            0.21,
            -0.5,
            0.1,
            -0.1,
            0.0,
            0.5,

            0.1,
            0.0,

            0.1,
            0.2,
            0.01]
        point2 = JointTrajectoryPoint()
        point2.positions = [
            0.22,
            -0.51,
            0.2,
            -0.1,
            0.0,
            0.0,

            0.1,
            0.0,

            0.15,
            0.1,
            0.02]
        traj.points = [point1, point2]
        req.trajectory = traj
        res = self._filter_srv.call(req)
        assert_true(res.is_success)
        eq_(self._joint_names, res.trajectory.joint_names)
        non_base_joints = [
            'arm_lift_joint',
            'arm_flex_joint',
            'arm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint',
            'hand_motor_joint',
            'head_pan_joint',
            'head_tilt_joint']

        traj_joints = res.trajectory.joint_names
        for point in res.trajectory.points:
            for joint in non_base_joints:
                vel = point.velocities[traj_joints.index(joint)]
                acc = point.accelerations[traj_joints.index(joint)]
                assert_less_equal(abs(vel), _LIMITS[joint]['velocity'])
                assert_less_equal(abs(acc), _LIMITS[joint]['acceleration'])

    def test_no_base_roll_joint(self):
        req = FilterJointTrajectory.Request()
        req.start_state.joint_state.name = self._joint_names
        req.start_state.joint_state.position = [
            0.2,
            -0.5,
            0.0,
            0.0,
            0.0,
            1.0,

            0.0,
            0.0,

            0.0,
            0.0,
            0.0]

        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        point1 = JointTrajectoryPoint()
        point1.positions = [
            0.21,
            -0.5,
            0.1,
            -0.1,
            0.0,
            0.5,

            0.1,
            0.0,

            0.1,
            0.2,
            0.01]
        point2 = JointTrajectoryPoint()
        point2.positions = [
            0.22,
            -0.51,
            0.2,
            -0.1,
            0.0,
            0.0,

            0.1,
            0.0,

            0.15,
            0.1,
            0.02]
        traj.points = [point1, point2]
        req.trajectory = traj
        res = self._filter_srv.call(req)
        assert_false(res.is_success)

    def test_too_small_movement(self):
        req = FilterJointTrajectory.Request()
        req.start_state.joint_state.name = \
            self._joint_names[:2] + ['base_roll_joint']
        req.start_state.joint_state.position = [0.2, -0.5, 0.0]
        traj = JointTrajectory()
        traj.joint_names = self._joint_names[:2]
        point1 = JointTrajectoryPoint()
        point1.positions = [0.2, -0.5]
        point2 = JointTrajectoryPoint()
        point2.positions = [0.2 + 1e-5, -0.5]
        traj.points = [point1, point2]
        req.trajectory = traj
        res = self._filter_srv.call(req)

        assert_true(res.is_success)
        eq_(len(res.trajectory.points), 1)
        assert_almost_equal(res.trajectory.points[0].positions[0], 0.2 + 1e-5)
        assert_almost_equal(
            Duration.from_msg(res.trajectory.points[0].time_from_start).nanoseconds / 1000000000.0, 0.1)

    def test_joint_in_use_joint_param(self):
        req = FilterJointTrajectory.Request()
        req.start_state.joint_state.name = ['dummy_joint', 'base_roll_joint']
        req.start_state.joint_state.position = [0.0, 0.0]
        traj = JointTrajectory()
        traj.joint_names = ['dummy_joint']
        point1 = JointTrajectoryPoint()
        point1.positions = [0.5]
        traj.points = [point1]
        req.trajectory = traj

        res = self._filter_srv.call(req)
        assert_true(res.is_success)
        eq_(['dummy_joint'], res.trajectory.joint_names)

        traj_joints = res.trajectory.joint_names
        for point in res.trajectory.points:
            vel = point.velocities[traj_joints.index('dummy_joint')]
            acc = point.accelerations[traj_joints.index('dummy_joint')]
            assert_less_equal(abs(vel), _LIMITS['dummy_joint']['velocity'])
            assert_less_equal(abs(acc), _LIMITS['dummy_joint']['acceleration'])
