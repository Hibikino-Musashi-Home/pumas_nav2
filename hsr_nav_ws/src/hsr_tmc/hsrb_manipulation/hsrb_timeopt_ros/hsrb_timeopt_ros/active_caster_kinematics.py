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
u"""Active Caster model"""

from math import cos
from math import sin

from tmc_timeopt.target import Target
from tmc_timeopt.trajectory import LinearTrajectory
from tmc_timeopt.trajectory import Trajectory


class ActiveCasterKinematicsTarget(Target):
    u"""ActiveCasterkinematixStarget class.

    Output variable name:
      - caster
      - left_wheel
      - right_wheel
    """

    _DEFAULT_PARAM = {'caster_offset': 0.11,
                      'tread_length': 0.266,
                      'wheel_radius': 0.04}

    def __init__(self, param={}):
        u"""Set the parameter and initialize Active Caster.

        Args:
           Param: DICT with the following three elements
                  Caster_offset: Offset between the center axis and wheel axis [m]
                  Tread_length: Tred [M]
                  Wheel_raduis: Wheel diameter [m]
        Note:
           Regarding variable names
           [x, y, a]: Each represents the rotation of X coordinates, Y coordinates, Z axes
           [0,1,2]: Each represents world coordinates, bogie coordinates, top plate coordinates
        """
        self.ac_names = ['x02', 'y02', 'a02', 'x01', 'y01', 'a01',
                         'a12', 'x11', 'caster', 'left_wheel', 'right_wheel']
        self.base_names = ['odom_x', 'odom_y', 'odom_t']
        self.names = self.base_names + self.ac_names

        if 'caster_offset' in param:
            self.xd12 = param['caster_offset']
        else:
            self.xd12 = self._DEFAULT_PARAM['caster_offset']

        if 'tread_length' in param:
            self.d = param['tread_length']
        else:
            self.d = self._DEFAULT_PARAM['tread_length']

        if 'wheel_radius' in param:
            self.r = param['wheel_radius']
        else:
            self.r = self._DEFAULT_PARAM['wheel_radius']

    def update_kinematics(self, point):
        u"""Update TARGET athletic.

        Args:
           Point: DICT of each state
        Note:
           Regarding variable names
           [x, y, a]: Each represents the rotation of X coordinates, Y coordinates, Z axes
           [0,1,2]: Each represents world coordinates, bogie coordinates, top plate coordinates
           [D, v, a]: Express displacement, speed, acceleration, respectively
        """
        # Calculate the position and posture of the bogie
        (xd02, xv02, xa02) = (point['x02'][i] for i in range(3))
        (yd02, yv02, ya02) = (point['y02'][i] for i in range(3))
        (ad02, av02, aa02) = (point['a02'][i] for i in range(3))
        ad12 = point['a12'][0]
        # Calculation of bogie displacement
        ad01 = ad02 - ad12
        # Speed ​​calculation
        av01 = (yv02 * cos(ad01) - xv02 * sin(ad01)) / self.xd12
        xv01 = xv02 + self.xd12 * av01 * sin(ad01)
        yv01 = yv02 - self.xd12 * av01 * cos(ad01)
        av12 = av02 - av01
        # Acceleration calculation
        xa01 = (xa02 + (2.0 * av01 * yv02 + xa02) * cos(2.0 * ad01)
                + (ya02 - 2.0 * av01 * xv02) * sin(2.0 * ad01)) / 2.0
        ya01 = (ya02 + (2.0 * av01 * xv02 - ya02) * cos(2 * ad01) + (xa02 + 2.0 * av01 * yv02) * sin(2 * ad01)) / 2.0
        aa01 = ((ya02 - av01 * xv02) * cos(ad01) - (xa02 + av01 * yv02) * sin(ad01)) / self.xd12
        aa12 = aa02 - aa01
        # Calculation of bogie parallel speed
        if abs(cos(ad01)) > 0.5:
            xv11 = xv01 / cos(ad01)
            xa11 = xa01 / cos(ad01) + (xv01 * sin(ad01) * av01 / (cos(ad01) ** 2.0))
        else:
            xv11 = yv01 / sin(ad01)
            xa11 = ya01 / sin(ad01) - (yv01 * cos(ad01) * av01 / (sin(ad01) ** 2.0))
        # Save the wheel speed orbital from the bogie speed orbit
        wl = (2.0 * xv11 - av01 * self.d) / self.r / 2.0
        wr = (2.0 * xv11 + av01 * self.d) / self.r / 2.0
        dwl = (2.0 * xa11 - aa01 * self.d) / self.r / 2.0
        dwr = (2.0 * xa11 + aa01 * self.d) / self.r / 2.0

        # Return Point
        point['left_wheel'] = [0.0, wl, dwl]
        point['right_wheel'] = [0.0, wr, dwr]
        point['caster'] = [ad12, av12, aa12]
        # Update DICT of state
        self.point = point

    def get_dynamics(self):
        u"""Return the dynamic spalameter (A, B, C, D).

        Args:
            (a, b, c, d), each is a dict with ('variable name', 'restriction type'), respectively, and the value is Value.
        """
        (a, b, c, d) = ({}, {}, {}, {})
        a['left_wheel', 'acceleration'] = self.point['left_wheel'][1]
        b['left_wheel', 'acceleration'] = self.point['left_wheel'][2]
        c['left_wheel', 'acceleration'] = 0.0
        d['left_wheel', 'acceleration'] = 0.0
        a['right_wheel', 'acceleration'] = self.point['right_wheel'][1]
        b['right_wheel', 'acceleration'] = self.point['right_wheel'][2]
        c['right_wheel', 'acceleration'] = 0.0
        d['right_wheel', 'acceleration'] = 0.0
        a['caster', 'acceleration'] = self.point['caster'][1]
        b['caster', 'acceleration'] = self.point['caster'][2]
        c['caster', 'acceleration'] = 0.0
        d['caster', 'acceleration'] = 0.0

        return (a, b, c, d)

    def generate_base_trajectory(self, traj, caster_angle, step):
        u"""Calculate and add the orbit of the casta shaft angle from the car body command orbital ('oDOM_X', 'oDOM_Y', 'oDOM_T').

        Args:
            Traj: TrajectoryDict
            Caster_angle: Initial angle of Caster [RAD]
            STEP: Division of parameters
        """
        # Organization generation
        traj['x01'] = Trajectory(traj.length)
        traj['y01'] = Trajectory(traj.length)
        traj['a01'] = Trajectory(traj.length)
        traj['x11'] = Trajectory(traj.length)
        traj['a12'] = LinearTrajectory(traj.length)
        traj['left_wheel'] = Trajectory(traj.length)
        traj['right_wheel'] = Trajectory(traj.length)
        # Define an alias for ease of internal processing
        traj['x02'] = traj['odom_x']
        traj['y02'] = traj['odom_y']
        traj['a02'] = traj['odom_t']
        traj['caster'] = LinearTrajectory(traj.length)

        # Save the trajectory of the caster axis by integrating from the initial value
        traj['a01'][0] = (traj['a02'](0)[0] - caster_angle, 0, 0)
        size = int((traj.length - 1) / step)
        for i in range(size + 1):
            s = step * i
            (xd02, xv02, xa02) = traj['x02'](s)
            (yd02, yv02, ya02) = traj['y02'](s)
            (ad02, av02, aa02) = traj['a02'](s)
            ad01 = traj['a01'][s][0]
            av01 = (yv02 * cos(ad01) - xv02 * sin(ad01)) / self.xd12
            aa01 = ((ya02 - av01 * xv02) * cos(ad01) - (xa02 + av01 * yv02) * sin(ad01)) / self.xd12
            traj['a01'][s][1] = av01
            traj['a01'][s][2] = aa01
            traj['a12'][s] = (ad02 - ad01, av02 - av01, aa02 - aa01)
            traj['caster'][s] = traj['a12'][s]
            xd01 = xd02 - self.xd12 * cos(ad01)
            xv01 = xv02 + self.xd12 * av01 * sin(ad01)
            xa01 = (xa02 + (2 * av01 * yv02 + xa02) * cos(2 * ad01) + (ya02 - 2 * av01 * xv02) * sin(2 * ad01)) / 2
            traj['x01'][s] = (xd01, xv01, xa01)
            yd01 = yd02 - self.xd12 * sin(ad01)
            yv01 = yv02 - self.xd12 * av01 * cos(ad01)
            ya01 = (ya02 + (2 * av01 * xv02 - ya02) * cos(2 * ad01) + (xa02 + 2 * av01 * yv02) * sin(2 * ad01)) / 2
            traj['y01'][s] = (yd01, yv01, ya01)
            # Calculation of bogie parallel speed
            if abs(cos(ad01)) > 0.5:
                xv11 = xv01 / cos(ad01)
                xa11 = xa01 / cos(ad01) + (xv01 * sin(ad01) * av01 / (cos(ad01) ** 2))
            else:
                xv11 = yv01 / sin(ad01)
                xa11 = ya01 / sin(ad01) - (yv01 * cos(ad01) * av01 / (sin(ad01) ** 2))
            traj['x11'][s] = (0, xv11, 0)
            # Save the wheel speed orbital from the bogie speed orbit
            wl = (2 * xv11 - av01 * self.d) / self.r / 2
            wr = (2 * xv11 + av01 * self.d) / self.r / 2
            dwl = (2 * xa11 - aa01 * self.d) / self.r / 2
            dwr = (2 * xa11 + aa01 * self.d) / self.r / 2
            traj['left_wheel'][s] = (0, wl, dwl)
            traj['right_wheel'][s] = (0, wr, dwr)
            # Call integral of the bogie
            if i != size:
                ad01 += av01 * step + 0.5 * aa01 * step ** 2
                traj['a01'][step * (i + 1)] = (ad01, av01, aa01)

    def plot_trajectory(self, traj, step=0.01):
        import matplotlib
        import matplotlib.pyplot as plt
        # Change AGG to TKAGG when Debug
        matplotlib.use('Agg')

        names = ['x02', 'y02', 'a02', 'x01', 'y01', 'a01',
                 'a12', 'x11', 'left_wheel', 'right_wheel', 'caster']
        x = [i * step for i in range(int(traj.length / step) + 1)]

        (y0, y1, y2) = ({}, {}, {})
        for name in names:
            (y0[name], y1[name], y2[name]) = ([], [], [])
        for s in x:
            point = traj(s)
            self.update(point)
            for name in names:
                y0[name].append(point[name][0])
                y1[name].append(point[name][1])
                y2[name].append(point[name][2])

        plt.title('x-y')
        plt.plot(y0['x02'], y0['y02'], '-', label='Body')
        plt.plot(y0['x01'], y0['y01'], '-', label='Base')
        plt.legend()
        plt.show()

        for name in names:
            plt.title(name)
            plt.plot(x, y0[name], '-', label='%s[0]' % name)
            plt.plot(x, y1[name], '-', label='%s[1]' % name)
            plt.plot(x, y2[name], '-', label='%s[2]' % name)
            plt.legend()
            plt.show()
