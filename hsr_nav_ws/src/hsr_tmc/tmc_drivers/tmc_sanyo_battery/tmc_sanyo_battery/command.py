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
# vim: fileencoding=utf-8 :

import argparse
import traceback

from .driver import driver


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--device', type=str,
                        action='store', dest='device_name',
                        default='/dev/sanyo-battery',
                        help='Specify device file name')
    parser.add_argument('-b', '--baudrate', type=int,
                        action='store', dest='baudrate',
                        default=38400,
                        help='Baudrate[bps]')
    parser.add_argument('-v', '--verbose', action='store_true',
                        default=False, dest='verbose', help='Detailed output')
    args = parser.parse_args()
    try:
        with driver.connect(args.device_name, args.baudrate) as conn:
            data = conn.read()
            if args.verbose:
                print('battery_level         [%]: {0:8.2f}'.format(
                    data['battery_level']))
                print('remaining_charge     [Ah]: {0:8.2f}'.format(
                    data['remaining_charge']))
                print('full_charge_capacity [Ah]: {0:8.2f}'.format(
                    data['full_charge_capacity']))
                print('temperature       [deg C]: {0:8d}'.format(
                    data['temperature']))
                print('electric_current      [A]: {0:8.2f}'.format(
                    data['electric_current']))
                print('voltage               [V]: {0:8.2f}'.format(
                    data['voltage']))
                print('zero_percent_detected    : {0:5s}'.format(
                    str(data['zero_percent_detected'])))
                print('discharge_enabled        : {0:5s}'.format(
                    str(data['discharge_enabled'])))
                print('charge_enabled           : {0:5s}'.format(
                    str(data['charge_enabled'])))
                print('over_discharge           : {0:5s}'.format(
                    str(data['over_discharge'])))
                print('full_charge              : {0:5s}'.format(
                    str(data['full_charge'])))
                print('learning_enabled         : {0:5s}'.format(
                    str(data['learning_enabled'])))
                print('triple_parallel          : {0:5s}'.format(
                    str(data['triple_parallel'])))
                print('over_charge              : {0:5s}'.format(
                    str(data['over_charge'])))
            else:
                print('{0:2.2f} %'.format(data['battery_level']))
    except Exception:
        traceback.print_exc()


if __name__ == '__main__':
    main()
