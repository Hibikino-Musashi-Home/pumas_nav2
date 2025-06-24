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
import threading

import serial


class SerialConnection:

    _COLOR_INPUT_MAX = 255

    def __init__(self, port_name, baudrate):
        self._connect = serial.Serial(port_name, baudrate, timeout=0.1)
        self._lock = threading.Lock()

        self._color_r = 255
        self._color_g = 255
        self._color_b = 255
        self._head_led = 0
        self._hand_led = 0
        self._drive_power = 1
        self._suction = 0

        self._magnetic_sensor_1 = False
        self._magnetic_sensor_2 = False
        self._emergency_button = False
        self._pressure_sensor = False
        self._front_bumper_sensor = False
        self._rear_bumper_sensor = False

    def run(self, interval):
        self._run_impl(interval)

    def set_color(self, r, g, b):
        with self._lock:
            self._color_r = self._to_color_input(r)
            self._color_g = self._to_color_input(g)
            self._color_b = self._to_color_input(b)

    def set_head_led(self, state):
        with self._lock:
            self._head_led = int(state)

    def set_hand_led(self, state):
        with self._lock:
            self._hand_led = int(state)

    def set_drive_power(self, state):
        with self._lock:
            self._drive_power = int(state)

    def set_suction(self, state):
        with self._lock:
            self._suction = int(state)

    def magnetic_sensor_1(self):
        with self._lock:
            return self._magnetic_sensor_1

    def magnetic_sensor_2(self):
        with self._lock:
            return self._magnetic_sensor_2

    def emergency_button(self):
        with self._lock:
            return self._emergency_button

    def pressure_sensor(self):
        with self._lock:
            return self._pressure_sensor

    def front_bumper_sensor(self):
        with self._lock:
            return self._front_bumper_sensor

    def rear_bumper_sensor(self):
        with self._lock:
            return self._rear_bumper_sensor

    def _to_color_input(self, value):
        return int(min(max(0, value * self._COLOR_INPUT_MAX), self._COLOR_INPUT_MAX))

    def _write(self):
        send_data = bytes([self._color_r, self._color_g, self._color_b,
                           self._head_led, self._hand_led, self._drive_power, self._suction])
        self._connect.write(send_data)
        self._connect.flush()

    def _read(self):
        values = self._connect.read(6).decode()
        if len(values) < 6:
            return
        self._magnetic_sensor_1 = bool(int(values[0]))
        self._magnetic_sensor_2 = bool(int(values[1]))
        self._emergency_button = bool(int(values[2]))
        self._pressure_sensor = bool(int(values[3]))
        self._front_bumper_sensor = bool(int(values[4]))
        self._rear_bumper_sensor = bool(int(values[5]))

    def _run_impl(self, interval):
        with self._lock:
            self._write()
            self._read()
        timer = threading.Timer(interval, self._run_impl, (interval, ))
        timer.start()
