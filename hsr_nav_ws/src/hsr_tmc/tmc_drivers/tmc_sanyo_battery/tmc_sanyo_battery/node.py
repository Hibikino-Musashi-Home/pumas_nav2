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

import threading

from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
import rclpy
from sensor_msgs.msg import BatteryState

from .driver import driver


class BatteryDiag:

    def __init__(self, init):
        self.status = init
        self.error = None
        self.is_charging = True

    def update(self, stat):
        data = self.status
        level = data['battery_level']
        if self.error is not None:
            stat.summary(DiagnosticStatus.ERROR, f'Battery Error: {self.error}')
        elif self.is_charging:
            stat.summary(DiagnosticStatus.OK, f'Battery Level: {level} %')
        elif level > 50:
            stat.summary(DiagnosticStatus.OK, f'Battery Level: {level} %')
        elif level > 20:
            stat.summary(DiagnosticStatus.WARN, f'Battery Level: {level} %')
        else:
            stat.summary(DiagnosticStatus.ERROR, f'Battery Level: {level} %')
        for k, v in data.items():
            stat.add(k, str(v))
        return stat


def main():
    rclpy.init()
    node = rclpy.create_node('sanyo_battery')

    updater = diagnostic_updater.Updater(node)
    updater.setHardwareID('SANYO Battery')

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    state_pub = node.create_publisher(BatteryState, 'battery_state', 1)

    device_name = node.declare_parameter('device_name', '/dev/sanyo-battery').value
    baudrate = node.declare_parameter('baudrate', 38400).value
    pub_rate = node.declare_parameter('publish_rate', 1.0).value

    with driver.connect(device_name, baudrate) as conn:
        data = driver.default_status()
        diag = BatteryDiag(data)
        updater.add("Battery updater", diag.update)

        rate = node.create_rate(pub_rate)
        while rclpy.ok():
            rate.sleep()

            error = None
            try:
                data = conn.read()
            except driver.BatteryError as e:
                error = str(e)
                node.get_logger().warn(error)

            state = BatteryState()
            state.header.stamp = node.get_clock().now().to_msg()
            state.voltage = data['voltage']
            state.temperature = float(data['temperature'])
            state.current = data['electric_current']
            state.charge = data['remaining_charge']
            # FULL_CHARGE_CAPACITY is Capace or Design_capacity, but I don't care because it's not important
            state.capacity = data['full_charge_capacity']
            state.design_capacity = float('nan')
            # state.design_capacity = data['full_charge_capacity']
            state.percentage = data['battery_level']
            state.present = True
            state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

            if data['full_charge']:
                state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            elif data['electric_current'] < 0.0:
                state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:
                state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

            if data['over_charge']:
                state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
            elif data['over_discharge']:
                state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
            elif error is None:
                state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            else:
                state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            state_pub.publish(state)

            if data['electric_current'] < 0.0:
                diag.is_charging = True
            else:
                diag.is_charging = False
            diag.status = data
            diag.error = error
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
