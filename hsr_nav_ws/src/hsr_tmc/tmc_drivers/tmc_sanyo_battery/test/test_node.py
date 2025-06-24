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

from diagnostic_msgs.msg import (
    DiagnosticArray,
    DiagnosticStatus,
)
import pytest
import rclpy
import rclpy.duration
from sensor_msgs.msg import BatteryState
from tmc_sanyo_battery.driver.driver import BatteryError
from tmc_sanyo_battery.node import main


def make_test_status():
    test_status = {}
    test_status['voltage'] = 1.0
    test_status['temperature'] = 2
    test_status['electric_current'] = 3.0
    test_status['remaining_charge'] = 4.0
    test_status['full_charge_capacity'] = 5.0
    test_status['battery_level'] = 6.0
    test_status['full_charge'] = False
    test_status['over_charge'] = False
    test_status['over_discharge'] = False
    return test_status


@pytest.fixture
def setup(mocker):
    driver_mock = mocker.patch('tmc_sanyo_battery.node.driver.connect')
    connect_mock = driver_mock.return_value.__enter__.return_value
    connect_mock.read.return_value = make_test_status()

    rclpy.init()
    test_node = rclpy.create_node('test_node')
    mocker.patch('rclpy.init')
    ok_mock = mocker.patch('rclpy.ok')
    ok_mock.return_value = True

    do_spin = True

    def spin_func(node):
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(test_node)
        while do_spin:
            try:
                # Throw exception after rclpy.ok is set False
                executor.spin_once()
            except Exception:
                pass

    spin_mock = mocker.patch('rclpy.spin')
    spin_mock.side_effect = spin_func

    main_thread = threading.Thread(target=main)
    main_thread.start()

    yield test_node, connect_mock

    ok_mock.return_value = False
    main_thread.join()

    do_spin = False

    test_node.destroy_node()
    # rclpy.shutdown is called in main


def capture_diagnostic_status(node, level):
    values = []

    def callback(msg):
        # Use level as capture trigger since diagnostic publisher run async
        status = msg.status[0]
        if status.level == level:
            values.append(status)

    node.create_subscription(DiagnosticArray, 'diagnostics', callback, 1)
    timeout = node.get_clock().now() + rclpy.duration.Duration(seconds=5.0)
    rate = node.create_rate(10.0)
    while len(values) == 0 and node.get_clock().now() < timeout:
        rate.sleep()
    return values[0]


def extract_value(key_values, target_key):
    for key_value in key_values:
        if key_value.key == target_key:
            return key_value.value
    return None


def test_diagnostics(setup):
    node = setup[0]
    status = capture_diagnostic_status(node, DiagnosticStatus.ERROR)
    assert status.message == 'Battery Level: 6.0 %'
    assert extract_value(status.values, 'voltage') == '1.0'
    assert extract_value(status.values, 'temperature') == '2'
    assert extract_value(status.values, 'electric_current') == '3.0'
    assert extract_value(status.values, 'remaining_charge') == '4.0'
    assert extract_value(status.values, 'full_charge_capacity') == '5.0'
    assert extract_value(status.values, 'battery_level') == '6.0'
    assert extract_value(status.values, 'full_charge') == 'False'
    assert extract_value(status.values, 'over_charge') == 'False'
    assert extract_value(status.values, 'over_discharge') == 'False'

    test_status = make_test_status()
    test_status['electric_current'] = -1.0

    connect_mock = setup[1]
    connect_mock.read.return_value = test_status

    status = capture_diagnostic_status(node, DiagnosticStatus.OK)
    assert status.message == 'Battery Level: 6.0 %'

    test_status['electric_current'] = 1.0
    test_status['battery_level'] = 21.0
    connect_mock.read.return_value = test_status

    status = capture_diagnostic_status(node, DiagnosticStatus.WARN)
    assert status.message == 'Battery Level: 21.0 %'

    test_status['battery_level'] = 51.0
    connect_mock.read.return_value = test_status

    status = capture_diagnostic_status(node, DiagnosticStatus.OK)
    assert status.message == 'Battery Level: 51.0 %'


def capture_battery_state(node):
    values = []

    def callback(msg):
        values.append(msg)

    node.create_subscription(BatteryState, 'battery_state', callback, 1)
    timeout = node.get_clock().now() + rclpy.duration.Duration(seconds=5.0)
    rate = node.create_rate(10.0)
    while len(values) == 0 and node.get_clock().now() < timeout:
        rate.sleep()
    return values[0]


def test_battery_state_normal(setup):
    node = setup[0]
    state = capture_battery_state(node)
    assert state.voltage == 1.0
    assert state.temperature == 2.0
    assert state.current == 3.0
    assert state.charge == 4.0
    assert state.capacity == 5.0
    assert state.percentage == 6.0
    assert state.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    assert state.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_GOOD


def test_battery_state_full_charge(setup):
    test_status = make_test_status()
    test_status['full_charge'] = True

    connect_mock = setup[1]
    connect_mock.read.return_value = test_status

    node = setup[0]
    state = capture_battery_state(node)
    assert state.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_FULL


def test_battery_state_charging(setup):
    test_status = make_test_status()
    test_status['electric_current'] = -1.0

    connect_mock = setup[1]
    connect_mock.read.return_value = test_status

    node = setup[0]
    state = capture_battery_state(node)
    assert state.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING


def test_battery_state_over_charge(setup):
    test_status = make_test_status()
    test_status['over_charge'] = True

    connect_mock = setup[1]
    connect_mock.read.return_value = test_status

    node = setup[0]
    state = capture_battery_state(node)
    assert state.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE


def test_battery_state_over_discharge(setup):
    test_status = make_test_status()
    test_status['over_discharge'] = True

    connect_mock = setup[1]
    connect_mock.read.return_value = test_status

    node = setup[0]
    state = capture_battery_state(node)
    assert state.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE


def test_battery_state_read_error(setup):
    connect_mock = setup[1]
    connect_mock.read.side_effect = BatteryError

    node = setup[0]
    state = capture_battery_state(node)
    assert state.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
