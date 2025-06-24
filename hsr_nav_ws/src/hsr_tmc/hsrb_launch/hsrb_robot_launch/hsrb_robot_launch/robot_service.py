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
#!/usr/bin/python3
from __future__ import print_function

import atexit
import os
import signal
import subprocess
import sys
import threading

import psutil
import rclpy
from rclpy.executors import MultiThreadedExecutor
import std_msgs.msg

from tmc_voice_msgs.msg import Voice

from .robot_service_utils import exxx_read_hash, RobotServiceDiagPublisher


MESSAGES = {
    'HSR start': {'ja': [u"HSRスタート", 3.0],
                  'en': [u"HSR start", 2.0]},
    'HSR stopped': {'ja': [u"停止しました", 3.0],
                    'en': [u"HSR stopped", 1.0]},
    'Killing all nodes': {'ja': (u"全てのノードを停止します", 4.0),
                          'en': (u"Killing all the nodes", 4.0)},
    'Failed to launch. Retrying..': {'ja': (u"起動に失敗しました。再起動します", 5.0),
                                     'en': (u'Failed to launch. Retrying', 4.0)},
    'Timeout. Start shutdown': {'ja': (u"{0}分経過しました", 3.0),
                                'en': (u"{0} minutes passed", 3.0)},
}

"""
If this node is standing up, it will be generally stood up, and if you can Kill, you will be able to do other things.
Node name set used
I suppressed basic things
"""
CHECK_NODE = {"robot_state_publisher", "joint_state_publisher",
              "joint_state_broadcaster", "servo_diagnostic_broadcaster",
              "base_bumper_node", "dynpick_driver_node", "urg_node",
              "controller_manager", "arm_trajectory_controller",
              "omni_base_controller", "gripper_controller",
              "head_trajectory_controller"}


class RobotService:
    def __init__(self, watch_motor_id=11, watch_timeout=15, keep_nodes_time=10, lang='ja'):
        rclpy.init()
        self.node = rclpy.create_node("robot_service")
        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()
        self.watch_motor_id = watch_motor_id
        self.watch_timeout = watch_timeout
        self.keep_nodes_time = keep_nodes_time
        self.lang = lang
        personal_name = os.getenv("PERSONAL_NAME", "HSR")
        MESSAGES['HSR start']['ja'][0] = personal_name + u"スタート"
        MESSAGES['HSR start']['en'][0] = personal_name + u" start"

        if personal_name != "HSR":
            # I intentionally include comma to give the utterance between them
            MESSAGES['HSR stopped']['ja'][0] = personal_name + u",停止しました"
        MESSAGES['HSR stopped']['en'][0] = personal_name + u" stopped"

        self.was_emergency_on = False
        self.button_was_reset = False
        self.running = True

        self.proc_app = None
        self.proc_device = None
        self.previous_signal = None

        self.make_subscriber()

        self.node.get_logger().info("Connecting to /talk_request")
        self.talk_request_pub = self.node.create_publisher(Voice, 'talk_request', 1)
        result = self.wait_until(lambda: self.talk_request_pub.get_subscription_count() > 0,
                                 timeout=3.0, polling=0.1)
        if not result:
            self.node.get_logger().error("Failed to connect to /talk_request")

        self.node.get_logger().info("Connecting to /diagnostics")
        self.diag_pub = RobotServiceDiagPublisher(self.node)
        result = self.wait_until(lambda: self.diag_pub.get_subscription_count() > 0,
                                 timeout=3.0, polling=0.1)
        if not result:
            self.node.get_logger().error("Failed to connect to /diagnostics")

        signal.signal(signal.SIGTERM, self.signal)
        signal.signal(signal.SIGINT, self.signal)
        # Drop the child process without any problems even when it ends with an internal error
        atexit.register(self.term_nodes)

    def make_subscriber(self):
        pass

    def signal(self, sig, stack):
        u"""Signal handler"""
        self.running = False
        if sig != self.previous_signal:
            # If the same signal comes multiple times while stopping the node, ignore it
            self.previous_signal = sig
            self.term_nodes()
        rclpy.try_shutdown()
        self.node.destroy_node()
        sys.exit(sig)

    def kill_process(self, proc):
        u"""Drop all processes including grandchildren"""
        try:
            parent = psutil.Process(proc.pid)
        except psutil.NoSuchProcess:
            return
        # Get all the child processes recursively
        children = parent.children(recursive=True)
        # Drop the target process
        proc.terminate()
        proc.wait()
        # End the remaining child process
        for child in children:
            if child.is_running():
                child.kill()

    def term_nodes(self, all_nodes=True):
        u"""Stop node"""
        if all_nodes:
            if self.proc_app:
                self.notify('Killing all nodes', True)
                self.kill_process(self.proc_app)
                self.proc_app = None
        if self.proc_device:
            self.kill_process(self.proc_device)
            self.proc_device = None

    def launch_nodes(self, all_nodes=True, timeout=15.0):
        u"""Start a node"""
        self.node.get_logger().info("Start launching")
        # Start a node other than the device system
        # TODO(Masayuki Masuda): 起動スクリプトが出来たら追加
        if all_nodes:
            pass
        # Start the device node
        self.proc_device = subprocess.Popen(
            ['/opt/ros/{0}/bin/ros2'.format(os.environ['ROS_DISTRO']), 'launch',
             'hsrb_bringup', 'robot.launch.py'],
            stdout=None, stderr=None, close_fds=True)

        return self.wait_until(lambda: CHECK_NODE <= set(self.node.get_node_names()),
                               timeout=timeout, polling=0.5)

    def wait_until(self, func, timeout=None, polling=1.0, *args, **kwargs):
        if timeout is None:
            duration = rclpy.time.Duration(seconds=0)
        else:
            duration = rclpy.time.Duration(seconds=timeout)
        start = self.node.get_clock().now()
        if polling > 0:
            rate = self.node.create_rate(1 / polling)
        else:
            rate = self.node.create_rate(1.0)
        while ((self.node.get_clock().now() - start) < duration or timeout is None):
            cond = func(*args, **kwargs)
            if cond:
                return True
            if not self.running:
                break
            rate.sleep()

        return False

    def watch(self):
        u"""Surveillance until the emergency stop is ON/OFF at regular intervals.

        If the publisher falls and does not have more than Duration or more, if it is no longer Running.
        """
        self.last_subscribed = self.node.get_clock().now()
        duration = rclpy.time.Duration(seconds=self.watch_timeout)
        rate = self.node.create_rate(1)
        while ((self.node.get_clock().now() - self.last_subscribed) < duration):
            if self.was_emergency_on:
                return False
            if not self.running:
                break
            rate.sleep()
        return True

    def notify(self, message, wait, *args, **kwargs):
        u"""Development notification"""
        self.node.get_logger().info(message)
        if message in MESSAGES:
            if self.lang in ('ja', 'en'):
                sentence = MESSAGES[message][self.lang][0].format(
                    *args, **kwargs)
                self.talk_request_pub.publish(
                    Voice(sentence=sentence, language={'ja': 0, 'en': 1}[self.lang]))
                if wait:
                    self.node.create_rate(1 / MESSAGES[message][self.lang][1]).sleep()

    def wait_until_servo_is_not_ready(self):
        self.node.get_logger().info("Waiting until servo is not ready")
        self.wait_until(lambda: exxx_read_hash(self.watch_motor_id) is None, polling=0.5)

    def wait_until_servo_is_ready(self):
        self.node.get_logger().info("Waiting until servo is ready")
        self.wait_until(lambda: exxx_read_hash(self.watch_motor_id) is not None, polling=0.5)

    def run(self):
        self.wait_until_servo_is_not_ready()
        is_all = True
        while rclpy.ok() and self.running:
            self.wait_until_servo_is_ready()
            self.previous_signal = None
            self.was_emergency_on = False
            self.button_was_reset = False

            self.notify('HSR start', True)
            self.diag_pub.publish()
            success = self.launch_nodes(is_all)
            # When unintended behavior occurs, make TRUE to restart all nodes
            is_all = True
            if success:
                is_all = self.watch()
                self.notify('HSR stopped', False)

                if not is_all:
                    # If there is no operation for a certain period, stop all nodes
                    detected = self.wait_until(lambda: self.button_was_reset,
                                               timeout=60.0 * self.keep_nodes_time)
                    if not detected:
                        is_all = True
                        self.notify('Timeout. Start shutdown', True, self.keep_nodes_time)

            else:
                self.notify('Failed to launch. Retrying..', True)

            self.term_nodes(is_all)

            # If you re -set up before the node falls, you will get an error such as parameter duplication, so wait.
            is_down = self.wait_until(lambda: CHECK_NODE.intersection(self.node.get_node_names()) == set(),
                                      timeout=60.0,
                                      polling=0.5)
            if self.running and not is_down:
                self.term_nodes()
                rclpy.try_shutdown()
                self.node.destroy_node()
                raise RuntimeError("Node down timeout, something went wrong")

    def stop_button_cb(self, msg):
        self.last_subscribed = self.node.get_clock().now()
        if msg.data:
            self.was_emergency_on = True
        self.button_was_reset = self.was_emergency_on and not msg.data


class HsrbRobotService(RobotService):
    u"""HSRB automatic start -up control class"""

    def __init__(self, watch_motor_id=11, watch_timeout=15, keep_nodes_time=10, lang='ja'):
        super(HsrbRobotService, self).__init__(watch_motor_id=watch_motor_id,
                                               watch_timeout=watch_timeout,
                                               keep_nodes_time=keep_nodes_time,
                                               lang=lang)

    def make_subscriber(self):
        self.node.create_subscription(
            std_msgs.msg.Bool, 'runstop_button',
            self.stop_button_cb, 10)


class HsrcRobotService(RobotService):
    u"""HSRC automatic start -up control class"""

    def __init__(self, watch_motor_id=11, watch_timeout=15, keep_nodes_time=10, lang='ja'):
        super(HsrcRobotService, self).__init__(watch_motor_id=watch_motor_id,
                                               watch_timeout=watch_timeout,
                                               keep_nodes_time=keep_nodes_time,
                                               lang=lang)

    def make_subscriber(self):
        self.node.create_subscription(
            std_msgs.msg.Bool, 'emergency_stop_button',
            self.stop_button_cb, 10)
        self.node.create_subscription(
            std_msgs.msg.Bool, 'wireless_stop_button',
            self.stop_button_cb, 10)
