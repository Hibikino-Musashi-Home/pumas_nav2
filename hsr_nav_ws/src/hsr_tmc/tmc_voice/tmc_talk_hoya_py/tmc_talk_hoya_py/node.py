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
import threading

import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
)
from std_msgs.msg import String

from tmc_voice_msgs.action import TalkRequest
from tmc_voice_msgs.msg import Voice

from .voicetext import (
    VoiceTextRuntimeError,
    VoiceTextSpeaker
)


class VoiceTextNode(Node):
    def __init__(self):
        super().__init__('text_to_speech')

        self._pitch = self._get_voicetext_param('pitch', 50, 200)
        self._speed = self._get_voicetext_param('speed', 50, 400)
        self._volume = self._get_voicetext_param('volume', 0, 500)
        self._pause = self._get_voicetext_param('pause', 0, 65535)

        root_path = self._get_root_path()
        jpn_voice = self._get_voices('jpn_voice', ['haruka'])
        for voice in jpn_voice:
            try:
                self._vt_jpn = VoiceTextSpeaker(path=root_path + '/vt', voice=voice)
                self.get_logger().info(f'Voicetext {voice} is ready')
                break
            except VoiceTextRuntimeError as err:
                self.get_logger().info(str(err))
                self._vt_jpn = None

        eng_voice = self._get_voices('eng_voice', ['julie'])
        for voice in eng_voice:
            try:
                self._vt_eng = VoiceTextSpeaker(path=root_path + '/vt', voice=voice)
                self.get_logger().info(f'Voicetext {voice} is ready')
                break
            except VoiceTextRuntimeError as err:
                self.get_logger().info(str(err))
                self._vt_eng = None

        self._is_speaking = False
        self._end_time = None
        self._lock = threading.Lock()

        self._subscriber = self.create_subscription(Voice, 'talk_request', self._subscriber_callback, 1)
        self._publisher = self.create_publisher(
            String, 'talking_sentence',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL))
        self._publisher.publish(String())

        self._timer = self.create_timer(0.1, self._run)

        self._action_server = ActionServer(
            self,
            TalkRequest,
            'talk_request_action',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._preempt_callback,
            callback_group=ReentrantCallbackGroup())
        self._goal_handle = None

    def _get_voicetext_param(self, name, min_value, max_value):
        self.declare_parameter(name, -1)
        value = self.get_parameter(name).get_parameter_value().integer_value
        if min_value <= value <= max_value:
            return value
        else:
            return -1

    def _get_root_path(self):
        self.declare_parameter('root_path', '/opt/tmc')
        return self.get_parameter('root_path').get_parameter_value().string_value

    def _get_voices(self, name, default_voices):
        self.declare_parameter(name, default_voices)
        return self.get_parameter(name).get_parameter_value().string_array_value

    def __enter__(self):
        if self._vt_jpn is not None:
            self._vt_jpn.__enter__()
        if self._vt_eng is not None:
            self._vt_eng.__enter__()
        return self

    def __exit__(self, *args):
        if self._vt_jpn is not None:
            self._vt_jpn.__exit__(*args)
        if self._vt_eng is not None:
            self._vt_eng.__exit__(*args)
        return True

    def _subscriber_callback(self, data):
        with self._lock:
            if self._is_speaking:
                self._is_speaking = False
                self._cancel_talking()
                if self._goal_handle is not None:
                    self._goal_handle = None
            self._speak_sentence(data)

    async def _execute_callback(self, goal_handle):
        with self._lock:
            if self._is_speaking:
                self._is_speaking = False
                self._cancel_talking()

            self._speak_sentence(goal_handle.request.data)
            if not self._is_speaking:
                goal_handle.abort()
                return TalkRequest.Result()

        self._goal_handle = goal_handle

        rate = self.create_rate(20.0)
        while self._is_speaking and self._goal_handle is not None:
            rate.sleep()
        if self._goal_handle is None and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif self._goal_handle is None and not goal_handle.is_cancel_requested:
            goal_handle.abort()
        else:
            self._goal_handle = None
        return TalkRequest.Result()

    def _goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def _preempt_callback(self, goal_handle):
        with self._lock:
            self._is_speaking = False
            self._cancel_talking()
            self._goal_handle = None
        return CancelResponse.ACCEPT

    def _speak_sentence(self, data):
        duration = self._send_sentence_to_speaker(data)
        if duration > 0.0:
            msg = String()
            msg.data = data.sentence
            self._publisher.publish(msg)
            self._end_time = self.get_clock().now() + Duration(seconds=duration)
            self._is_speaking = True
        else:
            self._end_time = None
            self._is_speaking = False

    def _send_sentence_to_speaker(self, data):
        if data.language == Voice.JAPANESE:
            if self._vt_jpn is None:
                self.get_logger().warn("Japanese license is not available.")
                return 0.0
            else:
                vt = self._vt_jpn
        elif data.language == Voice.ENGLISH:
            if self._vt_eng is None:
                self.get_logger().warn("English license is not available.")
                return 0.0
            else:
                vt = self._vt_eng
        else:
            self.get_logger().error("Requested language is not supported.")
            return 0.0
        try:
            duration = vt.speak(data.sentence,
                                pitch=self._pitch,
                                speed=self._speed,
                                volume=self._volume,
                                pause=self._pause)
            self._cancel_talking = vt.cancel
            return duration
        except Exception as err:
            self.get_logger().error(str(err))
            return 0.0

    def _run(self):
        if self._is_speaking:
            remaining = self._end_time - self.get_clock().now()
            if remaining.nanoseconds < 0:
                if self._goal_handle is not None:
                    self._goal_handle.succeed()
                self._publisher.publish(String())
                self._end_time = None
                self._is_speaking = False
            else:
                if self._goal_handle is not None:
                    feedback = TalkRequest.Feedback()
                    feedback.remaining_time = remaining.to_msg()
                    self._goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)
    with VoiceTextNode() as node:
        try:
            executor = MultiThreadedExecutor()
            rclpy.spin(node, executor)
        except Exception as err:
            node.get_logger().error(str(err))
        rclpy.try_shutdown()
        node.destroy_node()
