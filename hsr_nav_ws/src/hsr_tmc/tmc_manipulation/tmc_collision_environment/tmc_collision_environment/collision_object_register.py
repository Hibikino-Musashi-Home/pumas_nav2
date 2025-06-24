#!/usr/bin/env python3
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
import time

from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningSceneWorld
import rclpy
from rclpy.node import Node


class CollisionObjectRegister(Node):

    def __init__(self, collision_objects: list[CollisionObject]):
        super().__init__('collision_object_register')

        self._collision_objects = collision_objects
        self._removing_ids = [x.id for x in collision_objects]

        self._pub = self.create_publisher(CollisionObject, 'collision_environment_server/collision_object', 10)
        self._sub = self.create_subscription(
            PlanningSceneWorld, 'collision_environment_server/environment', self._callback, 1)

    def remove(self):
        for id in self._removing_ids:
            collision_object = CollisionObject()
            collision_object.id = id
            collision_object.operation = CollisionObject.REMOVE
            self._pub.publish(collision_object)

    def regist(self):
        for collision_object in self._collision_objects:
            collision_object.operation = CollisionObject.ADD
            self._pub.publish(collision_object)

    @property
    def not_removed_object_num(self):
        return len(self._removing_ids)

    @property
    def not_registered_object_num(self):
        return len(self._collision_objects)

    def _callback(self, msg: PlanningSceneWorld):
        registered = [x.id for x in msg.collision_objects]
        if len(self._removing_ids):
            self._removing_ids = [x for x in self._removing_ids if x in registered]
        else:
            self._collision_objects = [x for x in self._collision_objects if x.id not in registered]


def main(collision_objects: list[CollisionObject]):
    rclpy.init()
    node = CollisionObjectRegister(collision_objects)

    try:
        while node.not_removed_object_num > 0:
            rclpy.spin_once(node)
            node.remove()
            time.sleep(0.2)
        while node.not_registered_object_num > 0:
            rclpy.spin_once(node)
            node.regist()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
