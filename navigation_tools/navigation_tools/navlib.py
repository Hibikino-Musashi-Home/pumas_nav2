#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Union

import numpy as np
import rclpy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped
from pumas_interfaces.msg import Joints, StartAndEndJoints
from pumas_interfaces.srv import ParamReadWrite
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, Float32MultiArray
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

default_arm_pose = {
    "arm_flex_joint": -0.26,  # default is 0.0
    "arm_lift_joint": 0.0,
    "arm_roll_joint": -1.57,
    "wrist_flex_joint": -1.57,
    "wrist_roll_joint": 0.0,
    "head_pan_joint": 0.0,
    "head_tilt_joint": np.deg2rad(0.0),
}


class NavModule:
    """Navigation Module for the robot"""

    def __init__(self, node: Optional[Union[str, Node]] = None):

        context = rclpy.get_default_context()
        if not context.ok():
            rclpy.init()

        if node is None:
            self._node: Node = rclpy.create_node("nav_module_node")
            self._external_node = False
        elif isinstance(node, str):
            self._node: Node = rclpy.create_node(node)
            self._external_node = False
        elif isinstance(node, Node):
            self._node: Node = node
            self._external_node = True
        else:
            raise TypeError(
                "NavModule.__init__: 'node' must be of type str or rclpy.node.Node"
            )

        self.marker = Marker()
        self.marker_num = 0

        self.global_goal_reached = False
        self.goal_reached = False
        self.robot_stop = False

        self.motion_synth_start_pose = None
        self.motion_synth_end_pose = None

        # Publishers
        self.pub_marker = self.create_publisher(Marker, "/nav_goal_marker", 10)
        self.pub_global_goal = self.create_publisher(
            PoseStamped, "/move_base_simple/goal", 10
        )
        self.pub_dist_angle = self.create_publisher(
            Float32MultiArray, "/simple_move/goal_dist_angle", 10
        )
        self.pub_robot_stop = self.create_publisher(
            Empty, "/navigation/stop", 10)
        self.pub_move_joint_pose = self.create_publisher(
            StartAndEndJoints, "/motion_synth/joint_pose", 10
        )

        # Subscribers
        self.create_subscription(
            GoalStatus, "/simple_move/goal_reached", self.callback_goal_reached, 10
        )
        self.create_subscription(
            GoalStatus, "/navigation/status", self.callback_global_goal_reached, 10
        )
        self.create_subscription(
            Empty, "/stop", self.callback_stop, 10)

        # Service Clients
        self.param_rw_client = self.create_client(
            ParamReadWrite, "/param_read_write")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self._node)

        self.get_logger().info("NavModule.->initialized")

    def __getattr__(self, name):
        return getattr(self._node, name)

    def call_param_rw(
        self, node_name, param_name, param_value: str = "", write: bool = False
    ):
        req = ParamReadWrite.Request()
        req.node_name = node_name
        req.param_name = param_name
        req.write = write
        req.value = param_value

        future = self.param_rw_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().param_value
        else:
            self.get_logger().error("NavModule.->param_read_write service call failed")
            return None

    def callback_goal_reached(self, msg):
        self.goal_reached = False
        if msg.status == GoalStatus.SUCCEEDED:
            self.goal_reached = True

    def callback_global_goal_reached(self, msg):
        if msg.status == GoalStatus.SUCCEEDED:
            self.global_goal_reached = True

    def callback_stop(self, msg):
        self.robot_stop = True

    def get_global_pose_from_tf(self, target_frame="map", source_frame="base_footprint"):
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw = euler[2]

            # self.get_logger().info(
            #    f"NavModule.->TF Pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            # )
            return x, y, yaw

        except TransformException as ex:
            self.get_logger().warn(
                f"NavModule.->TF lookup failed: {target_frame} -> {source_frame}: {ex}"
            )
            return None

    def pose_stamped2pose_2d(self, pose_stamped):
        pose2d = Pose2D()
        pose2d.x = pose_stamped.pose.position.x
        pose2d.y = pose_stamped.pose.position.y
        orientation = pose_stamped.pose.orientation
        euler = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        pose2d.theta = euler[2]
        return pose2d

    def create_arm_joint_goal(self, joint_poses):
        joints = Joints()
        joints.arm_lift_joint = joint_poses["arm_lift_joint"]
        joints.arm_flex_joint = joint_poses["arm_flex_joint"]
        joints.arm_roll_joint = joint_poses["arm_roll_joint"]
        joints.wrist_flex_joint = joint_poses["wrist_flex_joint"]
        joints.wrist_roll_joint = joint_poses["wrist_roll_joint"]
        joints.head_pan_joint = joint_poses["head_pan_joint"]
        joints.head_tilt_joint = joint_poses["head_tilt_joint"]
        return joints

    def create_goal_pose(self, x, y, yaw, frame_id):
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        return goal

    def send_goal(self, goal):

        self.get_logger().info("NavModule.->Sending Nav Goal")

        if (
            self.motion_synth_start_pose is not None
            or self.motion_synth_end_pose is not None
        ):

            start_and_end_joints = StartAndEndJoints()
            start_and_end_joints.has_arm_start_pose = False
            start_and_end_joints.has_arm_end_pose = False

            if self.motion_synth_start_pose is not None:
                start_and_end_joints.has_arm_start_pose = True
                start_and_end_joints.start_pose = self.create_arm_joint_goal(
                    joint_poses=self.motion_synth_start_pose
                )
            else:
                start_and_end_joints.has_arm_start_pose = True
                # startが無いなら自動でgo_pose()代入
                start_and_end_joints.start_pose = self.create_arm_joint_goal(
                    joint_poses=default_arm_pose
                )

            if self.motion_synth_end_pose is not None:
                start_and_end_joints.has_arm_end_pose = True
                start_and_end_joints.end_pose = self.create_arm_joint_goal(
                    joint_poses=self.motion_synth_end_pose
                )
            else:
                # goalが無い場合
                start_and_end_joints.has_arm_end_pose = False

            self.pub_move_joint_pose.publish(start_and_end_joints)

        self.get_logger().info(
            f"NavModule.->Publishing Global Goal: x={goal.pose.position.x}, y={goal.pose.position.y}"
        )
        self.marker_plot(goal)
        self.pub_global_goal.publish(goal)

    def handle_robot_stop(self):
        if not self.global_goal_reached:
            msg_stop = Empty()
            self.pub_robot_stop.publish(msg_stop)

    def marker_plot(self, goal):
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "goal_markers"
        self.marker.id = self.marker_num
        self.marker_num += 1
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose = goal.pose
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.pub_marker.publish(self.marker)

    def initialize_before_new_goal(self, send_stop=True):
        self.global_goal_reached = False
        self.goal_reached = False
        self.robot_stop = False

        if send_stop:
            self.pub_robot_stop.publish(Empty())
        self.motion_synth_start_pose = None
        self.motion_synth_end_pose = None

    def go_abs(self, goal: Pose2D, timeout, goal_distance=None, keep_motion_synth=False) -> bool:
        self.get_logger().info(
            f"NavModule.->Go Absolute Goal: x={goal.x}, y={goal.y}, theta={goal.theta}"
        )

        self.global_goal_reached = False
        self.goal_reached = False
        self.robot_stop = False

        if not keep_motion_synth:
            self.motion_synth_start_pose = None
            self.motion_synth_end_pose = None

        current_pose = self.get_global_pose_from_tf("map", "base_footprint")
        if current_pose is not None:
            current_x, current_y, _ = current_pose
            current_distance = math.sqrt(
                (goal.x - current_x) ** 2 + (goal.y - current_y) ** 2
            )
            if current_distance <= 0.05:
                self.get_logger().info("NavModule.->Already at goal")
                return True

        attempts = int(timeout * 10) if timeout != 0 else float("inf")
        goal_pose = self.create_goal_pose(goal.x, goal.y, goal.theta, "map")
        self.send_goal(goal_pose)

        executor = SingleThreadedExecutor()
        executor.add_node(self._node)

        result = False

        while (
            not self.global_goal_reached
            and rclpy.ok()
            and not self.robot_stop
            and attempts >= 0
        ):
            if goal_distance is not None:
                current_pose = self.get_global_pose_from_tf(
                    "map", "base_footprint")
                if current_pose is not None:
                    current_x, current_y, _ = current_pose
                    current_distance = math.sqrt(
                        (goal.x - current_x) ** 2 + (goal.y - current_y) ** 2
                    )
                    if current_distance <= goal_distance:
                        result = True
                        break

            attempts -= 1
            executor.spin_once(timeout_sec=0.1)

        if self.global_goal_reached:
            result = True
        elif self.robot_stop:
            result = False
        else:
            result = False

        if not result:
            self.handle_robot_stop()

        return result

    def set_use_point_cloud(self, enabled: bool):
        value = "true" if enabled else "false"

        self.call_param_rw(
            node_name="potential_fields",
            param_name="use_point_cloud",
            param_value=value,
            write=True,
        )
        self.call_param_rw(
            node_name="map_augmenter",
            param_name="use_point_cloud",
            param_value=value,
            write=True,
        )

        self.get_logger().info(
            f"NavModule.->use_point_cloud set to {value}"
        )

    def nav_goal(self, goal, timeout, motion_synth_pose=None, goal_distance=None, use_point_cloud=True):

        self.initialize_before_new_goal(send_stop=True)

        if motion_synth_pose is not None:

            self.get_logger().info("NavModule.->Motion Synth Nav Goal with Pose Config")
            self.set_use_point_cloud(False)

            if "start" in motion_synth_pose:
                self.motion_synth_start_pose = motion_synth_pose["start"]
            if "goal" in motion_synth_pose:
                self.motion_synth_end_pose = motion_synth_pose["goal"]

        else:
            self.get_logger().info("NavModule.->Standard Nav Goal")
            self.set_use_point_cloud(use_point_cloud)

        return self.go_abs(goal, timeout, goal_distance, keep_motion_synth=True)


if __name__ == "__main__":
    rclpy.init()
    nav = NavModule()

    # goal = Pose2D(x=1.0, y=3.7, theta=0.0)
    # goal = Pose2D(x=0.8, y=3.44, theta=0.0)
    start_pose = {
        "arm_lift_joint": 0.0,
        "arm_flex_joint": np.deg2rad(0.0),
        "arm_roll_joint": np.deg2rad(0.0),
        "wrist_flex_joint": np.deg2rad(-90.0),
        "wrist_roll_joint": 0.0,
        "head_pan_joint": 0.0,
        "head_tilt_joint": np.deg2rad(0.0),
    }
    goal_pose = {
        "arm_lift_joint": 0.4,
        "arm_flex_joint": np.deg2rad(-90.0),
        "arm_roll_joint": np.deg2rad(0.0),
        "wrist_flex_joint": np.deg2rad(-90.0),
        "wrist_roll_joint": 0.0,
        "head_pan_joint": 0.0,
        "head_tilt_joint": np.deg2rad(0.0),
    }
    ms_config = {
        "start": start_pose,
        "goal": goal_pose,
    }

    goal = Pose2D(x=0.0, y=0.0, theta=0.0)
    success = nav.nav_goal(goal, motion_synth_pose=None,
                           timeout=0, goal_distance=None, use_point_cloud=True)

    success = nav.nav_goal(goal, motion_synth_pose=None,
                           timeout=0, goal_distance=None, use_point_cloud=True)

    goal = Pose2D(x=0.0, y=0.0, theta=0.0)
    success = nav.nav_goal(goal, motion_synth_pose=None,
                           timeout=0, goal_distance=None, use_point_cloud=True)

    if success:
        nav.get_logger().info("NavStatus.->Nav Goal Reached")
    else:
        nav.get_logger().warn("NavStatus.->Failed to Reach Goal")

    nav.get_logger().warn("all complete")

    # for _ in range(10):
    #    success = nav.nav_goal(goal, motion_synth_pose=None, timeout=0, goal_distance=0)

    #    if success:
    #        nav.get_logger().info("NavStatus.->Nav Goal Reached")
    #    else:
    #        nav.get_logger().warn("NavStatus.->Failed to Reach Goal")

    #    time.sleep(2)
