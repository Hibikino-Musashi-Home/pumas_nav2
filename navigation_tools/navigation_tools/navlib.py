#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Optional, Union, Dict, Any

# import copy
import math

# import time

import numpy as np

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty, Float32MultiArray
from geometry_msgs.msg import (
    PoseStamped,
    Pose2D,
    # Pose,
    # PoseWithCovariance,
    PoseWithCovarianceStamped,
)

from tf_transformations import euler_from_quaternion, quaternion_from_euler

from pumas_interfaces.msg import StartAndEndJoints, Joints
from pumas_interfaces.srv import ParamReadWrite

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
        """_summary_

        Args:
            node (Optional[Union[str, Node]], optional): _description_. Defaults to None.

        Raises:
            TypeError: _description_
        """

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
            raise TypeError("NavModule.__init__: 'node' must be of type str or rclpy.node.Node")

        self.marker = Marker()
        self.marker_num = 0

        self.global_goal_reached = False
        self.goal_reached = False
        self.robot_stop = False

        self.motion_synth_start_pose = None
        self.motion_synth_end_pose = None

        # Publishers
        self.pub_marker = self._node.create_publisher(
            Marker,
            "/nav_goal_marker",
            10,
        )
        self.pub_global_goal = self._node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.pub_dist_angle = self._node.create_publisher(Float32MultiArray, "/simple_move/goal_dist_angle", 10)
        self.pub_robot_stop = self._node.create_publisher(Empty, "/navigation/stop", 10)
        self.pub_move_joint_pose = self._node.create_publisher(StartAndEndJoints, "/motion_synth/joint_pose", 10)

        # Subscribers
        self._node.create_subscription(GoalStatus, "/simple_move/goal_reached", self._callback_goal_reached, 10)
        self._node.create_subscription(GoalStatus, "/navigation/status", self._callback_global_goal_reached, 10)
        self._node.create_subscription(Empty, "/navigation/stop", self._callback_stop, 10)
        # self.create_subscription(PoseStamped, "/global_pose", self.global_pose_callback, 10)
        self._node.create_subscription(PoseWithCovarianceStamped, "/pose", self._global_pose_callback, 10)

        # Service Clients
        self.param_rw_client = self._node.create_client(ParamReadWrite, "/param_read_write")

        self._node.get_logger().info("NavModule.->initialized")

    def call_param_rw(
        self,
        node_name: str,
        param_name: str,
        param_value: str = "",
        write: bool = False,
    ) -> Optional[str]:
        """_summary_

        Args:
            node_name (str): _description_
            param_name (str): _description_
            param_value (str, optional): _description_. Defaults to "".
            write (bool, optional): _description_. Defaults to False.

        Returns:
            Optional[str]: _description_
        """
        req = ParamReadWrite.Request()
        req.node_name = node_name
        req.param_name = param_name
        req.write = write
        req.value = param_value

        future = self.param_rw_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is not None:
            return future.result().param_value
        else:
            self._node.get_logger().error("NavModule.->param_read_write service call failed")
            return None

    def _callback_goal_reached(self, msg: GoalStatus):
        self.goal_reached = False
        if msg.status == GoalStatus.SUCCEEDED:
            self.goal_reached = True

    def _callback_global_goal_reached(self, msg: GoalStatus):
        self.goal_reached = False
        if msg.status == GoalStatus.SUCCEEDED:
            self.global_goal_reached = True

    def _callback_stop(self, msg: Empty):
        self.robot_stop = True

    def _global_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.global_pose = msg
        self._node.get_logger().info(
            f"NavModule.->Global Pose: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}"
        )

    def pose_stamped2pose_2d(self, pose_stamped: PoseStamped) -> Pose2D:
        """_summary_

        Args:
            pose_stamped (geometry_msgs.msg.PoseStamped): _description_

        Returns:
            geometry_msgs.msg.Pose2D: _description_
        """
        pose2d = Pose2D()
        pose2d.x = pose_stamped.pose.position.x
        pose2d.y = pose_stamped.pose.position.y
        orientation = pose_stamped.pose.orientation
        euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        pose2d.theta = euler[2]
        return pose2d

    def create_arm_joint_goal(self, joint_poses: Dict[str, float]) -> Joints:
        """_summary_

        Args:
            joint_poses (Dict[str, float]): _description_

        Returns:
            pumas_interfaces.msg.Joints: _description_
        """
        joints = Joints()
        joints.arm_lift_joint = joint_poses["arm_lift_joint"]
        joints.arm_flex_joint = joint_poses["arm_flex_joint"]
        joints.arm_roll_joint = joint_poses["arm_roll_joint"]
        joints.wrist_flex_joint = joint_poses["wrist_flex_joint"]
        joints.wrist_roll_joint = joint_poses["wrist_roll_joint"]
        joints.head_pan_joint = joint_poses["head_pan_joint"]
        joints.head_tilt_joint = joint_poses["head_tilt_joint"]
        return joints

    def create_goal_pose(self, x: float, y: float, yaw: float, frame_id: float) -> PoseStamped:
        """_summary_

        Args:
            x (float): _description_
            y (float): _description_
            yaw (float): _description_
            frame_id (float): _description_

        Returns:
            geometry_msgs.msg.PoseStamped: _description_
        """
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

    def _send_goal(self, goal: PoseStamped):

        self._node.get_logger().info("NavModule.->Sending Nav Goal")

        if self.motion_synth_start_pose is not None or self.motion_synth_end_pose is not None:

            start_and_end_joints = StartAndEndJoints()
            start_and_end_joints.has_arm_start_pose = False
            start_and_end_joints.has_arm_end_pose = False

            if self.motion_synth_start_pose is not None:
                start_and_end_joints.has_arm_start_pose = True
                start_and_end_joints.start_pose = self.create_arm_joint_goal(joint_poses=self.motion_synth_start_pose)
            else:
                start_and_end_joints.has_arm_start_pose = True
                # startが無いなら自動でgo_pose()代入
                start_and_end_joints.start_pose = self.create_arm_joint_goal(joint_poses=default_arm_pose)

            if self.motion_synth_end_pose is not None:
                start_and_end_joints.has_arm_end_pose = True
                start_and_end_joints.end_pose = self.create_arm_joint_goal(joint_poses=self.motion_synth_end_pose)
            else:
                # goalが無い場合
                start_and_end_joints.has_arm_end_pose = False

            self.pub_move_joint_pose.publish(start_and_end_joints)

            start_and_end_joints.has_arm_start_pose = False
            start_and_end_joints.has_arm_end_pose = False

        self.marker_plot(goal)
        self.pub_global_goal.publish(goal)

    def handle_robot_stop(self):
        """_summary_
        """
        if not self.global_goal_reached:
            msg_stop = Empty()
            self.pub_robot_stop.publish(msg_stop)

    def marker_plot(self, goal: PoseStamped):
        """_summary_

        Args:
            goal (geometry_msgs.msg.PoseStamped): _description_
        """
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = self._node.get_clock().now().to_msg()
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

    def go_abs(self, goal: Pose2D, timeout: float, goal_distance: Optional[float] = None) -> bool:
        """_summary_

        Args:
            goal (geometry_msgs.msg.Pose2D): _description_
            timeout (float): _description_
            goal_distance (Optional[float], optional): _description_. Defaults to None.

        Returns:
            bool: _description_
        """

        goal_pose = self.create_goal_pose(goal.x, goal.y, goal.theta, "map")

        self.global_goal_reached = False
        self.robot_stop = False

        # NOTE: replace loop-count timeout with ROS time-based timeout
        start_time = self._node.get_clock().now()
        timeout_duration = rclpy.duration.Duration(seconds=timeout) if timeout > 0 else None

        self._send_goal(goal_pose)  # send nav goal

        executor = SingleThreadedExecutor()
        executor.add_node(self._node)

        result = False

        # NOTE: replace loop-count timeout with ROS time-based timeout
        # check goal reached or stop signal
        while rclpy.ok() and not self.global_goal_reached and not self.robot_stop:

            if timeout_duration is not None:
                elapsed = self._node.get_clock().now().nanoseconds - start_time.nanoseconds
                if elapsed > timeout_duration.nanoseconds:
                    break

            if goal_distance:
                current_x = self.global_pose.pose.pose.position.x
                current_y = self.global_pose.pose.pose.position.y
                current_distance = math.sqrt((goal.x - current_x) ** 2 + (goal.y - current_y) ** 2)
                if current_distance < goal_distance:
                    result = True
                    break

            executor.spin_once(timeout_sec=0.1)

        # NOTE: Stop managing this node callbacks.
        executor.remove_node(self._node)

        if self.global_goal_reached:
            result = True
        elif self.robot_stop:
            self._node.get_logger().info("NavModule.->Nav Signal Stop")
            result = False
        else:
            self._node.get_logger().warn("NavModule.->Nav Failed")
            result = False

        self.handle_robot_stop()

        return result

    def nav_goal(
        self,
        goal: Pose2D,
        timeout: float,
        motion_synth_pose: Optional[Dict[str, Any]] = None,
        goal_distance: Optional[float] = None,
    ) -> bool:
        """_summary_

        Args:
            goal (geometry_msgs.msg.Pose2D): _description_
            timeout (float): _description_
            motion_synth_pose (Optional[Dict[str, Any]], optional): _description_. Defaults to None.
            goal_distance (Optional[float], optional): _description_. Defaults to None.

        Returns:
            bool: _description_
        """
        self.motion_synth_start_pose = None
        self.motion_synth_end_pose = None

        if motion_synth_pose is not None:

            self._node.get_logger().info("NavModule.->Motion Synth Nav Goal with Pose Config")

            self.call_param_rw(
                node_name="potential_fields",
                param_name="use_point_cloud",
                param_value="false",
                write=True,
            )
            self.call_param_rw(
                node_name="map_augmenter",
                param_name="use_point_cloud",
                param_value="false",
                write=True,
            )

            if "start" in motion_synth_pose:
                self.motion_synth_start_pose = motion_synth_pose["start"]
            if "goal" in motion_synth_pose:
                self.motion_synth_end_pose = motion_synth_pose["goal"]

        elif motion_synth_pose is None:

            self._node.get_logger().info("NavModule.->Standard Nav Goal")

            self.call_param_rw(
                node_name="potential_fields",
                param_name="use_point_cloud",
                param_value="true",
                write=True,
            )
            self.call_param_rw(
                node_name="map_augmenter",
                param_name="use_point_cloud",
                param_value="true",
                write=True,
            )

        return self.go_abs(goal, timeout, goal_distance)

    def destory(self):
        """_summary_
        """
        try:
            if not self._external_node:
                self._node.destroy_node()
        except Exception:
            pass


if __name__ == "__main__":
    rclpy.init()
    nav = NavModule()

    goal = Pose2D(x=1.0, y=3.7, theta=0.0)
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

    # success = nav.go_abs(goal, timeout=0, goal_distance=0)
    # success = nav.nav_goal(goal, motion_synth_pose=ms_config, timeout=0, goal_distance=0)
    success = nav.nav_goal(goal, motion_synth_pose=None, timeout=0, goal_distance=0)

    if success:
        nav._node.get_logger().info("NavStatus.->Nav Goal Reached")
    else:
        nav._node.get_logger().warn("NavStatus.->Failed to Reach Goal")

    nav.destory()
    if rclpy.ok():
        rclpy.shutdown()
