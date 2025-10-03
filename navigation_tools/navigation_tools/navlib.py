#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import copy
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Pose2D, Pose, PoseWithCovariance, PoseWithCovarianceStamped

from tf_transformations import euler_from_quaternion, quaternion_from_euler

from pumas_interfaces.msg import StartAndEndJoints, Joints

default_arm_pose = {
               'arm_flex_joint': -0.26, #default is 0.0
               'arm_lift_joint': 0.0, 
               'arm_roll_joint': -1.57,
               'wrist_flex_joint': -1.57,
               'wrist_roll_joint': 0.0,
               'head_pan_joint': 0.0,
               'head_tilt_joint': np.deg2rad(0.0),}

class NavModule(Node):
    """Navigation Module for the robot"""
    __instance = None

    def __new__(cls, *args, **kargs):
        if cls.__instance is None:
            cls.__instance = super(NavModule, cls).__new__(cls)
            cls.__initialized = False
        return cls.__instance

    def __init__(self):

        super().__init__("nav_module")
        
        self.marker = Marker()
        self.marker_num = 0

        self.global_goal_reached = False
        self.goal_reached = False
        self.robot_stop = False

        # Publishers
        self.pub_marker = self.create_publisher(Marker, "/nav_goal_marker", 10)
        self.pub_global_goal = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.pub_dist_angle = self.create_publisher(Float32MultiArray, "/simple_move/goal_dist_angle", 10)
        self.pub_robot_stop = self.create_publisher(Empty, "/navigation/stop", 10)
        self.pub_move_joint_pose = self.create_publisher(StartAndEndJoints, "/motion_synth/joint_pose", 10)

        # Subscribers
        self.create_subscription(GoalStatus, "/simple_move/goal_reached", self.callback_goal_reached, 10)
        self.create_subscription(GoalStatus, "/navigation/status", self.callback_global_goal_reached, 10)
        self.create_subscription(Empty, "/navigation/stop", self.callback_stop, 10)
        #self.create_subscription(PoseStamped, "/global_pose", self.global_pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.global_pose_callback, 10)

        time.sleep(1.0)
        self.get_logger().info("NavModule.->initialized")

    def callback_goal_reached(self, msg):
        self.goal_reached = False
        if msg.status == GoalStatus.SUCCEEDED:
            self.goal_reached = True

    def callback_global_goal_reached(self, msg):
        self.goal_reached = False
        if msg.status == GoalStatus.SUCCEEDED:
            self.global_goal_reached = True

    def callback_stop(self, msg):
        self.robot_stop = True

    def global_pose_callback(self, msg):
        self.global_pose = msg
        self.get_logger().info(f"NavModule.->Global Pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")

    def pose_stamped2pose_2d(self, pose_stamped):
        pose2d = Pose2D()
        pose2d.x = pose_stamped.pose.position.x
        pose2d.y = pose_stamped.pose.position.y
        orientation = pose_stamped.pose.orientation
        euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
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

        self.get_logger().info('NavModule.->Sending Nav Goal')

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

    def go_abs(self, goal: Pose2D, timeout, goal_distance=None) -> bool:

        goal_pose = self.create_goal_pose(goal.x, goal.y, goal.theta, "map")

        self.global_goal_reached = False
        self.robot_stop = False
        attempts = int(timeout * 10) if timeout != 0 else float('inf')

        self.send_goal(goal_pose) # send nav goal

        executor = SingleThreadedExecutor()
        executor.add_node(self)

        result = False

        while not self.global_goal_reached and rclpy.ok() and not self.robot_stop and attempts >= 0: # check goal reached or stop signal
            if goal_distance:
                current_x, current_y = self.global_pose.pose.position.x, self.global_pose.pose.position.y
                current_distance = math.sqrt((goal.x - current_x) ** 2 + (goal.y - current_y) ** 2)
                if current_distance < goal_distance:
                    result = True
                    break

            attempts -= 1
            executor.spin_once(timeout_sec=0.1)

        if self.global_goal_reached:
            result = True
        elif self.robot_stop:
            self.get_logger().info('NavModule.->Nav Signal Stop')
            result = False
        else:
            self.get_logger().warn('NavModule.->Nav Failed')
            result = False

        self.handle_robot_stop()

        return result

    def nav_goal(self, goal, timeout, motion_synth_pose=None, goal_distance=None):
        self.motion_synth_start_pose = None
        self.motion_synth_end_pose = None

        if motion_synth_pose is not None:
            if "start" in motion_synth_pose:
                self.motion_synth_start_pose = motion_synth_pose["start"]
            if "goal" in motion_synth_pose:
                self.motion_synth_end_pose = motion_synth_pose["goal"]

        return self.go_abs(goal, timeout, goal_distance)


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
    
    #success = nav.go_abs(goal, timeout=0, goal_distance=0)
    success = nav.nav_goal(goal, motion_synth_pose=ms_config, timeout=0, goal_distance=0)

    if success:
        nav.get_logger().info("NavStatus.->Nav Goal Reached")
    else:
        nav.get_logger().warn("NavStatus.->Failed to Reach Goal")
    

