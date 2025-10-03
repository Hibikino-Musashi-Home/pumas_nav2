#!/usr/bin/env python3

import copy
import asyncio
import numpy as np

import tf_transformations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float32

from pumas_interfaces.action import MotionSynthesis


class MotionSynth(Node):
    def __init__(self):
        super().__init__("motion_synth_server")

        self._action_server = ActionServer(
            self,
            MotionSynthesis,
            "/motion_synth/joint_goal",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.arm_pub = self.create_publisher(Float32MultiArray, '/hardware/arm/goal_pose', 10)
        self.lift_pub = self.create_publisher(Float32, '/hardware/torso/goal_pose', 10)
        self.head_pub = self.create_publisher(Float32MultiArray, '/hardware/head/goal_pose', 10)

        self.current_pose = None
        self.global_nav_goal_reached = False
        self.path_received = False
        self.path_points = []

        self.create_subscription(PoseStamped, '/global_pose', self.global_pose_callback, 10)
        self.create_subscription(GoalStatus, '/navigation/status', self.navigation_status_callback, 10)
        self.create_subscription(Path, '/simple_move/goal_path', self.path_callback, 10)

        self.get_logger().info('motion_synth_server.-> is ready')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def path_callback(self, msg: Path):
        if not msg.poses:
            return
        self.path_points = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self.path_received = True
        self.get_logger().info(f"motion_synth -> Received Path Length: {len(self.path_points)}")

    def global_pose_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        yaw = tf_transformations.euler_from_quaternion(q)[2]
        self.current_pose = (position.x, position.y, yaw)

    def navigation_status_callback(self, msg):
        if msg.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('motion_synth_server.-> Global Nav Goal Reached')
            self.global_nav_goal_reached = True

    def send_pose(self, joints):
        lift_msg = Float32(data=joints.arm_lift_joint)
        arm_msg = Float32MultiArray(data=[
            joints.arm_flex_joint,
            joints.arm_roll_joint,
            joints.wrist_flex_joint,
            joints.wrist_roll_joint,
        ])
        head_msg = Float32MultiArray(data=[
            joints.head_pan_joint,
            joints.head_tilt_joint,
        ])
        self.lift_pub.publish(lift_msg)
        self.arm_pub.publish(arm_msg)
        self.head_pub.publish(head_msg)

    def check_self_collision_risk(self, goal_pose):
        if goal_pose.arm_flex_joint < -1.0:
            return True
        elif goal_pose.arm_flex_joint > -0.35 and goal_pose.arm_lift_joint > 0.15:
            return True
        return False

    def create_temporary_pose(self, goal_pose):
        tmp = copy.deepcopy(goal_pose)
        if goal_pose.arm_flex_joint < -1.0:
            tmp.arm_flex_joint = -0.6
        elif goal_pose.arm_flex_joint > -0.35 and goal_pose.arm_lift_joint > 0.15:
            tmp.arm_flex_joint = -0.60
        return tmp

    def joint_goal_reached(self, goal_joints, threshold=0.5):
        # TODO: Replace with real check
        return True

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing motion_synth goal...')
        goal = goal_handle.request
        feedback = MotionSynthesis.Feedback()

        # 初期ポーズ送信（必要に応じて）
        if goal.apply_start_pose:
            self.send_pose(goal.start_pose)

        # 経路取得を待つ（最大5秒）
        for i in range(50):
            if self.path_received:
                break
            await asyncio.sleep(0.1)
        if not self.path_received or not self.path_points:
            self.get_logger().warn("No path received, aborting.")
            goal_handle.abort()
            return MotionSynthesis.Result(result=False)

        arm_start_index = int(len(self.path_points) * 0.75)
        triggered = False
        temporary_pose = None
        temporary_pose_sent = False
        final_pose_sent = False
        collision_risk = self.check_self_collision_risk(goal.goal_pose)
        if collision_risk:
            self.get_logger().warn("Detected self-collision risk. Using temporary pose.")
            temporary_pose = self.create_temporary_pose(goal.goal_pose)

        rate = self.create_rate(10)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled.")
                goal_handle.canceled()
                return MotionSynthesis.Result(result=False)

            if not triggered and self.current_pose:
                cx, cy, _ = self.current_pose
                tx, ty = self.path_points[arm_start_index]
                dist = np.linalg.norm([cx - tx, cy - ty])
                if dist < 1.0:
                    self.get_logger().info("Reached 3/4 of path. Triggering arm motion.")
                    if collision_risk:
                        self.send_pose(temporary_pose)
                        temporary_pose_sent = True
                    else:
                        self.send_pose(goal.goal_pose)
                        final_pose_sent = True
                    triggered = True

            if triggered:
                if temporary_pose_sent and not final_pose_sent:
                    yaw_error = abs(self.current_pose[2] - goal.goal_location.theta)
                    if yaw_error > np.pi:
                        yaw_error = 2 * np.pi - yaw_error
                    if yaw_error < 0.3 or self.global_nav_goal_reached:
                        self.send_pose(goal.goal_pose)
                        final_pose_sent = True
                        self.global_nav_goal_reached = False

            if triggered and final_pose_sent:
                if self.joint_goal_reached(goal.goal_pose):
                    self.get_logger().info("Final arm pose reached.")
                    goal_handle.succeed()
                    return MotionSynthesis.Result(result=True)

            goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = MotionSynth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

