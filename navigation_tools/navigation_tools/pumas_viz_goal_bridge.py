#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from navigation_tools.navlib import outcome_name
from pumas_interfaces.action import PumasNav
from pumas_interfaces.msg import StartAndEndJoints
from rclpy.action import ActionClient
from rclpy.node import Node


class RvizGoalBridge(Node):
    """Forwards RViz "2D Nav Goal" clicks to the /pumas_nav action.

    A new click always replaces whatever is running: mvn_pln preempts the
    active goal in its handle_accepted, so this node just sends and lets the
    server drop the old one. Callbacks of the goal that was replaced are
    ignored here via a generation counter -- otherwise a preempted goal's
    result (an abort carrying OUTCOME_PREEMPTED) would be logged as if the
    goal the user just clicked had failed.
    """

    def __init__(self):
        super().__init__('rviz_goal_bridge')
        self.cli = ActionClient(self, PumasNav, '/pumas_nav')

        # Incremented per click; every callback carries the value it was
        # registered under and bails out once it no longer matches.
        self._generation = 0

        self.sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.cb_goal,
            10,
        )

        self.get_logger().info('RvizGoalBridge started')

    def _is_stale(self, generation: int) -> bool:
        return generation != self._generation

    def cb_goal(self, msg: PoseStamped):
        self.get_logger().info(
            f'Received RViz goal: x={msg.pose.position.x}, y={msg.pose.position.y}'
        )

        if not self.cli.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('PumasNav action server not available')
            return

        self._generation += 1
        generation = self._generation

        goal = PumasNav.Goal()
        goal.goal = msg
        goal.arm_joints = StartAndEndJoints()
        goal.use_arm = False
        # goal.patience = True
        # goal.proximity_criterion = 2.0

        future = self.cli.send_goal_async(
            goal,
            feedback_callback=lambda fb: self.feedback_callback(
                fb, generation),
        )
        future.add_done_callback(
            lambda f: self.goal_response_callback(f, generation))

    def goal_response_callback(self, future, generation: int):
        goal_handle = future.result()

        if self._is_stale(generation):
            # Another click landed before the server answered this one. Cancel
            # the orphan so it cannot drive alongside the newer goal.
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('PumasNav goal rejected')
            return

        self.get_logger().info('PumasNav goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self.result_callback(f, generation))

    def feedback_callback(self, feedback_msg, generation: int):
        if self._is_stale(generation):
            return

        # logger disabled by ry0hei.kobayashi
        # fb = feedback_msg.feedback
        # self.get_logger().info(
        #    f"feedback: state={fb.state_name}, "
        #    f"dist={fb.remaining_distance:.3f}, "
        #    f"near={fb.near_goal_reached}, "
        #    f"msg={fb.message}"
        # )

    def result_callback(self, future, generation: int):
        result = future.result().result

        # if self._is_stale(generation):
        #    self.get_logger().info(
        #        f'superseded goal finished: '
        #        f'outcome={outcome_name(result.outcome)}, '
        #        f'message={result.message}'
        #    )
        #    return

        # self.get_logger().info(
        #    f'result: success={result.success}, '
        #    f'outcome={outcome_name(result.outcome)}, '
        #    f'near_goal_reached={result.near_goal_reached}, '
        #    f'message={result.message}'
        # )


def main():
    rclpy.init()
    node = RvizGoalBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
