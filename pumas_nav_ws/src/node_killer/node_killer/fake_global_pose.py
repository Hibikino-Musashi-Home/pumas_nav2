#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from geometry_msgs.msg import Point, Quaternion

from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import traceback

class FakeGlobalPoseNode(Node):
    def __init__(self):
        super().__init__("fake_global_pose")

        # Internal state
        self.global_pose_ = PoseStamped()
        self.global_pose_.header.frame_id = "map"

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Subscribers
        self.subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, 
            "/amcl_pose",
            self.callback_amcl, 
            10
        )

        # Publisher
        self.pub_global_ = self.create_publisher(PoseStamped, "/global_pose", 10)

        #Processing
        self.processing_timer_ = self.create_timer(0.01, self.fake_global_pose) 

        self.get_logger().info("Starting fake_global_pose application in python...")

    def callback_amcl(self, msg):
        try:
            self.global_pose_.pose = msg.pose.pose

        except:
            self.get_logger().error(traceback.format_exc())

    def fake_global_pose(self):
        try:
            now = self.get_clock().now().to_msg()
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                #self.get_logger().warn(f"TF lookup failed: {e}")
                return

            self.global_pose_.header.stamp = now
            self.global_pose_.header.frame_id = 'map'

            self.global_pose_.pose.position = Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z
            )

            self.global_pose_.pose.orientation = Quaternion(
                x=transform.transform.rotation.x,
                y=transform.transform.rotation.y,
                z=transform.transform.rotation.z,
                w=transform.transform.rotation.w
            )

        except Exception as e:
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = FakeGlobalPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
