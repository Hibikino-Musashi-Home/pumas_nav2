#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class EnablePublisher(Node):
    def __init__(self):
        super().__init__("enable_publisher")

        pub = self.create_publisher(Bool, '/navigation/potential_fields/enable', 10)
        msg = Bool(data=True)
        pub.publish(msg)

        self.get_logger().info('Published enable message')


def main(args=None):
    rclpy.init(args=args)
    node = EnablePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
