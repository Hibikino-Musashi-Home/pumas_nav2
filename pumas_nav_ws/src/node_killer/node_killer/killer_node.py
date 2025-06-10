#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import subprocess as sub
import traceback
import time


class PoseIntegratorKillerNode(Node):
    def __init__(self):
        super().__init__("pose_integrator_killer_node")

        # Processing timer: check every 5 seconds
        self.processing_timer_ = self.create_timer(5.0, self.pose_integrator_killer)

        self.get_logger().info("Starting pose_integrator_killer_node application in python...")


    def pose_integrator_killer(self):
        try:
            node_names = self.get_node_names()
            if '/pose_integrator' in node_names:
                self.get_logger().info("Detected /pose_integrator node!")

                # Insert custom shutdown logic here, like sending a shutdown signal
                # It is here for compatibility 
            else:
                self.get_logger().info("/pose_integrator not running.")

        except Exception:
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = PoseIntegratorKillerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
