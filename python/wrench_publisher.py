#!/usr/bin/python3
#!python

import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class WrenchPublisher(Node):
    def __init__(self):
        super().__init__("wrench_node")

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.wrench_publisher = self.create_publisher(WrenchStamped, "/wrench", 5)
        self.wrench_object = WrenchStamped()
        self.frame_id = "_frankalink8"

        self.wrench_object.header.frame_id = self.frame_id

        self.wrench_object.wrench.force.x = 1.0
        self.wrench_object.wrench.force.y = 1.0
        self.wrench_object.wrench.force.z = 0.0

    def timer_callback(self):
        # print("4. WRENCH ")
        self.wrench_publisher.publish(self.wrench_object)


def main(args=None):
    rclpy.init(args=args)
    wrench_basic = AttractorPublisher()
    rclpy.spin(wrench_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
