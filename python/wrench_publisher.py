#!/usr/bin/python3
#!python

import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class WrenchPublisher(Node):
    def __init__(self, attractor_position, franka):
        super().__init__("pose_node")

        self.attractor_position = attractor_position
        self.franka = franka

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.wrench_publisher = self.create_publisher(WrenchStamped, "/wrench", 5)
        self.wrench_object = WrenchStamped()
        self.frame_id = "_frankalink8"

        self.wrench_object.header.frame_id = self.frame_id

        ee_pos = franka.get_end_effector_position()
        breakpoint()
        unit_wrench_vec = attractor_position - ee_pos
        print(wrench_vec)
        norm = np.linalg.norm(unit_wrench_vec)

        if norm:
            unit_wrench_vec = unit_wrench_vec / norm

        self.wrench_object.wrench.force.x = unit_wrench_vec[0]
        self.wrench_object.wrench.force.y = unit_wrench_vec[1]
        self.wrench_object.wrench.force.z = unit_wrench_vec[2]

    def timer_callback(self):
        print("4. WRENCH ")
        self.wrench_publisher.publish(self.wrench_object)


def main(args=None):
    rclpy.init(args=args)
    wrench_basic = AttractorPublisher()
    rclpy.spin(wrench_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
