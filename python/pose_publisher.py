#!/usr/bin/python3
#!python

import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped


class PosePublisher(Node):
    def __init__(self):
        super().__init__("pose_node")

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pose_publisher = self.create_publisher(PoseStamped, "/pose", 5)
        self.pose_object = PoseStamped()
        self.frame_id = "_frankalink8"

        self.pose_object.header.frame_id = self.frame_id

        self.pose_object.pose.position.x = 0.0
        self.pose_object.pose.position.y = 0.0
        self.pose_object.pose.position.z = 0.0

        print(self.pose_object)

    def timer_callback(self):
        print("4. POSE ")
        self.pose_publisher.publish(self.pose_object)

        # print(self.obstacles_array.markers)


def main(args=None):
    rclpy.init(args=args)
    pose_basic = PosePublisher()
    rclpy.spin(pose_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
