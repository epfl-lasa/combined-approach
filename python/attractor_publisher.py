#!/usr/bin/python3
#!python

import rclpy
import math
import numpy as np

from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class AttractorPublisher(Node):
    def __init__(self, pos):
        super().__init__("pose_node")

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.attractor_publisher = self.create_publisher(Marker, "/attractor", 5)
        self.attractor_object = Marker()
        self.frame_id = "world"

        self.attractor_object.header.frame_id = self.frame_id

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.attractor_pos = pos
        self.attractor_object.type = Marker.CUBE
        self.attractor_object.pose.position.x = pos[0]
        self.attractor_object.pose.position.y = pos[1]
        self.attractor_object.pose.position.z = pos[2]

        # self.attractor_object.pose.orientation.x = 0.402
        # self.attractor_object.pose.orientation.y = 0.191
        # self.attractor_object.pose.orientation.z = 0.462
        # self.attractor_object.pose.orientation.w = 0.733

        self.attractor_object.pose.orientation.x = 0.5
        self.attractor_object.pose.orientation.y = 0.5
        self.attractor_object.pose.orientation.z = 0.5
        self.attractor_object.pose.orientation.w = 0.5

        self.cube_length = 0.05
        self.attractor_object.scale.x = self.cube_length
        self.attractor_object.scale.y = self.cube_length
        self.attractor_object.scale.z = self.cube_length

        self.attractor_object.color.r = 0.913
        self.attractor_object.color.g = 0.117
        self.attractor_object.color.b = 0.388

        # This has to be, otherwise it will be transparent
        self.attractor_object.color.a = 1.0

    def get_transformation(self, from_frame_rel, to_frame_rel):
        print("Attractor Transformation")
        pass
        # try:
        #     now = rclpy.time.Time()
        #     # now = self.get_clock().now().to_msg()

        #     trans = self.tf_buffer.lookup_transform(
        #         to_frame_rel,
        #         from_frame_rel,
        #         now)
        #     return trans

        # except TransformException as ex:
        #     self.get_logger().info(
        #         f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
        #     )
        #     return None

    def timer_callback(self):
        print("5. Attractor ")
        self.attractor_publisher.publish(self.attractor_object)


def main(args=None):
    rclpy.init(args=args)
    attractor_basic = AttractorPublisher()
    rclpy.spin(attractor_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
