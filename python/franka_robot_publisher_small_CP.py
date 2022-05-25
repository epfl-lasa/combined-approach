#!/usr/bin/python3
#!python

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np

import tf2_ros
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from franka_robot_base import FrankaRobotBase, RigidLink, ControlPoint


class FrankaRobotPublisher(FrankaRobotBase):
    def initialize_control_point(self):
        self.link_dict = {}
        self.control_publisher = self.create_publisher(
            MarkerArray, "/control_points", 5
        )
        self.control_point_array = MarkerArray()
        self.cp_radius = 0.16

        # self.control_publishers = []
        # self.marker_object_list = []

        self.br = TransformBroadcaster(self)
        self.frame_id_base = "_frankalink"

        link_0 = RigidLink(
            [
                ControlPoint([0.0, 0.0, 0.06], 0.21),  # X:FRONT Y:RIGHT Z:UP
                ControlPoint([-0.10, 0.0, 0.03], 0.15),  # X:FRONT Y:RIGHT Z:UP
            ],
            link_id=0,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_0.link_id), link=link_0)

        link_1 = RigidLink(
            [
                ControlPoint([0.0, 0.0, -0.15], self.cp_radius),  # X:FRONT Y:RIGHT Z:UP
                ControlPoint([0.0, -0.03, -0.05], 0.15),  # X:FRONT Y:RIGHT Z:UP
                ControlPoint(
                    [0.0, -0.055, 0.01], self.cp_radius
                ),  # X:FRONT Y:RIGHT Z:UP
            ],
            link_id=1,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_1.link_id), link=link_1)

        link_2 = RigidLink(
            [
                ControlPoint([0.0, -0.009, 0.068], 0.14),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.0, -0.009, 0.034], 0.14),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.0, -0.054, 0.038], 0.12),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.0, -0.10, 0.015], 0.12),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.0, -0.145, 0.00], 0.13),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.0, -0.163, 0.00], 0.13),  # X:FRONT Y:UP Z:RIGHT
            ],
            link_id=2,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_2.link_id), link=link_2)

        link_3 = RigidLink(
            [
                ControlPoint([0.0, 0.0, -0.10], 0.12),  # X:FRONT Y:RIGHT Z:UP
                ControlPoint([0.008, 0.0, -0.060], 0.12),  # X:FRONT Y:RIGHT Z:UP
                ControlPoint([0.06, 0.03, 0.0], 0.10),  # X:FRONT Y:RIGHT Z:UP
                ControlPoint([0.09, 0.07, 0.0], 0.13),  # X:FRONT Y:RIGHT Z:UP
                ControlPoint([0.09, 0.03, 0.0], 0.13),  # X:FRONT Y:RIGHT Z:UP
            ],
            link_id=3,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_3.link_id), link=link_3)

        link_4 = RigidLink(
            [
                ControlPoint([-0.006, 0.0, 0.066], 0.12),  # X:UP Y:FRONT Z:RIGHT
                ControlPoint([-0.004, 0.0, 0.026], 0.12),  # X:UP Y:FRONT Z:RIGHT
                ControlPoint([-0.06, 0.036, 0.010], 0.10),  # X:UP Y:FRONT Z:RIGHT
                ControlPoint([-0.08, 0.067, 0.00], 0.12),  # X:UP Y:FRONT Z:RIGHT
                ControlPoint([-0.081, 0.105, 0.00], 0.12),  # X:UP Y:FRONT Z:RIGHT
            ],
            link_id=4,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_4.link_id), link=link_4)

        link_5 = RigidLink(
            [
                ControlPoint([-0.003, 0.0, -0.24], 0.12),  # X:UP Y:RIGHT Z:FRONT
                ControlPoint([0.0, 0.0, -0.21], 0.11),  # X:UP Y:RIGHT Z:FRONT
                ControlPoint([0.0, 0.04, -0.2], 0.11),  # X:UP Y:RIGHT Z:FRONT
                ControlPoint([0.0, 0.065, -0.17], 0.081),  # X:UP Y:RIGHT Z:FRONT
                ControlPoint([0.0, 0.082, -0.135], 0.08),  # X:UP Y:RIGHT Z:FRONT
                ControlPoint([0.0, 0.085, -0.095], 0.08),  # X:UP Y:RIGHT Z:FRONT
                ControlPoint([0.0, 0.090, -0.056], 0.08),  # X:UP Y:RIGHT Z:FRONT
                ControlPoint([0.0, 0.077, -0.0032], 0.121),  # X:UP Y:RIGHT Z:FRONT
                ControlPoint([0.0, 0.033, -0.0032], 0.12),  # X:UP Y:RIGHT Z:FRONT
            ],
            link_id=5,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_5.link_id), link=link_5)

        link_6 = RigidLink(
            [
                ControlPoint([-0.002, 0.00, 0.015], 0.098),  # X:FRONT Y:DOWN Z:RIGHT
                ControlPoint([0.02, 0.004, 0.012], 0.11),  # X:FRONT Y:DOWN Z:RIGHT
                ControlPoint([0.06, 0.022, 0.012], 0.12),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.084, 0.048, 0.0], 0.09),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.084, 0.034, 0.0], 0.10),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.084, 0.014, 0.0], 0.10),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.087, -0.004, 0.0], 0.105),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.087, -0.0285, 0.0], 0.105),  # X:FRONT Y:UP Z:RIGHT
            ],
            link_id=6,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_6.link_id), link=link_6)

        link_7 = RigidLink(
            [
                ControlPoint([0.0, 0.0, 0.076], 0.10),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.01, 0.04, 0.079], 0.05),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.04, 0.04, 0.082], 0.05),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.04, 0.01, 0.079], 0.05),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.024, 0.055, 0.083], 0.04),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.055, 0.024, 0.083], 0.04),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.07, 0.04, 0.082], 0.042),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.04, 0.07, 0.082], 0.042),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.055, 0.06, 0.082], 0.042),  # X:FRONT Y:UP Z:RIGHT
                # ControlPoint([0.06, -0.02, 0.082],0.04),    # X:FRONT Y:UP Z:RIGHT
                # ControlPoint([0.04, 0.7, 0.076],0.05)       # X:FRONT Y:UP Z:RIGHT
            ],
            link_id=7,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_7.link_id), link=link_7)

        link_8 = RigidLink([ControlPoint([0.0, 0.0, 0.0], 0.05)], link_id=8)
        self.add_link(frame_id=self.frame_id_base + str(link_8.link_id), link=link_8)


def main(args=None):
    rclpy.init(args=args)
    marker_basic = FrankaRobotPublisher()
    rclpy.spin(marker_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
