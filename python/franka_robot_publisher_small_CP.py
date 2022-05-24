#!/usr/bin/python3
#!python

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from franka_robot_base import FankaRobotBase, RigidLink, ControlPoint


class FrankaRobotPublisher(FankaRobotBase):
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
                ControlPoint(
                    [0.0, -0.009, 0.054], self.cp_radius
                ),  # X:FRONT Y:UP Z:RIGHT
                ControlPoint([0.0, -0.125, 0.0], self.cp_radius),  # X:FRONT Y:UP Z:UP
            ],
            link_id=2,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_2.link_id), link=link_2)

        link_3 = RigidLink(
            [
                ControlPoint(
                    [0.0, 0.0, -0.075], self.cp_radius
                ),  # X:FRONT Y:RIGHT Z:UP
                ControlPoint([0.09, 0.07, 0.0], self.cp_radius),
            ],
            link_id=3,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_3.link_id), link=link_3)

        link_4 = RigidLink(
            [
                ControlPoint([0.003, 0.0, 0.066], self.cp_radius),
                ControlPoint([-0.076, 0.076, 0.0], self.cp_radius),
            ],
            link_id=4,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_4.link_id), link=link_4)

        link_5 = RigidLink(
            [
                ControlPoint([0.0, 0.0, -0.21], self.cp_radius),
                ControlPoint([0.0, 0.08, -0.129], self.cp_radius),
                ControlPoint([0.0, 0.073, 0.0], self.cp_radius),
            ],
            link_id=5,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_5.link_id), link=link_5)

        link_6 = RigidLink(
            [
                ControlPoint([0.0, 0.0, 0.042], self.cp_radius),
                ControlPoint([0.1, 0.019, 0.0], self.cp_radius),
            ],
            link_id=6,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_6.link_id), link=link_6)

        link_7 = RigidLink([ControlPoint([0.0, 0.0, 0.076], self.cp_radius)], link_id=7)
        self.add_link(frame_id=self.frame_id_base + str(link_7.link_id), link=link_7)

        link_8 = RigidLink([ControlPoint([0.0, 0.0, 0.0], self.cp_radius)], link_id=8)
        self.add_link(frame_id=self.frame_id_base + str(link_8.link_id), link=link_8)


def main(args=None):
    rclpy.init(args=args)
    marker_basic = FrankaRobotPublisher()
    rclpy.spin(marker_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
