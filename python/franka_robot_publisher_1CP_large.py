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

from franka_robot_base import FrankaRobotBase, RigidLink, ControlPoint


class FrankaRobotPublisher(FrankaRobotBase):
    def initialize_control_point(self):
        self.link_dict = {}
        self.control_publisher = self.create_publisher(
            MarkerArray, "/control_points", 5
        )
        self.control_point_array = MarkerArray()

        # self.control_publishers = []
        # self.marker_object_list = []

        self.br = TransformBroadcaster(self)
        self.frame_id_base = "_frankalink"
        self.cp_radius = 0.3

        link_0 = RigidLink(
            [ControlPoint([0.0, 0.0, 0.08], self.cp_radius)],
            link_id=0,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_0.link_id), link=link_0)

        link_1 = RigidLink(
            [ControlPoint([0.0, -0.035, -0.0], self.cp_radius)],
            link_id=1,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_1.link_id), link=link_1)

        link_2 = RigidLink(
            [ControlPoint([0.0, -0.074, 0.03], self.cp_radius)],
            link_id=2,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_2.link_id), link=link_2)

        link_3 = RigidLink(
            [ControlPoint([0.05, 0.03, -0.02], self.cp_radius)],
            link_id=3,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_3.link_id), link=link_3)

        link_4 = RigidLink(
            [ControlPoint([-0.07, 0.03, 0.02], self.cp_radius)],
            link_id=4,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_4.link_id), link=link_4)

        link_5 = RigidLink(
            [ControlPoint([0.0, 0.05, -0.11], self.cp_radius)],
            link_id=5,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_5.link_id), link=link_5)

        link_6 = RigidLink(
            [ControlPoint([0.08, 0.0, 0.05], self.cp_radius)],
            link_id=6,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_6.link_id), link=link_6)


def main(args=None):
    rclpy.init(args=args)
    marker_basic = FrankaRobotPublisher()
    rclpy.spin(marker_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
