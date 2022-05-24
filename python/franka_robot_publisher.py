#!/usr/bin/python3
import logging

# from scipy.spatial.transform import Rotation

import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration

# import tf2_ros
from tf2_ros import TransformBroadcaster, TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener

# from std_msgs.msg import String
# from geometry_msgs.msg import Point, TransformStamped, PointStamped

# from tf2_geometry_msgs import PointStamped
# from sensor_msgs.msg import JointState
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

        link_1 = RigidLink(
            [
                ControlPoint([0.0, 0.0, -0.15], 0.18),
                ControlPoint([0.0, -0.075, 0.01], 0.18),
            ],
            link_id=1,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_1.link_id), link=link_1)

        link_2 = RigidLink(
            [
                ControlPoint([0.0, -0.007, 0.07], 0.18),
                ControlPoint([0.0, -0.125, 0.0], 0.18),
            ],
            link_id=2,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_2.link_id), link=link_2)

        link_3 = RigidLink(
            [
                ControlPoint([0.0, 0.0, -0.075], 0.15),
                ControlPoint([0.09, 0.07, 0.0], 0.18),
            ],
            link_id=3,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_3.link_id), link=link_3)

        link_4 = RigidLink(
            [
                ControlPoint([0.003, 0.0, 0.066], 0.18),
                ControlPoint([-0.076, 0.076, 0.0], 0.15),
            ],
            link_id=4,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_4.link_id), link=link_4)

        link_5 = RigidLink(
            [
                ControlPoint([0.0, 0.0, -0.21], 0.15),
                ControlPoint([0.0, 0.08, -0.129], 0.16),
                ControlPoint([0.0, 0.073, 0.0], 0.15),
            ],
            link_id=5,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_5.link_id), link=link_5)

        link_6 = RigidLink(
            [
                ControlPoint([0.0, 0.0, 0.042], 0.15),
                ControlPoint([0.1, 0.019, 0.0], 0.16),
            ],
            link_id=6,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_6.link_id), link=link_6)

        link_7 = RigidLink([ControlPoint([0.0, 0.0, 0.076], 0.1)], link_id=7)
        self.add_link(frame_id=self.frame_id_base + str(link_7.link_id), link=link_7)

        link_8 = RigidLink([ControlPoint([0.0, 0.0, 0.0], 0.1)], link_id=8)
        self.add_link(frame_id=self.frame_id_base + str(link_8.link_id), link=link_8)


def main(args=None):
    rclpy.init(args=args)
    marker_basic = FrankaRobotPublisher()
    rclpy.spin(marker_basic)

    marker_basic.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
