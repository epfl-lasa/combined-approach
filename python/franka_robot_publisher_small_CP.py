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


class ControlPoint:
    def __init__(self, position, radius):
        self.position = position
        self.radius = radius


class RigidLink:
    def __init__(self, control_point_list, link_id):
        self.control_point_list = control_point_list
        self.link_id = link_id


class FrankaRobotPublisher(Node):
    def __init__(self):

        super().__init__("franka_markers")

        # Declare and acquire `target_frame` parameter
        self.declare_parameter("world", "_frankalink8")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initialize_control_point()
        self.subscription = self.create_subscription(
            JointState, "franka/joint_states", self.callback_jointstate, 3
        )
        # self.subscription  # prevent unused variable warning

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

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

    def add_link(self, frame_id, link: RigidLink):

        self.link_dict[frame_id] = link
        i = link.link_id * 10

        for ii, cp in enumerate(link.control_point_list):
            self.marker_object = Marker()
            self.marker_object.header.frame_id = frame_id

            self.marker_object.ns = "frank_emika"

            self.marker_object.id = ii + i
            self.marker_object.type = Marker.SPHERE
            self.marker_object.action = Marker.ADD

            self.marker_object.pose.position.x = cp.position[0]
            self.marker_object.pose.position.y = cp.position[1]
            self.marker_object.pose.position.z = cp.position[2]

            self.marker_object.pose.orientation.x = 0.0
            self.marker_object.pose.orientation.y = 0.0
            self.marker_object.pose.orientation.z = 0.0
            self.marker_object.pose.orientation.w = 1.0

            self.marker_object.scale.x = cp.radius
            self.marker_object.scale.y = cp.radius
            self.marker_object.scale.z = cp.radius

            self.marker_object.color.r = 2.0
            self.marker_object.color.g = 1.0
            self.marker_object.color.b = 0.0

            # This has to be, otherwise it will be transparent
            self.marker_object.color.a = 1.0

            self.control_point_array.markers.append(self.marker_object)

    def get_transformation(self, from_frame_rel, to_frame_rel):
        try:
            now = rclpy.time.Time()
            # now = self.get_clock().now().to_msg()

            dur = Duration()
            dur.sec = 1
            dur.nsec = 0

            trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, now)
            print(now)
            # print(trans.header.frame_id)
            # breakpoint()
            return trans

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
            )
            return None

    def get_end_effector_position(self):
        ee_trans = self.get_transformation("_frankalink8", "world")

        breakpoint()
        if ee_trans is None:
            return None

        ee_pos = [
            ee_trans.transform.translation.x,
            ee_trans.transform.translation.y,
            ee_trans.transform.translation.z,
        ]

        breakpoint()

        return np.array(ee_pos)

    def callback_jointstate(self, msg):
        self.msg_jointstate = msg

    #     # print("callback_jointstate")
    #     # print('header', msg.header.stamp)
    #     # print(self.get_clock().now().to_msg())

    #     # self.joint_positions = msg.position
    #     # self.joint_velocity = msg.velocity
    #     # self.joint_effort = msg.effort

    def timer_callback(self):
        print("1. CONTROL POINTS")
        self.control_publisher.publish(self.control_point_array)
        # print(self.get_end_effector_position())

        self._done = True

    def spin_short(self):
        self._done = False
        rclpy.spin_until_future_complete(self, self)

    def done(self):
        return self._done


def main(args=None):
    rclpy.init(args=args)
    marker_basic = FrankaRobotPublisher()
    rclpy.spin(marker_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
