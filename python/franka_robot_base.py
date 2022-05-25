#!/usr/bin/python3
import logging

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import String
from geometry_msgs.msg import Point, TransformStamped, PointStamped

# from tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


class ControlPoint:
    def __init__(self, position, radius):
        self.point = Point()
        self.position = position

        self.point.x = position[0]
        self.point.y = position[1]
        self.point.z = position[2]

        self.radius = radius


class RigidLink:
    def __init__(self, control_point_list, link_id, frame_id_base="_frankalink"):
        self.control_point_list = control_point_list
        self.link_id = link_id

        self.link_name = frame_id_base + str(self.link_id)

    def get_control_points_stamped(self, time_now):
        # if time_now is None:
        # rclpy.time.Time.to_msg()

        point_list = []
        for cp in self.control_point_list:
            point_stamped = PointStamped()
            point_stamped.point = cp.point
            point_stamped.header.frame_id = self.link_name
            point_stamped.header.stamp = time_now

            point_list.append(point_stamped)

        return point_list


class FrankaRobotBase(Node):
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
        print("Robot init done")

    @property
    def n_links(self):
        return len(self.link_dict)

    @property
    def n_joints(self):
        return len(self.link_dict)

    def get_control_radii_list(self):
        radii_list = []
        for key, links in self.link_dict.items():
            radii_list.append([point.radius for point in links.control_point_list])

        return radii_list

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

            self.marker_object.pose.position.x = cp.point.x
            self.marker_object.pose.position.y = cp.point.y
            self.marker_object.pose.position.z = cp.point.z

            self.marker_object.pose.orientation.x = 0.0
            self.marker_object.pose.orientation.y = 0.0
            self.marker_object.pose.orientation.z = 0.0
            self.marker_object.pose.orientation.w = 1.0

            self.marker_object.scale.x = cp.radius
            self.marker_object.scale.y = cp.radius
            self.marker_object.scale.z = cp.radius

            self.marker_object.color.r = 2.0
            self.marker_object.color.g = 5.0
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
            # print(now)
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

        if ee_trans is None:
            return None

        ee_pos = [
            ee_trans.transform.translation.x,
            ee_trans.transform.translation.y,
            ee_trans.transform.translation.z,
        ]

        return np.array(ee_pos)

    def set_end_effector_position(self, position):
        pass
        # print(self.msg_jointstate.pose)
        # self.msg_jointstate.pose
        # how do I make the whole system change to a specific position based on where I want the end effector to be

    def callback_jointstate(self, msg):
        self.msg_jointstate = msg

    #     # self.joint_positions = msg.position
    #     # self.joint_velocity = msg.velocity
    #     # self.joint_effort = msg.effort

    def timer_callback(self):
        logging.info("[CONTROL POINTS] callback starting.")
        self.control_publisher.publish(self.control_point_array)
        # self.get_end_effector_position()

    def spin_short(self):
        self._done = False
        rclpy.spin_until_future_complete(self, self)

    def done(self):
        return self._done

    def initialize_control_point(self):
        raise NotImplementedError("[TODO] implement this class")
