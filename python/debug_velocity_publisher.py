#!/usr/bin/python3

import numpy as np
from numpy import linalg as LA
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray


class VisualizeVelocityPublisher:
    """Publishes the visualization arrow for better debugging and understanding
    of what is happening. Note, that this is not a Node, but only an extension
    of the avoider."""

    def __init__(self, robot_arm_avoider, franka, timer_period=0.2):
        self.avoider = robot_arm_avoider
        self.franka = franka

        self.initial_velocity_markers = None
        self.modulated_velocity_markers = None
        # self.correction_velocity_markers = None

        self.publisher_initial = self.avoider.create_publisher(
            MarkerArray, "initial_velocity", 1
        )
        self.publisher_modulated = self.avoider.create_publisher(
            MarkerArray, "modulated_velocity", 1
        )

        self.publisher_ee = self.avoider.create_publisher(
            Marker, "intial_velocity", 1)

        self._it_initial = None
        self._it_modulation = None
        # self.publisher_correction

        # Initialize Topics
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def reset_initial_array(self):
        self.initial_velocity_markers = MarkerArray()
        self._it_initial = 1000

    def reset_modulated_array(self):
        self.modulated_velocity_markers = MarkerArray()
        self._it_modulation = 2000

    def get_initial_velocity_marker(self):
        marker = self.get_new_arrow(
            position=self.franka.get_end_effector_position(),
            vector=self.avoider.ee_twist.get_linear_velocity(),
            scale=1,
        )

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.id = 3001

        return marker

    def append_modulated_velocity_marker(self, position, vector):
        marker = self.get_new_arrow(position, vector, scale=0.5)

        marker.color.r = 0.0
        marker.color.g = 0.1
        marker.color.b = 1.0
        marker.color.a = 0.5

        marker.id = self._it_modulation

        self._it_modulation += 1
        self.modulated_velocity_markers.markers.append(marker)

    def append_initial_velocity_marker(self, position, vector, scale=0.5):
        marker = self.get_new_arrow(position, vector)

        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.0
        marker.color.a = 0.5

        marker.id = self._it_initial

        self._it_initial += 1
        self.initial_velocity_markers.markers.append(marker)

    def get_new_arrow(self, position, vector, scale=1.0):
        """Create a new marker-arrow."""
        magnitude_vec = LA.norm(vector)
        # rotation_align = Rotation.align_vectors([[1, 0, 0]], [vector])
        # rotation_vec = rotation_align[0].as_quat()

        v1 = np.array([1, 0, 0])
        v2 = vector
        qq = np.zeros(4)
        qq[:3] = np.cross(v1, v2)
        qq[3] = np.sqrt((LA.norm(v1) ** 2) * (LA.norm(v2) ** 2)) + np.dot(v1, v2)

        try:
            rotation_vec = qq / LA.norm(qq)
        except ZeroDivisionError:
            return None

        marker_object = Marker()
        marker_object.header.frame_id = "world"

        marker_object.ns = "frank_emika"

        # marker_object.id = 1e4 + it
        marker_object.type = Marker.ARROW
        # marker_object.action = Marker.ADD

        marker_object.pose.position.x = position[0]
        marker_object.pose.position.y = position[1]
        marker_object.pose.position.z = position[2]

        marker_object.pose.orientation.x = rotation_vec[0]
        marker_object.pose.orientation.y = rotation_vec[1]
        marker_object.pose.orientation.z = rotation_vec[2]
        marker_object.pose.orientation.w = rotation_vec[3]

        marker_object.scale.x = magnitude_vec * scale
        marker_object.scale.y = magnitude_vec * 0.2 * scale
        marker_object.scale.z = magnitude_vec * 0.2 * scale

        # marker_object.color.r = 1.0
        # marker_object.color.g = 0.2
        # marker_object.color.b = 0.0
        # marker_object.color.a = 1.0

        return marker_object

    def publish_velocities(self):
        self.publisher_initial.publish(self.initial_velocity_markers)
        self.publisher_modulated.publish(self.modulated_velocity_markers)

        self.publisher_ee.publish(self.get_initial_velocity_marker())
