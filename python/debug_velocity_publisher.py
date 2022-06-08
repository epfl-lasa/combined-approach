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
        self._it_initial = None
        
        self.publisher_modulated = self.avoider.create_publisher(
            MarkerArray, "modulated_velocity", 1
        )
        self._it_modulation = None
        
        self.publisher_modulated_scaled = self.avoider.create_publisher(
            MarkerArray, "publisher_modulated_scaled", 1
        )

        self.publisher_ee = self.avoider.create_publisher(
            Marker, "intial_velocity", 1
        )
        self._it_modulation_scaled = None

        # self.publisher_rotdir = self.avoider.create_publisher(
        #     MarkerArray, "rotation_directions", 1
        # )
        # self._it_rotdir = None

        # self.publisher_linkjoint = self.avoider.create_publisher(
            # MarkerArray, "link_joints", 1
        # )

        self.publisher_correction_velocity = self.avoider.create_publisher(
            MarkerArray, "correction_velocity", 1
        )

    def reset_initial_array(self):
        self.initial_velocity_markers = MarkerArray()
        self._it_initial = 1000

    def reset_modulated_array(self):
        self.modulated_velocity_markers = MarkerArray()
        self._it_modulation = 2000

    def reset_modulated_scaled_array(self):
        self.scaled_modulated_velocity_markers = MarkerArray()
        self._it_modulation_scaled = 3000

    def reset_rotdir_array(self):
        self.rotdir_markers = MarkerArray()
        self._it_rotdir = 3000

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
        marker = self.get_new_arrow(position, vector, scale=0.5, rgba=[0.0, 0.1, 1.0, 0.5])
        marker.id = self._it_modulation

        self._it_modulation += 1
        self.modulated_velocity_markers.markers.append(marker)

    def append_scaled_modulated_velocity_marker(self, position, vector):
        marker = self.get_new_arrow(
            position, vector, scale=0.5, rgba=[0.0, 0.8, 1.0, 0.5])
        marker.id = self._it_modulation_scaled

        self._it_modulation_scaled += 1
        self.scaled_modulated_velocity_markers.markers.append(marker)

    def append_initial_velocity_marker(self, position, vector, scale=0.5):
        marker = self.get_new_arrow(
            position, vector, rgba=[1.0, 0.1, 0.0, 0.5])
        
        marker.id = self._it_initial
        marker.ns = "initial_velocity"

        self._it_initial += 1
        self.initial_velocity_markers.markers.append(marker)

    def append_rotation_direction_marker(self, position, vector, scale=0.2):
        marker = self.get_new_arrow(
            position, vector, scale=scale, rgba=[1.0, 1.0, 1.0, 0.8])
        
        marker.id = self._it_rotdir
        marker.ns = "rotation_directions"

        self._it_rotdir += 1
        self.rotdir_markers.markers.append(marker)

    def append_correction_velocity(self, position, vector, marker_id, scale=1):
        marker = self.get_new_arrow(
            position, vector, scale=scale, rgba=[1.0, 0.0, 0.0, 0.8]
        )
        
        marker.id = marker_id
        marker.ns = "correction_velocity"

        self.correction_velocity_markers.markers.append(marker)


    def get_new_arrow(
        self, position, vector, scale=1.0, rgba=[1.0, 0.0, 0.0, 1.0]
    ):
        """Create a new marker-arrow.

        RGB-A color directly as a list argument, since it is never changed, but read-only.
        """
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
        
        marker_object.color.r = rgba[0]
        marker_object.color.g = rgba[1]
        marker_object.color.b = rgba[2]
        marker_object.color.a = rgba[3]

        return marker_object
    
    def reset_arrays(self):
        self.reset_initial_array()
        self.reset_modulated_array()
        self.reset_modulated_scaled_array()
        # self.reset_rotdir_array()

        self.correction_velocity_markers = MarkerArray()

    def publish_link_joint_array(self):
        link_joint_array = MarkerArray()

        print(f"{np.round(self.avoider.joint_origins_global, 2)}")
        for ii in range(self.avoider.joint_origins_global.shape[1]):
            marker_object = Marker()
            marker_object.header.frame_id = "world"
            marker_object.ns = "link_joints"
            marker_object.id = ii

            marker_object.type = Marker.SPHERE

            marker_object.pose.orientation.x = 0.0
            marker_object.pose.orientation.y = 0.0
            marker_object.pose.orientation.z = 0.0
            marker_object.pose.orientation.w = 1.0

            marker_object.scale.x = 0.05
            marker_object.scale.y = 0.05
            marker_object.scale.z = 0.05
        
            marker_object.color.r = 0.2
            marker_object.color.g = 0.2
            marker_object.color.b = 0.2
            marker_object.color.a = 1.0

            pos = self.avoider.joint_origins_global[:, ii]
            
            print("ii ", ii, "  pos", pos)
            marker_object.pose.position.x = pos[0]
            marker_object.pose.position.y = pos[1]
            marker_object.pose.position.z = pos[2]

            link_joint_array.markers.append(marker_object)
        
        self.publisher_linkjoint.publish(link_joint_array)

    def publish_velocities(self):
        self.publisher_initial.publish(self.initial_velocity_markers)
        self.publisher_modulated.publish(self.modulated_velocity_markers)
        self.publisher_modulated_scaled.publish(
            self.scaled_modulated_velocity_markers
        )
        self.publisher_ee.publish(self.get_initial_velocity_marker())

        # self.publisher_rotdir.publish(self.rotdir_markers)

        self.publisher_correction_velocity.publish(
            self.correction_velocity_markers
        )
