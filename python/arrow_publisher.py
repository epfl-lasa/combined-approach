#!/usr/bin/python3
#!python

import rclpy
import math
import numpy as np


from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker


import tf2_ros
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class ArrowPublisher(Node):
    def __init__(self, attractor_position,franka_publisher):
        super().__init__("Arrow_node")

        self.attractor_position = attractor_position
        self.franka = franka_publisher

        self.declare_parameter("world", "_frankalink8")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.Arrow_publisher = self.create_publisher(Marker, "/Arrow", 5)
        self.Arrow_object = Marker()
        self.frame_id = "world"
        self.Arrow_object.header.frame_id = self.frame_id
        self.Arrow_object.type = Marker.ARROW



        self.arrow_length = 0.15
        self.arrow_width = 0.013
        self.Arrow_object.scale.x = self.arrow_length
        self.Arrow_object.scale.y = self.arrow_width
        self.Arrow_object.scale.z = self.arrow_width


        self.Arrow_object.color.r = 0.192
        self.Arrow_object.color.g = 0.596
        self.Arrow_object.color.b = 0.929

        # This has to be, otherwise it will be transparent
        self.Arrow_object.color.a = 1.0

    def timer_callback(self):
        print("5. Arrow ")

        # POSITION THE ARROW AT THE EE
        pos_ee = self.franka.get_end_effector_position()
        # print(self.pos_ee)
        self.Arrow_object.pose.position.x = pos_ee[0]
        self.Arrow_object.pose.position.y = pos_ee[1]
        self.Arrow_object.pose.position.z = pos_ee[2]

        
        # POINT IT TO THE ORIGIN
        x_axis = [1, 0 , 0]
        wrench_vec =  self.attractor_position-pos_ee 
        q = self.get_quaternion_from_vectors(x_axis,wrench_vec)

        self.Arrow_object.pose.orientation.x = q[0]
        self.Arrow_object.pose.orientation.y = q[1]
        self.Arrow_object.pose.orientation.z = q[2]
        self.Arrow_object.pose.orientation.w = q[3]

        self.Arrow_publisher.publish(self.Arrow_object)

    @staticmethod
    def get_quaternion_from_vectors (v1, v2):
        q = np.zeros(4)
        a = np.cross(v1, v2)
        q[:3] = a
        q[3] = np.linalg.norm(v1) * np.linalg.norm(v2) + np.dot(v1, v2)
        q = q/np.linalg.norm(q)
        return q


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


def main(args=None):
    rclpy.init(args=args)
    Arrow_basic = ArrowPublisher()
    rclpy.spin(Arrow_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
