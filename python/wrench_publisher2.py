#!/usr/bin/python3
#!python

import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, TransformStamped, PointStamped


class WrenchPublisher(Node):
    def __init__(self, attractor_position, franka):
        super().__init__("wrench_node")

        self.attractor_position = np.array(attractor_position)
        self.franka = franka

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.wrench_publisher = self.create_publisher(WrenchStamped, "/wrench", 5)
        self.wrench_object = WrenchStamped()
        self.point_object = PointStamped()
        # self.t = TransformStamped()

        # self.frame_id = "world"
        self.frame_id = "_frankalink8"
        # self.frame_id = "_frankalink8"
        self.point_object.header.frame_id = self.frame_id
        self.wrench_object.header.frame_id = self.frame_id

    def timer_callback(self):
        print("4. WRENCH ")
        ee_pos = self.franka.get_end_effector_position2()

        
        self.point_object.point.x = ee_pos[0]
        self.point_object.point.y = ee_pos[1]
        self.point_object.point.z = ee_pos[2]


        print(self.point_object)

        # self.t.transform.translation = self.ee_pos[0]
        # self.t.transform.translation = self.ee_pos[1]
        # self.t.transform.translation = self.ee_pos[2]

        # self.t.transform.translation = self.ee_pos[0]
        # self.t.transform.translation = self.ee_pos[1]
        # self.t.transform.translation = self.ee_pos[2]
        # ee_ori = self.franka.get_end_effector_orientation()

        if(ee_pos is None):
            return

        # unit_wrench_vec =  ee_pos + ee_pos - self.attractor_position 
        unit_wrench_vec =  ee_pos
        # print(wrench_vec)
        norm = np.linalg.norm(unit_wrench_vec)

        if norm :
            unit_wrench_vec = unit_wrench_vec / norm



        # print(ee_pos)
        self.wrench_object.wrench.force.x = unit_wrench_vec[0]
        self.wrench_object.wrench.force.y = unit_wrench_vec[1]
        self.wrench_object.wrench.force.z = unit_wrench_vec[2]

        # vt = tf2_geometry_msgs.do_transform_vector3(unit_wrench_vec,self.t)

        self.wrench_publisher.publish(self.wrench_object)


def main(args=None):
    rclpy.init(args=args)
    wrench_basic = AttractorPublisher()
    rclpy.spin(wrench_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
