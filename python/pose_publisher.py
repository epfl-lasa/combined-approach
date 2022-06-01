#!/usr/bin/python3
#!python

import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Quaternion
import tf2
# from tf.transformations import quaternion_from_euler



class PosePublisher(Node):
    def __init__(self,attractor_publisher,franka_publisher):
        super().__init__("pose_node")

        self.attractor = attractor_publisher
        self.franka = franka_publisher

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pose_publisher = self.create_publisher(PoseStamped, "/pose", 5)
        self.pose_object = PoseStamped()
        
        self.frame_id = "world"
        self.pose_object.header.frame_id = self.frame_id

        

    def timer_callback(self):
        # print("4. POSE ")

        ee_pos = self.franka.get_end_effector_position()
        ee_ori = self.franka.get_end_effector_orientation()
        # q_ee = Quaternion(ee_ori[0],ee_ori[1],ee_ori[2],ee_ori[3])

        a_pos = self.attractor.get_attractor_position()
        a_ori = self.attractor.get_attractor_orientation()


        # q_a = Quaternion(0.,0.,0.,1)
        # q_a = Quaternion(a_ori[0],a_ori[1],a_ori[2],a_ori[3])

        # breakpoint()
        
        # Set the position of the arrow in the the world frame to that of the ee
        self.pose_object.pose.position.x = ee_pos[0]
        self.pose_object.pose.position.y = ee_pos[1]
        self.pose_object.pose.position.z = ee_pos[2]

        q = quaternion_from_euler(0.0, 0.0, 0.0)

        # Set the orientation to match that of the attractor
        # self.pose_object.pose.orientation.x = a_ori[0]
        # self.pose_object.pose.orientation.y = a_ori[1]
        # self.pose_object.pose.orientation.z = a_ori[2]
        # self.pose_object.pose.orientation.w = a_ori[3]
        




        self.pose_publisher.publish(self.pose_object)




def main(args=None):
    rclpy.init(args=args)
    pose_basic = PosePublisher()
    rclpy.spin(pose_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
