#!/usr/bin/python3
from __future__ import print_function

import os
from threading import Thread

import pinocchio
from pinocchio.robot_wrapper import RobotWrapper

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter

from main_executor import StandardExecutor

from franka_robot_publisher import FrankaRobotPublisher
from obstacle_publisher import ObstaclePublisher


class RobotArmAvoider(Node):
    def __init__(self, franka, obstacles_publisher):
        print("Start init.")
        
        super().__init__("obstace_avoidance")
        self.franka = franka
        self.obstacles_publisher = obstacles_publisher

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.client = self.create_client(
            GetParameters, 'franka/robot_state_publisher/get_parameters'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        request = GetParameters.Request()
        request.names = ['robot_description']
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        urdf_string = future.result().values[0].string_value

        # Create temp-file to store it for later use with pinocchio
        urdf_filename = os.path.join("/tmp", request.names[0] + ".urdf")
        with open(urdf_filename, "w") as file:
            file.write(urdf_string)

        self.model = pinocchio.buildModelFromUrdf(urdf_filename)
        self.control_radii = self.franka.get_control_radii_list()

    def get_forward_kinematics(self,):
        pass
    
    def get_forward_kinematics(self,):
        pass

    def run_avoider(self):
        pass
    
    def get_gamma_control_points(self):
        self.world_control_point_list = []
        for it_link, link_id in enumerate(self.franka.link_dict.keys()):
            self.world_control_point_list.append(
                self.franka.get_link_points_in_global_frame(link_id)
            )

        self.gamma_list = []
        for cp_list in self.world_control_point_list:
            self.gamma_list.append([])
            for cp in cp_list:
                self.gamma_list(self.obstacles_publisher.get_gamma(cp))

    def timer_callback(self):
        print("ROBOT_ARM_AVOIDER.")

        self.get_gamma_control_points()


def main(args=None):
    rclpy.init(args=args)
    try:
        franka_publisher = FrankaRobotPublisher()
        obstacles_publisher = ObstaclePublisher()
        # self.avoidance_publisher = AvoidancePublisher(
        robot_arm_avoider = RobotArmAvoider(
            franka_publisher, obstacles_publisher
        )
        
        executor = StandardExecutor()
        executor.add_node(franka_publisher)
        executor.add_node(obstacles_publisher)
        executor.add_node(robot_arm_avoider)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            
            franka_publisher.destroy_node()
            obstacles_publisher.destroy_node()
            robot_arm_avoider.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    # get_franka()
    main()
