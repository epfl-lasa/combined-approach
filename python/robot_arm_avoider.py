#!/usr/bin/python3

from __future__ import print_function

import rclpy
import pinocchio

from franka_robot_publisher import FrankaRobotPublisher
from obstacle_publisher import ObstaclePublisher

from main_exectuor import StandardExecutor


class RobotArmAvoider(rclpy.Node):
    def __init__(self, franka, obstacles_publisher):
        super().__init__("obstace_avoidance")
        self.franka = franka
        self.obstacles_publisher = obstacles_publisher

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        print("Do full body avoidance.")


def main(args=None):
    rclpy.init(args=args)
    try:
        franka_publisher = FrankaRobotPublisher()
        obstacles_publisher = ObstaclePublisher()
        # self.avoidance_publisher = AvoidancePublisher(
        avoidance_publisher = RobotArmAvoider(
            franka_publisher, obstacles_publisher
        )
        
        executor = RobotArmAvoider()
        executor.add_node(franka_publisher)
        executor.add_node(obstacles_publisher)
        executor.add_node(avoidance_publisher)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            
            franka_publisher.destroy_node()
            obstacles_publisher.destroy_node()
            avoidance_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
