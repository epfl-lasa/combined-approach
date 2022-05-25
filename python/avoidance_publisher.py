#!/usr/bin/python3
#!python

import rclpy
import numpy as np

from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String
from vartools.dynamical_systems import LinearSystem


class AvoidancePublisher(Node):
    # Helper class
    def __init__(self, franka, obstacles_publisher):
        super().__init__("obstace_avoidance")
        self.franka = franka
        self.obstacles_publisher = obstacles_publisher

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.initial_dynamics = LinearSystem(
            attractor_position=np.array([0.0, 0.0, 0.0]),
            maximum_velocity=1,
            distance_decrease=0.3,
        )

        print("before get transformation")
        franka.get_transformation("_frankalink8", "world")

    def update_step(self, ii):
        if not ii % 10:
            print(f"it={ii}")

        # iterate over ii to just use the pos of the ee
        velocity = self.dynamic_avoider.evaluate(self.position_list[:, ii - 1])

    def has_converged(self, ii) -> bool:
        return np.allclose(self.position_list[:, ii], self.position_list[:, ii - 1])

    def timer_callback(self):
        print("3. AVOIDANCE")
        ee_pos = self.franka.get_end_effector_position()
        # print(ee_pos)

        # obstacles = self.obstacles_publisher.get_obstacles()
        # print(obstacles)


def main(args=None):
    rclpy.init(args=args)
    avoidance = AvoidancePublisher()
    rclpy.spin(avoidance)

    avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
