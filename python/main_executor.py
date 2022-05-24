#!/usr/bin/python3

from concurrent.futures import ThreadPoolExecutor
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import Executor

from std_msgs.msg import String

from franka_robot_publisher import FrankaRobotPublisher

# from franka_robot_publisher_small_CP import FrankaRobotPublisher

# from franka_robot_publisher_1CP import FrankaRobotPublisher
# from franka_robot_publisher_1CP_large import FrankaRobotPublisher
from obstacle_publisher import ObstaclePublisher

# from avoidance_publisher import AvoidancePublisher
# from robot_arm_avoider import RobotArmAvoider as AvoidancePublisher
from robot_arm_avoider import RobotArmAvoider

from wrench_publisher import WrenchPublisher
from attractor_publisher import AttractorPublisher


class StandardExecutor(Executor):
    def __init__(self):
        super().__init__()
        # self.high_priority_nodes = set()
        # self.hp_executor = ThreadPoolExecutor(max_workers=os.cpu_count() or 4)
        self.executor = ThreadPoolExecutor(max_workers=os.cpu_count() or 4)
        # self.lp_executor = ThreadPoolExecutor(max_workers=1)

    def spin_once(self, timeout_sec=None):
        try:
            handler, group, node = self.wait_for_ready_callbacks(
                timeout_sec=timeout_sec
            )
        except StopIteration:
            pass
        else:
            self.executor.submit(handler)


def main(args=None):
    rclpy.init(args=args)

    try:
        franka_publisher = FrankaRobotPublisher()
        obstacles_publisher = ObstaclePublisher()
        # avoidance_publisher = AvoidancePublisher(franka_publisher, obstacles_publisher)
        avoidance_publisher = RobotArmAvoider(franka_publisher, obstacles_publisher)

        wrench_publisher = WrenchPublisher()
        attractor_publisher = AttractorPublisher([0.5, 0.0, 0.5])

        executor = StandardExecutor()
        # pose_publisher = PosePublisher()
        executor.add_node(franka_publisher)
        executor.add_node(obstacles_publisher)
        executor.add_node(avoidance_publisher)

        # executor.add_node(pose_publisher)

        executor.add_node(wrench_publisher)
        executor.add_node(attractor_publisher)

        try:
            print("Spinning")
            executor.spin()
            print("Spinning Done")

        finally:
            executor.shutdown()

            franka_publisher.destroy_node()
            obstacles_publisher.destroy_node()
            avoidance_publisher.destroy_node()
            # pose_publisher.destroy_node()
            wrench_publisher.destroy_node()
            attractor_publisher.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    main()
