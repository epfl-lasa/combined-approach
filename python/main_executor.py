from concurrent.futures import ThreadPoolExecutor
import os

from examples_rclpy_executors.listener import Listener
from examples_rclpy_executors.talker import Talker
import rclpy
from rclpy.executors import Executor
from rclpy.node import Node
from std_msgs.msg import String

from franka_robot_publisher import FrankaRobotPublisher
from obstacle_publisher import ObstaclePublisher
from avoidance_publisher import AvoidancePublisher



class PriorityExecutor(Executor):
    
    def __init__(self):
        super().__init__()
        # self.high_priority_nodes = set()
        # self.hp_executor = ThreadPoolExecutor(max_workers=os.cpu_count() or 4)
        self.executor = ThreadPoolExecutor(max_workers=os.cpu_count() or 4)
        # self.lp_executor = ThreadPoolExecutor(max_workers=1)

    def spin_once(self, timeout_sec=None):
        try:
            handler, group, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except StopIteration:
            pass
        else:
            
            self.executor.submit(handler)
            


def main(args=None):
    rclpy.init(args=args)
    try:
        franka_publisher = FrankaRobotPublisher()
        obstacles_publisher = ObstaclePublisher()
        avoidance_publisher = AvoidancePublisher(franka_publisher, obstacles_publisher)
        
        executor = PriorityExecutor()
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