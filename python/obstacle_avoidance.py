#!/usr/bin/python3
#!python

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker , MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String

# Helper class

class AvoidancePublisher (Node):
    def __init__(self, franka, obstacles_publisher ):
        super().__init__('obstace_avoidance')
        self. franka = franka
        self.obstacles_publisher=obstacles_publisher

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    
    def timer_callback(self):
        print('3. AVOIDANCE')
        ee_pos = self.franka.get_end_effector_position()
        

        print(ee_pos)

    
            

    
def main(args=None):
    rclpy.init(args=args)
    avoidance = AvoidancePublisher()
    rclpy.spin(avoidance)
    

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

