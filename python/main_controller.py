#!/usr/bin/python3
#!python
import rclpy
from rclpy.node import Node



# import state_representation as sr
import std_msgs.msg
# from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM
# from combined_approach.robot_interface_node import RobotInterfaceNode

# import dynamic_obstacle_avoidance as doa
# mabye make into a libary instead of a local import
# install script 

from visualization_msgs.msg import Marker , MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String


from robot_controller import FrankaRobotPublisher
from obstacles_environment import ObstaclePublisher
from obstacle_avoidance import AvoidancePublisher
from test import FrameListener



class MainController(Node):
    def __init__ (self):
        self.franka = FrankaRobotPublisher()
        self.obstacles_publisher = ObstaclePublisher()
        self.obstacle_avoidance = AvoidancePublisher(self.franka, self.obstacles_publisher)

        super().__init__('main_controller')

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        rclpy.spin_once(self.franka)
        rclpy.spin_once(self.obstacles_publisher)
        rclpy.spin_once(self.obstacle_avoidance)
        
        
def main():
    rclpy.init()
    node = MainController()

    # Print the position of the obstacles

    # for ii, obs in enumerate (node.obstacles_publisher.obstacles_array.markers):
        
    #     x= obs.pose.position.x
    #     y= obs.pose.position.y
    #     z= obs.pose.position.z  

    #     if obs.type == Marker.SPHERE:
    #         print("Sphere Position: "+ str(x) + " "+str(y)+" "+str(z))
    #     elif obs.type == Marker.CUBE:
    #         print("Cube Postions: "+ str(x) + " "+str(y)+" "+str(z))


    # print(node.obstacles_publisher.obstacles_array)
    # position_obs = node.obstacles_publisher.
    # breakpoint()

    try:
        rclpy.spin(node)                        # calls all the nodes
    except KeyboardInterrupt:
        node.stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
