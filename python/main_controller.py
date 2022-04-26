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


from robot_controller import FrankaRobotPublisher



class MainController(Node):
    def __init__ (self):
        self.franka = FrankaRobotPublisher()



def main():
    rclpy.init()
    node = MainController()
    breakpoint()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
