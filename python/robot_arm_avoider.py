#!/usr/bin/python3
from __future__ import print_function

import os
from threading import Thread

import numpy as np
from numpy import linalg as LA

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter

import pinocchio
from pinocchio.robot_wrapper import RobotWrapper

import state_representation as sr
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from combined_approach.robot_interface_node import RobotInterfaceNode

from main_executor import StandardExecutor
from franka_robot_publisher import FrankaRobotPublisher
from obstacle_publisher import ObstaclePublisher


class RobotArmAvoider(RobotInterfaceNode):
    
    def __init__(
        self, franka,
        obstacles_publisher,
        node_name="obstacle_avoider",
        timer_period=0.1,
        robot_name="franka"
    ):
        print("Start main-avoider.")
        
        super().__init__(node_name, "joint_states", "velocity_controller/command")

        # robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        
        print(robot_name)
        self.init_robot_model(robot_name)
        
        self.franka = franka
        self.obstacles_publisher = obstacles_publisher

        target = sr.CartesianPose(
            self.robot.get_frames()[-1], self.robot.get_frames()[0]
        )
        target.set_position([0.6, +0.3, 0.5])
        target.set_orientation([0, 1, 0, 0])
        self._ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        # self._ds.set_parameter_value(
            # "attractor", target, sr.StateType.PARAMETER_CARTESIANPOSE
        # )
        # self._ds.set_parameter_value(
            # "gain", [50, 50, 50, 10, 10, 10],
            # sr.StateType.PARAMETER_DOUBLE_ARRAY
        # )
        
        # Seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.control_radii = self.franka.get_control_radii_list()
        self.weights_section = []
        self.weights_link = []

    def initialize_urdf_from_parameterserver_with_pinocchio(self):
        # => this is currently not used anymore
        self.client = self.create_client(
            GetParameters, 'franka/robot_state_publisher/get_parameters'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('This service not available, waiting again...')
            
        request = GetParameters.Request()
        request.names = ['robot_description']
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        urdf_string = future.result().values[0].string_value

        # Create temp-file to store it for later use with pinocchio
        urdf_filename = os.path.join("/tmp", request.names[0] + ".urdf")
        with open(urdf_filename, "w") as file:
            file.write(urdf_string)

        self.robot_model = pinocchio.buildModelFromUrdf(urdf_filename)
        

    def get_forward_kinematics(self,):
        pass
    
    def get_inverse_kinematics(self, velocity, link=None):
        # self.
        pass

    def get_modulated_ds_at_position(self, position):
        velocity = self.nominal_dynamics(position)
        return self.obstacle_publisher.avoid(position, velocity)

    def run_avoider(self):
        attractor_position = self.franka.get_end_effector_position()
        modulated_velocity = self.get_modulated_ds_at_position(
            attractor_position
        )
        self.get_inverse_kinematics(modulated_velocity)
        

        self.get_gamma_control_points()
        
        self.get_influence_weights()

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

    def get_influence_weights(self, gamma_min=1, gamma_cutoff=1e2):
        self.link_factor = 1
        
        self.weights_section = []
        self.weights_link = np.zeros(len(self.gamma_list))

        for it_joint, g_list in enumerate(self.gamma_list):
            g_list = np.array(g_list)
            weights_base = np.zeros(g_list.shape[1])
            
            ind_range = g_list < gamma_cutoff

            if not np.sum(ind_range):
                # All are too far away
                continue

            weights_base = (gamma_cutoff - gamma_min) / (g_list - gamma_min)

            max_weight = np.max(weights_base)
            if not max_weight:
                self.weight_section.append(np.zeros(self.section_weights.shape))
                
            self.weights_section.append(
                (np.arange(weights_base.shape)+1)
                / self.section_weights
                * weights_base
            )
            
            # Normalize
            self.weights_section = self.weights_section / LA.norm(
                self.weights_section
            )

            self.weights_link[it_joint] = (
                self.link_factor *
                abs(self.joint_vel_goal[it_joint])
                (it_joint+1) / self.franka.n_links * max_weight
            )

        weight_sum = np.sum(self.weights_link)
        if weight_sum:
            self.weights_link = self.weights_link / weight_sum
            
    def timer_callback(self):
        print("ROBOT_ARM_AVOIDER.")

        self.get_gamma_control_points()


    def publish(self, command):
        msg = std_msgs.msg.Float64MultiArray()
        msg.data = command.get_velocities().tolist()
        self.publisher.publish(msg)


    def stop(self):
        self.publish(
            sr.JointVelocities().Zero(
                self.robot.get_robot_name(), self.robot.get_number_of_joints()
            )
        )
        rclpy.shutdown()

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
