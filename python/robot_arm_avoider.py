#!/usr/bin/python3
from __future__ import print_function

import os
import logging
import copy

# from threading import Thread

import numpy as np
from numpy import linalg as LA
from scipy.spatial.transform import Rotation

import rclpy

# from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

# from rclpy.parameter import Parameter
# import std_msgs
from std_msgs.msg import Float64MultiArray

import pinocchio

# from pinocchio.robot_wrapper import RobotWrapper
import state_representation as sr
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE

from dynamic_obstacle_avoidance.avoidance import ModulationAvoider
from vartools.dynamical_systems import LinearSystem

from combined_approach.robot_interface_node import RobotInterfaceNode

from franka_robot_publisher import FrankaRobotPublisher
from obstacle_publisher import ObstaclePublisher
from attractor_publisher import AttractorPublisher

from debug_velocity_publisher import VisualizeVelocityPublisher


class RobotArmAvoider(RobotInterfaceNode):
    dimension = 3

    def __init__(
        self,
        franka,
        obstacles_publisher,
        node_name="obstacle_avoider",
        timer_period=0.1,
        robot_name="franka",
        target_position=None,
        target_orientation=None,
        visualize_modulation=False,
        max_linear_velocity=0.25,
    ):
        logging.info(f"Starting main-avoier with robot={robot_name}.")
        # robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        publish_topic = "franka/velocity_controller/command"
        super().__init__(node_name, "joint_states", publish_topic)
        self.init_robot_model(robot_name)

        # self.initialize_urdf_from_parameterserver_with_pinocchio()

        if target_position is None:
            target_position = [0.5, 0.0, 0.5]

        if target_orientation is None:
            target_orientation = [0, 0, 0, -1]

        self.max_linear_velocity = max_linear_velocity

        self.franka = franka
        self.obstacles_publisher = obstacles_publisher

        target = sr.CartesianPose(
            self.robot.get_frames()[-1], self.robot.get_frames()[0]
        )
        target.set_position(target_position)
        target.set_orientation(target_orientation)
        self._ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)

        self._ds.set_parameter_value(
            "attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE
        )

        self._ds.set_parameter_value(
            "gain", [50, 50, 50, 10, 10, 10], sr.ParameterType.DOUBLE_ARRAY
        )

        self._ds_position_only = LinearSystem(
            attractor_position=target_position, maximum_velocity=max_linear_velocity
        )

        self._modulator = ModulationAvoider(
            obstacle_environment=self.obstacles_publisher.obstacle_environment,
        )

        self.control_radii = self.franka.get_control_radii_list()
        self.weights_section = []
        self.weights_link = []

        # TODO: this should be automatically interpreted from the values
        n_links = 7
        self.link_trunk_array = np.zeros((self.dimension, n_links))
        self.world_control_point_list = [None for ii in range(self.n_links)]

        self.initial_control_velocities = np.zeros(self.n_links)
        self.final_control_velocities = np.zeros(self.n_links)

        if visualize_modulation:
            self._visualizer = VisualizeVelocityPublisher(
                robot_arm_avoider=self, franka=franka
            )
        else:
            self._visualizer = None

        self.velocity_publisher = self.create_publisher(
            Float64MultiArray, publish_topic, 5
        )
        self.timer = self.create_timer(timer_period, self.timer_callback)

        print("Controller init done")

    @property
    def n_links(self) -> int:
        return self.link_trunk_array.shape[1]

    def timer_callback(self):
        """The joint-control velocities in an ascending order (starting from the base)"""
        logging.info("[ROBOT_ARM_AVOIDER] Starting timer_callback.")
        self.update_initial_control()

        logging.info("Updating global control points.")
        self.update_control_points()

        logging.info("Getting gamma-weights")
        self.update_gamma_weights()

        logging.info("Getting influence weights")
        self.update_influence_weights()

        print("link weights", repr(np.round(self.weights_link, 1)))

        if self._visualizer:
            self._visualizer.reset_initial_array()
            self._visualizer.reset_modulated_array()

        # Iterate over all links and ensure collision free avoidance, while
        # trying to follow the initial control command
        self.final_control_velocities = (
            1 - np.sum(self.weights_link)
        ) * self.initial_control_velocities

        self.update_modulation_control(it_joint=0)
        logging.info("First link modulation done.")

        for it_joint in range(1, self.n_links):
            if not self.weights_link[it_joint]:
                continue

            self.update_correction_control(it_joint)
            self.update_modulation_control(it_joint)

        breakpoint()
        self.publish_final_control_velocity()
        # print('publish', self.final_control_velocities)

        if self._visualizer:
            self._visualizer.publish_velocities()

    def update_initial_control(self):
        self.ee_twist = self.get_modulated_ds_at_endeffector()

        # breakpoint()
        logging.info("Got the first twist.")
        initial_control = self.robot.inverse_velocity(
            self.ee_twist, sr.JointPositions(self.joint_state)
        )

        self.initial_control_velocities = initial_control.get_velocities()

    def get_direction_of_joint_rotation(self, it_joint):
        """Returns the direction of rotation of a joint along the robot.
        Assumption to have only rotational joints."""
        # [WARNING]: This is now hard-coded to always be the z-axis.
        # TODO: Investigate the pinocchio to get more information, on the rotation-axis
        # https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html#a64ff8505bc8abbba289b1946c58283ba
        axes_of_rot = [0, 0, 1]

        joint_name = "_frankajoint" + str(int(it_joint + 1))
        joint_pose = self.robot.forward_kinematics(
            sr.JointPositions(self.joint_state), joint_name
        )

        orientation = joint_pose.get_orientation()

        orientation_quat = Rotation.from_quat(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        # Transform to global frame
        axes_of_rot = orientation_quat.apply(axes_of_rot)
        return axes_of_rot

    def update_modulation_control(self, it_joint):
        """Update final control to ensure to have collision free velocity of
        each control point."""
        mean_vel_cps = np.zeros(self.dimension)
        mean_omega_cps = np.zeros((self.dimension))

        print(f"it join {it_joint}: {self.world_control_point_list[it_joint]}")

        for ii, control_point in enumerate(self.world_control_point_list[it_joint]):
            if not self.weights_section[it_joint][ii]:  # Zero weight
                continue

            # cp_pose = sr.CartesianPose()
            # cp_pose.set_position(control_point)

            # init_twist = sr.CartesianTwist(self._ds.evaluate(cp_pose))
            # init_twist.clamp(0.25, 0.25)
            # init_linear_vel = init_twist.get_linear_velocity()

            init_linear_vel = self._ds_position_only.evaluate(control_point)
            mod_velocity = self._modulator.avoid(
                control_point, velocity=init_linear_vel
            )

            if self._visualizer:
                self._visualizer.append_initial_velocity_marker(
                    vector=init_linear_vel, position=control_point
                )

                self._visualizer.append_modulated_velocity_marker(
                    vector=mod_velocity, position=control_point
                )

                print(f"[it joint | ii] = [{it_joint} | {ii} ]")
                print("init vel", init_linear_vel)
                print("modu vel", mod_velocity)

            mean_vel_cps += self.weights_section[it_joint][ii] * mod_velocity

        for ii, control_point in enumerate(self.world_control_point_list[it_joint]):
            if not self.weights_section[it_joint][ii]:  # Zero weight
                continue

            mean_omega_cps += self.weights_section[it_joint][ii] * np.cross(
                self.link_trunk_array[:, it_joint] - control_point,
                mod_velocity - mean_vel_cps,
            )

        mean_twist = sr.CartesianTwist(
            name="mean_vel",
            linear_velocity=mean_vel_cps,
            angular_velocity=mean_omega_cps,
            reference="world",
        )

        modulation_control = self.robot.inverse_velocity(
            mean_twist, sr.JointPositions(self.joint_state)
        )

        self.final_control_velocities[it_joint] = (
            self.final_control_velocities[it_joint]
            + self.weights_link[it_joint]
            * modulation_control.get_velocities()[it_joint]
        )

    def update_correction_control(self, it_joint):
        """Correct the control of the following link."""
        frame_name = self.robot.get_joint_frames()[it_joint]
        delta_control_velocity = np.zeros(self.final_control_velocities.shape)
        delta_control_velocity[:it_joint] = (
            self.final_control_velocities[:it_joint]
            - self.initial_control_velocities[:it_joint]
        )

        robot_delta_state = copy.deepcopy(self.joint_state)
        robot_delta_state.set_velocities(delta_control_velocity)

        twist_delta_control = self.robot.forward_velocity(
            joint_state=robot_delta_state, frame_name=frame_name
        )

        dir_joint = self.get_direction_of_joint_rotation(it_joint=it_joint)
        delta_omega = np.cross(twist_delta_control.get_linear_velocity(), dir_joint)

        delta_control = np.dot(delta_omega, dir_joint)

        self.final_control_velocities[it_joint] += delta_control * np.sum(
            self.weights_link[: it_joint - 1]
        )

    def control_loop(self):
        if self.state_received and rclpy.ok():
            twist = sr.CartesianTwist(
                self._ds.evaluate(
                    self.robot.forward_kinematics(sr.JointPositions(self.joint_state))
                )
            )
            twist.clamp(self.max_linear_velocity, 0.25)
            command = self.robot.inverse_velocity(
                twist, sr.JointPositions(self.joint_state)
            )
            self.publish(command)

    def get_modulated_ds_at_endeffector(self):
        ee_state = self.robot.forward_kinematics(sr.JointPositions(self.joint_state))
        twist = sr.CartesianTwist(self._ds.evaluate(ee_state))
        twist.clamp(self.max_linear_velocity, 0.25)

        # velocity = twist.get_linear_velocity()
        # velocity = self._modulator.avoid(ee_state.get_position(), velocity=velocity)
        # twist.set_linear_velocity(velocity)

        return twist

    def update_control_points(self):
        """Get control points in world / link0 frame."""
        # TODO: initialize only once and then update the values
        # self.world_control_point_list = []
        for it_link, link_id in enumerate(self.franka.link_dict.keys()):
            if int(link_id[-1]) == 8:
                continue

            trafo = self.franka.get_transformation(link_id, "world")
            if trafo is None:
                self.world_control_point_list.append([])
                logging.warn(
                    "Transformation for link not found."
                    + "There might be unexpected behavior."
                )
                continue

            link_start = np.array(
                [
                    trafo.transform.translation.x,
                    trafo.transform.translation.y,
                    trafo.transform.translation.z,
                ]
            )

            self.link_trunk_array[:, it_link] = link_start

            rot = Rotation.from_quat(
                [
                    trafo.transform.rotation.x,
                    trafo.transform.rotation.y,
                    trafo.transform.rotation.z,
                    trafo.transform.rotation.w,
                ]
            )

            self.world_control_point_list[it_link] = []
            for cp in self.franka.link_dict[link_id].control_point_list:
                position = rot.apply(cp.position)
                self.world_control_point_list[it_link].append(position + link_start)

    def update_gamma_weights(self) -> None:
        self.gamma_list = []
        for cp_list in self.world_control_point_list:
            self.gamma_list.append([])
            for cp in cp_list:
                self.gamma_list[-1].append(self.obstacles_publisher.get_gamma(cp))

    def get_relative_velocity_factors(self) -> np.array:
        """Returns the factor of the relative velocity to the obstacle."""
        # TODO: smarter way to do this; currently it only looks at the robot velocity
        # Higher up joints have to take into account the lower ones, too
        return np.cumsum(np.abs(self.initial_control_velocities))

    def update_influence_weights(
        self,
        gamma_min: float = 1,
        gamma_cutoff: float = 1e2,
        weight_factor: float = 5e-4,
    ) -> None:
        self.link_factor = 1

        self.weights_section = []
        self.weights_link = np.zeros(len(self.gamma_list))

        velocity_weights = self.get_relative_velocity_factors()

        for it_joint, g_list in enumerate(self.gamma_list):
            gamma_array = np.array(g_list)
            weights_base = np.zeros(len(g_list))

            ind_range = gamma_array < gamma_cutoff

            if not np.sum(ind_range):
                # All are too far away
                continue

            weights_base = (gamma_cutoff - gamma_min) / (gamma_array - gamma_min)
            weights_base *= weight_factor

            max_weight = np.max(weights_base)
            if not max_weight:  # Zero weight
                self.weights_section.append(np.zeros(self.section_weights.shape))

            weights_base *= (np.arange(weights_base.shape[0]) + 1) / weights_base.shape[
                0
            ]
            self.weights_section.append(weights_base / LA.norm(weights_base))

            self.weights_link[it_joint] = (
                self.link_factor
                * velocity_weights[it_joint]
                * (it_joint + 1)
                / self.franka.n_joints
                * max_weight
            )

        weight_sum = np.sum(self.weights_link)
        if weight_sum:
            self.weights_link = self.weights_link / weight_sum

    def publish_final_control_velocity(self):
        msg = Float64MultiArray()
        # msg.data = self.final_control_velocities.tolist()
        msg.data = self.initial_control_velocities.tolist()
        self.velocity_publisher.publish(msg)

    def stop(self):
        self.publish(
            sr.JointVelocities().Zero(
                self.robot.get_robot_name(), self.robot.get_number_of_joints()
            )
        )
        rclpy.shutdown()

    def initialize_urdf_from_parameterserver_with_pinocchio(self):
        # => this is currently not used anymore
        self.client = self.create_client(
            GetParameters, "franka/robot_state_publisher/get_parameters"
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("This service not available, waiting again...")

        request = GetParameters.Request()
        request.names = ["robot_description"]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        urdf_string = future.result().values[0].string_value

        # Create temp-file to store it for later use with pinocchio
        urdf_filename = os.path.join("/tmp", request.names[0] + ".urdf")
        with open(urdf_filename, "w") as file:
            file.write(urdf_string)

        self.pinocchio = pinocchio.buildModelFromUrdf(urdf_filename)

    def destroy_node(self):
        msg = Float64MultiArray()
        msg.data = [0.0 for _ in range(self.n_links)]
        self.velocity_publisher.publish(msg)

        super().destroy_node()

    def send_zero_command(self):
        msg = Float64MultiArray()
        msg.data = [0.0 for _ in range(self.n_links)]
        self.velocity_publisher.publish(msg)


def main(args=None):
    from main_executor import StandardExecutor

    rclpy.init(args=args)

    from obstacle_publisher import Obstacle, Obstacles
    from visualization_msgs.msg import Marker

    obstacles_array = Obstacles(
        [Obstacle([0.5, 0.0, 0.25], 0.3)],
        obstacle_type=Marker.SPHERE,
    )

    try:
        attractor_position = np.array([0.5, 0.5, 0.0])
        franka_publisher = FrankaRobotPublisher()
        obstacles_publisher = ObstaclePublisher(obstacles_array)
        robot_arm_avoider = RobotArmAvoider(
            franka_publisher,
            obstacles_publisher,
            target_position=attractor_position,
            visualize_modulation=True,  # DEBUGGING
        )

        attractor_publisher = AttractorPublisher(attractor_position)

        executor = StandardExecutor()
        executor.add_node(franka_publisher)
        executor.add_node(obstacles_publisher)
        executor.add_node(robot_arm_avoider)

        executor.add_node(attractor_publisher)

        try:
            executor.spin()
        finally:
            executor.shutdown()

            franka_publisher.destroy_node()
            obstacles_publisher.destroy_node()
            robot_arm_avoider.destroy_node()

            attractor_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    # logger.setLevel(logging.INFO)
    # get_franka()
    main()
