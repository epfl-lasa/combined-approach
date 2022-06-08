#!/usr/bin/python3
""" Control Node which Allows to Modulate a RobotArm."""
from __future__ import print_function

import os
import logging
import copy
import time

import numpy as np
from numpy import linalg as LA
from scipy.spatial.transform import Rotation

import rclpy

# from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

# from rclpy.parameter import Parameter
# import std_msgs
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# import pinocchio

# from pinocchio.robot_wrapper import RobotWrapper
import state_representation as sr
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE

from dynamic_obstacle_avoidance.avoidance import ModulationAvoider
# from vartools.dynamical_systems import LinearSystem

from combined_approach.robot_interface_node import RobotInterfaceNode

from angular_ds import AngularDynamics, LinearDynamics

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
        timer_period=0.15,
        robot_name="franka",
        attractor_publisher=None,
        target_position=None,
        target_orientation=None,
        visualize_modulation=False,
    ):
        logging.info(f"Starting main-avoier with robot={robot_name}.")
        # robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        publish_topic = "franka/velocity_controller/command"
        subscription_topic = "franka/joint_states"

        super().__init__(node_name, "franka/joint_states", publish_topic)
        self.init_robot_model(robot_name)

        # self.initialize_urdf_from_parameterserver_with_pinocchio()

        if target_position is None:
            target_position = attractor_publisher.attractor_position
            self.attractor_publisher = attractor_publisher
            # target_position = np.array([0.3, 0.0, 0.3])

        if target_orientation is None:
            # target_orientation = np.array([1, 0, 0, 0])
            # target_orientation = [0, 1, 0, 0]
            # target_orientation = [0, 0, 0, 1]
            target_orientation = np.array([1, 0, 0, 0])
            # target_orientation = np.array([0, 0, 0, 1])

        self.franka = franka
        self.obstacles_publisher = obstacles_publisher

        n_links = 7
        self.joint_origins_global = np.zeros((self.dimension, n_links))
        self.world_control_point_list = [[] for ii in range(self.n_links)]

        target = sr.CartesianPose(
            self.robot.get_frames()[-1], self.robot.get_frames()[0]
        )
        target.set_position(target_position)
        target.set_orientation(target_orientation)
        print('target', target)
        
        # gg_linear = 200
        # gg_ang = 100

        # self._ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        # self._ds.set_parameter_value(
        #     "attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE
        # )
        # self._ds.set_parameter_value(
        #     "gain",
        #     [gg_linear, gg_linear, gg_linear, gg_ang, gg_ang, gg_ang],
        #     sr.ParameterType.DOUBLE_ARRAY
        # )
        # print(f"Setup with linear gain: {gg_linear} | angular gain: {gg_ang}.")

        self.linear_velocity_scaling = 2.0
        self.max_linear_velocity = 0.2
        self.angular_velocity_scaling = 10.0
        self.max_angular_velocity = 1.0
        
        self.joint_velocity_scaling = 0.3
        self.max_joint_velocity = 0.2 * np.ones(self.n_links)

        self._ds_position_only = LinearDynamics(
            target_position=target_position,
            gain=self.linear_velocity_scaling,
            clamp=self.max_linear_velocity,
        )

        self._ds_angular_only = AngularDynamics(
            target_orientation=Rotation.from_quat(target_orientation),
            gain=self.angular_velocity_scaling,
            clamp=self.max_angular_velocity,
        )

        self._modulator = ModulationAvoider(
            obstacle_environment=self.obstacles_publisher.obstacle_environment,
        )

        self.control_radii = self.franka.get_control_radii_list()
        self.weights_section = []
        self.weights_link = []

        # TODO: this should be automatically interpreted from the values

        self.initial_control_velocities = np.zeros(self.n_links)
        self.final_control_velocities = np.zeros(self.n_links)

        if visualize_modulation:
            self._visualizer = VisualizeVelocityPublisher(
                robot_arm_avoider=self, franka=franka
            )
        else:
            self._visualizer = None

        self.velocity_publisher = self.create_publisher(
            Float64MultiArray, publish_topic, 1
        )
        
        # self._joint_state = sr.JointState(
            # robot_name=robot_name, joint_names=self.robot.get_joint_frames())
        # self.sub_jointstate = self.create_subscription(
            # JointState, subscription_topic, self.joint_state_callback, 5)
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        print("Controller init done")

        # self.update_local_joint_positions()

    @property
    def n_links(self) -> int:
        return self.joint_origins_global.shape[1]

    def update_local_joint_positions(self) -> None:
        """ Extract the joint positions and orientations form urdf file."""
        from bs4 import BeautifulSoup
        urdf_data = BeautifulSoup(self.robot_description, "xml")

        self.joint_origins_local = np.zeros((self.dimension, self.n_links))
        
        self.joint_axes_local = np.zeros((self.dimension, self.n_links))

        for ii, jointname in enumerate(self.robot.get_joint_frames()):
            joint_urdf = urdf_data.find('joint', {'name': jointname})
            
            origin = joint_urdf.find('origin')
            position = np.array(origin.get('xyz').split()).astype(float)
            # position = np.zeros(3)
            
            # rotation_rpy = np.array(origin.get('rpy').split()).astype(float)
            # rot_joint = Rotation.from_euler('XYZ', rotation_rpy)
            # position = rot_joint.apply(position)
            
            self.joint_origins_local[:, ii] = position

            joint_axis = joint_urdf.find('axis').get('xyz')
            self.joint_axes_local[:, ii] = np.array(joint_axis.split()).astype(float)

    def update_robot_kinematics(self) -> None:
        # print("joint pos", self.joint_state.get_positions())
        pass

    def check_convergence(self):
        ee_state = self.robot.forward_kinematics(sr.JointPositions(self.joint_state))
        if not self.attractor_publisher.has_convergend(position=ee_state.get_position()):
            return

        print("Attractor publisher has converged - Switching")
        self.attractor_publisher.switch_double_attractor(ee_state.get_position())
            
        self._ds_position_only.target_position = self.attractor_publisher.attractor_position
        # self._ds.set_parameter_value(
        #     "attractor", target, sr.ParameterType.STATE,
        #     sr.StateType.CARTESIAN_POSE
        # )

    def timer_callback(self) -> None:
        """The joint-control velocities in an ascending order (starting from the base)"""
        time_start = time.perf_counter()
        logging.info("[ROBOT_ARM_AVOIDER] Starting timer_callback.")
        
        if not self.state_received:
            logging.info("Waiting for the first sate.")
            return

        self.check_convergence()
        
        self.update_robot_kinematics()
        self.update_initial_control()

        logging.info("Updating global control points.")
        self.update_control_points()

        logging.info("Getting gamma-weights")
        self.update_gamma_weights()

        logging.info("Getting influence weights")
        self.update_influence_weights()   #

        # print("link weights", repr(np.round(self.weights_link, 1)))
        if self._visualizer:
            self._visualizer.reset_arrays()

        # Iterate over all links and ensure collision free avoidance, while
        # trying to follow the initial control command
        self.final_control_velocities = (
            1 - np.sum(self.weights_link)
        ) * self.initial_control_velocities

        for it_joint in range(0, self.n_links-1):
            if self.weights_link[it_joint]:
                self.update_modulation_control(it_joint)
                # logging.info(f"it={it_joint} : Link modulation done.")

            logging.info(f"it={it_joint} : Partially done.")
            # try:
            self.update_correction_control(it_joint)
            # except:
            #     # TODO: remove after debugging
            #     print("258 | DID NOT WORK (!!!!!)")
            #     raise
            #     breakpoint()
                
            # logging.info(f"it={it_joint} : Update done")

        logging.info("Entering last.")
        # Finally: update the last joint
        if self.weights_link[it_joint=(self.n_links-1)]:
            self.update_modulation_control(it_joint=(self.n_links-1))

        logging.info("Done all link modulations")

        self.initial_control_velocities = np.minimum(
            self.initial_control_velocities, self.max_joint_velocity
        )
        self.initial_control_velocities = (
            self.initial_control_velocities * self.joint_velocity_scaling
        )
                
        # And do the publishing
        self.publish_final_control_velocity()

        if self._visualizer:
            self._visualizer.publish_velocities()

            # self._visualizer.publish_link_joint_array()
        time_stop = time.perf_counter()

        print(f"Total function time: {time_stop-time_start}s.")

    def update_initial_control(self):
        self.ee_twist = self.get_ds_at_endeffector()
        
        logging.info("Got the first twist.")
        initial_control = self.robot.inverse_velocity(
            self.ee_twist, sr.JointPositions(self.joint_state)
        )

        self.initial_control_velocities = initial_control.get_velocities()
        self.initial_control_velocities = np.minimum(
            self.max_joint_velocity, self.initial_control_velocities
        )

    def get_direction_of_joint_rotation(self, it_joint):
        """Returns the direction of rotation of a joint along the robot.
        Assumption to have only rotational joints."""
        # [WARNING]: This is now hard-coded to always be the z-axis.
        # TODO: Investigate the pinocchio to get more information, on the rotation-axis
        # https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html#a64ff8505bc8abbba289b1946c58283ba
        axes_of_rot = [0, 0, 1]

        joint_name = "_frankajoint" + str(int(it_joint + 1))
        joint_pose = self.robot.forward_kinematics(
            joint_positions=sr.JointPositions(self.joint_state),
            # frame_name=joint_name
        )

        orientation = joint_pose.get_orientation()

        orientation_quat = Rotation.from_quat(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        # Transform to global frame
        axes_of_rot = orientation_quat.apply(axes_of_rot)

        # if self._visualizer:
        #     self._visualizer.append_rotation_direction_marker(
        #         vector=axes_of_rot, position=joint_pose.get_position(),
        #     )
            
        return axes_of_rot

    def update_modulation_control(self, it_joint):
        """Update final control to ensure to have collision free velocity of
        each control point."""
        mean_vel_cps = np.zeros(self.dimension)
        mean_omega_cps = np.zeros((self.dimension))
        
        try:
            for ii, control_point in enumerate(self.world_control_point_list[it_joint]):
                if not self.weights_section[it_joint][ii]:  # Zero weight
                    continue

                init_linear_vel = self._ds_position_only.evaluate(control_point)
                mod_velocity = self._modulator.avoid(
                    control_point, velocity=init_linear_vel
                )

                if self._visualizer:
                    self._visualizer.append_modulated_velocity_marker(
                        vector=mod_velocity, position=control_point
                    )

                    scaled_mod_velocity = (
                        mod_velocity
                        * self.weights_section[it_joint][ii]
                        * self.weights_link[it_joint]
                    )

                    self._visualizer.append_scaled_modulated_velocity_marker(
                        vector=scaled_mod_velocity,
                        position=control_point,
                    )
                    
            mean_vel_cps += self.weights_section[it_joint][ii] * mod_velocity
            
        except:
            breakpoint()

        logging.info("359 | Still a way to go.")

        for ii, control_point in enumerate(self.world_control_point_list[it_joint]):
            if not self.weights_section[it_joint][ii]:  # Zero weight
                continue

            mean_omega_cps += self.weights_section[it_joint][ii] * np.cross(
                self.joint_origins_global[:, it_joint] - control_point,
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

        logging.info(f"it={it_joint} : Modulation update done")

    def update_correction_control(self, it_joint):
        """Correct the control of the following link."""
        dir_joint_to_joint = (
            self.joint_origins_global[:, it_joint+1]
            - self.joint_origins_global[:, it_joint]
        )

        if not LA.norm(dir_joint_to_joint):
            return

        # TODO: is the following 'it_joint' correct?
        frame_name = self.robot.get_joint_frames()[it_joint+1]
        delta_control_velocity = np.zeros(self.final_control_velocities.shape)
        delta_control_velocity[:it_joint+1] = (
            self.final_control_velocities[:it_joint+1]
            - self.initial_control_velocities[:it_joint+1]
        )

        robot_delta_state = copy.deepcopy(self.joint_state)
        robot_delta_state.set_velocities(delta_control_velocity)
        twist_delta_control = self.robot.forward_velocity(
            joint_state=robot_delta_state, frame_name=frame_name
        )
        
        delta_omega = np.cross(
            dir_joint_to_joint, twist_delta_control.get_linear_velocity()
        )

        dir_joint = self.get_direction_of_joint_rotation(it_joint)
        delta_control = np.dot(delta_omega, dir_joint)

        self.final_control_velocities[it_joint] += (
            delta_control * (1 - np.sum(self.weights_link[:it_joint]))
        )

        logging.info(f"it={it_joint} : Correction update done")

        if self._visualizer:
            pose_ctrl = self.robot.forward_kinematics(
                joint_positions=sr.JointPositions(robot_delta_state),
                frame_name=frame_name
            )
            
            self._visualizer.append_correction_velocity(
                position=pose_ctrl.get_position(),
                vector=twist_delta_control.get_linear_velocity(),
                marker_id=it_joint
            )
        
    def control_loop(self):
        if self.state_received and rclpy.ok():
            twist = sr.CartesianTwist(
                self._ds.evaluate(
                    self.robot.forward_kinematics(sr.JointPositions(
                        self.joint_state))
                )
            )
            twist.clamp(self.max_linear_velocity, self.max_angular_velocity)
            command = self.robot.inverse_velocity(
                twist, sr.JointPositions(self.joint_state)
            )
            self.publish(command)

    def get_ds_at_endeffector(self):
        ee_state = self.robot.forward_kinematics(sr.JointPositions(self.joint_state))
        # twist = sr.CartesianTwist(self._ds.evaluate(ee_state))
        # twist.clamp(self.max_linear_velocity, self.max_angular_velocity)

        twist = sr.CartesianTwist(name="ee_velocity", reference="world")
        # twist = sr.CartesianTwist()
        
        orientation = ee_state.get_orientation()
        rot_quat = Rotation.from_quat([orientation.x,
                                       orientation.y,
                                       orientation.z,
                                       orientation.w
                                       ])
        
        vel_angular = self._ds_angular_only.evaluate(rot_quat)
        twist.set_angular_velocity(vel_angular)
        
        # velocity = twist.get_linear_velocity()
        velocity = self._ds_position_only.evaluate(ee_state.get_position())
        velocity = self._modulator.avoid(ee_state.get_position(), velocity=velocity)
        twist.set_linear_velocity(velocity)

        return twist

    def update_control_points(self):
        """Get control points in world / link0 frame."""
        # TODO: initialize only once and then update the values
        # self.world_control_point_list = []

        link_start = np.array([0, 0, 0])
        rot = Rotation.from_quat([0, 0, 0, 1])
        
        for it_link, link_id in enumerate(self.franka.link_dict.keys()):
            if int(link_id[-1]) == 8:
                continue

            # The joint is always before the corresponding link
            # self.joint_origins_global[:, it_link] = (
            #     link_start + rot.apply(self.joint_origins_local[:, it_link])
            # )
            
            # Get the new transformation
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

            rot = Rotation.from_quat(
                [
                    trafo.transform.rotation.x,
                    trafo.transform.rotation.y,
                    trafo.transform.rotation.z,
                    trafo.transform.rotation.w,
                ]
            )

            self.joint_origins_global[:, it_link] = link_start
            
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

        # print('gamma', self.gamma_list)

    def get_relative_velocity_factors(self) -> np.array:
        """Returns the factor of the relative velocity to the obstacle."""
        # TODO: smarter way to do this; currently it only looks at the robot velocity
        # Higher up joints have to take into account the lower ones, too
        return np.cumsum(np.abs(self.initial_control_velocities))

    def update_influence_weights(
        self,
        gamma_min: float = 1,
        gamma_cutoff: float = 1e2,
        weight_factor: float = 1,
    ) -> None:
        self.link_factor = 1

        self.weights_section = [[] for _ in range(len(self.gamma_list))]
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
                breakpoint()
                # TODO (remove): this should never be -> already checked before
                continue

            weights_base *= (
                (np.arange(weights_base.shape[0]) + 1) / weights_base.shape[0]
            )

            weight_sum = np.sum(weights_base)
            if weight_sum > 1:
                weights_sec = weights_base / weight_sum
            else:
                weights_sec[-1] = weights_base / weight_sum
                
            self.weights_section[it_joint] = weights_sec

            self.weights_link[it_joint] = (
                self.link_factor
                * velocity_weights[it_joint]
                * (it_joint + 1)
                / self.franka.n_joints
                * max_weight
            )
            
        weight_sum = np.sum(self.weights_link)
        if weight_sum > 1:
            self.weights_link = self.weights_link / weight_sum

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

    def publish_final_control_velocity(self):
        # Scale and crop control velocity
        self.final_control_velocities = (
            self.final_control_velocities * self.joint_velocity_scaling
        )
        self.final_control_velocities = np.minimum(
            self.final_control_velocities, self.max_joint_velocity
        )

        # Publish velocity
        msg = Float64MultiArray()
        # msg.data = self.initial_control_velocities.tolist()
        msg.data = self.final_control_velocities.tolist()
        
        # self.velocity_publisher.publish(msg)
        self.publisher.publish(msg)

    def destroy_node(self):
        """Send zero-control command before destroying the node."""
        msg = Float64MultiArray()
        msg.data = [0.0 for _ in range(self.n_links)]
        # self.velocity_publisher.publish(msg)
        self.publisher.publish(msg)

        super().destroy_node()

    def send_zero_command(self):
        msg = Float64MultiArray()
        msg.data = [0.0 for _ in range(self.n_links)]
        # self.velocity_publisher.publish(msg)
        self.publisher.publish(msg)


def main(args=None):
    from main_executor import StandardExecutor

    rclpy.init(args=args)

    from obstacle_publisher import Obstacle, Obstacles
    from visualization_msgs.msg import Marker

    obstacles_array = Obstacles(
        [Obstacle([0.3, 0.0, -0.1], 0.5)],
        obstacle_type=Marker.SPHERE,
    )

    try:
        attractor_position = np.array([0.4, 0.4, 0.0])
        attractor_publisher = AttractorPublisher(attractor_position)
        
        franka_publisher = FrankaRobotPublisher()
        obstacles_publisher = ObstaclePublisher(obstacles_array)
        robot_arm_avoider = RobotArmAvoider(
            franka_publisher,
            obstacles_publisher,
            attractor_publisher=attractor_publisher,
            visualize_modulation=True,  # DEBUGGING
        )

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
    # logging.basicConfig(level=logging.WARNING)
    logging.basicConfig(level=logging.INFO)
    # logger.setLevel(logging.INFO)
    # get_franka()
    main()
