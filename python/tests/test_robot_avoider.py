import numpy as np

import rclpy
from visualization_msgs.msg import Marker

from robot_arm_avoider import RobotArmAvoider

from franka_robot_base import RigidLink

from franka_robot_publisher import FrankaRobotPublisher
from obstacle_publisher import ObstaclePublisher
from attractor_publisher import AttractorPublisher


class TestArmAvoider(RobotArmAvoider):
    """Modified version of  RobotArmAvoider class to allow for simplified
    testing without ROS."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._active_joints = ["_frankalink2", "_frankalink4"]

        self.franka.control_point_array.markers = []
        for key in self.franka.link_dict.keys():
            if key not in self._active_joints:
                self.franka.link_dict[key] = RigidLink([], link_id=key[-1])
                continue

            self.franka.add_link(frame_id=key, link=self.franka.link_dict[key])

    def update_modulation_control(self, it_joint):
        frame_name = self.robot.get_joint_frames()[it_joint]
        if frame_name not in self._active_joints:
            return

        super().update_modulation_control(it_joint)

    def update_correction_control(self, it_joint):
        frame_name = self.robot.get_joint_frames()[it_joint]
        if frame_name not in self._active_joints:
            return

        super().update_correction_control(it_joint)

    def update_initial_control(self):
        super().update_initial_control()

        for it_joint in range(self.n_links):
            frame_name = self.robot.get_joint_frames()[it_joint]
            if frame_name not in self._active_joints:
                continue

            self.initial_control_velocities[it_joint] = 0


def test_main_class():
    from main_executor import StandardExecutor

    from obstacle_publisher import Obstacle, Obstacles

    obstacles_array = Obstacles(
        [Obstacle([0.5, 0.0, 0.25], 0.3)],
        obstacle_type=Marker.SPHERE,
    )

    rclpy.init()

    try:
        attractor_position = np.array([0.5, 0.5, 0.0])

        franka_publisher = FrankaRobotPublisher()
        obstacles_publisher = ObstaclePublisher(obstacles_array=obstacles_array)
        robot_arm_avoider = TestArmAvoider(
            franka_publisher, obstacles_publisher, target_position=attractor_position
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
    pass


if __name__ == "__main__":
    test_main_class()
