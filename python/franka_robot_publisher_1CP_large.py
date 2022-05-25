#!/usr/bin/python3
#!python

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class ControlPoint:
    def __init__(self, position, radius):
        self.position = position
        self.radius = radius


class RigidLink:
    def __init__(self, control_point_list, link_id):
        self.control_point_list = control_point_list
        self.link_id = link_id


class FrankaRobotPublisher(Node):
    def __init__(self):

        super().__init__("franka_markers")

        # Declare and acquire `target_frame` parameter
        self.declare_parameter("world", "_frankalink8")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initialize_control_point()
        self.subscription = self.create_subscription(
            JointState, "franka/joint_states", self.callback_jointstate, 3
        )
        # self.subscription  # prevent unused variable warning

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def initialize_control_point(self):
        self.link_dict = {}
        self.control_publisher = self.create_publisher(
            MarkerArray, "/control_points", 5
        )
        self.control_point_array = MarkerArray()

        # self.control_publishers = []
        # self.marker_object_list = []

        self.br = TransformBroadcaster(self)
        self.frame_id_base = "_frankalink"
        self.cp_radius = 0.3

        link_0 = RigidLink(
            [ControlPoint([0.0, 0.0, 0.08], self.cp_radius)],
            link_id=0,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_0.link_id), link=link_0)

        link_1 = RigidLink(
            [ControlPoint([0.0, -0.035, -0.0], self.cp_radius)],
            link_id=1,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_1.link_id), link=link_1)

        link_2 = RigidLink(
            [ControlPoint([0.0, -0.074, 0.03], self.cp_radius)],
            link_id=2,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_2.link_id), link=link_2)

        link_3 = RigidLink(
            [ControlPoint([0.05, 0.03, -0.02], self.cp_radius)],
            link_id=3,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_3.link_id), link=link_3)

        link_4 = RigidLink(
            [ControlPoint([-0.07, 0.03, 0.02], self.cp_radius)],
            link_id=4,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_4.link_id), link=link_4)

        link_5 = RigidLink(
            [ControlPoint([0.0, 0.05, -0.11], self.cp_radius)],
            link_id=5,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_5.link_id), link=link_5)

        link_6 = RigidLink(
            [ControlPoint([0.08, 0.0, 0.05], self.cp_radius)],
            link_id=6,
        )
        self.add_link(frame_id=self.frame_id_base + str(link_6.link_id), link=link_6)

        # link_7 = RigidLink([ControlPoint([0.1, 0.0, self.cp_radius], 0.1)], link_id=7)
        # self.add_link(frame_id=self.frame_id_base + str(link_7.link_id), link=link_7)

        # link_8 = RigidLink([ControlPoint([0.0, 0.0, 0.0], self.cp_radius)], link_id=8)
        # self.add_link(frame_id=self.frame_id_base + str(link_8.link_id), link=link_8)

    def add_link(self, frame_id, link: RigidLink):

        self.link_dict[frame_id] = link
        i = link.link_id * 10

        for ii, cp in enumerate(link.control_point_list):
            self.marker_object = Marker()
            self.marker_object.header.frame_id = frame_id

            self.marker_object.ns = "frank_emika"

            self.marker_object.id = ii + i
            self.marker_object.type = Marker.SPHERE
            self.marker_object.action = Marker.ADD

            self.marker_object.pose.position.x = cp.position[0]
            self.marker_object.pose.position.y = cp.position[1]
            self.marker_object.pose.position.z = cp.position[2]

            self.marker_object.pose.orientation.x = 0.0
            self.marker_object.pose.orientation.y = 0.0
            self.marker_object.pose.orientation.z = 0.0
            self.marker_object.pose.orientation.w = 1.0

            self.marker_object.scale.x = cp.radius
            self.marker_object.scale.y = cp.radius
            self.marker_object.scale.z = cp.radius

            self.marker_object.color.r = 2.0
            self.marker_object.color.g = 0.0
            self.marker_object.color.b = 3.0

            # This has to be, otherwise it will be transparent
            self.marker_object.color.a = 1.0

            self.control_point_array.markers.append(self.marker_object)

    def get_transformation(self, from_frame_rel, to_frame_rel):
        try:
            now = rclpy.time.Time()
            # now = self.get_clock().now().to_msg()

            dur = Duration()
            dur.sec = 1
            dur.nsec = 0

            trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, now)
            # print(now)
            # print(trans.header.frame_id)
            # breakpoint()
            return trans

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
            )
            return None

    def get_end_effector_position(self):
        ee_trans = self.get_transformation("_frankalink8", "world")

        if ee_trans is None:
            return None

        ee_pos = [
            ee_trans.transform.translation.x,
            ee_trans.transform.translation.y,
            ee_trans.transform.translation.z,
        ]

        return ee_pos

    def set_end_effector_position(self, position):
        pass
        # print(self.msg_jointstate.pose)
        # self.msg_jointstate.pose
        # how do I make the whole system change to a specific position based on where I want the end effector to be

    def callback_jointstate(self, msg):
        self.msg_jointstate = msg

    #     # print("callback_jointstate")
    #     # print('header', msg.header.stamp)
    #     # print(self.get_clock().now().to_msg())

    #     # self.joint_positions = msg.position
    #     # self.joint_velocity = msg.velocity
    #     # self.joint_effort = msg.effort

    def timer_callback(self):
        print("1. CONTROL POINTS")
        self.control_publisher.publish(self.control_point_array)
        # self.get_end_effector_position()

        self._done = True

    def spin_short(self):
        self._done = False
        rclpy.spin_until_future_complete(self, self)

    def done(self):
        return self._done


def main(args=None):
    rclpy.init(args=args)
    marker_basic = FrankaRobotPublisher()
    rclpy.spin(marker_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
