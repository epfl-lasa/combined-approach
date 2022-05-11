#!/usr/bin/python3
#!python
import rclpy
import math

from rclpy.node import Node

from visualization_msgs.msg import Marker
from std_msgs.msg import String


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("marker_basic")
        self.objectlisher = self.create_publisher(Marker, "/marker_basic", 1)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.init_marker(index=0, x_val=0.0, y_val=0.0, z_val=0.0)

    def timer_callback(self):

        print("publishing marker")

        # if self.i == max_val:
        # self.i=0
        sinus_value = (math.sin(self.i / 10) + 1 + 0.14) / 2
        print(sinus_value)
        self.marker_object.pose.position.z = sinus_value
        self.objectlisher.publish(self.marker_object)
        self.i += 1

    def init_marker(self, index=0, x_val=0.0, y_val=0.0, z_val=0.0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "world"

        # self.marker_object.header.stamp    = rospy.get_rostime()
        # self.marker_object.header.stamp    = MarkerPublisher.get_clock().now()

        self.marker_object.ns = "frank_emika"
        self.marker_object.id = index
        self.marker_object.type = Marker.SPHERE
        self.marker_object.action = Marker.ADD

        self.marker_object.pose.position.x = 0.3
        self.marker_object.pose.position.y = 0.3
        self.marker_object.pose.position.z = 0.0

        self.marker_object.pose.orientation.x = x_val
        self.marker_object.pose.orientation.y = y_val
        self.marker_object.pose.orientation.z = z_val
        self.marker_object.pose.orientation.w = 1.0

        self.marker_object.scale.x = 0.2
        self.marker_object.scale.y = 0.2
        self.marker_object.scale.z = 0.2

        self.marker_object.color.r = 1.0
        self.marker_object.color.g = 4.0
        self.marker_object.color.b = 1.0

        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0

        # If we want it forever, 0, otherwise seconds before desapearing
        # self.marker_object.lifetime = rclpy.Duration(0)


def main(args=None):
    rclpy.init(args=args)
    marker_basic = MarkerPublisher()
    rclpy.spin(marker_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
