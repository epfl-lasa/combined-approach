import rclpy
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String


class MarkerArrayPublisher(Node):
    def __init__(self):
        super().__init__("marker_array")
        self.objectlisher = self.create_publisher(MarkerArray, "/marker_array", 1)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.init_marker(index=0, z_val=0)

    def timer_callback(self):

        print("publishing marker array")
        self.objectlisher.publish(self.marker_object)

    def init_marker_array(self, index=0):

        self.marker_object = Marker()
        self.marker_object.header.frame_id = "/_frankalink7"

        # self.marker_object.header.stamp    = rospy.get_rostime()
        # self.marker_object.header.stamp    = MarkerPublisher.get_clock().now()

        self.marker_object.ns = "frank_emika"
        self.marker_object.id = index
        self.marker_object.type = Marker.SPHERE
        self.marker_object.action = Marker.ADD

        self.marker_object.pose.position.x = 0.0
        self.marker_object.pose.position.y = 0.0
        self.marker_object.pose.position.z = 0.0

        self.marker_object.pose.orientation.x = 0.0
        self.marker_object.pose.orientation.y = 0.0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0

        self.marker_object.scale.x = 0.1
        self.marker_object.scale.y = 0.1
        self.marker_object.scale.z = 0.1

        self.marker_object.color.r = 1.0
        self.marker_object.color.g = 1.0
        self.marker_object.color.b = 1.0

        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0

        # If we want it forever, 0, otherwise seconds before desapearing
        # self.marker_object.lifetime = rclpy.Duration(0)


def main(args=None):
    rclpy.init(args=args)
    marker_basic = MarkerArrayPublisher()
    rclpy.spin(marker_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
