import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            JointState, "franka/joint_states", self.listener_callback, 10
        )
        # self.subscription = self.create_subscription(
        #     String,
        #     'topic',
        #     self.listener_callback,
        #     10)
        # self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(
            Float64MultiArray, "franka/velocity_controller/command", 10
        )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        print("finished")

    def run(self):
        pass

    def listener_callback(self, msg):
        print("finished")
        self.get_logger().info('I heard: "%s"' % msg.position)

    def timer_callback(self):
        msg2 = Float64MultiArray()
        msg2.data = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg = String()
        msg.data = "Hello World: %d" % self.i
        # self.publisher_.publish(msg)
        self.publisher_.publish(msg2)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_subscriber.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
