#!/usr/bin/python3
#!python

import rclpy
import math

from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String


class Obstacle:
    def __init__(self, position, radius_length):
        self.position = position
        self.radius_length = radius_length


class Obstacles:
    def __init__(self, obstacle_array, obstacle_type):
        self.obstacle_array = obstacle_array
        self.obstacle_type = obstacle_type


class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__("obstacle_node")
        print("obstalce init")

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.obstacles_dict = {}
        self.obstacles_publisher = self.create_publisher(MarkerArray, "/obstacles", 5)
        self.obstacles_array = MarkerArray()
        self.frame_id_base = "world"
        self.i = 0
        self.j = 1

        spheres = Obstacles(
            [Obstacle([0.0, 0.5, 0.0], 0.3), Obstacle([0.3, -0.5, 0.0], 0.3)],
            obstacle_type=Marker.SPHERE,
        )
        self.add_obstacle(frame_id=self.frame_id_base, obstacles=spheres)

        cubes = Obstacles(
            [
                Obstacle([0.3, 0.0, 0.15], 0.3)
                # Obstacle([0.0 ,-0.125, 0.0], 0.18),
            ],
            obstacle_type=Marker.CUBE,
        )
        self.add_obstacle(frame_id=self.frame_id_base, obstacles=cubes)

    def add_obstacle(self, frame_id, obstacles: Obstacles):

        self.obstacles_dict[frame_id] = obstacles
        i = obstacles.obstacle_type * 10

        for ii, obs in enumerate(obstacles.obstacle_array):
            self.marker_object = Marker()
            self.marker_object.header.frame_id = frame_id

            # self.marker_object.header.stamp    = rospy.get_rostime()
            # self.marker_object.header.stamp    = MarkerPublisher.get_clock().now()

            self.marker_object.ns = "frank_emika"

            self.marker_object.id = ii + i
            self.marker_object.type = obstacles.obstacle_type
            self.marker_object.action = Marker.ADD

            self.marker_object.pose.position.x = obs.position[0]
            self.marker_object.pose.position.y = obs.position[1]
            self.marker_object.pose.position.z = obs.position[2]

            self.marker_object.pose.orientation.x = 0.0
            self.marker_object.pose.orientation.y = 0.0
            self.marker_object.pose.orientation.z = 0.0
            self.marker_object.pose.orientation.w = 1.0

            self.marker_object.scale.x = obs.radius_length
            self.marker_object.scale.y = obs.radius_length
            self.marker_object.scale.z = obs.radius_length

            self.marker_object.color.r = 0.623529412
            self.marker_object.color.g = 0.439215686
            self.marker_object.color.b = 0.278431373

            # This has to be, otherwise it will be transparent
            self.marker_object.color.a = 1.0

            self.obstacles_array.markers.append(self.marker_object)

    def get_gamma(self):
        

    def timer_callback(self):
        print("2. OBSTACLES ")
        sinus_value = math.sin(self.i / 10) / 2
        for ii, obs in enumerate(self.obstacles_array.markers):

            if obs.type == Marker.SPHERE:
                obs.pose.position.z = sinus_value * self.j + 0.5 + 0.12

                self.j *= -1
            elif obs.type == Marker.CUBE:
                pass
                # obs.pose.position.z=-sinus_value

        self.obstacles_publisher.publish(self.obstacles_array)
        self.i += 1

        # print(self.obstacles_array.markers)


def main(args=None):
    rclpy.init(args=args)
    marker_basic = ObstaclePublisher()
    rclpy.spin(marker_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
