#!/usr/bin/python3
#!python

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker , MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String




class ControlPoint: 
    def __init__(self, position, radius):
        self.position=position
        self.radius=radius

class RigidLink:
    def __init__(self, control_point_list, link_id):
        self.control_point_list= control_point_list
        self.link_id = link_id
        


class FrankaRobotPublisher (Node):
    def __init__(self):
        super().__init__('franka_markers')

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.link_dict = {}
        self.control_publisher=self.create_publisher(MarkerArray,'/control_points',5)
        self.control_point_array = MarkerArray()

        # self.control_publishers = []
        # self.marker_object_list = []



        self.frame_id_base = "_frankalink"

 

        link_1 = RigidLink([
            ControlPoint([0.0 ,0.0, -0.15], 0.18),
            ControlPoint([0.0 ,-0.075, 0.01], 0.18),
            ],
            link_id=1)
        self.add_link (
            frame_id = self.frame_id_base + str(link_1.link_id),
            link = link_1
            ) 

        

        link_2 = RigidLink([
            ControlPoint([0.0 ,-0.007, 0.07], 0.18),
            ControlPoint([0.0 ,-0.125, 0.0], 0.18),
            ],
            link_id=2)
        self.add_link (
            frame_id = self.frame_id_base + str(link_2.link_id),
            link = link_2
            )

        link_3 = RigidLink([
            ControlPoint([0.0 ,0.0, -0.075], 0.15),
            ControlPoint([0.09 ,0.07, 0.0], 0.18),
            ],
            link_id=3)
        self.add_link (
            frame_id = self.frame_id_base + str(link_3.link_id),
            link = link_3
            )
        
        link_4 = RigidLink([
            ControlPoint([0.003 ,0.0, 0.066], 0.18),
            ControlPoint([-0.076 ,0.076, 0.0], 0.15),
            ],
            link_id=4)
        self.add_link (
            frame_id = self.frame_id_base + str(link_4.link_id),
            link = link_4
            )

        link_5 = RigidLink([
            ControlPoint([0.0 ,0.0, -0.21], 0.15),
            ControlPoint([0.0 ,0.08,-0.129], 0.16),
            ControlPoint([0.0 ,0.073, 0.0], 0.15),

            ],
            link_id=5)
        self.add_link (
            frame_id = self.frame_id_base + str(link_5.link_id),
            link = link_5
            )
        
        link_6 = RigidLink([
            ControlPoint([0.0 ,0.0, 0.042], 0.15),
            ControlPoint([0.1 ,0.019, 0.0], 0.16),
            ],
            link_id=6)
        self.add_link (
            frame_id = self.frame_id_base + str(link_6.link_id),
            link = link_6
            )

        link_7 = RigidLink([
            ControlPoint([0.0 ,0.0, 0.076], 0.1),
            ],
            link_id=7)
        self.add_link (
            frame_id = self.frame_id_base + str(link_7.link_id),
            link = link_7
            )

        link_8 = RigidLink([
            ControlPoint([0.0 ,0.0, 0.0], 0.1),
            ],
            link_id=8)
        self.add_link (
            frame_id = self.frame_id_base + str(link_8.link_id),
            link = link_8
            )    
    
    def add_link(self,frame_id, link : RigidLink ):
        
        self.link_dict[frame_id ] = link
        i = link.link_id*10

        for ii, cp in enumerate(link.control_point_list):
            self.marker_object = Marker()
            self.marker_object.header.frame_id = frame_id
            
            # self.marker_object.header.stamp    = rospy.get_rostime()
            # self.marker_object.header.stamp    = MarkerPublisher.get_clock().now()

            self.marker_object.ns = "frank_emika"
            
            self.marker_object.id = ii +i
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
            self.marker_object.color.g = 5.0
            self.marker_object.color.b = 0.0

            # This has to be, otherwise it will be transparent
            self.marker_object.color.a = 1.0

            self.control_point_array.markers.append(self.marker_object)

    
    def timer_callback(self):
        print('control points publisher')
        self.control_publisher.publish(self.control_point_array)
            

    


def main(args=None):
    rclpy.init(args=args)
    marker_basic = FrankaRobotPublisher()
    rclpy.spin(marker_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()

