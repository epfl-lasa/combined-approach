import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import String




class ControlPoint: 
    def __init__(self, position, radius):
        self.position=position
        self.radius=radius

class RigidLink:
    def __init__(self, control_point_list):
        self.control_point_list= control_point_list
        


class FrankaRobotPublisher (Node):
    def __init__(self):
        super().__init__('franka_markers')

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.link_dict = {}
        self.control_publishers = []
        self.marker_object_list = []

        self.frame_id_base = "_frankalink"
        new_link = RigidLink([
            ControlPoint([0.0 ,0.0, 0.0], 0.1),
            ControlPoint([0.0 ,0.0, -0.2], 0.1),
            ])



        self.add_link (
            frame_id = self.frame_id_base + "8",
            link = new_link

            ) 
    
    def add_link(self,frame_id, link : RigidLink ):
        
        self.link_dict[frame_id ] = link

        for ii, cp in enumerate (link.control_point_list) :
            self.control_publishers.append(self.create_publisher(Marker,'/control_point'+frame_id+"_"+str(ii),1))
            self.marker_object = Marker()
            self.marker_object.header.frame_id = frame_id
            
            # self.marker_object.header.stamp    = rospy.get_rostime()
            # self.marker_object.header.stamp    = MarkerPublisher.get_clock().now()

            self.marker_object.ns = "frank_emika"
            # self.marker_object.id = index
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

            self.marker_object.color.r = 1.0
            self.marker_object.color.g = 1.0
            self.marker_object.color.b = 1.0

            # This has to be, otherwise it will be transparent
            self.marker_object.color.a = 1.0

            self.marker_object_list.append(self.marker_object)

    
    def timer_callback(self):
        print('control points publisher')

        for ii in range (len(self.marker_object_list)):
            self.control_publishers[ii].publish(self.marker_object_list[ii])
            

        

class MarkerPublisher(Node):

    def __init__(self):
        super().__init__('marker_basics')
        self.objectlisher= self.create_publisher(Marker,'/marker_basic',1)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.init_marker(index=0,z_val=0)
        

    def timer_callback(self):
        
        print('publishing marker')
        self.objectlisher.publish(self.marker_object)
        
      

    def init_marker(self, index=0, z_val=0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "/_frankalink8"
        
        # self.marker_object.header.stamp    = rospy.get_rostime()
        # self.marker_object.header.stamp    = MarkerPublisher.get_clock().now()

        self.marker_object.ns = "frank_emika"
        self.marker_object.id = index
        self.marker_object.type = Marker.SPHERE
        self.marker_object.action = Marker.ADD
      
        self.marker_object.pose.position.x = 0.0;
        self.marker_object.pose.position.y = 0.0;
        self.marker_object.pose.position.z = 0.0;

      
        self.marker_object.pose.orientation.x = 0.0
        self.marker_object.pose.orientation.y = 0.0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0
        
        self.marker_object.scale.x = 0.3
        self.marker_object.scale.y = 0.3
        self.marker_object.scale.z = 0.3

        self.marker_object.color.r = 0.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 1.0

        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0

        # If we want it forever, 0, otherwise seconds before desapearing
        # self.marker_object.lifetime = rclpy.Duration(0)


def main(args=None):
    rclpy.init(args=args)
    marker_basic = FrankaRobotPublisher()
    rclpy.spin(marker_basic)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()



# 	#!/usr/bin/env python

# import rospy
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point


# class MarkerBasics(object):

#     def __init__(self):
#         self.marker_objectlisher = rospy.Publisher('/marker_basic', Marker, queue_size=1)
#         self.rate = rospy.Rate(1)
#         self.init_marker(index=0,z_val=0)
    
#     def init_marker(self,index=0, z_val=0):
#         self.marker_object = Marker()
#         self.marker_object.header.frame_id = "/odom"
#         self.marker_object.header.stamp    = rospy.get_rostime()
#         self.marker_object.ns = "haro"
#         self.marker_object.id = index
#         self.marker_object.type = Marker.SPHERE
#         self.marker_object.action = Marker.ADD
        
#         my_point = Point()
#         my_point.z = z_val
#         self.marker_object.pose.position = my_point
        
#         self.marker_object.pose.orientation.x = 0
#         self.marker_object.pose.orientation.y = 0
#         self.marker_object.pose.orientation.z = 0.0
#         self.marker_object.pose.orientation.w = 1.0
#         self.marker_object.scale.x = 1.0
#         self.marker_object.scale.y = 1.0
#         self.marker_object.scale.z = 1.0
    
#         self.marker_object.color.r = 0.0
#         self.marker_object.color.g = 0.0
#         self.marker_object.color.b = 1.0
#         # This has to be, otherwise it will be transparent
#         self.marker_object.color.a = 1.0
            
#         # If we want it for ever, 0, otherwise seconds before desapearing
#         self.marker_object.lifetime = rospy.Duration(0)
    
#     def start(self):
#         while not rospy.is_shutdown():
#             self.marker_objectlisher.publish(self.marker_object)
#             self.rate.sleep()
   

# if __name__ == '__main__':
#     rospy.init_node('marker_basic_node', anonymous=True)
#     markerbasics_object = MarkerBasics()
#     try:
#         markerbasics_object.start()
#     except rospy.ROSInterruptException:
#         pass