#!/usr/bin/env python

"""
    Publish a OPW skeleton message as a list of visualization markers for RViz
"""

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from openpose_wrapper.msg import SkeletonGroup

class SkeletonViewer():
    def __init__(self):
        rospy.init_node('skeleton_viewer')
        
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("Initializing Skeleton Viewer Node...")
        
        self.sensor_name        = rospy.get_param('~sensor_name', 'node_01')
        self.in_skeleton_group  = rospy.get_param('~in_skeleton', 'opw_3d/' + self.sensor_name + '/skeleton_group')
        #self.frame_id          = rospy.get_param('~frame_id'   , self.sensor_name + '_color_optical_frame')
        
        self.scale    = rospy.get_param('~scale', 0.07)
        self.lifetime = rospy.get_param('~lifetime', 0) # 0 is forever
        self.ns       = rospy.get_param('~ns', self.sensor_name)
        self.id       = rospy.get_param('~id', 0)
        self.color    = rospy.get_param('~color', {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0})
        
        # NB: you can remap private paramters directly from command-line:
        # $ rosrun openpose_wrapper opw_skeleton_viewer.py _sensor_name:=realsense01 _color/r:=1.0
        print('Sensor name:', self.sensor_name )
        print('Input topic:', self.in_skeleton_group )
        
        # Subscribe to the skeleton topic.
        rospy.Subscriber(self.in_skeleton_group, SkeletonGroup, self.skeleton_handler)
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('skeleton_markers', Marker, queue_size=10)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = self.ns
        self.markers.id = self.id
        self.markers.type = Marker.POINTS
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(self.lifetime)
        self.markers.scale.x = self.scale
        self.markers.scale.y = self.scale
        self.markers.color.r = self.color['r']
        self.markers.color.g = self.color['g']
        self.markers.color.b = self.color['b']
        self.markers.color.a = self.color['a']

        rospy.loginfo("Done. Start publishing skeleton markers...")
    
    
    def spin(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        
    def skeleton_handler(self, msg):
        self.markers.header.frame_id = msg.header.frame_id
        self.markers.header.stamp    = msg.header.stamp

        self.markers.points = list()
        for skeleton in msg.skeletons: 
            for joint in skeleton.markers:         
                p = Point()
                p = joint.center.pose.position
                self.markers.points.append(p)

        self.marker_pub.publish(self.markers)            


    def shutdown(self):
        rospy.loginfo('Shutting down Skeleton Viewer Node.')
        
if __name__ == '__main__':
    viewer_node = SkeletonViewer()
    viewer_node.spin()