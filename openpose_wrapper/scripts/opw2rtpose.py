#!/usr/bin/env python

"""
    Convert a OPW skeleton msg to a RTPOSE skeleton msgs

"""

import rospy
from sensor_msgs.msg import CameraInfo
from openpose_wrapper.msg import SkeletonGroup
from rtpose_wrapper.msg import SkeletonArrayMsg, SkeletonMsg, Joint3DMsg

# TODO: read the mapping from a config file
map_openpose25_to_rtposeMPI = {
    0:  0,
    1:  1,
    2:  2,
    3:  3,
    4:  4,
    5:  5,
    6:  6,
    7:  7,
    8:  14,
    9:  8,
    10: 9,
    11: 10,
    12: 11,
    13: 12,
    14: 13,
    15: None,
    16: None,
    17: None,
    18: None,
    19: None,
    20: None,
    21: None,
    22: None,
    23: None,
    24: None,
}


class SkeletonConverter():
    def __init__(self):
        rospy.init_node('skeleton_converter')
        rospy.on_shutdown(self.shutdown)

        # ROS Params
        self.sensor_name         = rospy.get_param('~sensor_name',  'node_01')
        self.in_camera_info      = rospy.get_param('~in_camera_info_topic', self.sensor_name + '/color/camera_info')
        self.in_skeleton_topic   = rospy.get_param('~in_skeleton_topic',  '/opw_3d/' + self.sensor_name + '/skeleton_group')
        self.out_skeleton_topic  = rospy.get_param('~out_skeleton_topic', '/detector/skeletons')

        # Retrieve camera parameters from the camera_info topic
        camera_info_msg = rospy.wait_for_message(self.in_camera_info, CameraInfo)
        self.color_width   = camera_info_msg.width
        self.color_height  = camera_info_msg.height
        self.camera_matrix = camera_info_msg.K

        print('Sensor name:'  , self.sensor_name )
        print('Input  topic:' , self.in_skeleton_topic )
        print('Output topic:' , self.out_skeleton_topic )
        print('Camera: ({:d}x{:d})'.format(self.color_width, self.color_height) )
        print('Camera matrix: ', self.camera_matrix )

        # Subscribe to the skeleton topic.
        rospy.Subscriber(self.in_skeleton_topic, SkeletonGroup, self.skeleton_handler)

        self.rtpose_skeleton_pub = rospy.Publisher(self.out_skeleton_topic, SkeletonArrayMsg, queue_size=1)

        rospy.loginfo("Initialization done. Start converting skeletons to rtpose skeletons...")


    def skeleton_handler(self, msg):
        sk_rtpose_array = SkeletonArrayMsg()
        sk_rtpose_array.header = msg.header
        sk_rtpose_array.intrinsic_matrix = self.camera_matrix
        sk_rtpose_array.rgb_header = msg.header

        for sk_openpose in msg.skeletons:
            sk_rtpose = SkeletonMsg()
            sk_rtpose.header = msg.header
            sk_rtpose.joints = self.map_to_rtpose_joints(sk_openpose.markers, msg.header)        

            # Hard-coded values as in rtpose_wrapper
            sk_rtpose.info   = "rtpose with MPI model"
            sk_rtpose.skeleton_type  = 1
            sk_rtpose.height     = 0.0
            sk_rtpose.distance   = 0.0
            sk_rtpose.confidence = 100.0
            sk_rtpose.occluded   = False

            sk_rtpose_array.skeletons.append(sk_rtpose)

        # Publish
        print(sk_rtpose_array)
        self.rtpose_skeleton_pub.publish(sk_rtpose_array)


    def map_to_rtpose_joints(self, in_joints, in_header):
        # Initialize the list of joints
        out_joint_array = []
        for j in range(15):
            out_joint = Joint3DMsg() 
            out_joint.header     = in_header
            out_joint.max_width  = self.color_width 
            out_joint.max_height = self.color_height  
            out_joint.x = float('nan')
            out_joint.y = float('nan')
            out_joint.z = float('nan')
            out_joint.confidence = 0.0 
            out_joint_array.append(out_joint)

        # Map the coordinates from OpenPose to rtpose
        for joint in in_joints:
            out_joint_id = map_openpose25_to_rtposeMPI[joint.id]
            
            if out_joint_id is not None:
                out_joint_array[ out_joint_id ].x = joint.center.pose.position.x
                out_joint_array[ out_joint_id ].y = joint.center.pose.position.y
                out_joint_array[ out_joint_id ].z = joint.center.pose.position.z
                out_joint_array[ out_joint_id ].confidence = joint.confidence

        return out_joint_array

    
    def spin(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

        
    def shutdown(self):
        rospy.loginfo('Shutting down Skeleton Viewer Node.')

        
if __name__ == '__main__':
    converter_node = SkeletonConverter()
    converter_node.spin()