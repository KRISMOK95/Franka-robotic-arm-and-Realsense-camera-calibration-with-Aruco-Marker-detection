"""
Adapted from autolab/perception package's register camera script:
Script to register sensors to a chessboard for the YuMi setup
Authors: Jeff Mahler and Brenton Chu
""" 
import rospy
from geometry_msgs.msg import PointStamped
import tf
import random
import numpy as np
from std_msgs.msg import String
import os
import subprocess
import numpy as np
import rospy
from sensor_msgs.msg import Image
from perception import CameraIntrinsics
import cv2
from cv_bridge import CvBridge, CvBridgeError

from frankapy import FrankaArm

from autolab_core import RigidTransform, Point

from geometry_msgs.msg import PointStamped
import tf
import random
from std_msgs.msg import String

# ROS subscriber
global x,y
x = 0.0
y = 0.0
'''
def get_coordinate(topic='/aruco_single/pixel'):

    coordinates = rospy.wait_for_message(topic, PointStamped).pose
    try:
        x = coordinates[0]
        y = coordinates[1]
        print(coordinates)
        print(x ,y)

        
        rospy.loginfo("coordinates:x=%f y=%f" %(x, y))
        return x , y 
        

    except rospy.ROSException as e:
        print(e)


if __name__ == '__main__':
    get_coordinate()

'''
'''
def callback(msg):
    point = PointStamped()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "stereo_gazebo_left_camera_optical_frame"
    point.point.x = msg.point.x         # access to point data structure and its x, y, z components
    point.point.y = msg.point.y
    xx = point.point.x
    yy = point.point.y
    print(xx)
    print(yy)
    rospy.loginfo("coordinates:x=%f y=%f" %(point.point.x, point.point.y))

    return point.point.x , point.point.y



def listener_marker_pose():


    rospy.init_node('listener_marker_pose', anonymous=True)

    rospy.point_pub = rospy.Subscriber("/aruco_single/pixel", PointStamped , callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener_marker_pose()
'''

def get_object_center_point_in_world(
    object_image_center_x, 
    object_image_center_y, 
    depth_image, 
    intrinsics, 
    transform
):    
    
    object_center = Point(
        np.array([object_image_center_x, object_image_center_y]), 
        'azure_kinect_overhead'
    )
    object_depth = depth_image[int(object_image_center_y), int(object_image_center_x)] * 0.001
    print(
        "x, y, z: ({:.4f}, {:.4f}, {:.4f})".format(
        object_image_center_x, object_image_center_y, object_depth
        )
    )
    
    object_center_point_in_world = transform * intrinsics.deproject_pixel(
        object_depth, object_center
    )    
    print(object_center_point_in_world)

    return object_center_point_in_world 


