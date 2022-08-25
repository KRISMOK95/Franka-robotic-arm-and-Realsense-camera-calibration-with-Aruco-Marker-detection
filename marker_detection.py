from frankapy import FrankaArm
import numpy as np
import argparse
import cv2
from cv_bridge import CvBridge
from autolab_core import RigidTransform, Point

from perception import CameraIntrinsics
from utils import *

import rospy
from geometry_msgs.msg import PointStamped

global x,y
x = 0.0
y = 0.0

REALSENSE_INTRINSICS = "/home/kalong/Documents/camera-calibration/calib/realsense_intrinsics.intr"
REALSENSE_EE_TF = "/home/kalong/Documents/camera-calibration/calib/realsense_ee.tf"

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--intrinsics_file_path", type=str, default=REALSENSE_INTRINSICS
    )
    parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_EE_TF)
    args = parser.parse_args()

    print("Starting robot")
    fa = FrankaArm()

    print("Opening Grippers")
    # Open Gripper
    fa.open_gripper()

    # Reset Pose
    fa.reset_pose()
    # Reset Joints
    fa.reset_joints()

    current_pose = fa.get_pose()
    current_pose.translation=[0.49751928, -0.27072591,  0.38351772]
    fa.goto_pose(current_pose)



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

        return xx , yy



    def listener_marker_pose():


        rospy.init_node('listener_marker_pose', anonymous=True)

        rospy.point_pub = rospy.Subscriber("/aruco_single/pixel", PointStamped , callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()



    print("robot is ready")
    cv_bridge = CvBridge()
    realsense_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)
    realsense_to_ee_transform = RigidTransform.load(args.extrinsics_file_path)

    rgb_image = get_realsense_rgb_image(cv_bridge)
    depth_image = get_realsense_depth_image(cv_bridge)

    object_image_position = np.array([200, 300])

    
    listener_marker_pose()

    object_z_height = 0.020
    intermediate_pose_z_height = 0.19

    current_pose = fa.get_pose()

    # the x y should not move, what if it move.

    object_center_point_in_world = get_object_center_point_in_world_realsense(
        xx,
        yy,
        depth_image,
        realsense_intrinsics,
        realsense_to_ee_transform,
        current_pose
    )

    object_center_pose = current_pose

    object_center_pose.translation = [
        object_center_point_in_world[0],
        object_center_point_in_world[1],
        object_z_height,
    ]

    intermediate_robot_pose = object_center_pose.copy()
    intermediate_robot_pose.translation = [
        object_center_point_in_world[0],
        object_center_point_in_world[1],
        intermediate_pose_z_height,
    ]

    # Move to intermediate robot pose
    fa.goto_pose(intermediate_robot_pose)

    fa.goto_pose(object_center_pose, 5, force_thresholds=[10, 10, 10, 10, 10, 10])

    # Close Gripper
    fa.goto_gripper(0.045, grasp=True, force=10.0)

    # Move to intermediate robot pose
    fa.goto_pose(intermediate_robot_pose)

    fa.goto_pose(object_center_pose, 5, force_thresholds=[10, 10, 20, 10, 10, 10])

    print("Opening Grippers")
    # Open Gripper
    fa.open_gripper()

    fa.goto_pose(intermediate_robot_pose)

    # Reset Pose
    fa.reset_pose()
    # Reset Joints
    fa.reset_joints()














