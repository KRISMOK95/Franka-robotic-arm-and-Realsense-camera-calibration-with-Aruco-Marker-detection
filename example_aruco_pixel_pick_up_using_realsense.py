from frankapy import FrankaArm
import numpy as np
import argparse
import cv2
from cv_bridge import CvBridge
from autolab_core import RigidTransform, Point
from geometry_msgs.msg import PointStamped

from perception import CameraIntrinsics
from utils import *

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
    current_pose.translation=[0.57, -0.27072591,  0.28351772] # [0.49751928, -0.27072591,  0.38351772]
    fa.goto_pose(current_pose)


    cv_bridge = CvBridge()
    realsense_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)
    realsense_to_ee_transform = RigidTransform.load(args.extrinsics_file_path)

    rgb_image = get_realsense_rgb_image(cv_bridge)
    depth_image = get_realsense_depth_image(cv_bridge)
    aruco_point = rospy.wait_for_message('/aruco_single/pixel', PointStamped, timeout=10)
    print("the aruco point is:")
    print(aruco_point)
    print("The depth of image is:")
    print(depth_image)

    object_image_position = np.array([aruco_point.point.y, aruco_point.point.x])
    print("The object image position is:")
    print(object_image_position)

    pregrasp_pose_height = 0.13

    current_pose = fa.get_pose()
    print("got pose")

    object_center_point_in_world = get_object_center_point_in_world_realsense(
        object_image_position[1],
        object_image_position[0],
        depth_image,
        realsense_intrinsics,
        realsense_to_ee_transform,
        current_pose
    )
    print("The get_object_center_point_in_world_realsense is:")
    print(get_object_center_point_in_world_realsense)
    print("The object_center_point_in_world:")
    print(object_center_point_in_world)
    print("The object image position is:")
    print(object_image_position)
    

    object_center_pose = current_pose
    print("got pose 2")

    object_center_pose.translation = [
        object_center_point_in_world[0] -0.005,#-0.009
        object_center_point_in_world[1],
        object_center_point_in_world[2]  + 0.03
    ]

    print(object_center_point_in_world[2]+ 0.06)  # 0.1889798239607534
    print(object_center_point_in_world[2]) # 0.05897982

    intermediate_robot_pose = object_center_pose.copy()
    intermediate_robot_pose.translation = [
        object_center_point_in_world[0] -0.005, #-0.009
        object_center_point_in_world[1],
        object_center_point_in_world[2]+pregrasp_pose_height
    ]


    # Move to intermediate robot pose
    fa.goto_pose(intermediate_robot_pose)

    fa.goto_pose(object_center_pose, 5, force_thresholds=[10, 10, 10, 10, 10, 10])

    # Close Gripper
    fa.goto_gripper(0.045, grasp=True, force=10.0)

    print("Is it graspped?")
    print(fa.get_gripper_is_grasped())

    # Move to intermediate robot pose
    fa.goto_pose(intermediate_robot_pose)

    ####### my codes


    fa.goto_joints([-1.52498538,  0.03338459 , 0.07795439 ,-2.22849252 , 0.0411701  , 2.26262835,-0.67404007])



    fa.goto_joints([-1.50054569 , 0.50220558 , 0.03934883, -2.12346325 , 0.03274582 , 2.52555007,-0.69513546])

    fa.open_gripper()

    fa.goto_joints([-1.52498538,  0.03338459 , 0.07795439 ,-2.22849252 , 0.0411701  , 2.26262835,-0.67404007])
'''
    # Reset Pose
    fa.reset_pose()
    # Reset Joints
    fa.reset_joints()
'''