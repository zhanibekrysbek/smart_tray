#!/usr/bin/env python

import rospy
import sys
sys.path.insert(1, '/home/zhanibek/catkin_ws/src/smart_tray/scripts')
# sys.path.insert(1, './phri')
# sys.path.insert(1, '..')
# sys.path.append('..')

import conf

import cv2

import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge as bridge


from PoseEstimation import BoardLocalization



def main():

    rospy.init_node("camera_1_pose_estimation")
    rospy.loginfo('pose_estimation_1 is initialized!')

    tloc1 = BoardLocalization(
        board=conf.board,
        calib_data=conf.angetube_calibration,
        mrklen=conf.mrklen,
        aruco_dict=conf.aruco_dict,
        name='cam1_tray_pose',
        camname='camera_1'
    )

    rospy.Subscriber('/camera_1', Image, callback=tloc1.callback)


    rospy.spin()


if __name__ == '__main__':
    main()
