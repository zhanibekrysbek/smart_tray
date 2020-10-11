#!/usr/bin/env python

import cv2
import rospy
import sys
sys.path.insert(1, '/home/zhanibek/catkin_ws/src/smart_tray/scripts')

import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge as bridge

import conf

from PoseEstimation import BoardLocalization







def main():

    rospy.init_node("pose_camera_3_estimation")
    rospy.loginfo('pose_estimation_3 is initialized!')



    tloc3 = BoardLocalization(
        board=conf.board,
        calib_data=conf.logitech_t2_calibration,
        mrklen=conf.mrklen,
        aruco_dict=conf.aruco_dict,
        name='cam3_tray_pose',
        camname='camera_3'
        )

    rospy.Subscriber('/camera_3', Image, callback=tloc3.callback)

    rospy.spin()


if __name__ == '__main__':
    main()
