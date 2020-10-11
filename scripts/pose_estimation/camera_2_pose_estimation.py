#!/usr/bin/env python

import rospy
import sys
sys.path.insert(1, '/home/zhanibek/catkin_ws/src/smart_tray/scripts')
import cv2

import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge as bridge

import conf

from PoseEstimation import BoardLocalization





def main():

    rospy.init_node("camera_2_pose_estimation")
    rospy.loginfo('pose_estimation_2 is initialized!')


    tloc2 = BoardLocalization(
        board=conf.board,
        calib_data=conf.logitech_t1_calibration,
        bname='tray',
        mrklen=conf.mrklen,
        aruco_dict=conf.aruco_dict,
        name='cam2_tray_pose',
        camname='camera_2'
        )

    rospy.Subscriber('/camera_2', Image, callback=tloc2.callback)

    rospy.spin()


if __name__ == '__main__':
    main()
