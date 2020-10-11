#!/usr/bin/env python

import rospy
from PoseEstimation import BoardLocalization
import sys
sys.path.insert(1, '/home/zhanibek/catkin_ws/src/smart_tray/scripts')

import conf
from sensor_msgs.msg import Image

def main():

    rospy.init_node("camera_1_grf_estimation")
    rospy.loginfo('grf_estimation is initialized!')


    grf1 = BoardLocalization(
        board=conf.board_grf,
        calib_data=conf.angetube_calibration,
        mrklen=conf.mrklen_grf,
        aruco_dict=conf.aruco_dict,
        bname='grf',
        name='cam1_grf_pose',
        camname='camera_1'
        )

    rospy.Subscriber('/camera_1', Image, callback=grf1.callback)


    rospy.spin()


if __name__=='__main__':

    main()
