#!/usr/bin/env python

import rospy
from PoseEstimation import BoardLocalization
import sys
sys.path.insert(1, '/home/zhanibek/catkin_ws/src/smart_tray/scripts')

import conf
from sensor_msgs.msg import Image


def main():

    rospy.init_node("camera_2_grf_estimation")
    rospy.loginfo('grf_estimation is initialized!')


    grf2 = BoardLocalization(
        board=conf.board_grf,
        bname='grf',
        calib_data=conf.logitech_t1_calibration,
        mrklen=conf.mrklen_grf,
        aruco_dict=conf.aruco_dict,
        name='cam2_grf',
        camname='camera_2'
        )

    rospy.Subscriber('/camera_2', Image, callback=grf2.callback)



    rospy.spin()


if __name__=='__main__':

    main()
