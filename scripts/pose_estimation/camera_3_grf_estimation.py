#!/usr/bin/env python

import rospy
import sys
sys.path.insert(1, '/home/zhanibek/catkin_ws/src/smart_tray/scripts')

from PoseEstimation import BoardLocalization
import conf
from sensor_msgs.msg import Image


def main():

    rospy.init_node("camera_3_grf_estimation")
    rospy.loginfo('grf_estimation is initialized!')


    grf3 = BoardLocalization(
        board=conf.board_grf,
        bname='grf',            
        calib_data=conf.logitech_t2_calibration,
        mrklen=conf.mrklen_grf,
        aruco_dict=conf.aruco_dict,
        name='cam3_grf',
        camname='camera_3'
        )

    rospy.Subscriber('/camera_3', Image, callback=grf3.callback)


    rospy.spin()


if __name__=='__main__':

    main()
