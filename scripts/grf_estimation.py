#!/usr/bin/env python

import rospy
from pose_estimation import BoardLocalization
import conf
from sensor_msgs.msg import Image

def main():

    rospy.init_node("grf_estimation")
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


    # grf2 = BoardLocalization(
    #     board=conf.board_grf,
    #     calib_data=conf.logitech_t1_calibration,
    #     mrklen=conf.mrklen_grf,
    #     aruco_dict=conf.aruco_dict,
    #     name='cam2_grf',
    #     camname='camera_2'
    #     )

    # rospy.Subscriber('/camera_2', Image, callback=grf2.callback)



    # grf3 = BoardLocalization(
    #     board=conf.board_grf,
    #     calib_data=conf.logitech_t2_calibration,
    #     mrklen=conf.mrklen_grf,
    #     aruco_dict=conf.aruco_dict,
    #     name='cam3_grf',
    #     camname='camera_3'
    #     )

    # rospy.Subscriber('/camera_3', Image, callback=grf3.callback)


    rospy.spin()


if __name__=='__main__':

    main()
