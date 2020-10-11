#!/usr/bin/env python

import rospy
import sys
sys.path.insert(1, '/home/zhanibek/catkin_ws/src/smart_tray/scripts')

import conf
from phri import utils

from PoseEstimation import TrayInGRF


def main():

    rospy.init_node("process_tray_location")
    rospy.loginfo('process_tray_location!')


    t1grf = TrayInGRF(
        cam2grf=conf.cam3togrf,
        topic='/cam3_tray_grf',
        tray_est_top='/cam3_tray_pose_estimation'
    )


    rospy.spin()

if __name__=='__main__':
    main()
