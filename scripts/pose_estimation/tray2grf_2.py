#!/usr/bin/env python

import rospy
import sys
sys.path.insert(1, '/home/zhanibek/catkin_ws/src/smart_tray/scripts')

import conf
from phri import utils

from PoseEstimation import TrayInGRF
from geometry_msgs.msg import PoseStamped



def main():

    rospy.init_node("tray2grf_2")
    rospy.loginfo('tray2grf_2!')


    t2grf = TrayInGRF(
        cam2grf=conf.cam2togrf,
        topic='/cam2_tray_grf',
        tray_est_top='/cam2_tray_pose_estimation'
    )

    rospy.Subscriber('/cam2_tray_pose_estimation', PoseStamped, t2grf.callback)

    rospy.spin()

if __name__=='__main__':
    main()
