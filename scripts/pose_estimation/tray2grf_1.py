#!/usr/bin/env python

import rospy
import sys
sys.path.insert(1, '/home/zhanibek/catkin_ws/src/smart_tray/scripts')


import conf
from phri import utils

from PoseEstimation import TrayInGRF



def main():

    rospy.init_node("tray2grf_1")
    rospy.loginfo('tray2grf_1!')

    
    t1grf = TrayInGRF(
        cam2grf=conf.cam1togrf, 
        topic='/cam1_tray_grf', 
        tray_est_top='/cam1_tray_pose_estimation' 
        )


    rospy.spin()

if __name__=='__main__':
    main()
