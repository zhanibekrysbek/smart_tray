#!/usr/bin/env python

import rospy
import sys
sys.path.insert(1,'/home/zhanibek/.local/lib/python2.7/site-packages/cv2')
import cv2
from cv2 import aruco
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge as bridge

import conf
from phri import utils



class TrayInGRF(object):

    def __init__(self, cam2grf, topic='/cam1_tray_processed', tray_est_top='/cam1_tray_pose_estimation' ):
        
        # Initialize Set of Params:
        
        # Name of this module
        self.name = name
        self.topic = topic
        self.tray_est_top = tray_est_top
        self.cam2grf = cam2grf
        
        # Publishers
        self.pub = rospy.Publisher(self.topic, PoseStamped, queue_size=10)  # Pose Publisher object 
        


        rospy.Subscriber(self.tray_est_top, PoseStamped, self.callback)


    def callback(self, data):

        '''
        TODO:
        publish the PoseStamped message at self.pub:

        pose = g_
        '''

        
        pose = PoseStamped()

        pose.header = data.header
        pose.header.frame_id = 'GRF repr'

        cam2tray = {
            'position': np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]),
            'orientation': np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        }

        self.perform_transformation(self.cam2grf, cam2tray)




    def perform_transformation(self, cam2grf_pose, cam2tray_pose):

        Gc_grf = utils.g_from_pose(cam2grf_pose)
        Gc_tray = utils.g_from_qv(cam2tray_pose)
        Ggrf_tray = np.matmul(utils.inverse(Gc_grf),Gc_tray)
        



        pose.position.x = 
        pose.orientation.w




def main():

    rospy.init_node("pose_estimation")
    rospy.loginfo('pose_estimation is initialized!')


    cam1togrf = {
        'position': np.array([-0.288765, 0.21742400, 2.4659026]),
        'orientation': np.array([0.84192191, 0.02008486, 0.0575091, 0.5361498])
    }
    
    tloc1 = TrayInGRF(
        cam2grf=cam1togrf, 
        topic='/cam1_tray_processed', 
        tray_est_top='/cam1_tray_pose_estimation' 
        )

    rospy.Subscriber('/camera_1', Image,callback=tloc1.callback)



    rospy.spin()

if __name__=='__main__':
    main()
