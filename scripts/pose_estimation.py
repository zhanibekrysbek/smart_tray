#!/usr/bin/env python

import rospy
import sys
sys.path.insert(1,'/home/zhanibek/.local/lib/python2.7/site-packages/cv2')
import cv2
from cv2 import aruco
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge as bridge

import conf



class TrayLocalization(object):

    def __init__(self, board, calib_data, mrklen, 
                aruco_dict, name='cam1_pose'):
        
        # Initialize Set of Params:
        
        # Name of this module
        self.name = name
        self.board = board
        self.aruco_dict = aruco_dict
        self.mrklen = mrklen
        
        # Camera Calibration Data
        self.camera_matrix = calib_data['camera_matrix']
        self.dist_coeffs = calib_data['dist_coeffs']
        
        # Publishers
        self.posePub = rospy.Publisher('/'+self.name+'_estimation', PoseStamped, queue_size=10)  # Pose Publisher object 
        self.processedImagePub = rospy.Publisher('/aruco_'+self.name, Image, queue_size=10) # Annotated image publisher
        self.logPub = NotImplemented
        
        #
        self.recursiveTracking = True
        self.rvec_prev = None
        self.tvec_prev = None        


    def callback(self, data):

        trayPose = PoseStamped()
        trayPose.header = data.header

        frame = bridge().imgmsg_to_cv2(data,"bgr8")

        retval, frame, rvec, tvec = self.estimatePose(frame)

        if retval:
            trayPose.pose.position.x, trayPose.pose.position.y, trayPose.pose.position.z = tvec.flatten()
            trayPose.pose.orientation = NotImplemented
        # else:
            # rospy.loginfo('No Markers Detected')

        frame_msg = bridge().cv2_to_imgmsg(frame,'bgr8')
        self.posePub.publish(trayPose)
        self.processedImagePub.publish(frame_msg)


    # Aruco 3D-Board Tracking and Pose Estimation
    def estimatePose(self, frame):

        rvec = None
        tvec = None
        res = False

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, self.aruco_dict, parameters=parameters)
        corners, ids, rjcorners, recids = aruco.refineDetectedMarkers(frame, self.board, corners, ids, 
                                                            rejectedImgPoints, self.camera_matrix, self.dist_coeffs)
        
        rospy.loginfo(ids)
        # if no any marker is detected:
        if ids is None:
            # code to show when no markers are found
            cv2.putText(frame, "No Markers", (3,64), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1, cv2.LINE_AA)
            self.rvec_prev = None
            self.tvec_prev = None

        # if some markers are detected: Run Pose Estimation
        else:
            retval = False
            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients
            if not self.recursiveTracking:
                retval, rvec, tvec = aruco.estimatePoseBoard( 
                                                corners, ids, self.board, self.camera_matrix, 
                                                self.dist_coeffs, None, None, False )

            elif type(self.rvec_prev)==np.ndarray and self.recursiveTracking:
                retval, rvec, tvec = aruco.estimatePoseBoard(
                                            corners, ids, self.board, 
                                            self.camera_matrix, self.dist_coeffs,
                                            self.rvec_prev, self.tvec_prev, True)
                self.rvec_prev = rvec
                self.tvec_prev = tvec

            if retval:
                # Draw the Tray Coordinate system
                aruco.drawAxis( frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 2*self.mrklen )
                # draw a square around the markers
                aruco.drawDetectedMarkers(frame, corners, ids)
                res = True

        return res, frame, rvec, tvec


def main():

    rospy.init_node("camera_1_pose_estimation")
    rospy.loginfo('camera_1_pose_estimation is initialized!')

    
    tloc1 = TrayLocalization(
        board=conf.board, 
        calib_data=conf.logitech_t1_calibration, 
        mrklen=conf.mrklen,
        aruco_dict=conf.aruco_dict, 
        name='cam1_pose'
        )

    rospy.Subscriber('/camera_1', Image,callback=tloc1.callback)

    rospy.spin()

if __name__=='__main__':
    main()
