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



class BoardLocalization(object):

    def __init__(self, board, calib_data, mrklen, 
                aruco_dict, bname = 'tray', name='cam1_tray', camname='camera_1'):
        
        # Initialize Set of Params:
        
        # Name of this module
        self.name = name
        self.camname = camname
        self.board = board
        self.aruco_dict = aruco_dict
        self.mrklen = mrklen
        
        # Camera Calibration Data
        self.camera_matrix = calib_data['camera_matrix']
        self.dist_coeffs = calib_data['dist_coeffs']
        
        # Publishers
        self.posePub = rospy.Publisher('/'+self.name+'_estimation', PoseStamped, queue_size=10)  # Pose Publisher object 
        self.processedImagePub = rospy.Publisher('/aruco_image_'+bname+'_'+self.camname, Image, queue_size=10) # Annotated image publisher
        
        # Tracking params
        self.recursiveTracking = True
        self.rvec_prev = None
        self.tvec_prev = None        


    def callback(self, data):

        trayPose = PoseStamped()
        trayPose.header = data.header

        frame = bridge().imgmsg_to_cv2(data,"bgr8")

        retval, frame, rvec, tvec = self.estimatePose(frame)
        
        # If pose data is received then publish
        if retval:
            trayPose.pose.position.x, trayPose.pose.position.y, trayPose.pose.position.z = tvec.flatten()
            
            quat = trayPose.pose.orientation 
            quat.x, quat.y, quat.z, quat.w =  utils.quat_from_R(cv2.Rodrigues(rvec)[0])
            trayPose.pose.orientation = quat

        frame_msg = bridge().cv2_to_imgmsg(frame,'bgr8')
        frame_msg.header = data.header
        self.posePub.publish(trayPose)
        self.processedImagePub.publish(frame_msg)


    # Aruco 3D-Board Tracking and Pose Estimation
    def estimatePose(self, frame):

        rvec = None
        tvec = None

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, self.aruco_dict, parameters=parameters)
        corners, ids, rjcorners, recids = aruco.refineDetectedMarkers(frame, self.board, corners, ids, 
                                                            rejectedImgPoints, self.camera_matrix, self.dist_coeffs)
        
        # estimate pose of each marker and return the values
        # rvet and tvec-different from camera coefficients

        if self.recursiveTracking and type(self.tvec_prev)==np.ndarray:
            retval, rvec, tvec = aruco.estimatePoseBoard(
                                        corners, ids, self.board, 
                                        self.camera_matrix, self.dist_coeffs,
                                        self.rvec_prev, self.tvec_prev, True)
            self.rvec_prev = rvec
            self.tvec_prev = tvec

        else:
            retval, rvec, tvec = aruco.estimatePoseBoard( 
                                            corners, ids, self.board, 
                                            self.camera_matrix, self.dist_coeffs, 
                                            None, None, False )
            self.rvec_prev = rvec
            self.tvec_prev = tvec

        if retval:
            # Draw the Tray Coordinate system
            aruco.drawAxis( frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 2*self.mrklen )
            # draw a square around the markers
            aruco.drawDetectedMarkers(frame, corners, ids)

        
        # If no any markers are detected:
        else:
            cv2.putText(frame, "No Markers", (3,64), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1, cv2.LINE_AA)
            self.rvec_prev = None
            self.tvec_prev = None

        return retval, frame, rvec, tvec


def main():

    rospy.init_node("pose_estimation")
    rospy.loginfo('pose_estimation is initialized!')

    
    tloc1 = BoardLocalization(
        board=conf.board, 
        calib_data=conf.angetube_calibration, 
        mrklen=conf.mrklen,
        aruco_dict=conf.aruco_dict, 
        name='cam1_tray_pose',
        camname='camera_1'
        )

    rospy.Subscriber('/camera_1', Image,callback=tloc1.callback)


    # tloc2 = BoardLocalization(
    #     board=conf.board, 
    #     calib_data=conf.logitech_t1_calibration, 
    #     mrklen=conf.mrklen,
    #     aruco_dict=conf.aruco_dict, 
    #     name='cam2_pose',
    #     camname='camera_2'
    #     )

    # rospy.Subscriber('/camera_2', Image, callback=tloc2.callback)



    # tloc3 = BoardLocalization(
    #     board=conf.board, 
    #     calib_data=conf.logitech_t2_calibration, 
    #     mrklen=conf.mrklen,
    #     aruco_dict=conf.aruco_dict, 
    #     name='cam3_pose',
    #     camname='camera_3'
    #     )

    # rospy.Subscriber('/camera_3', Image, callback=tloc3.callback)


    rospy.spin()

if __name__=='__main__':
    main()
