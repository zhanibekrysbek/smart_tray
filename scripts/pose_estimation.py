#!/usr/bin/env python

import rospy
import sys
sys.path.insert(1,'/home/zhanibek/.local/lib/python2.7/site-packages/cv2')
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge as bridge
import numpy as np
# from scipy.linalg import norm
import cv2
from cv2 import aruco
import yaml
import utils

rvec_prev = None
tvec_prev = None

pub = None


class TrayLocalization(object):

    def __init__(self, board, posePub, calib_data, mrklen, 
                aruco_dict, name='cam1_pose', camtopic = '/camera_1' ):
        
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
        self.posePub = posePub  # Pose Publisher object 
        self.processedImagePub = rospy.Publisher('/aruco_'+self.name, Image, queue_size=10) # Annotated image publisher
        self.logPub = NotImplemented
        
        #
        self.recursiveTracking = True
        self.rvec_prev = None
        self.tvec_prev = None        


    def callback(self, data):

        pose = PoseStamped()
        pose.header = data.header

        frame = bridge().imgmsg_to_cv2(data,"bgr8")

        retval, frame, rvec, tvec, loggs = self.estimatePose(frame)

        if retval:
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = tvec.flatten()
            pose.operation = NotImplemented
        else:
            rospy.loginfo('No Markers Detected')

        frame_msg = bridge().cv2_to_imgmsg(frame,'bgr8')
        self.posePub.publish(pose)
        self.processedImagePub.publish(frame)


    # Aruco Tracking and Calculating the distance to the camera
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
        
        # if no any marker is detected:
        if ids is None:
            # code to show when no markers are found
            cv2.putText(frame, "No Markers", (3,64), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1, cv2.LINE_AA)
            self.rvec_prev = None
            self.tvec_prev = None

        # if some markers are detected: Run Pose Estimation
        else:
            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients
            retval, rvec, tvec = aruco.estimatePoseBoard( 
                                            corners, ids, self.board, self.camera_matrix, 
                                            self.dist_coeffs, None, None, False )

            if type(self.rvec_prev)==np.ndarray and self.recursiveTracking:
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


        return frame, rvec, tvec


def main():
    global pub
    rospy.init_node("camera_1_pose_estimation")
    
    pub = rospy.Publisher('/cam_1_pose_estimation', PoseStamped, queue_size=10)
    rospy.Subscriber("/camera_1", Image, callback)
    

    rospy.spin()


    # When everything done, release the capture
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
