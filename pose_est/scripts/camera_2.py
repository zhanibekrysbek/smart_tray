#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge as bridge
import numpy as np
from scipy.linalg import norm
import cv2
import cv2.aruco as aruco
import glob

def callback(data):
    frame = bridge().imgmsg_to_cv2(data,"bgr8")

    # marker size of a single aruco marker(in meters)
    global marker_size
    marker_size=0.032

    # termination criteria for the iterative algorithm
    global criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    mtx, dist = camera_calib()
    frame,ids,cam_marker_dist = aruco_track(frame,mtx,dist)

    cv2.imshow('frame',frame)
    cv2.waitKey(1)
    if ids is None:
        print "There are no markers."
    else:
        disp = ""
        for i in range(0, ids.size):
            disp+="Distance btw Marker id {} and camera:{} (cm)\n".format(ids[i],cam_marker_dist[i])
        print disp

# Camera Calibration
def camera_calib():
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    # checkerboard of size (9 x 6) is used
    objp = np.zeros((5*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:5].T.reshape(-1,2)

    # arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    # iterating through all calibration images
    # in the folder
    images = glob.glob("calib_images/*.jpg")
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # find the chess board (calibration pattern) corners
        ret, corners = cv2.findChessboardCorners(gray, (9,6), None)

        # if calibration pattern is found, add object points,
        # image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            # Refine the corners of the detected corners
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (9,6), corners2, ret)

        else:
            break

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    return [mtx, dist]

# Aruco Tracking and Calculating the distance to the camera
def aruco_track(frame,mtx,dist):
    # operations on the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # set dictionary size depending on the aruco marker selected
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    # detector parameters can be set here (List of detection parameters[3])
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10

    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    

    # check if the ids list is not empty
    # if no check is added the code will crash
    if ids is None:
        # code to show when no markers are found
        cv2.putText(frame, "No Markers", (0,64), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1, cv2.LINE_AA)
        cam_marker_dist = [None]
    else:
        # estimate pose of each marker and return the values
        # rvet and tvec-different from camera coefficients
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, marker_size, mtx, dist)
        cam_marker_dist = [None]*len(ids)

        for i in range(0, ids.size):
            # draw axis for the aruco markers
            aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.04)
            cam_marker_dist[i] = 100*norm(tvec[i])

        # draw a square around the markers
        aruco.drawDetectedMarkers(frame, corners, ids)

    return [frame,ids,cam_marker_dist]


def main():
    rospy.init_node("retrieve_image")
    rospy.Subscriber("/color_image", Image, callback)
    rospy.spin()
    # When everything done, release the capture
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()



