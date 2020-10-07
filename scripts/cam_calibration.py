#!/usr/bin/env python

import numpy as np
import cv2
from matplotlib import pyplot as plt
import cv2.aruco as aruco
import glob
import yaml

class Calibrate_Camera:
    """Calibrate an individual camera"""
    def __init__(self):
        self.dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.row = 7  # horizontal
        self.col = 5  # vertical
        self.sqr = 1  # solid black squares
        self.mrk = 0.8 # markers, must be smaller than squares
        self.board = aruco.CharucoBoard_create(
            self.col,self.row,
            self.sqr,
            self.mrk,
            self.dictionary)

    def calculateReprojectionError(self, imgpoints, objpoints, rvecs, tvecs, mtx, dist):
        """
        imgpts: features found in image, (num_imgs, 2)
        objpts: calibration known features in 3d, (num_imgs, 3)
        """
        imgpoints = [c.reshape(-1,2) for c in imgpoints]
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            imgpoints2 = imgpoints2.reshape(-1,2)

            # if not all markers were found, then the norm below will fail
            if len(imgpoints[i]) != len(imgpoints2):
                continue
                
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error

        total_error = mean_error/len(objpoints)
        return total_error

    def calibrate(self):
        corners_all = []  # corners in all images
        ids_all = []  # ids found in all images
        image_size = None

        images = glob.glob('./calib_images/*.jpg')

        for imname in images:

            img = cv2.imread(imname)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
              
            corners, ids, rejectedImgPts = aruco.detectMarkers(gray, self.dictionary)
            
            # if ids were found, then
            if ids is not None and len(ids) > 0:
                ret, ch_corners, ch_ids = aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.board)

                corners_all.append(ch_corners)
                ids_all.append(ch_ids)

        image_size = gray.shape[::-1]

        # Make sure at least one image was found
        if len(images) < 1:
            #Calibration failed because there were no images, warn the user
            print("Calibration was unsuccessful. No images of charucoboards were found. Add images of charucoboards and use or alter the naming conventions used in this file.")
            # Exit for failure
            exit()

        rms, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            corners_all, ids_all, self.board, image_size, None, None)

        h, w = self.board.chessboardCorners.shape
        objpts = [self.board.chessboardCorners.reshape((h,1,3)) for c in corners_all]
        imgpts = corners_all
        
        rep_error = self.calculateReprojectionError(imgpts, objpts, rvecs, tvecs, cameraMatrix, distCoeffs)
        
        cam_cal_data = {
            'cameraMatrix': cameraMatrix,
            'distCoeffs': distCoeffs,
            'rotationVector': rvecs,
            'translationVector': tvecs,
            'reprojectionError': rep_error,
            'rms': rms
        }

        return cam_cal_data
        
        

def main():
    c = Calibrate_Camera()
    c_data = c.calibrate()
    print c_data
    f = open('calibration.yaml', 'w')
    yaml.dump(c_data, f)
    f.close()


if __name__=='__main__':
    main()