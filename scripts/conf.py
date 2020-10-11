
import numpy as np
# import sys
# sys.path.insert(1,'/home/zhanibek/.local/lib/python2.7/site-packages/cv2')
from cv2 import aruco


# set dictionary size depending on the aruco marker selected
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
mrklen = 0.115

'''
Every marker in the tray is viewed as linearly translated squares from the origin.
Therefore, we only need to define 3 template markers to describe all 8 markers across 3 different planes.
As a result, Coordinates of each marker is found by summing their respective translation from the origin.

In Aruco library, canonical representation of corner points of markers are defined as below:
    - The order of corners are clock-wise
    - first corner point is the one where you get red small circle when aruco.drawDetectedMarkers is called.
'''
# Template Locations for three planes
side_marker0_right = np.array(
             [[0., 0., -mrklen],
              [0., 0., 0.], 
              [mrklen, 0., 0.],
              [mrklen, 0., -mrklen]], dtype=np.float32)

side_marker0_left = np.array(
             [[0., 0., -mrklen],
              [0., 0., 0.], #1
              [-mrklen, 0., 0.],
              [-mrklen, 0., -mrklen]], dtype=np.float32)

top_marker0 = np.array(
             [[0., 0., 0.],
              [mrklen, 0., 0.],
              [mrklen, -mrklen, 0.],
              [0., -mrklen, 0.]], dtype=np.float32)

# Linear translations of markers from template positions specified in above variables 
# [x_off,y_off,z_off]
offsets_right = np.array(
                   [[-0.1930, -0.15796, -0.025],  # 20
                    [-0.05514,-0.15796, -0.025],  # 21
                    [0.0871,  -0.15796, -0.025],  # 22
                   ], dtype=np.float32) 

# [x_off,y_off,z_off]
offsets_left = np.array(
                   [[0.1931, 0.15796, -0.02],   # 24
                    [0.053,  0.15796, -0.02],   # 25
                    [-0.079,  0.15796, -0.02],  # 23
                   ], dtype=np.float32) 


# [x_off,y_off,z_off]
offsets_top = np.array(
                   [[0.07623, 0.1252, 0.],  # 26
                    [0.07623, -0.0129, 0.]  # 27
                   ], dtype=np.float32) 

# Define all corner points for 8 markers
board_corners = [side_marker0_right + offset for offset in offsets_right]
board_corners += [side_marker0_left + offset for offset in offsets_left]
board_corners += [top_marker0 + offset for offset in offsets_top]
# Order of ids must coincide with board corners
board_ids = np.array( [[20],[21], [22], [24], [25], [23], [26], [27]], dtype=np.int32)
board = aruco.Board_create(board_corners, aruco_dict, board_ids)


logitech_t1_calibration = {
    'camera_matrix':
        np.array([[524.10342668,   0.        , 323.49844061],
                  [  0.        , 522.73949737, 250.24476893],
                  [  0.        ,   0.        ,   1.        ]]),
    'dist_coeffs': 
        np.array([[ 0.06786745, -0.25414535,  0.0063491 ,  0.00304427, -0.00894483]])}



logitech_t2_calibration = {
    'camera_matrix': 
        np.array([[691.12066304,   0.        , 326.75591214],
               [  0.        , 690.78628974, 223.17308235],
               [  0.        ,   0.        ,   1.        ]]),
    'dist_coeffs': 
        np.array([[-9.49565343e-02,  2.82949612e+00, -9.81941816e-03, -1.96461492e-03, -1.68993667e+01]])}


angetube_calibration = {
    'camera_matrix': 
        np.array([[869.41374442,   0.        , 986.72010981],
               [  0.        , 868.79524322, 491.84396142],
               [  0.        ,   0.        ,   1.        ]]),
    'dist_coeffs': 
    np.array([[-2.72916381e-02,  5.84333722e-02, -5.61004719e-03, 1.21938899e-05, -1.00107964e-01]]),}
