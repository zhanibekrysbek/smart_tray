#!/usr/bin/env python

import numpy as np
import pickle
import os
import cv2
import sys
sys.path.append('../pose_estimation')
sys.path.append('../')

from PoseEstimation import BoardLocalization, TrayInGRF
import conf
from tqdm import tqdm

meta_data_paths = [
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/koh_sanket/trial_0/koh_sanket_trial_0_2020-10-11-20-48-27_meta_data.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/koh_zhanibek/trial_0/koh_zhanibek_trial_0_2020-10-11-21-19-07_meta_data.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/sanket_vignesh/trial_0/trial_0_2020-10-12-14-47-43_meta_data.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/sanket_vignesh/trial_1/trial_1_2020-10-12-14-51-53_meta_data.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/zhanibek_sanket/trial_0/trial_0_2020-10-12-15-02-32_meta_data.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/zhanibek_vignesh/trial_0/trial_0_2020-10-12-14-57-11_meta_data.pkl'
]

class LocalizationOffline:

    def __init__(self):

        pass



def main():

    meta_data_path = meta_data_paths[0]
    meta_data = pickle.load(open(meta_data_path))

    cams = ['camera_1', 'camera_2', 'camera_3']

    for cam in tqdm(cams):
        cam_df = meta_data[cam]
        print 'Processing '+cam+' topic...'

        frame_ids = []
        seqs = []
        tstamps = []

        raw_positions = []
        raw_orientations = []
        grf_positions = []
        grf_orientations = []
        for index, row in tqdm(cam_df.iterrows()):

            seqs.append(row['seq'])
            tstamps.append(row['time_stamp'])
            frame_ids.append(row['frame_id']+'_offline')


            im = cv2.imread(row['image_path'])



    return


if '__name__'=='__main__':

    main()
