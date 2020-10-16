#!/usr/bin/env python

import numpy as np
import pickle
import argparse
import os
import cv2
import sys
import pandas as pd
from tqdm import tqdm
sys.path.insert(1, '/home/zhanibek/catkin_ws/src/smart_tray/scripts')
sys.path.insert(1, '/home/zhanibek/catkin_ws/src/smart_tray/scripts/pose_estimation')

from PoseEstimation import BoardLocalization, TrayInGRF
import conf
from phri import utils

meta_data_paths = [
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/koh_sanket/trial_0/koh_sanket_trial_0_2020-10-11-20-48-27_meta_data.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/koh_zhanibek/trial_0/koh_zhanibek_trial_0_2020-10-11-21-19-07_meta_data.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/sanket_vignesh/trial_0/trial_0_2020-10-12-14-47-43_meta_data.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/sanket_vignesh/trial_1/trial_1_2020-10-12-14-51-53_meta_data.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/zhanibek_sanket/trial_0/trial_0_2020-10-12-15-02-32_meta_data.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/zhanibek_vignesh/trial_0/trial_0_2020-10-12-14-57-11_meta_data.pkl'
]



def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter

    parser = argparse.ArgumentParser(
        formatter_class=arg_fmt, description=main.__doc__)
    parser.add_argument(
        '-i', '--index', dest='ind', required=True,
        help='Meta Data file index'
    )

    args = parser.parse_args(sys.argv[1:])
    ind = int(args.ind)


    print('Starting the program! ')
    meta_data_path = meta_data_paths[ind]

    print 'Loading Meta Data: '+os.path.basename(meta_data_path)

    meta_data = pickle.load(open(meta_data_path))

    cams = ['camera_1', 'camera_2', 'camera_3']

    cam2grf = [conf.cam1togrf, conf.cam2togrf, conf.cam3togrf]
    cam2grf = dict(zip(cams, cam2grf))

    calib = [conf.angetube_calibration, conf.logitech_t1_calibration, conf.logitech_t2_calibration]
    calib = dict(zip(cams, calib))

    # iterate over cameras
    for cam in tqdm(cams):
        cam_df = meta_data[cam]
        print '\n\tProcessing '+cam+' topic...'

        raw_df = pd.DataFrame()
        grf_df = pd.DataFrame()

        frame_ids = []
        seqs = []
        tstamps = []

        raw_positions = []
        raw_orientations = []
        grf_positions = []
        grf_orientations = []

        trayLoc = BoardLocalization(
            board=conf.board,
            calib_data=calib[cam],
            mrklen=conf.mrklen,
            aruco_dict=conf.aruco_dict,
            name='cam1_tray_pose',
            camname='camera_1'
        )
        tray2grf = TrayInGRF(
            cam2grf=cam2grf[cam],
            topic='/cam1_tray_grf',
            tray_est_top='/cam1_tray_pose_estimation'
        )

        # Itereate over every image from a single camera
        for _, row in tqdm(cam_df.iterrows(), total=cam_df.shape[0]):
            seqs.append(row['seq'])
            tstamps.append(row['time_stamp'])
            frame_ids.append(row['frame_id']+'_offline')

            impath = row['image_path']
            im = cv2.imread(impath)
            # Run Pose Estimation
            retval, frame, rvec, tvec = trayLoc.estimatePose(im)
            # If success then compute tray pose in global reference frame and save the data
            if retval:
                pos = tvec.squeeze()
                quat = utils.quat_from_R(cv2.Rodrigues(rvec)[0])
                
                raw_positions.append(pos)
                raw_orientations.append(quat)

                grf2tray = tray2grf.perform_transformation({
                    'position':pos,
                    'orientation': quat
                })
                grf_positions.append(grf2tray['position'])
                grf_orientations.append(grf2tray['orientation'])
            else:
                raw_positions.append(np.nan)
                raw_orientations.append(np.nan)
                grf_positions.append(np.nan)
                grf_orientations.append(np.nan)

            aruco_impath = os.path.basename(impath).split('.')[0]+'_aruco_offline.jpg'
            dirname = os.path.dirname(impath)
            dirname = dirname + '_aruco_offline'
            if not os.path.exists(dirname):
                os.mkdir(dirname)
            aruco_impath = os.path.join(dirname, aruco_impath)
            cv2.imwrite(aruco_impath,frame)

        raw_df['seq'] = seqs
        raw_df['time_stamp'] = tstamps
        raw_df['frame_id'] = frame_ids
        raw_df['position'] = raw_positions
        raw_df['quaternion'] = raw_orientations

        grf_df['seq'] = seqs
        grf_df['time_stamp'] = tstamps
        grf_df['frame_id'] = frame_ids
        grf_df['position'] = grf_positions
        grf_df['quaternion'] = grf_orientations

        meta_data[cam+'_raw_pose_offline'] = raw_df
        meta_data[cam+'_grf_offline'] = grf_df
    
    new_mname = os.path.basename(meta_data_path).split('.')[0]+'_aruco_offline.pkl'
    dirname = os.path.dirname(meta_data_path)
    new_mpath = os.path.join(dirname, new_mname)

    print 'Saving the results'
    pickle.dump(meta_data, open(new_mpath, 'w'))



if __name__=='__main__':

    main()
