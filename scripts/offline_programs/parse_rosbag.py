#!/usr/bin/env python

# Common Python packages
import pandas as pd 
import argparse
import numpy as np
import os
import sys
sys.path.insert(1,'/home/zhanibek/.local/lib/python2.7/site-packages/cv2')
import cv2
import pickle
from tqdm import tqdm

# ROS packages
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, WrenchStamped


meta_data_paths = [
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/koh_sanket/trial_0/koh_sanket_trial_0_2020-10-11-20-48-27_meta_data_aruco_offline.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/koh_zhanibek/trial_0/koh_zhanibek_trial_0_2020-10-11-21-19-07_meta_data_aruco_offline.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/sanket_vignesh/trial_0/trial_0_2020-10-12-14-47-43_meta_data_aruco_offline.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/sanket_vignesh/trial_1/trial_1_2020-10-12-14-51-53_meta_data_aruco_offline.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/zhanibek_sanket/trial_0/trial_0_2020-10-12-15-02-32_meta_data_aruco_offline.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/zhanibek_vignesh/trial_0/trial_0_2020-10-12-14-57-11_meta_data_aruco_offline.pkl'
]

imu_calibration_data = [
	'/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/imu_calibration/imu_calibration_2020-11-02-14-58-29.bag']


class trayDataParser(object):


	def __init__(self, fname, destination, topiclist):

		self.fname = fname
		self.destination = destination
		self.topiclist = topiclist
		self.base_name = fname.split('/')[-1].split('.bag')[0]

		self.bag = rosbag.Bag(self.fname)

		self.bridge = CvBridge() 

		self.t0 = self.bag.get_start_time()
		self.tf = self.bag.get_end_time()

		if not os.path.exists(destination):
			os.mkdir(destination)



	def process_rosbag_file(self, res={}):

		print(' <<< Processing file: %s  >>>'%self.fname)

		# res = {}
		
		res['summary'] = self.data_summary()
		for topic_type in self.topiclist.keys():

			for topic in self.topiclist[topic_type]:
				print('\n\n\t %s topic is being processed!\n'%topic)

				msgs = self.bag.read_messages(topic)
				cnts = self.bag.get_message_count(topic)
				if topic_type=='rft':
					resdf = self.parse_rft_topic(msgs,total_msgs=cnts)
				elif topic_type=='imu':
					resdf = self.parse_imu_topic(msgs, total_msgs=cnts)
				elif topic_type=='pose':
					resdf = self.parse_pose_topic(msgs, total_msgs=cnts)
				elif topic_type=='images':
					resdf = self.parse_img_topic(msgs, topic, total_msgs=cnts)
				elif topic_type == 'camera':
					resdf = self.parse_image_files(topic)

					message_count = resdf.seq.nunique()
					duration = resdf.time_stamp.iloc[-1]-resdf.time_stamp.iloc[0]
					message_type = 'sensor_msgs/Image'
					freq = message_count/duration
					tempinfo = pd.DataFrame([['/'+topic, message_count, freq,
                                            message_type]], columns=res['summary'].columns[:-2])
					res['summary'] = res['summary'].append(tempinfo, sort=False, ignore_index=True)

				res[topic] = resdf
				
		meta_data_name = os.path.join(self.destination, self.base_name + '_meta_data_v2.pkl')
		
		print '\nSaving results to: ', os.path.basename(meta_data_name)
		
		pickle.dump(res, open(meta_data_name,'w'), protocol=2)

		return

	def data_summary(self):
		descr = self.bag.get_type_and_topic_info()[1]

		topics = []
		# for temp in self.topiclist.values():
		# 	topics+=temp

		res = pd.DataFrame()

		msg_types = []
		freqs = []
		counts = []
		
		for topic in descr.keys():
			des = descr[topic]

			msg_types.append(des.msg_type)
			freqs.append(des.frequency)
			counts.append(des.message_count)
			topics.append(topic)
		
		res['topic_name'] = topics
		res['message_count'] = counts
		res['frequency'] = freqs
		res['msg_type'] = msg_types
		res['start_time'] = [self.t0] * len(counts)
		res['end_time'] = [self.tf] * len(counts)
		res.index = np.arange(len(counts))
		print '\n==================================== ROS bag Info ======================================\n'
		print res
		print '\n========================================================================================\n'
		
		self.summary = res
		return res

	def parse_image_files(self, topic):
		res = pd.DataFrame()

		seqs = []
		timestamps = []
		frame_ids = []
		impaths = []

		basedir = os.path.join(self.destination, topic, 'images')
		imnames = np.array(os.listdir(basedir))

		order = np.argsort([int(p.split("_")[-3]) for p in imnames])
		imnames = imnames[order]

		for im in tqdm(imnames):
			if im.endswith('.png'):
				vals = im.split('_')
				if 'angetube' in im:
					frame_id = vals[0]+'_'+vals[1]
					seq = int(vals[2])
					sec = int(vals[3])
					nsec = int(vals[-1].split('.')[0])
				else: 
					frame_id = vals[0]+'_'+vals[1] + '_'+vals[2]
					seq = int(vals[3])
					sec = int(vals[4])
					nsec = int(vals[-1].split('.')[0])

				frame_ids.append(frame_id)
				seqs.append(seq)
				impaths.append(os.path.join(basedir,im))
				timestamps.append(sec+nsec*1e-9)

		res['seq'] = seqs
		res['time_stamp'] = timestamps
		res['frame_id'] = frame_ids
		res['image_path'] = impaths
		res.index = np.arange(len(seqs))

		return res

	def parse_img_topic(self, msgs, topic_name, total_msgs):
	
		topic_name = topic_name.split('/')[-1]
		mpath_base = os.path.join(self.destination, topic_name)

		if not os.path.exists(mpath_base):
			os.mkdir(mpath_base)

		res = pd.DataFrame()	
		seqs = []
		timestamps = []
		frame_ids = []
		impaths = []

		for ms in tqdm(msgs, total=total_msgs):
			ms = ms.message
			seq = ms.header.seq
			t = ms.header.stamp.to_time()
			frame_id = ms.header.frame_id
			
			im = self.bridge.imgmsg_to_cv2(ms)
			impath = os.path.join(mpath_base, self.base_name+'_'+topic_name+'_'+str(seq)+'.png')
			cv2.imwrite(impath, im)

			seqs.append(seq)
			timestamps.append(t)
			frame_ids.append(frame_id)
			impaths.append(impath)

		res['seq'] = seqs
		res['time_stamp'] = timestamps
		res['frame_id'] = frame_ids
		res['image_path'] = impaths
		res.index = np.arange(len(seqs))
		return res

	def parse_imu_topic(self, msgs, total_msgs):
		res = pd.DataFrame()

		seqs = []
		timestamps = []
		frame_ids = []
		accels = []
		gyros = []
		mags = []

		for ms in tqdm(msgs, total=total_msgs):
			ms = ms.message
			seq = ms.header.seq
			t = ms.header.stamp.to_time()
			frame_id = ms.header.frame_id

			seqs.append(seq)
			timestamps.append(t)
			frame_ids.append(frame_id)
			accels.append(np.array([ms.accel.x, ms.accel.y, ms.accel.z]))
			gyros.append(np.array([ms.gyro.x, ms.gyro.y, ms.gyro.z]))
			mags.append(np.array([ms.mag.x, ms.mag.y, ms.mag.z]))

		res['seq'] = seqs
		res['time_stamp'] = timestamps
		res['frame_id'] = frame_ids
		res['accel'] = accels
		res['gyro'] = gyros
		res['mag'] = mags
		res.index = np.arange(len(seqs))

		return res

	def parse_pose_topic(self, msgs, total_msgs):
		res = pd.DataFrame()

		seqs = []
		timestamps = []
		frame_ids = []
		positions = []
		quaternions = []

		for ms in tqdm(msgs, total=total_msgs):
			ms = ms.message
			seq = ms.header.seq
			t = ms.header.stamp.to_time()
			frame_id = ms.header.frame_id

			seqs.append(seq)
			timestamps.append(t)
			frame_ids.append(frame_id)
			positions.append(np.array([ms.pose.position.x, ms.pose.position.y, ms.pose.position.z]))
			quaternions.append(np.array([ms.pose.orientation.x, ms.pose.orientation.y, 
										 ms.pose.orientation.z, ms.pose.orientation.w]))


		res['seq'] = seqs
		res['time_stamp'] = timestamps
		res['frame_id'] = frame_ids
		res['position'] = positions
		res['quaternion'] = quaternions
		res.index = np.arange(len(seqs))

		return res

	def parse_rft_topic(self, msgs,total_msgs):
		res = pd.DataFrame()

		seqs = []
		timestamps = []
		frame_ids = []
		forces = []
		torques = []

		for ms in tqdm(msgs, total=total_msgs):
			ms = ms.message
			seq = ms.header.seq
			t = ms.header.stamp.to_time()
			frame_id = ms.header.frame_id

			seqs.append(seq)
			timestamps.append(t)
			frame_ids.append(frame_id)
			forces.append(np.array([ms.wrench.force.x, ms.wrench.force.y, ms.wrench.force.z]))
			torques.append(np.array([ms.wrench.torque.x, ms.wrench.torque.y, ms.wrench.torque.z]))


		res['seq'] = seqs
		res['time_stamp'] = timestamps
		res['frame_id'] = frame_ids
		res['force'] = forces
		res['torque'] = torques
		res.index = np.arange(len(seqs))

		return res


def main():

	arg_fmt = argparse.RawDescriptionHelpFormatter

	parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
	parser.add_argument(
		'-f', '--file_name', dest='fname', required=True,
		help='Specify rosbag file to process.'
	)

	parser.add_argument(
		'-o', '--output path', dest='destination', required=False,
		help='path to save the results.'
	)

	parser.add_argument(
		'-w', '--overwrite this file', dest='old_ind', required=False,
		help='index of meta_data_paths'
        )

	args = parser.parse_args(sys.argv[1:])


	# topiclist = {'rft': ['/RFT_FORCE', '/RFT_FORCE_2'],
	# 			 'imu': ['/imu_data'],
	# 			 'pose': ['/cam1_tray_pose_estimation', '/cam2_tray_pose_estimation', '/cam3_tray_pose_estimation',
    #           			'/cam1_tray_grf', '/cam2_tray_grf', '/cam3_tray_grf'],
	# 			 'camera': ['camera_1', 'camera_2', 'camera_3']
	# 			}

	topiclist = {
              'imu': ['/imu_data']
              }

				#  'images':['/camera_1', '/aruco_cam1_pose']}

	# import pdb; pdb.set_trace()

	# topiclist = {'rft': ['/RFT_FORCE', '/RFT_FORCE_2'],
    #           }

	print('Received argument: %s'%args.fname)
	
	if args.destination is None:
		args.destination = os.getcwd()
	
	if args.old_ind is not None:
		path = meta_data_paths[int(args.old_ind)]
		print 'Loading ', os.path.basename(path)
		old_res = pickle.load(open(path))
		
	else:
		old_res={}

	parser = trayDataParser(args.fname, args.destination, topiclist)
	parser.process_rosbag_file(old_res)

	print('Done!')


if __name__=='__main__':

	main()

