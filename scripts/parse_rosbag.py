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



	def process_rosbag_file(self):

		print(' <<< Processing file: %s  >>>'%self.fname)

		res = {}
		
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

				res[topic] = resdf
				

		meta_data_name = os.path.join(self.destination, self.base_name + '_meta_data.pkl')
		pickle.dump(res, open(meta_data_name,'w'))

		return

	def data_summary(self):
		descr = self.bag.get_type_and_topic_info()[1]

		topics = []
		for temp in self.topiclist.values():
			topics+=temp

		res = pd.DataFrame()

		msg_types = []
		freqs = []
		counts = []
		
		for topic in topics:
			des = descr[topic]

			msg_types.append(des.msg_type)
			freqs.append(des.frequency)
			counts.append(des.message_count)
		
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
			torques.append(np.array([ms.wrench.torque.x, ms.wrench.force.y, ms.wrench.force.z]))


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

	args = parser.parse_args(sys.argv[1:])


	topiclist = {'rft': ['/RFT_FORCE', '/RFT_FORCE_2'],
				 'imu': ['/imu_data'],
				 'pose':['/cam1_pose_estimation'],
				 'images':['/camera_1', '/aruco_cam1_pose']}

	# import pdb; pdb.set_trace()


	print('Received argument: %s'%args.fname)
	
	if args.destination is None:
		args.destination = os.getcwd()

	parser = trayDataParser(args.fname, args.destination, topiclist)
	parser.process_rosbag_file()

	print('Done!')


if __name__=='__main__':

	main()

