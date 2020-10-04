#!/usr/bin/env python

import rosbag
import pandas as pd 
import argparse
import sys
import numpy as np


def process_topic(tlk_gen):

	tlk_df = pd.DataFrame()

	lt = []
	words = []
	ms_ids = []
	rtime = []

	for ms in tlk_gen:
		rtime.append(ms.timestamp.to_time())
		lt.append(ms.message.time)
		words.append(ms.message.word)
		ms_ids.append(ms.message.msg_id)

	tlk_df['ms_ids'] = ms_ids
	tlk_df['ros_time'] = rtime
	tlk_df['local_time'] = lt
	tlk_df['words'] = words
	tlk_df.index = np.arange(len(ms_ids))

	return tlk_df



def process_rosbag_file(fname):

	print(' <<< Processing file: %s  >>>'%fname)

	bag = rosbag.Bag(fname)

	tlk1 = bag.read_messages('talker_1')
	tlk2 = bag.read_messages('talker_2')

	tlk_1_df = process_topic(tlk1)
	print('< Done with talker_1 >')
	tlk_2_df = process_topic(tlk2)
	print('< Done with talker_1 >')

	base_path = fname.split('.bag')[0]

	tlk_1_df.to_pickle(base_path + '_talker_1.pkl')
	tlk_2_df.to_pickle(base_path + '_talker_2.pkl')



def main():

	arg_fmt = argparse.RawDescriptionHelpFormatter

	parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
	parser.add_argument(
		'-f', '--file_name', dest='fname', required=True,
		help='Specify rosbag file to process.'
	)

	args = parser.parse_args(sys.argv[1:])

	# import pdb; pdb.set_trace()

	print('Received argument: %s'%args.fname)
	process_rosbag_file(args.fname)

	print('Done!')


if __name__=='__main__':

	main()

