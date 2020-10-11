#!/usr/bin/env python

import os
import sys

import cv2
import rospy
import argparse
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

'''
This code is used save the images from given Image topic.
'''

path2save = '/home/ki-hwan/catkin_ws/src/smart_tray/data/pose_estimation/calibration/logitech_t2/images'



class ImageRecorder(object):

    def __init__(self, destination, img_topic, ext = '.png'):
        # Initialize Set of Params:
        
        # Name of this module
        self.img_topic = img_topic
        self.ext = ext
        self.destination = destination
        self.basedir = os.path.join(self.destination, self.img_topic.split('/')[-1], 'images')
        self.bridge = CvBridge()

        print(self.basedir)

        if not os.path.exists(self.basedir):
            os.makedirs(self.basedir)
        
        
        # Subscriber

        rospy.Subscriber(self.img_topic, Image, self.callback)


    def callback(self, data):


        im = self.bridge.imgmsg_to_cv2(data,"bgr8")

        seq = data.header.seq
        secs = data.header.stamp.secs
        nsecs = data.header.stamp.nsecs
        frame_id = data.header.frame_id

        imname = frame_id.split('/')[-1] + '_' + str(seq) + '_' + str(secs) + '_' + str(nsecs) + self.ext
        impath = os.path.join(self.basedir, imname)
        
        cv2.imwrite(impath, im)


def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter

    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                        description=main.__doc__)

    parser.add_argument(
        '-d', '--destination', dest='destination', required=False,
        help='name of the data'
    )


    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node...")
    rospy.init_node("image_recorder")

    if args.destination is None:
		args.destination = os.getcwd()

    print args.destination

    imrec1 = ImageRecorder( args.destination ,img_topic='/camera_1', ext='.png')
    imrec1_aruco = ImageRecorder( args.destination ,img_topic='/aruco_image_camera_1', ext='.jpg')

    rospy.loginfo('Recording in Progress! Press CTRL+C to stop')

    rospy.spin()

    return

if __name__ == '__main__':
    main()
