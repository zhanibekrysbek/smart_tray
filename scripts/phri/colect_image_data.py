#!/usr/bin/env python

import os
import sys
sys.path.insert(1,'/home/KOH/.local/lib/python2.7/site-packages/cv2')
import cv2
import rospy
import argparse
from datetime import datetime
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

'''
This code is used to collect images from a camera.
'''

path2save = '/home/catkin_ws/src/smart_tray/data/pose_estimation/calibration/logitech_t1/images'

# A class to fetch color images from Kinect
class color_image:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera_1', Image, self.callback)
        self.image = None

    def callback(self,data):

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

# performs camera projection
# 
def record(name, freq=3):

    # initialize image-data retriever classes
    ci = color_image()
    # wait some time to get the data flow
    rospy.sleep(1.)
    # Publish visuals of bbox extraction
    rate = rospy.Rate(freq)

    print 'START recording..'

    seq = 0
    while not rospy.is_shutdown(): # and n <=10:

        img = ci.image # np.ndarray
        
        now = datetime.now()
        stamp = now.strftime("%m%d%Y%H%M%S_%f")
        
        color_name = os.path.join(path2save, name+'_'+str(seq)+'_'+stamp+'.png')
        
        retval = cv2.imwrite(color_name, img)
        
        # import pdb;pdb.set_trace()
        print seq
        print '\t', retval
        seq += 1
        rate.sleep()

    print 'done!'

    return

def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter

    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                        description=main.__doc__)

    parser.add_argument(
        '-n', '--name', dest='name',
        help='name of the data'
    )
    parser.add_argument(
        '-r', '--rate', dest='rate',
        help='frequency rate to record at'
    )


    args = parser.parse_args(rospy.myargv()[1:])

    print args

    print("Initializing node...")
    rospy.init_node("color_image_collection")

    record(name=args.name, freq=float(args.rate))

    return

if __name__ == '__main__':
    main()
