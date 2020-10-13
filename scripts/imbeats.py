#!/usr/bin/env python

import os
import sys

import cv2
import rospy
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tqdm import tqdm
from smart_tray.srv import trigger_srv
from std_msgs.msg import Bool

'''
This code is used save the images from given Image topic.
'''
def imbeats():

    pub = rospy.Publisher('/imbeats',Bool, queue_size=60)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        pub.publish(True)
        rate.sleep()



def main():

    rospy.init_node("imbeats")
    rospy.loginfo('imbeats is initialized!')

    imbeats()

    return


if __name__ == '__main__':
    main()
