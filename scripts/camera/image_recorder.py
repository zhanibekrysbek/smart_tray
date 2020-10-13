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


class ImageRecorder(object):

    def __init__(self, destination, img_topic, beat_topic = '/imbeat', ext = '.png'):
        # Initialize Set of Params:
        
        # Name of this module
        self.img_topic = img_topic
        self.ext = ext
        self.destination = destination
        self.basedir = os.path.join(self.destination, self.img_topic.split('/')[-1], 'images')
        self.bridge = CvBridge()

        self.img_msgs = []
        self.imnames = []

        print(self.basedir)

        if not os.path.exists(self.basedir):
            os.makedirs(self.basedir)
        
        
        # Subscriber
        self.sub = rospy.Subscriber(self.img_topic, Image, self.callback)
        # self.sub = rospy.Subscriber(self.img_topic, Bool, self.save_callback)


    def callback(self, data):

        # self.img_msgs.append(data)

        im = self.bridge.imgmsg_to_cv2(data, "bgr8")

        seq = data.header.seq
        secs = data.header.stamp.secs
        nsecs = data.header.stamp.nsecs
        frame_id = data.header.frame_id

        imname = frame_id.split(
            '/')[-1] + '_' + str(seq) + '_' + str(secs) + '_' + str(nsecs) + self.ext
        impath = os.path.join(self.basedir, imname)

        cv2.imwrite(impath, im)

    def save_callback(self, flag):

        if len(self.img_msgs)!=0:
            data = self.img_msgs.pop(0)

            im = self.bridge.imgmsg_to_cv2(data, "bgr8")

            seq = data.header.seq
            secs = data.header.stamp.secs
            nsecs = data.header.stamp.nsecs
            frame_id = data.header.frame_id

            imname = frame_id.split('/')[-1] + '_' + str(seq) + '_' + str(secs) + '_' + str(nsecs) + self.ext
            impath = os.path.join(self.basedir, imname)

            cv2.imwrite(impath, im)


    def save(self):

        rospy.loginfo('Saving %s topic!', self.img_topic)

        # for im, imname in tqdm(zip(self.ims, self.imnames)):
        #     cv2.imwrite(imname, im)
        imnum = len(self.img_msgs)
        # import pdb; pdb.set_trace()
        for i in tqdm(range(imnum)):

            data = self.img_msgs.pop()

            im = self.bridge.imgmsg_to_cv2(data,"bgr8")

            seq = data.header.seq
            secs = data.header.stamp.secs
            nsecs = data.header.stamp.nsecs
            frame_id = data.header.frame_id

            imname = frame_id.split('/')[-1] + '_' + str(seq) + '_' + str(secs) + '_' + str(nsecs) + self.ext
            impath = os.path.join(self.basedir, imname)

            cv2.imwrite(impath, im)


class Recoreders:

    def __init__(self, recorders):
        self.recorders = recorders
        self.state = False

    def service_callback(self, req):
        rospy.loginfo('image_recorder service is called: %s', req.req)


        if req.req == 'stop':
            rospy.loginfo('Image recorder is stopping!')

            for rec in self.recorders:
                rec.sub.unregister()
            
            self.state = True
            
            return 'Saving images!'

        else:
            return 'Unknown command! Use "stop" to stop recording'

    def run(self):

        while not rospy.is_shutdown() and not self.state:
            continue

        self.saveall()



    def saveall(self):

        for rec in self.recorders:
            rospy.loginfo('%s topic: %d', rec.img_topic, len(rec.img_msgs))

        for rec in self.recorders:
            rec.save()

        rospy.loginfo('Done!')



def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter

    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                        description=main.__doc__)

    parser.add_argument(
        '-d', '--destination', dest='destination', required=False,
        help='name of the data'
    )


    args = parser.parse_args(rospy.myargv()[1:])

    
    rospy.init_node("image_recorder")
    rospy.loginfo('image_recorder is initialized! ')


    if args.destination is None:
		args.destination = os.getcwd()


    print args.destination

    imrec1 = ImageRecorder( args.destination ,img_topic='/camera_1', ext='.png')
    # imrec1_aruco = ImageRecorder( args.destination ,img_topic='/aruco_image_tray_camera_1', ext='.jpg')

    imrec2 = ImageRecorder(args.destination, img_topic='/camera_2', ext='.png')

    imrec3 = ImageRecorder(args.destination, img_topic='/camera_3', ext='.png')


    # recs = Recoreders(recorders=[imrec1, imrec2, imrec3])

    # rospy.on_shutdown(recs.saveall)
    # rospy.Service('image_recorder', trigger_srv, recs.service_callback)

    # rospy.loginfo('Recording in Progress! Send "stop" request to image_recorder service!')

    rospy.loginfo('Recording in Progress! Press CTRL+C to stop recording...')

    # recs.run()

    rospy.spin()
    

    return

if __name__ == '__main__':
    main()
