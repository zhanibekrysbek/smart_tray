#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge as bridge
import os

'''
 Camera object used across smart_tray package that is responsible acquiring images and publishing in specified topic.
'''
class Camera(object):

    def __init__(self, name, model='logitech_t1', program_id=1, freq = 30, height = 720, width = 1280):
        # false: silent
        # true: talking
        self.state = False
        self.name = name
        self.freq = freq
        self.rate = rospy.Rate(self.freq)
        self.program_id = program_id
        self.model = model
        self.height = height
        self.width = width


        # self.destination = '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/test/trial_2_im_rec/camera_'+str(program_id)
        # if not os.path.exists(self.destination):
        #     os.makedirs(self.destination)



    # Service callback meant to be referenced by parent thread 
    def service_callback(self, req):
        rospy.loginfo('%s service is called: %s', self.name, req.req)

        if req.req == 'start':
            self.state = True

            rospy.loginfo('%s: Broadcasting images ', self.name)
            return 'Broadcasting images from ' + self.model

        elif req.req == 'stop':
            self.state = False
            self.cap.release()
            rospy.loginfo('%s: going silent', self.name)
            return 'Going silent ' + self.model

        else:
            return 'Unknown command! Use either "start" -or- "stop" '

    # Will output images based on request. Otherwise stays silent.
    def run(self, cam_id, pub):

        self.cap = cv2.VideoCapture(cam_id)
        codec = cv2.VideoWriter_fourcc( 'M', 'J', 'P', 'G')
        self.cap.set(cv2.CAP_PROP_FOURCC,codec)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FPS, self.freq)

        seq = 0
        while not rospy.is_shutdown():
            if self.state:
                ret, frame = self.cap.read()
                if ret==True:

                    # tt = rospy.get_rostime()
                    # secs, nsecs = tt.secs, tt.nsecs
                    # frame_id = self.model + '_' + str(seq)
                    # imname = frame_id.split('/')[-1] + '_' + str(seq) + '_' + str(secs) + '_' + str(nsecs) + '.png'
                    # impath = os.path.join(self.destination, imname)

                    # cv2.imwrite(impath, frame)



                    img_msg = bridge().cv2_to_imgmsg(frame,"bgr8")
                    img_msg.header.stamp = rospy.get_rostime()
                    img_msg.header.seq = seq
                    img_msg.header.frame_id = self.model + '_' + str(seq)
                    pub.publish(img_msg)      
                    
                    seq+=1

            self.rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()
