#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge as bridge


class Camera(object):

    def __init__(self, name, rate = 30):
        # false: silent
        # true: talking
        self.state = False
        self.name = name
        self.rate = rospy.Rate(rate)

    def service_callback(self, req):
        rospy.loginfo('%s service is called: %s', self.name, req.req)

        if req.req == 'start':
            self.state = True
            self.t0 = rospy.get_time()

            rospy.loginfo('%s: Broadcasting images ', self.name)
            return 'Broadcasting images from Camera_1 '

        elif req.req == 'stop':
            self.state = False
            rospy.loginfo('%s: going silent', self.name)
            return 'Going silent Camera_1'

        else:
            return 'Unknown command! Use either "start" -or- "stop" '

    def run(self, cam_id, pub):


        cap = cv2.VideoCapture(cam_id)
        seq = 0
        while not rospy.is_shutdown():
            if self.state:
                ret, frame = cap.read()
                if ret==True:
                    img_msg = bridge().cv2_to_imgmsg(frame,"bgr8")
                    img_msg.header.stamp = rospy.get_rostime()
                    img_msg.header.seq = seq
                    img_msg.header.frame_id = 'camera_1_' + str(seq)
                    pub.publish(img_msg)                
                    
                    seq+=1

            self.rate.sleep()

        cap.release()
        cv2.destroyAllWindows()