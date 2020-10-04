#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge as bridge

'''
 Camera object used across smart_tray package that is responsible acquiring images and publishing in specified topic.
'''
class Camera(object):

    def __init__(self, name, program_id=1, rate = 30):
        # false: silent
        # true: talking
        self.state = False
        self.name = name
        self.rate = rospy.Rate(rate)
        self.program_id = program_id

    # Service callback meant to be referenced by parent thread 
    def service_callback(self, req):
        rospy.loginfo('%s service is called: %s', self.name, req.req)

        if req.req == 'start':
            self.state = True

            rospy.loginfo('%s: Broadcasting images ', self.name)
            return 'Broadcasting images from Camera_1 '

        elif req.req == 'stop':
            self.state = False
            rospy.loginfo('%s: going silent', self.name)
            return 'Going silent Camera_1'

        else:
            return 'Unknown command! Use either "start" -or- "stop" '

    # Will output images based on request. Otherwise stays silent.
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
                    img_msg.header.frame_id = 'camera_' + str(self.program_id) + '_' + str(seq)
                    pub.publish(img_msg)                
                    
                    seq+=1

            self.rate.sleep()

        cap.release()
        cv2.destroyAllWindows()