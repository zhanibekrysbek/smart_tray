#!/usr/bin/env python

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge as bridge

pub = rospy.Publisher('/color_image',Image,queue_size=10)
rospy.init_node('send_image')
r = rospy.Rate(25)

def main():
    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret==True:
            conv_frame = bridge().cv2_to_imgmsg(frame,"bgr8")
            pub.publish(conv_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
        r.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()

