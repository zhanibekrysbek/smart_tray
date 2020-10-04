#!/usr/bin/env python

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge as bridge
from camera_publisher import Camera
from smart_tray.srv import trigger_srv



def main():

    rospy.init_node('camera_1_image_acquisition')
    rospy.on_shutdown(stop)
    rospy.loginfo(' Starting the camera_1 node! ')

    cam = Camera('camera_1')
    pub = rospy.Publisher('/camera_1', Image, queue_size=10)
    serv = rospy.Service('camera_1', trigger_srv, cam.service_callback )

    cam.run(0, pub)

    rospy.spin()

    # cap = cv2.VideoCapture(0)
    # r = rospy.Rate(30);

    # while not rospy.is_shutdown():
    #     ret, frame = cap.read()
    #     if ret==True:
    #         conv_frame = bridge().cv2_to_imgmsg(frame,"bgr8")
    #         pub.publish(conv_frame)

    #     r.sleep()

    # cap.release()
    # cv2.destroyAllWindows()


def stop():
    rospy.loginfo('Ending the program!')

if __name__=="__main__":
    main()

