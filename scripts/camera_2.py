#!/usr/bin/env python

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge as bridge
from camera_publisher import Camera
from smart_tray.srv import trigger_srv



def main():

    
    rospy.init_node('camera_2_image_acquisition')
    rospy.on_shutdown(stop)
    rospy.loginfo(' Starting the camera_2 node! ')

    cam = Camera('camera_2')
    pub = rospy.Publisher('/camera_2', Image, queue_size=10)
    serv = rospy.Service('camera_2', trigger_srv, cam.service_callback )

    cam.run(1, pub)

    rospy.spin()

def stop():
    rospy.loginfo('Ending the program!')

if __name__=="__main__":
    main()

