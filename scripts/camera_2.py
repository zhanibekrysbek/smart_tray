#!/usr/bin/env python

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge as bridge
from camera_publisher import Camera
from smart_tray.srv import trigger_srv


program_id = 2
camera_id = 0

def main():

    rospy.init_node('camera_%d_image_acquisition', program_id)
    rospy.on_shutdown(stop)
    rospy.loginfo(' Starting the camera_%d node! ', program_id)

    cam = Camera('/camera_' + str(program_id). program_id=program_id)

    topic_name = '/camera_' + str(program_id)
    pub = rospy.Publisher(topic_name, Image, queue_size=10)
    rospy.Service('/camera_' + str(program_id), trigger_srv, cam.service_callback )

    cam.run(camera_id, pub)

    rospy.spin()


def stop():
    rospy.loginfo('Ending the program!')

if __name__=="__main__":
    main()
