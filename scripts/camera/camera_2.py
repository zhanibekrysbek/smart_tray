#!/usr/bin/env python

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge as bridge
from camera_publisher import Camera
from smart_tray.srv import trigger_srv


'''
Logitech t1 camera
'''

program_id = 2
camera_id = 0

def main():

    rospy.init_node('camera_{}_image_acquisition'.format(program_id))
    rospy.on_shutdown(stop)
    rospy.loginfo(' Starting the camera_%d node! ', program_id)

    topic_name = '/camera_' + str(program_id)

    cam = Camera(
        name = topic_name, 
        model = 'logitech_t1',
        program_id=program_id, 
        freq=30, 
        height=720, 
        width=1280)


    pub = rospy.Publisher(topic_name, Image, queue_size=10)
    rospy.Service(topic_name, trigger_srv, cam.service_callback )

    cam.run(camera_id, pub)

    rospy.spin()


def stop():
    rospy.loginfo('Ending the program!')

if __name__=="__main__":
    main()
