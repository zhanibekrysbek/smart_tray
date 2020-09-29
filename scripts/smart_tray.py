#!/usr/bin/env python

import rospy
import subprocess
import time
from datetime import datetime
from std_msgs.msg import Float64, String
from rft_sensor_serial.srv import rft_operation, rft_operation_2

class smart_tray(object):

	def __init__(self):

		rospy.set_param('RFT_COM_PORT', '/dev/ttyUSB0')
		rospy.set_param('RFT_COM_PORT', '/dev/ttyUSB1')
		# wait until services will be available in the network
		rospy.wait_for_service('rft_serial_op_service')
		rospy.wait_for_service('rft_serial_operation_2')

		self.rft_srv_1 = rospy.ServiceProxy('rft_serial_op_service', rft_operation)
		self.rft_srv_2 = rospy.ServiceProxy('rft_serial_op_service_2', rft_operation_2)

	def start(self):
		# trigger two nodes to broadcast data
		self.rft_srv_1(11,0,0,0)
		# self.talker2_serv('talk')

	# to be called on rospy shutdown
	def stop(self):

		rospy.loginfo('Ending the program!')
		self.rft_srv_1(12,0,0,0)
		# self.talker2_serv('stop')

'''
	This node is a central program that initiates sensor readings.

	1. Service call for RFT_1
	2. Service call for RFT_2
	3. Service call for IMU
	4. Service call for camera_1
	5. Service call for camera_2
	6. ? Service call for pose estimation


'''
def main():

	rospy.init_node('smart_tray')

	rospy.loginfo('Starting smart_tray node!')

	tray = smart_tray()

	tray.start()

	rospy.on_shutdown(tray.stop)

	rospy.spin()


if __name__=='__main__':

	main()
