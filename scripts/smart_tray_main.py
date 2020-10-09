#!/usr/bin/env python

import rospy
import subprocess
import time
from datetime import datetime
from std_msgs.msg import Float64, String
from rft_sensor_serial.srv import rft_operation
from smart_tray.srv import trigger_srv


class smart_tray(object):
	'''
	Class handle of entire tray designed at Robotics Lab, UIC.
	Mainly, it triggers resonsible services for each device including:
		- 2 RFT60 sensors
		- 2 Cameras
		- 1 IMU
		- Camera Pose Estimation Module
	'''

	def __init__(self):

		# Set COM port params
		rospy.set_param('RFT_COM_PORT', '/dev/ttyUSB2')
		rospy.set_param('RFT_COM_PORT_2', '/dev/ttyUSB3')

		# wait until services will be available in the network
		rospy.wait_for_service('rft_serial_op_service')
		rospy.wait_for_service('rft_serial_op_service_2')
		rospy.wait_for_service('imu_srv')
		rospy.wait_for_service('camera_1')

		rospy.loginfo('Both RFT sensors are available!')

		self.rft_srv_1 = rospy.ServiceProxy('rft_serial_op_service', rft_operation)
		self.rft_srv_2 = rospy.ServiceProxy('rft_serial_op_service_2', rft_operation)
		self.imu_srv = rospy.ServiceProxy('imu_srv', trigger_srv)
		self.camera_1_srv = rospy.ServiceProxy('camera_1', trigger_srv)

	def start(self):
		# Call Serial Number so it gets written in message frame.
		res1 = self.rft_srv_1(2,0,0,0)
		res2 = self.rft_srv_2(2,0,0,0)
		res3 = self.imu_srv('start')
		res4 = self.camera_1_srv('start')

		if res1.result !=0 or res2.result !=0:
			rospy.logwarn('Something wrong with Sensor Communication!')

		rospy.sleep(0.5)

		# trigger two nodes to broadcast data
		self.rft_srv_1(11,0,0,0)
		self.rft_srv_2(11,0,0,0)

		rospy.sleep(0.5)
		# apply bias
		res1 = self.rft_srv_1(17,1,0,0)
		res2 = self.rft_srv_2(17,1,0,0)

	# to be called on rospy shutdown
	def stop(self):

		rospy.loginfo('Ending the program!')
		self.rft_srv_1(12,0,0,0)
		self.rft_srv_2(12,0,0,0)
		self.imu_srv('stop')
		self.camera_1_srv('stop')

'''
	This node is a central program that initiates sensor readings.

	1. Service call for RFT_1
	2. Service call for RFT_2
	3. Service call for IMU
	4. Service call for camera_1
	5. Service call for camera_2
	6. 


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
