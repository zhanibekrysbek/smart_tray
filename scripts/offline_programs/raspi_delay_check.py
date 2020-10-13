#!/usr/bin/env python

import rospy
from smart_tray.msg import imu_msg
import numpy as np


'''
Raspbery Pi's clock went out of sync accidentaly.

This code is to estimate its delay.
'''

class IMUDelay:


    def __init__(self):

        self.local_time = []
        self.raspi_time = []

    def callback(self, data):
        
        tl = rospy.get_time()
        secs = data.header.stamp.secs
        nsecs = data.header.stamp.nsecs
        tr = secs+nsecs*1e-9

        self.local_time.append(tl)
        self.raspi_time.append(tr)


    def print_results(self):

        tl = np.array(self.local_time)
        tr = np.array(self.raspi_time)
        dif = np.mean(tl-tr)
        rospy.loginfo('PC-RasPi time: %f', dif)


def main():

    rospy.init_node('raspi_delay_check')

    rospy.loginfo(' RasPi Delay Check! ')

    imudelay = IMUDelay()
    rospy.Subscriber('/imu_data', imu_msg, imudelay.callback)

    rospy.on_shutdown(imudelay.print_results)

    rospy.spin()


def stop():
    rospy.loginfo('Ending the program!')


if __name__ == "__main__":
    main()
