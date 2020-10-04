#!/usr/bin/env python

'''
    This code is meant to be executed in Raspberry Pi with 
    LSM9DS1 IMU sensor from Adafruit via I2C communication.
    It assumes that LSM9DS1_RaspberryPi_Library is installed 
    at "/home/pi" directory.
'''

import rospy
import sys
import time
from smart_tray.srv import trigger_srv
from smart_tray.msg import imu_msg


sys.path.append( '/home/pi/LSM9DS1_RaspberryPi_Library/example')

from LSM9DS1_Basic_I2C import lib 


class IMU_sensor():

    def __init__(self, srvname, rate = 200):
        # false: silent
        # true: talking
        self.state = False
        self.name = srvname
        self.rate = rospy.Rate(rate)

    def service_callback(self, req):
        rospy.loginfo('%s service is called: %s', self.name, req.req)

        if req.req == 'start':
            self.state = True

            rospy.loginfo('%s: broadcasting sensor readings!', self.name)
            return 'broadcasting sensor readings'

        elif req.req == 'stop':
            self.state = False
            rospy.loginfo('%s: going silent', self.name)
            return 'Going silent'


    def broadcast_sensor_values(self, pub):

        g = 9.803 # m/s^2

        ind = 0

        imu = lib.lsm9ds1_create()
        lib.lsm9ds1_begin(imu)
        if lib.lsm9ds1_begin(imu) == 0:
            rospy.loginfo("Failed to communicate with LSM9DS1.")
            quit()
        lib.lsm9ds1_calibrate(imu)

        while not rospy.is_shutdown():
            if self.state:
                while lib.lsm9ds1_gyroAvailable(imu) == 0:
                    pass
                lib.lsm9ds1_readGyro(imu)
                while lib.lsm9ds1_accelAvailable(imu) == 0:
                    pass
                lib.lsm9ds1_readAccel(imu)
                while lib.lsm9ds1_magAvailable(imu) == 0:
                    pass
                lib.lsm9ds1_readMag(imu)

                # gx = lib.lsm9ds1_getGyroX(imu)
                # gy = lib.lsm9ds1_getGyroY(imu)
                # gz = lib.lsm9ds1_getGyroZ(imu)

                gx, gy, gz = lib.lsm9ds1_getGyroX(imu), lib.lsm9ds1_getGyroY(imu), lib.lsm9ds1_getGyroZ(imu)
                ax, ay, az = lib.lsm9ds1_getAccelX(imu), lib.lsm9ds1_getAccelY(imu), lib.lsm9ds1_getAccelZ(imu)
                mx, my, mz = lib.lsm9ds1_getMagX(imu), lib.lsm9ds1_getMagY(imu), lib.lsm9ds1_getMagZ(imu)

                cgx, cgy, cgz = lib.lsm9ds1_calcGyro(imu, gx), lib.lsm9ds1_calcGyro(imu, gy), lib.lsm9ds1_calcGyro(imu, gz)
                cax, cay, caz = lib.lsm9ds1_calcAccel(imu, ax), lib.lsm9ds1_calcAccel(imu, ay), lib.lsm9ds1_calcAccel(imu, az)
                cmx, cmy, cmz = lib.lsm9ds1_calcMag(imu, mx), lib.lsm9ds1_calcMag(imu, my), lib.lsm9ds1_calcMag(imu, mz)


                # Define the message and publish
                msg = imu_msg()

                msg.accel.x = cax * g
                msg.accel.y = cay * g
                msg.accel.z = caz * g

                msg.gyro.x = cgx
                msg.gyro.y = cgy
                msg.gyro.z = cgz

                msg.mag.x = cmx
                msg.mag.y = cmy
                msg.mag.z = cmz

                rtime = rospy.get_rostime()
                msg.header.seq = ind
                msg.header.stamp.secs = rtime.secs
                msg.header.stamp.nsecs = rtime.nsecs
                msg.header.frame_id = "LSM9DS1_IMU_SENSOR_DATA"

                ind += 1
                pub.publish(msg)

            self.rate.sleep()

                # print('\n ---')
                # rospy.loginfo("Gyro: %f, %f, %f [deg/s]" % (cgx, cgy, cgz))
                # rospy.loginfo("Accel: %f, %f, %f [Gs]" % (cax, cay, caz))
                # rospy.loginfo("Mag: %f, %f, %f [gauss]" % (cmx, cmy, cmz))
    def stop(self):
        rospy.loginfo('Ending the program!')


def main():

    rospy.init_node('imu_lsm9ds1')
    rospy.loginfo('Initialized imu_lsm9ds1 node!')

    pub = rospy.Publisher('/imu_data', imu_msg, queue_size=10)

    sensor = IMU_sensor('imu_srv')

    rospy.Service('imu_srv', trigger_srv, sensor.service_callback )

    rospy.loginfo('Service started! Waiting for a call...')

    sensor.broadcast_sensor_values(pub)
    rospy.on_shutdown(sensor.stop)


if __name__=='__main__':

    main()


