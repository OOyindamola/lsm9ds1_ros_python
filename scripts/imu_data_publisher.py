#!/usr/bin/env python3
import time
import board
import busio
import adafruit_lsm9ds1

import numpy as np
import rospy
from sensor_msgs.msg import Imu

# I2C connection:
i2c = busio.I2C(board.SCL_1, board.SDA_1)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

def read_data():
    pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    imu_msg = Imu()
    
    while not rospy.is_shutdown():
        # Read acceleration, magnetometer, gyroscope, temperature.
        accel_x, accel_y, accel_z = sensor.acceleration
        mag_x, mag_y, mag_z = sensor.magnetic
        gyro_x, gyro_y, gyro_z = sensor.gyro
        temp = sensor.temperature
        
        #Header
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        
        #angular velocities from gyroscope
        imu_msg.angular_velocity.x = gyro_x * np.pi/180
        imu_msg.angular_velocity.y = gyro_y * np.pi/180
        imu_msg.angular_velocity.z = gyro_z  * np.pi/180
        
        #linear accelerations from accelerometer
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        
        # Print values.
        print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
        print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
        print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
        print('Temperature: {0:0.3f}C'.format(temp))

        pub.publish(imu_msg)
        rate.sleep()

        
if __name__ == '__main__':
    try:
        read_data()
    except rospy.ROSInterruptException:
        pass