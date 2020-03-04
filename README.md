## Introduction
This is a ROS package for reading Adafruit LSM9DS1 9DOF IMU data on Jetson Nano using i2c. It is tested on Python 3.6 using the Adafruit LSM9DS1 Library. The code for reading the data is derived from https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1/blob/master/examples/lsm9ds1_simpletest.py

## Usage
1. Wire up the sensor according to the datasheet at https://cdn-learn.adafruit.com/downloads/pdf/adafruit-lsm9ds1-accelerometer-plus-gyro-plus-magnetometer-9-dof-breakout.pdf?timestamp=1583324342

2. Connect the SDA and SCL port to i2c Bus 0:
  
      SDA - Pin 27

      SCL - Pin 28

## Dependencies
1. Install adafruitblinka
```
$ pip3 install adafruit-blinka
```
2. Install Adafruit CircuitPython LSM9ds1 library
```
$ pip3 install adafruit-circuitpython-lsm9ds1
```

## Installation Instructions - Ubuntu 18.04 with ROS Melodic
Create a catkin workspace and clone lsm9s1_ros_python:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/OOyindamola/lsm9ds1_ros_python/
```
Install Catkin dependencies
```
sudo pip3 install catkin_pkg
```
Build Workspace with Python 3
```
$ cd ~/catkin_ws
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```
> **Note** This will configure catkin_make with Python 3. You may then proceed to use just catkin_make for subsequent builds. 
## Basic Usage
```
roslaunch lsm9ds1_ros_python imu_publisher.launch
```

## References
1. Adafruit CircuitPython Library - https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1
2. Configuring ROS Environment with Python 3 - http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
