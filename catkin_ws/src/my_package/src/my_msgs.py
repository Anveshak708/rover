#! /usr/bin/env python

import rospy
from time import sleep
from STservo_sdk import *
from geometry_msgs.msg import Twist
from time import sleep
Left_velocity = 0
Right_velocity = 0
encoder_maximum = 32000
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'
MOTOR_SPEED = 300
MOTOR_ACCL = 0
portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()
def odom_callback(msg1):
    global Right_velocity , Left_velocity
    linear_vel = msg1.linear.x
    angular_vel = msg1.angular.z
    wheel_sep = 0.30
    Right_velocity =((linear_vel * 2) + (angular_vel * wheel_sep)) / (2.0)
    Left_velocity =((linear_vel * 2) - (angular_vel * wheel_sep)) / (2.0)
    Right_velocity = int(Right_velocity * 10000)
    Left_velocity = int(Left_velocity * 10000)
