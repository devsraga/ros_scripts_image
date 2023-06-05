#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16, Float64MultiArray
from sensor_msgs.msg import Joy
import cv2 as cv
import numpy as np
#rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
# lsusb is a linux camond to check all usb devises
# to know which port arduino is connected type "ls /dev/tty*"
# give the permition to the /dev/tty* ports if require by chmod 666 /dev/ttyACM0 ...etc
#   Serial.begin(115200);
#  nh.getHardware()->setBaud(115200);




def joy_call(data):
    j = data
    pitch = j.axes[1]
    yaw = j.axes[4]
    s = 90*(pitch+1)
    s2 = 90*(yaw+1)
    print("pitch:  " + str(s))
    print("yaw:  " + str(s2))
    array = [s, s2]
    servo_data = Float64MultiArray()  # the data to be sent, initialise the array
    servo_data.data = array # assign the array with the value you want to send
    pub.publish(servo_data)


if __name__ == '__main__':
    
    rospy.init_node("dev_joy_node", anonymous=True)
    rospy.loginfo("cam_sub node is initialised")
    rospy.Subscriber('/joy', Joy, joy_call)

    # rospy.init_node("dev_pub", anonymous=True)
    rospy.loginfo("dev_pub node is initialised")
    pub = rospy.Publisher('/servo', Float64MultiArray, queue_size=10)
    
    # pub.publish(s)
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.spin()
