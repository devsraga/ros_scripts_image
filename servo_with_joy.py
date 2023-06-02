#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy
import cv2 as cv
import numpy as np




def joy_call(data):
    j = data
    pitch = j.axes[1]
    s = int(90*(pitch+1))
    print("pitch:  " + str(s))
    pub.publish(s)


if __name__ == '__main__':
    
    rospy.init_node("dev_joy_node", anonymous=True)
    rospy.loginfo("cam_sub node is initialised")
    rospy.Subscriber('/joy', Joy, joy_call)

    # rospy.init_node("dev_pub", anonymous=True)
    rospy.loginfo("dev_pub node is initialised")
    pub = rospy.Publisher('/servo', UInt16, queue_size=10)
    # pub.publish(s)
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.spin()
