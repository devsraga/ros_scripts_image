#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge

def main():
    rospy.init_node("dev_pub", anonymous=True)
    rospy.loginfo("dev_pub node is initialised")
    pub = rospy.Publisher('dev_web_img', Image, queue_size=30)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        err, img = cam.read()
        img = bridge.cv2_to_imgmsg(img, "bgr8")
        pub.publish(img)
if __name__ == '__main__':
    cam = cv.VideoCapture(0)
    bridge = CvBridge()
    main()

