#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
def sub_callback(dev_pub_data):
    print(dev_pub_data)
def main():
    rospy.init_node("dev_pub", anonymous=True)
    rospy.loginfo("dev_sub node is initialised")
    rospy.Subscriber('dev_chatter', String, sub_callback)
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.spin()
if __name__ == '__main__':
    main()
