#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def float_call(data):
    rospy.loginfo("The receved data is: ")
    print(data)
def main():
    rospy.init_node("dev_pub", anonymous=True)
    rospy.loginfo("dev_pub node is initialised")
    rospy.Subscriber('dev_chatter', Float32, float_call)
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()

