#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("dev_pub", anonymous=True)
    rospy.loginfo("dev_pub node is initialised")
    pub = rospy.Publisher('dev_chatter', String, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish("tuhu tuhu tuhu...")
if __name__ == '__main__':
    main()
