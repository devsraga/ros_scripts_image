#!/usr/bin/env python3
import rospy

def main():
	rospy.init_node("test_node", anonymous=True)
	rospy.loginfo("test_node is initialised")
	rospy.logwarn("you can log any warning by rospy.logwarn('')")
	rospy.logerr("similerly you can also log err by rospy.logerr('error')")
	# rospy.spin()  # this line will not  shutdown the node



if __name__ == '__main__':
	main()
