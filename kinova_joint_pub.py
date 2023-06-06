#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16, Float64MultiArray, Float64
from sensor_msgs.msg import Joy, JointState
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
    j1 = j.axes[0]
    j2 = j.axes[1]
    j3 = j.axes[2]
    j4 = j.axes[3]
    j5 = j.axes[4]
    j6 = j.axes[5]
    j7 = j.buttons[0]
    j8 = j.buttons[1]
    j9 = j.buttons[2]	
    
    s1 = (180*(j1+1))*(3.14/180)
    s2 = (180*(j2+1))*(3.14/180)
    s3 = (180*(j3+1))*(3.14/180)
    s4 = (180*(j4+1))*(3.14/180)
    s5 = (180*(j5+1))*(3.14/180)
    s6 = (180*(j6+1))*(3.14/180)
    s7 = 2*j7-1
    s8 = 2*j8-1
    s9 = j9


    #print("pitch:  " + str(s))
    #print("yaw:  " + str(s2))
    array = [s1, s2, s3, s4, s5, s6, s7, s8]
    servo_data1 = Float64()
    servo_data2 = Float64()
    servo_data3 = Float64()
    servo_data4 = Float64()
    servo_data5 = Float64()
    servo_data6 = Float64() 
    servo_data7 = Float64()
    servo_data8 = Float64()
    servo_data9 = Float64() 
    servo_data1.data = s1 
    servo_data2.data = s2 
    servo_data3.data = s3 
    servo_data4.data = s4 
    servo_data5.data = s5 
    servo_data6.data = s6 
    servo_data7.data = s7 
    servo_data8.data = s8 
    servo_data9.data = s9 
    pub1.publish(servo_data1)
    pub2.publish(servo_data2)
    pub3.publish(servo_data3)
    pub4.publish(servo_data4)
    pub5.publish(servo_data5)
    pub6.publish(servo_data6)
    pub7.publish(servo_data7)
    pub8.publish(servo_data8)
    pub9.publish(servo_data9)


if __name__ == '__main__':
    
    rospy.init_node("dev_joy_node", anonymous=True)
    rospy.loginfo("cam_sub node is initialised")
    rospy.Subscriber('/joy', Joy, joy_call)

    #rospy.init_node("dev_kinova_joint", anonymous=True)
    rospy.loginfo("dev_pub node is initialised")

    pub1 = rospy.Publisher('/j2n6s300/joint_1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/j2n6s300/joint_2_position_controller/command', Float64, queue_size=10) 
    pub3 = rospy.Publisher('/j2n6s300/joint_3_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/j2n6s300/joint_4_position_controller/command', Float64, queue_size=10) 
    pub5 = rospy.Publisher('/j2n6s300/joint_5_position_controller/command', Float64, queue_size=10)
    pub6 = rospy.Publisher('/j2n6s300/joint_6_position_controller/command', Float64, queue_size=10) 
    pub7 = rospy.Publisher('/j2n6s300/finger_1_position_controller/command', Float64, queue_size=10) 
    pub8 = rospy.Publisher('/j2n6s300/finger_2_position_controller/command', Float64, queue_size=10)
    pub9 = rospy.Publisher('/j2n6s300/finger_3_position_controller/command', Float64, queue_size=10) 
    
    
    # pub.publish(s)
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.spin()
