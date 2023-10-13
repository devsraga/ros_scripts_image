#!/usr/bin/env python3
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from matplotlib import pyplot as plt
import numpy as np

def smita_call(data):
    rospy.loginfo("The receved data is: ")
    # print(data)
    # print(data.goal.trajectory.points[1].positions)
    # print(data.goal.trajectory.points[-1].positions)
    print("........................................")
    print(data.goal.trajectory.points[0].velocities[0])
    len_points = len(data.goal.trajectory.points)
    print("........................................")
    pos = [list(data.goal.trajectory.points[-i].positions) for i in range(0,len_points+1)]
    vel = [list(data.goal.trajectory.points[-i].velocities) for i in range(0,len_points+1)]
    acc = [list(data.goal.trajectory.points[-i].accelerations) for i in range(0,len_points+1)]
    effort = [list(data.goal.trajectory.points[-i].effort) for i in range(0,len_points+1)]
# Legend....................................
    pos_legend = ["Right_joint_1_position", "Right_joint_2_position", "Right_joint_3_position", "Right_joint_4_position"]
    vel_legend = ["Right_joint_1_velocity", "Right_joint_2_velocity", "Right_joint_3_velocity", "Right_joint_4_velocity"]
    acc_legend = ["Right_joint_1_acceleration", "Right_joint_2_acceleration", "Right_joint_3_acceleration", "Right_joint_4_acceleration"]
    effort_legend = ["Right_joint_1_effort", "Right_joint_2_effort", "Right_joint_3_effort", "Right_joint_4_effort"]


    # pos = data.goal.trajectory.points[i].positions
    # pos = append(pos)
    # i = i+1
    print(vel[0][0])
    vel_raw = [vel[j][2] for j in range(0,len_points)]
    pos_raw = [pos[j][2] for j in range(0,len_points)]
    print(vel_raw)
    print("3333333333333333333333333333333333333333333333")
    print(vel[0])
    print("3333333333333333333333333333333333333333333333")
    print(vel[1])
    print("33333333333333333333333333333333333333333333399")
    print(vel[-1])
    print(f"length {len_points}")
    # print(vel[:,0])
    print(np.pi)
    rad2deg = (180/np.pi)
    print(rad2deg)
    # d_points = data.goal.trajectory.points()
    # print()
    for i in range(0, 4):
        pos_raw[i] = [pos[j][i] for j in range(1,len_points)]
        vel_raw[i] = [vel[j][i] for j in range(1,len_points)]
        # acc_raw[i] = [acc[j][i] for j in range(0,len_points)]
        # effort_raw[i] = [effort[j][i] for j in range(0,len_points)]



    for i in range(0, 4):
        plt.figure(1)
        plt.plot(pos_raw[i])
    plt.legend(pos_legend, loc ="center") 

    for i in range(0, 4):
        plt.figure(2)
        plt.plot(vel_raw[i])
    plt.legend(vel_legend,loc ="center") 
    
    plt.grid()
    plt.show()
    
    # for i in range(0, 4):
    #     plt.plot(effort_raw[i])
    # plt.show()


    
def main():
    rospy.init_node("dev_pub", anonymous=True)
    rospy.loginfo("dev_pub node is initialised")
    # data = FollowJointTrajectoryAction()
    i = 0
    rospy.Subscriber('/robot_arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, smita_call)
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.spin()
    rospy.loginfo("test_1")
    


if __name__ == '__main__':
    
    main()
    
    
    