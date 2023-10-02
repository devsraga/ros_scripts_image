//################################################################################################
// 5. ROS callback functions + ROS callback functions + ROS callback functions + ROS callback func
//################################################################################################

// CF_0.........................................
//void servo_callback_multi_DOF_joint_states( const sensor_msgs::MultiDOFJointState& MultiDOFJointState_msg){   
//
//
//      String MultiDOFJointState_msg.joint_name[] = {DXL_joint_1, DXL_joint_2, DXL_joint_3, DXL_joint_4, DXL_joint_5, DXL_joint_6}
//      
//      String DXL_joints[] = MultiDOFJointState_msg.joint_name();
//      float  motor_transforms[] = MultiDOFJointState_msg.transforms();
//      float  motor_twists[] = MultiDOFJointState_msg.twist();
//      
//      for (int i = 0; i<=6; i++){
//      float  motor_position = MultiDOFJointState_msg.axes[0];
//      }
//      float  motor_1_position = MultiDOFJointState_msg.axes[0];
//      float  motor_2_position = MultiDOFJointState_msg.axes[1];
//      float  motor_3_position = MultiDOFJointState_msg.axes[2];
//      float  motor_4_position = MultiDOFJointState_msg.axes[2];
//      float  motor_5_position = MultiDOFJointState_msg.axes[2];
//      float  motor_6_position = MultiDOFJointState_msg.axes[2];
      

  // Set Goal Velocity using RPM
//  dxl.setGoalVelocity(DXL_ID, s, UNIT_RPM);
//  float goal_pos[6] = motor_positons;
//  mov(V_max, goal_pos);
//  
//}


// CF_1.........................................
void servo_callback_joystick_cammonds( const sensor_msgs::Joy& joy_msg){

      float motors_positions_raw[] = {joy_msg.axes[0], joy_msg.axes[1], joy_msg.axes[2], joy_msg.axes[3], joy_msg.axes[4], joy_msg.axes[5]};

      mov_pos_by_joy(motors_positions_raw); 
//      mov_vel(motors_positions_raw); 

}


// CF_2.........................................
void servo_callback_supplied_js_cammonds( const sensor_msgs::JointState& sup_js_msg){

//    mov_pos(sup_js_msg.position);
 
}
// CF_2.........................................
//void servo_callback_joint_states( const sensor_msgs::JointState& JointState_msgs){   
//
//
//      String DXL_joint[] = JointState_msgs.name[];
//      float motors_positions[] = JointState_msgs.position[];
//      float motors_velocity[] = JointState_msgs.velocity[];
//      float motors_effort[] = JointState_msgs.effort[];
//
//      
//
//  // Set Goal Velocity using RPM
//  dxl.setGoalVelocity(DXL_ID, s, UNIT_RPM);
//  mov(V_max, motors_positions);
//  
//}
