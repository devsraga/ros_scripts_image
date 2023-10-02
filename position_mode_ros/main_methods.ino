
//################################################################################################
// 10. Methods Methods Methods Methods Methods Methods Methods Methods Methods Methods Methods Met
//################################################################################################

   // 10.1  Movin the stewart platform.........................
      void mov(int V_max, float goal_pos[], uint8_t joint_cnt){
         
        int flag = 0;
        for(int i =0; i< joint_cnt; i++){
          if((i+1)%2!=0){
            if(goal_pos[i]>=60 and goal_pos[i]<=180){
              flag++; //flag++ --> flag=flag+1
              }
            else{  
              
            }
//      cout<<goal_pos[i]<<' of actuator' i+1 ' is out of range [60,180]'<<endl;
    }
    else if((i+1)%2==0){
            if(goal_pos[i]>=180 and goal_pos[i]<=300){
            flag++;
            }
    }
  }

  if(flag==6){

    for(int i=0; i< joint_cnt; i++){
      dxl.setGoalPosition(DXL_ID[i], goal_pos[i], UNIT_DEGREE);
      }
    digitalWrite(warning_pin_1,HIGH);
    digitalWrite(warning_pin_3,HIGH);
    digitalWrite(warning_pin_2,LOW);
    delay(50);

    }
    else{

    digitalWrite(warning_pin_2,HIGH);
    digitalWrite(warning_pin_3,HIGH);
    for(int j=0; j<100; j++){
    digitalWrite(warning_pin_1,LOW);
    delay(50);
    digitalWrite(warning_pin_1,HIGH);
    delay(50);
    }
    }
  }

   // 10.2  Movin the stewart platform.........................
         void mov_1(float* goal_pos, uint8_t joint_cnt){



float present_position[] = {0, 0, 0, 0, 0, 0}; 

    for (int i = 0; i < joint_cnt; i++){
         present_position[i] = dxl.getPresentPosition(DXL_ID[i], UNIT_DEGREE);
    }
             
  for (int j= 0; j< joint_cnt; j++){
    dxl.setGoalPosition(DXL_ID[j], goal_pos[j], UNIT_DEGREE);
  }
  
  while (abs(goal_pos[0] - present_position[0]) > 2.0 && abs(goal_pos[1] - present_position[1])> 2.0 && abs(goal_pos[2] - present_position[2])> 2.0 && abs(goal_pos[3] - present_position[3])> 2.0 && abs(goal_pos[4] - present_position[4])> 2.0 && abs(goal_pos[5] - present_position[5])> 2.0)
  {

    for (int i = 0; i< joint_cnt; i++){
         present_position[i] = dxl.getPresentPosition(DXL_ID[i], UNIT_DEGREE);
    }


  }
         }

   // 10.3  Movin motors velocity mode.........................
         void mov_vel(float goal_vel_raw[], uint8_t joint_cnt){
          float motors_velocity[joint_cnt];
          for (int i= 0; i< joint_cnt; i++){
            motors_velocity[i] = 30 * goal_vel_raw[i];
            dxl.setGoalVelocity(DXL_ID[i], motors_velocity[i], UNIT_RPM);
          }
 }

     
    // 10.5  Movin motors position mode.........................
         void mov_pos_by_joy(float goal_pos_raw[], uint8_t joint_cnt){
          float motors_pos[joint_cnt];
          for (int j= 0; j< joint_cnt; j++){
            motors_pos[j] = 180 * (goal_pos_raw[j] + 1);
            dxl.setGoalPosition(DXL_ID[j], (int (motors_pos[j]*100))/100, UNIT_DEGREE);
          }
 }

     // 10.5  Movin motors position mode.........................
         void mov_pos(float goal_pos_raw[], uint8_t joint_cnt){
          for (int j= 0; j< joint_cnt; j++){
            dxl.setGoalPosition(DXL_ID[j], goal_pos_raw[j], UNIT_DEGREE);
          }
 }
   




// geting motor joint states 
float* dxl_motor_states(int* DXL_ID, uint8_t joint_cnt)  {

     float motor_position[joint_cnt];
     float motor_velocity[joint_cnt];
     float motor_current[joint_cnt];
     for (int i = 0; i < joint_cnt; i++){
         motor_position[i] = dxl.getPresentPosition(DXL_ID[i], UNIT_DEGREE);
                  motor_velocity[i] = dxl.getPresentVelocity(DXL_ID[i], UNIT_RPM);
                           motor_current[i] = dxl.getPresentCurrent(DXL_ID[i], UNIT_MILLI_AMPERE);
                           
    } 
    return motor_position, motor_velocity,  motor_current ;
}

// real joint states publisher's parameter updates
void real_joint_states_update(int* DXL_ID, uint8_t joint_cnt){
      float real_joint_position[joint_cnt];
      float real_joint_velocity[joint_cnt];
      float real_joint_effort[joint_cnt];
      
      float* motor_states;    
      motor_states = dxl_motor_states(DXL_ID, joint_cnt);

real_joint_state.position = motor_states;
real_joint_state.velocity = motor_states;
real_joint_state.effort = motor_states;
}

// motor real joint states publisher's update with combined
void dxl_joint_states_update(int* DXL_ID, uint8_t joint_cnt ){

     float motor_position[joint_cnt];
     float motor_velocity[joint_cnt];
     float motor_current[joint_cnt];
     for (int i = 0; i < joint_cnt; i++){
         motor_position[i] = dxl.getPresentPosition(DXL_ID[i], UNIT_DEGREE);
                  motor_velocity[i] = dxl.getPresentVelocity(DXL_ID[i], UNIT_RPM);
                           motor_current[i] = dxl.getPresentCurrent(DXL_ID[i], UNIT_MILLI_AMPERE);                
    } 
real_joint_state.position = motor_position;
real_joint_state.velocity = motor_velocity;
real_joint_state.effort = motor_current;

}

/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(uint8_t joint_cnt, char* joint_state_header_frame_id)
{
  static char *joint_states_name[] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

  real_joint_state.header.frame_id = joint_state_header_frame_id;
  real_joint_state.name            = joint_states_name;

  real_joint_state.name_length     = joint_cnt;
  real_joint_state.position_length = joint_cnt;
  real_joint_state.velocity_length = joint_cnt;
  real_joint_state.effort_length   = joint_cnt;
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;

  if (isConnected)
  {
    if (wait_flag == false)
    {
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

bool* is_conected_motors(int* DXL_ID, uint8_t joint_cnt){
  bool is_conected_motor[joint_cnt];
  for(int i=0; i< joint_cnt; i++){
    is_conected_motor[i] = dxl.ping(DXL_ID[i]);
   }
   return is_conected_motor;
}

uint16_t* joints_model_number(int* DXL_ID, uint8_t joint_cnt){
  uint16_t model_number[joint_cnt];
  for(int i=0; i< joint_cnt; i++){
    model_number[i] = dxl.getModelNumber(DXL_ID[i]);
   }
   return model_number;
}



void switch_operating_modes(int* DXL_ID, uint8_t joint_cnt, uint8_t operating_mode){
 
 for(int i=0; i< joint_cnt; i++){
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], operating_mode);     //This function changes operating mode of DYNAMIXEL.
    dxl.torqueOn(DXL_ID[i]);
   }

}
 
