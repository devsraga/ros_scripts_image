 /*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

//################################################################################################
// 1. ROS include  +  message include +  node handeller + Dynamixel2Arduino include
//################################################################################################

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/MultiDOFJointState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <std_msgs/Time.h>

ros::NodeHandle  nh;

#include <Dynamixel2Arduino.h>


//################################################################################################
// 2. Please modify it to suit your hardware (Microcontroller)
//################################################################################################

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

//################################################################################################
// 3. Variables Variables Variables Variables Variables Variables Variables Variables Variables Va
//################################################################################################

   // 2.1  id assignments.........................
   
      int DXL_ID[]={1,2,3,4,5,6};
      int V_max = 0;
      
      
    // 2.2 LED + Button pin assignments...........
    
      int warning_pin_1 = 18; // red led
      int warning_pin_2 = 19; // green led
      int warning_pin_3 = 20; // blue led
      int button_pin_1 = 16;
    
   // 2.3  Various SP poses........................
   
      float goal_pos_0[6]={120,240,120,240,120,240};
      float goal_pos_1[6]={60,300,60,300,60,300};
      float goal_pos_2[6]={180,180,180,180,180,180};
      float goal_pos_3[6]={10,310,10,300,60,300};
      float goal_pos_4[6]={90,270,90,270,90,270};
      float motors_positions[] = {0, 0, 0, 0, 0, 0};
      float present_position[] = {0, 0, 0, 0, 0, 0};
      float real_joint_position[] = {0, 0, 0, 0, 0, 0};
      float real_joint_velocity[] = {0, 0, 0, 0, 0, 0};
      float real_joint_effort[] = {0, 0, 0, 0, 0, 0};
      char Joint_1[8] = "Joint_1";
      char Joint_2[8] = "Joint_2";
      char Joint_3[8] = "Joint_3";
      char Joint_4[8] = "Joint_4";
      char Joint_5[8] = "Joint_5";
      char Joint_6[8] = "Joint_6";
      String joint_names[] = {Joint_1, Joint_2, Joint_3, Joint_4, Joint_5, Joint_6};

      int n = 6;
      
   // 2.4 protocal..............

      const float DXL_PROTOCOL_VERSION = 2.0;

   // 2.5 actuator variables....
 

//################################################################################################
// 4. Objects Objects  Objects Objects Objects Objects Objects Objects Objects Objects Objects Obj
//################################################################################################

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
//This namespace is required to use Control table item names
  using namespace ControlTableItem;
  
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
      
        for (int i=0; i < 6; i++){
          motors_positions[i] = 180*(motors_positions_raw[i] +1);
        }


      mov_1(motors_positions);
//
//    dxl.setGoalPosition(DXL_ID[1], motors_positions[1], UNIT_DEGREE);
//      dxl.setGoalPosition(DXL_ID[2], motors_positions[3], UNIT_DEGREE);
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



//################################################################################################
// 6. ROS Subscriber Node + ROS Subscriber + ROS Subscriber + ROS Subscriber + ROS Subscriber + RO
//################################################################################################

ros::Subscriber<sensor_msgs::Joy> sub("/joy", servo_callback_joystick_cammonds);
//ros::Subscriber<sensor_msgs::JointState> sub("/JointState", servo_callback_joint_states);
//ros::Subscriber<sensor_msgs::MultiDOFJointState> sub("/MultiDOFJointState", servo_callback_joint_states);
//ros::Subscriber<sensor_msgs::Imu> sub("/Imu", Board_callback_joystick_cammonds);
//ros::Subscriber<sensor_msgs::Temperature> sub("/joy", servo_callback_joystick_cammonds);



//################################################################################################
// 7. ROS Publisher Node + ROS Publisher  + ROS Publisher  + ROS Publisher  + ROS Publisher  + ROS
//################################################################################################

//ros::Subscriber<geometry_msgs::AccelWithCovarianceStamped> sub("/joy", servo_callback_joystick_cammonds);
//ros::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub("/joy", servo_callback_joystick_cammonds);
//ros::Subscriber<geometry_msgs::TwistWithCovarianceStamped> sub("/joy", servo_callback_joystick_cammonds);
//ros::Subscriber<geometry_msgs::WrenchStamped> sub("/joy", servo_callback_joystick_cammonds);

sensor_msgs::JointState real_joint_state;
ros::Publisher real_joint_state__publisher("real_joint_state", &real_joint_state);







//################################################################################################
// 8. Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Se
//################################################################################################

void setup() {

//ROS Setups ................................................................
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  
//Dynamixel Setup ............................................................
  // put your setup code here, to run once:
 pinMode(warning_pin_1,OUTPUT); // 18 to 20 led in epansion board, 
 pinMode(warning_pin_2,OUTPUT); // 18 to 20 led in epansion board, 
 pinMode(warning_pin_3,OUTPUT); // 18 to 20 led in epansion board, 
 pinMode(button_pin_1,OUTPUT); // 16 to 17 button pin in epansion board, 


 
// Use UART port of DYNAMIXEL Shield to debug.
//  Serial.begin(115200);
     DEBUG_SERIAL.begin(115200);


  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information

  for(int i=0; i<6; i++){

     bool is_conected_motor = dxl.ping(DXL_ID[0]);
  }
uint16_t model_number[] = {0, 0, 0, 0, 0, 0};
for(int i=1; i<=6; i++){
    model_number[i] = dxl.getModelNumber(i);
   }






  for(int i=0; i<n; i++){
  // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_POSITION);     //This function changes operating mode of DYNAMIXEL.
    dxl.torqueOn(DXL_ID[i]);
 
    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], V_max);
  }

}
  

//################################################################################################
// 9. LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP  LOOP L
//################################################################################################

void loop(){ 

sensor_msgs::JointState real_joint_state;
ros::Publisher real_joint_state__publisher("real_joint_state", &real_joint_state);
    
    
    for (int i = 0; i<6; i++){
         real_joint_position[i] = dxl.getPresentPosition(DXL_ID[i+1], UNIT_DEGREE);
    }
    for (int i = 0; i<6; i++){
         real_joint_velocity[i] = dxl.getPresentVelocity(DXL_ID[i+1], UNIT_RPM);
    }
    for (int i = 0; i<6; i++){
         real_joint_effort[i] = dxl.getPresentCurrent(DXL_ID[i+1], UNIT_MILLI_AMPERE);
    }
//    joint_names = {Joint_1, Joint_/2, Joint_3, Joint_4, Joint_5, Joint_6}
//  real_joint_state.name = joint_names;
  real_joint_state.position = real_joint_position;
  real_joint_state.velocity = real_joint_velocity;
  real_joint_state.effort = real_joint_effort;
  
  real_joint_state__publisher.publish( &real_joint_state );

// 0.  Spinning the ROS cominication and call back functions.................................
   nh.spinOnce();
   delay(1); 

}

//################################################################################################
// 10. Methods Methods Methods Methods Methods Methods Methods Methods Methods Methods Methods Met
//################################################################################################

   // 10.1  Movin the stewart platform.........................
      void mov(int V_max, float goal_pos[]){
         
        int flag = 0;
        for(int i =0; i<n; i++){
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

    for(int i=0; i<n; i++){
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
         void mov_1(float* goal_pos){



//      for (int k = 0; k<6; k++){
//        dxl.setGoalVelocity(DXL_ID, goal_pos[0], UNIT_RPM);
//      }
float present_position[] = {0, 0, 0, 0, 0, 0}; 

    for (int i = 0; i<6; i++){
         present_position[i] = dxl.getPresentPosition(DXL_ID[i+1], UNIT_DEGREE);
    }
             
  for (int j= 0; j<6; j++){
    dxl.setGoalPosition(DXL_ID[j+1], goal_pos[j], UNIT_DEGREE);
  }
  
  while (abs(goal_pos[0] - present_position[0]) > 2.0 && abs(goal_pos[1] - present_position[1])> 2.0 && abs(goal_pos[2] - present_position[2])> 2.0 && abs(goal_pos[3] - present_position[3])> 2.0 && abs(goal_pos[4] - present_position[4])> 2.0 && abs(goal_pos[5] - present_position[5])> 2.0)
  {

    for (int i = 0; i<6; i++){
         present_position[i] = dxl.getPresentPosition(DXL_ID[i+1], UNIT_DEGREE);
    }


  }
         }
 
