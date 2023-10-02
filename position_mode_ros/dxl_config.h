//################################################################################################
// 1. ROS include  +  message include +  node handeller + Dynamixel2Arduino include
//################################################################################################
#ifndef dxl_config_H_
#define dxl_config_H_

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
#include <math.h>
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
      uint8_t joint_cnt = sizeof(DXL_ID)/sizeof(DXL_ID[0]);
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


 char joint_state_header_frame_id[30];
 unsigned long prev_update_time;
 static uint32_t tTime[10];


 #define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 
#define JOINT_CONTROL_FREQEUNCY                100  //hz 


//################################################################################################
// 4. ROS NodeHandle + ROS NodeHandle + ROS NodeHandle + ROS NodeHandle +ROS NodeHandle + ROS Node
//################################################################################################
ros::NodeHandle  nh;
ros::Time current_time;
uint32_t current_offset;

//################################################################################################
// 4. Function prototypes + Function prototypes + Function prototypes + Function prototypes + Func
//################################################################################################
// Callback function prototypes
void servo_callback_joystick_cammonds( const sensor_msgs::Joy& joy_msg);
void servo_callback_supplied_js_cammonds( const sensor_msgs::JointState& sup_js_msg);

// Function prototypes
void mov(int V_max, float goal_pos[], uint8_t joint_cnt = joint_cnt);
void mov_1(float* goal_pos, uint8_t joint_cnt = joint_cnt);
void mov_pos(float goal_pos_raw[], uint8_t joint_cnt = joint_cnt);
void mov_pos_by_joy(float goal_pos_raw[], uint8_t joint_cnt = joint_cnt);
void mov_vel(float goal_vel_raw[], uint8_t joint_cnt = joint_cnt);
float* dxl_motor_states(int* DXL_ID, uint8_t joint_cnt = joint_cnt);
void real_joint_states_update(int* DXL_ID, uint8_t joint_cnt= joint_cnt);
void dxl_joint_states_update(int* DXL_ID, uint8_t joint_cnt = joint_cnt);
void initJointStates(uint8_t joint_cnt = joint_cnt, char* header_frame_id = "");
bool* is_conected_motors(int* DXL_ID, uint8_t joint_cnt);
uint16_t* joints_model_number(int* DXL_ID, uint8_t joint_cnt);
void switch_operating_modes(int* DXL_ID, uint8_t joint_cnt, uint8_t operating_mode);
void updateTime();
ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated
void waitForSerialLink(bool isConnected);

// publisher function prototypes
void publishDriveInformation(void);

// Required function prototypes


//################################################################################################
// 5. Objects Objects  Objects Objects Objects Objects Objects Objects Objects Objects Objects Obj
//################################################################################################

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
//This namespace is required to use Control table item names
  using namespace ControlTableItem;
  


//################################################################################################
// 6. ROS Subscriber Node + ROS Subscriber + ROS Subscriber + ROS Subscriber + ROS Subscriber + RO
//################################################################################################

ros::Subscriber<sensor_msgs::Joy> sub("/joy", servo_callback_joystick_cammonds);
ros::Subscriber<sensor_msgs::JointState> joint_pos_subscriber("/supplied_joint_state", servo_callback_supplied_js_cammonds);
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
ros::Publisher real_joint_state_publisher("real_joint_state", &real_joint_state);





#endif // dxl_config_H_
