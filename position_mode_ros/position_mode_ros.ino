#include "dxl_config.h"

//################################################################################################
// 8. Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Setup Se
//################################################################################################

void setup() {

//ROS Setups ................................................................
    // Initialize ROS node handle, advertise and subscribe the topics

  nh.initNode();
  nh.getHardware()->setBaud(115200);
  
  nh.subscribe(sub);
   nh.subscribe(joint_pos_subscriber);

  nh.advertise(real_joint_state_publisher);

  
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
      is_conected_motors(DXL_ID, joint_cnt);
      joints_model_number(DXL_ID, joint_cnt);
      
  // Set DYNAMIXEL operating mode
      switch_operating_modes(DXL_ID, joint_cnt, OP_POSITION);
//      /dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[0], V_max);

  prev_update_time = millis();
  initJointStates(joint_cnt, joint_state_header_frame_id);
}
  

//################################################################################################
// 9. LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP  LOOP L
//################################################################################################

void loop(){ 

  uint32_t t = millis();
  updateTime();
//  updateVariable(nh.connected());
//  updateTFPrefix(nh.connected());

  if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
//    publishSensorStateMsg();
//    publishBatteryStateMsg();
    publishDriveInformation();
    tTime[2] = t;
  }

// 0.  Spinning the ROS cominication and call back functions.................................
   nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());

}
