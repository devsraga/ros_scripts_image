/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo;
Servo servo_2;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180 
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void servo_cb_2( const std_msgs::UInt16& cmd_msg_2){
  servo_2.write(cmd_msg_2.data); //set servo angle, should be from 0-180   
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);
ros::Subscriber<std_msgs::UInt16> sub_2("servo_2", servo_cb_2);

void setup(){
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  pinMode(13, OUTPUT);


  nh.initNode();
  nh.subscribe(sub);
    nh.initNode();
  nh.subscribe(sub_2);
  
  servo_2.attach(9); //attach it to pin 9
  servo.attach(5); //attach it to pin 9
//  digitalWrite(4,HIGH);
}

void loop(){
  
  nh.spinOnce();
  delay(1);
}
