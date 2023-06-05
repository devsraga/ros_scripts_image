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
#include <sensor_msgs/Joy.h>

ros::NodeHandle  nh;

Servo servo;
Servo servo_2;


void servo_cb( const sensor_msgs::Joy& cmd_msg){
   float   pitch = cmd_msg.axes[1];
   float yaw = cmd_msg.axes[4];
      float  s = 90*(pitch+1);
  float  s2 = 90*(yaw+1);
  servo.write(s); //set servo angle, should be from 0-180 
  servo_2.write(s2); //set servo angle, should be from 0-180 
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}



ros::Subscriber<sensor_msgs::Joy> sub("/joy", servo_cb);


void setup(){
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  pinMode(13, OUTPUT);


  nh.initNode();
  nh.subscribe(sub);
  
  servo_2.attach(9); //attach it to pin 9
  servo.attach(5); //attach it to pin 9
//  digitalWrite(4,HIGH);
}

void loop(){
  
  nh.spinOnce();
  delay(1);
}
