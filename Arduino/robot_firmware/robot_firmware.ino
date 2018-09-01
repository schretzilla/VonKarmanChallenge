/*
 * robot_firmware - Brains of our Von Karman Challenge (VKC) Robot
 * Copyright (C) 2018 Andrew Miyaguchi <andrewmiyaguchi@gmail.com>
 * 
 * Recieving communications based off ROS Simple Drive system
 * <https://github.com/danielsnider/simple_drive>
 * 
 * Pins 5 and 6 were ignored even though they support PWM as their
 * duty cycle are affected by timer0, which is used for delay and mills();
 * 
 * |--Left Side
 * |  |-Pin 7 (!PWM) -> Enable
 * |  |-Pin 10 (PWM) -> Pos
 * |  |-Pin 11 (PWM) -> Neg
 * |
 * |--Right Side
 * |  |-Pin 8 (!PWM) -> Enable
 * |  |-Pin 3  (PWM) -> Pos
 * |  |-Pin 9  (PWM) -> Neg
 * 
 * index  type
 * 0    Left/Right Axis stick left
 * 1    Up/Down Axis stick left
 * 2    LT
 * 3    Left/Right Axis stick right
 * 4    Up/Down Axis stick right
 * 5    RT
 * 6    cross key left/right
 * 7    cross key up/down
 * 
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

#include "AQMH2407ND.h"
#include "Utilities.h"

AQMH2407ND *leftDriver;
AQMH2407ND *rightDriver;

long ros_watchdog;
void readMessageCallback(const sensor_msgs::Joy& joy);

ros::NodeHandle nh;
ros::Subscriber<sensor_msgs::Joy> sub("joy", &readMessageCallback);


void setup() {
  /* Setup serial connections */
  Serial.begin(115200);
  Serial.println(F("Welcome to the VKC robot firmware"));
  Serial.println(F("INIT: Firmware booting..."));
  
  /* Setup the tank drive */
  leftDriver = new AQMH2407ND(7, 10, 11);
  rightDriver = new AQMH2407ND(8, 3, 9);
  
  leftDriver->setReversed(true);
  rightDriver->setReversed(true);

  leftDriver->enable();
  rightDriver->enable();

  /* Setup ROS Node */
  nh.initNode();
  nh.subscribe(sub);
  ros_watchdog = millis();
}

void loop() {
  /* avoid using delay by checking delta time */
  if(millis() - ros_watchdog > 1000) {
    nh.spinOnce();
  }
}

void readMessageCallback(const sensor_msgs::Joy& joy) {
  int leftJoyStick = fmap(joy.axes[1], -1, 1, -255, 255);
  int rightJoyStick = fmap(joy.axes[4], -1, 1, -255, 255);
  
  leftDriver->setSpeed(leftJoyStick);
  rightDriver->setSpeed(rightJoyStick);
}
