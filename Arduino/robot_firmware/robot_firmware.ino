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
 */

#include "AQMH2407ND.h"
#include "TankDrive.h"
#include "Utilities.h"

AQMH2407ND *leftDriver;
AQMH2407ND *rightDriver;
TankDrive *drive;

void setup() {
  /* Setup serial connections */
  Serial.begin(115200);
  Serial.println(F("Welcome to the VKC robot firmware"));
  Serial.println(F("INIT: Firmware booting..."));

  /* Setup the tank drive */
  leftDriver = new AQMH2407ND(7, 10, 11);
  rightDriver = new AQMH2407ND(8, 3, 9);

  drive = new TankDrive(leftDriver, rightDriver);
  dmesg("Finished setting up drivers");
  drive->setSpeed(0);
}

void loop() {
  
}
