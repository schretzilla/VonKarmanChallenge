/*
 * robot_firmware - Brains of our Von Karman Challenge (VKC) Robot
 * Copyright (C) 2018 Andrew Miyaguchi <andrewmiyaguchi@gmail.com>
 * 
 * Recieving communications based off ROS Simple Drive system
 * <https://github.com/danielsnider/simple_drive>
 */

#include "Utilities.h"

AQMH2407ND *frontDriver;
AQMH2407ND *rearDriver;

void setup() {
  /* Setup serial connections */
  Serial.begin(115200);
  Serial.println(F("Welcome to the VKC robot firmware"));
  Serial.println(F("INIT: Firmware booting..."));

  /* Setup the tank drive */
  frontDriver = AQMH2407ND(
  
}

void loop() {
  
}
