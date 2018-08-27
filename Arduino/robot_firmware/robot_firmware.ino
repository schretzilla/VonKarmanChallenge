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

struct DATA {
    float linear;
    float rotation;
    float servo;
} received;

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
  
  drive = new TankDrive(leftDriver, rightDriver);
  dmesg("Finished setting up drivers");
  drive->setSpeed(0);
}

void loop() {
   if (Serial.available() >= sizeof(uint8_t)) {
       delayMicroseconds(10);
       uint8_t cmd = (uint8_t) Serial.read();
       Serial.print("GOT a cmd");
       Serial.println(cmd);
       // TWIST MOTOR COMMAND
       if (cmd == 0) {
           Serial.readBytes((char *) &received.linear, sizeof(float));
           Serial.readBytes((char *) &received.rotation, sizeof(float));
           Serial.print("GOT TWIST");
           Serial.println((int) received.linear);
           Serial.println((int) received.rotation);
           //setWheelVelocity((int) ((received.linear + received.rotation) * 100), (int) ((received.linear - received.rotation) * 100));
       }
       // SERVO MOTOR COMMAND
       else if (cmd == 2) {
           Serial.readBytes((char *) &received.servo, sizeof(float));
           Serial.print("GOT SERVO");
           Serial.println((int) received.servo);
           //theServo.write((int) received.servo);
       }
   }
}
