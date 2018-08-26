/*
 * This file is part of robot_firmware
 * Copyright (C) 2018 Andrew Miyaguchi <andrewmiyaguchi@gmail.com>
 * 
 * Tank drive logic using two AQMH2407ND motor controllers
 * Speed is an 16 bit value (-255 to 255)
 * setSpeed: move the robot forward
 * setRotation: rotate the robot
 * 
 * TODO:
 * setAngle: offset the robot by n degrees
 */

#ifndef TANKDRIVE_H
#define TANKDRIVE_H

#define TIMEOUT_CONTROLS 10000

#include <Arduino.h>
#include "AQMH2407ND.h"

class TankDrive {
  public:
    TankDrive(AQMH2407ND *leftController, AQMH2407ND *rightController);
    void setSpeed(int speed);
    void setRotation(int speed);
    void setAngle(int deg);
    void run();
    
  private:
    AQMH2407ND *lc;  // left driver
    AQMH2407ND *rc;  // right driver
    int speed = 0;
    int angle = 0;
};

#endif /* TANKDRIVE_H */
