/*
 * This file is part of robot_firmware
 * Copyright (C) 2018 Andrew Miyaguchi <andrewmiyaguchi@gmail.com>
 * 
 * Basic arduino driver for the AQMH2407ND motor controller board.
 * Note that this motor controller only supports two motors for
 * both a forward and reverse configuration.
 * 
 * When an input is pulled low, the output is also pulled to ground.
 * 
 * |--Motor1
 * |  |- in1 --> Motor1 Leg 1
 * |  |- in2 --> Motor1 Leg 2
 * |
 * |--Motor2
 * |  |- in3 --> Motor 2 Leg 1
 * |  |- in4 --> Motor 2 Leg 2
 */

#ifndef AQMH2407ND_H
#define AQMH2407ND_H

#define SIDE_A 0  // Left
#define SIDE_B 1  // Right

#include <Arduino.h>

class AQMH2407ND {
  public:
    AQMH2407ND(int enA, int in1, int in2);
    AQMH2407ND(int enA, int enB, int in1, int in2, int in3, int in4);
    void setSpeed(int speed);
    void setSpeed(int side, int speed);
    void setMaxDuty(int duty);
    void setReversed(boolean a);
    void setReversed(boolean a, boolean b);
    void enable();
    void disable();

  private:
    int max_duty_cycle = 255;
    boolean mirrorMode = true;
    boolean reversed_a;
    boolean reversed_b;
    int pin_enA;
    int pin_enB;
    int pin_in[4];
};

#endif /* AQMH2407ND_H */
