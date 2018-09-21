/*
 * This file is part of robot_firmware
 * Copyright (C) 2018 Andrew Miyaguchi <andrewmiyaguchi@gmail.com>
 * 
 * Basic arduino driver for electromagnet.
 * 
 * The electromagnet uses about 220mA, so make sure to hook it up
 * to a MOSFET or power transistor!
 */

#ifndef ELECTROMAGNET_H
#define ELECTROMAGNET_H

#include <Arduino.h>
#include "Utilities.h"

class Electromagnet {
  public:
    Electromagnet(int control_pin);
    void setState(boolean state);
    void toggle();

  private:
    int c_pin;
    int c_state = false;
    
};

#endif /* ELECTROMAGNET_H */
