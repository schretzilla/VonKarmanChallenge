#include "AQMH2407ND.h"

AQMH2407ND::AQMH2407ND(int enA, int enB, int in1, int in2, int in3, int in4) {
  pin_enA = enA;
  pin_enB = enB;
  pin_in[0] = in1;
  pin_in[1] = in2;
  pin_in[2] = in3;
  pin_in[3] = in4;

  /* Setup the required pins for the motor controllers */
  pinMode(pin_enA, OUTPUT);
  pinMode(pin_enB, OUTPUT);

  for(int motor:pin_in)
    pinMode(motor, OUTPUT);

  /* Disable the controller just in case */
  disable();
}

/* set the speed of a motor */
void AQMH2407ND::setSpeed(int side, int speed) {
  int pos;
  int neg;
  
  /* Clear all previous set speeds */
  for(int motorNum:pin_in)
    analogWrite(motorNum, 0);

  /* Early return if speed is zero */
  if(speed == 0)
    return;

  /* Get the side to be set */
  switch(side) {
    case SIDE_A:
      pos = pin_in[0];
      neg = pin_in[1];
      break;
    case SIDE_B:
      pos = pin_in[2];
      neg = pin_in[3];
      break;
    default:
      /* Oops, something bad happened */
      return;
  }

  /* Add overduty protections */
  speed = min(max(speed, -255), 255);
  int mappedSpeed = map(speed, -255, 255, -max_duty_cycle, max_duty_cycle);

  /* Set the pwm duty cycle appropriate for forward or reverse */  
  if(speed > 0) {
    analogWrite(pos, mappedSpeed);
  } else if(speed < 0) {
    analogWrite(neg, mappedSpeed);
  }
}

/* 
 *  Limit the controller to a max duty cycle to allow
 *  lower voltage motors to run on higher voltage controllers
 */
void AQMH2407ND::setMaxDuty(int duty) {
  duty = min(max(duty, 0), 255);
  max_duty_cycle = duty;
}

/* Enable the controller */
void AQMH2407ND::enable() {
  digitalWrite(pin_enA, HIGH);
  digitalWrite(pin_enB, HIGH);
}


/* Disable the controller */
void AQMH2407ND::disable() {
  digitalWrite(pin_enA, LOW);
  digitalWrite(pin_enB, LOW);

  /* Clear the set speed */
  for(int motor:pin_in) {
    digitalWrite(motor, LOW);
    analogWrite(motor, 0);
  }
}

