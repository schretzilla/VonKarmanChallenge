#include "TankDrive.h"

TankDrive::TankDrive(AQMH2407ND *leftController, AQMH2407ND *rightController) {
  lc = leftController;
  rc = rightController;

  lc->enable();
  rc->enable();
}

void TankDrive::setSpeed(int speed) {
  lc->setSpeed(speed);
  rc->setSpeed(speed);
}

