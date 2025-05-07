// this is an [include guard](https://en.wikipedia.org/wiki/Include_guard)
#ifndef ROVER_OP_H
#define ROVER_OP_H

#include <inttypes.h>
#include <Servo.h>

#include "globals.h"
#include "motor_ctl.h"
#include "sonar_system.h"


/****************** OTHER ******************************/

enum struct RoverAction {
  turnLeft,
  turnRight,
  longStepForward,
  shortStepFoward,
  sweepScan,
};

void driveRover(int leftMotorSpeed, int rightMotorSpeed, unsigned long time);


#endif