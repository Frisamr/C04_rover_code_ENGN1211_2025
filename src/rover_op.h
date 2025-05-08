// this is an [include guard](https://en.wikipedia.org/wiki/Include_guard)
#ifndef ROVER_OP_H
#define ROVER_OP_H

#include <inttypes.h>
#include <Servo.h>


/****************** OTHER ******************************/

enum struct RoverAction {
  turnLeft,
  turnRight,
  longStepForward,
  shortStepFoward,
  sweepScan,
};

void driveRover(int LEFT_MOTORSpeed, int RIGHT_MOTORSpeed, unsigned long time);


#endif // ROVER_OP_H