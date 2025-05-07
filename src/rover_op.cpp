// import standard arduino functions, better int types, servo library
#include <Arduino.h>
#include <inttypes.h>
#include <Servo.h>


// import logging library
#include "ArduinoLog.h"

// the globals and constants of this program
#include "globals.h"
// custom motor control code
#include "motor_ctl.h"
// custom sonar system code
#include "sonar_system.h"



/****************** DATA TYPES ******************************/

enum struct RoverAction {
  turnLeft,
  turnRight,
  longStepForward,
  shortStepFoward,
  sweepScan,
};


/****************** MAZE-SOLVING HELPER FUNCTIONS ******************************/

void driveRover(int leftMotorSpeed, int rightMotorSpeed, unsigned long time) {
  ALog.traceln("`driveRover` called with leftMotorSpeed %d, rightMotorSpeed %d, and time %u", leftMotorSpeed, rightMotorSpeed, time);

  // spin both motors forwards
  setMotorSpeed(constants::LEFT_MOTOR, leftMotorSpeed);
  setMotorSpeed(constants::RIGHT_MOTOR, rightMotorSpeed);

  // drive forward for provided time
  delay(time);

  // stop
  setMotorSpeed(constants::LEFT_MOTOR, 0);
  setMotorSpeed(constants::RIGHT_MOTOR, 0);
}

