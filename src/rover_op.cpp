// import standard arduino functions, better int types, servo library
#include <Arduino.h>
#include <Servo.h>
#include <inttypes.h>

// import logging library
#include "ArduinoLog.h"

// the globals and constants of this program
#include "globals.h"
using namespace constants;
using namespace globals;

// components control functions
#include "components.h"

// import the data type and function declarations
#include "rover_op.h"

/****************** MAZE-SOLVING HELPER FUNCTIONS ******************************/

void driveRover(int leftMotorSpeed, int rightMotorSpeed, unsigned long time) {
    ALog.traceln(
        "`driveRover` called with leftMotorSpeed %d, rightMotorSpeed %d, and time %u", leftMotorSpeed, rightMotorSpeed,
        time
    );

    // spin both motors forwards
    setMotorSpeed(constants::LEFT_MOTOR, leftMotorSpeed);
    setMotorSpeed(constants::RIGHT_MOTOR, rightMotorSpeed);

    // drive forward for provided time
    delay(time);

    // stop
    setMotorSpeed(constants::LEFT_MOTOR, 0);
    setMotorSpeed(constants::RIGHT_MOTOR, 0);
}
