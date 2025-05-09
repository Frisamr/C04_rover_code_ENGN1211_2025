/****************** IMPORTS ******************************/
// import standard arduino functions, better int types, and servo library
#include <Arduino.h>
#include <Servo.h>

// import logging library
// this is a customised version of https://github.com/JSC-TechMinds/Arduino-Log
#include "src/ArduinoLog.h"

// the globals and constants of this program
#include "src/globals.h"
using namespace constants;
using namespace globals;

// components control
#include "src/components.h"

// rover movments and maze solving
#include "src/rover_op.h"

// testing routines and assessment level demos
#include "src/testing.h"

/****************** CONFIG ******************************/
// these are global variables declared in `global.h`

// approximately correct
unsigned long globals::MICROS_PER_cm = 182000;

// approximately correct
unsigned long globals::MICROS_PER_DEGREE = 19000;

MotorSettings globals::MOTOR_CONFIG = {
    255, //leftMotorDrive;
    255, //rightMotorDrive;
    255, //leftMotorTurn;
    255, //rightMotorTurn;
};

// set so the rover will drive approx 0.5cm
unsigned long globals::SHORT_STEP_TIME = globals::MICROS_PER_cm / 2;

// set so the rover will drive 3cm
unsigned long globals::LONG_STEP_TIME = globals::MICROS_PER_cm * 3;

/****************** OTHER GLOBALS ******************************/
// These will be overwritten as the program runs. Don't change them.

bool globals::RUN_START = true;

// global sonar system object
Servo globals::THE_SERVO = Servo();

// dummy servo angle value (this will be overridden)
int globals::SERVO_ANGLE = 90;

/****************** SETUP ******************************/

// forward declare this function so we can use it in setup()
// full function definition at the end of this file
void printPrefix(Print* _logOutput, int logLevel);

void setup() {
    // open the serial port
    Serial.begin(9600);

    // setup logging
    ALog.setPrefix(printPrefix);                       // set custom prefix that shows log level
    ALog.begin(LOG_LEVEL_INFO, &Serial, false, false); // logging settings: level, output, show level, show colour

    // initialise the motors and sonar module (set the control pins to output mode)
    initMotor(constants::LEFT_MOTOR);
    initMotor(constants::RIGHT_MOTOR);
    initSonarSystem();
}

/****************** MAIN LOOP ******************************/

// Comment out the routines you don't want to use in the loop function.
void loop() {
    /* test level demos */

    //demoLevel_1_part2();

    /* testing routines */

    //testSonarReliability();
    testSonarSweep();
    //testServoAngle();
    //testMovement(RoverMove::driveForward, 10 * globals::MICROS_PER_cm);
    //testMovement(RoverMove::turnLeft, 90 * globals::MICROS_PER_DEGREE);
}

/****************** CUSTOM LOGGING PREFIX ******************************/

void printPrefix(Print* _logOutput, int logLevel) {
    // Show log description based on log level
    switch (logLevel) {
    default:
    case LOG_LEVEL_SILENT:
        _logOutput->print(" [SILENT] ");
        break;
    case LOG_LEVEL_FATAL:
        _logOutput->print("  [FATAL] ");
        break;
    case LOG_LEVEL_ERROR:
        _logOutput->print("  [ERROR] ");
        break;
    case LOG_LEVEL_WARNING:
        _logOutput->print("[WARNING] ");
        break;
    case LOG_LEVEL_INFO:
        _logOutput->print("   [INFO] ");
        break;
    case LOG_LEVEL_TRACE:
        _logOutput->print("  [TRACE] ");
        break;
    case LOG_LEVEL_VERBOSE:
        _logOutput->print("[VERBOSE] ");
        break;
    }
}
