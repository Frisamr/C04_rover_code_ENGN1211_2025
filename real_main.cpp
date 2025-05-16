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

/****************** GLOBALS ******************************/
// how long (in ms) it takes the rover to drive 1cm forward/backward
uint16_t globals::MILLIS_PER_CM = 193;

// how long (in ms) it takes the rover to turn 1 degree
unsigned long globals::MILLIS_45_DEG = 863;

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
    ALog.setPrefix(printPrefix);                        // set custom prefix that shows log level
    ALog.begin(LOG_LEVEL_TRACE, &Serial, false, false); // logging settings: level, output, show level, show colour

    // initialise the motors, sonar module, and servo
    initMotor(constants::LEFT_MOTOR);
    initMotor(constants::RIGHT_MOTOR);
    initSonarMod();
    initServo();
}

/****************** MAIN LOOP ******************************/

// Comment out the routines you don't want to use in the `loop()` function.
void loop() {
    /* test level demos */

    //demo_level_1_part2();
    //demo_level_2();
    demo_level_345();

    /* testing routines */

    //testAngledSonar();
    //testSonarReliability();
    //testServoAngle();
    //testMovement(RvrMoveKind::driveFwd, timeToDriveDist(0.5), 1);
    //testMovement(RvrMoveKind::turnRight45, 999, 2);
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
