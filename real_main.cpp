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
unsigned long globals::MILLIS_PER_CM = 200;

// approximately correct
unsigned long globals::MILLIS_PER_DEGREE = 21;

// set so the rover will drive approx 0.5cm
unsigned long globals::SHORT_STEP_TIME = globals::MILLIS_PER_CM / 2;

// set so the rover will drive 3cm
unsigned long globals::LONG_STEP_TIME = globals::MILLIS_PER_CM * 3;

/****************** OTHER GLOBALS ******************************/

bool globals::RUN_START = true;

// global sonar system object
Servo globals::THE_SERVO = Servo();

/****************** SETUP ******************************/

// forward declaration
extern void printPrefix(Print *_logOutput, int logLevel);

void setup() {
    // open the serial port
    Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }

    // setup logging
    ALog.setPrefix(printPrefix);                       // set custom prefix
    ALog.begin(LOG_LEVEL_TRACE, &Serial, true, false); // logging settings: level, output, show level, show colour

    // initialise the motors and sonar module (set the control pins to output mode)
    initMotor(constants::LEFT_MOTOR);
    initMotor(constants::RIGHT_MOTOR);
    initSonarSystem();
}

/****************** MAIN LOOP ******************************/

// Comment out the routines you don't want to use in the loop function.
void loop() {
    /* test level demos */

    // demoLevel_1_part2();
    // demoLevel_1_part1();

    /* testing routines */

    testCollisionAvoidance();
    // testSonarReliability();
    // testConstantMotion(255, 255, 10 * globals::MILLIS_PER_CM);
}

/****************** CUSTOM LOGGING PREFIX ******************************/

void printPrefix(Print *_logOutput, int logLevel) {
    // Show log description based on log level
    switch (logLevel) {
    default:
    case 0:
        _logOutput->print("[SILENT]  ");
        break;
    case 1:
        _logOutput->print("[FATAL]   ");
        break;
    case 2:
        _logOutput->print("[ERROR]   ");
        break;
    case 3:
        _logOutput->print("[WARNING] ");
        break;
    case 4:
        _logOutput->print("[INFO]    ");
        break;
    case 5:
        _logOutput->print("[TRACE]   ");
        break;
    case 6:
        _logOutput->print("[VERBOSE] ");
        break;
    }
}
