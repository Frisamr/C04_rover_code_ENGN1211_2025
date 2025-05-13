/****************** IMPORTS AND BASE SETUP ******************************/
// import Arduino libs
#include <Arduino.h>

// utils for storing strings in prog mem (for mem optimisation)
#include <WString.h>
#include <avr/pgmspace.h>

// better int types
#include <inttypes.h>

// import logging library
#include "ArduinoLog.h"

// the globals and constants of this program
#include "globals.h"
using namespace constants;
using namespace globals;

// for components control functions
#include "components.h"

// for maze solving code
#include "rover_op.h"

// include function declarations for this file
#include "testing.h"

// buffer used for clearing out newline chars in the serial buffer
const int CLEAR_BUF_SIZE = 2;
char serialClearBuf[CLEAR_BUF_SIZE];

/****************** ASSESSMENT LEVELS ******************************/

// **LEVEL 2**

// **LEVEL 1**

void demoLevel_1_part2() {
    if (globals::RUN_START) {
        ALog.infoln(F("Level 1, part 2"));
        delay(2000);
    }

    for (int idx = 0; idx < 4; idx += 1) {
        doRvrMove(RvrMoveKind::driveFwd, 30 * globals::MICROS_PER_cm);
        delay(50);
        doRvrMove(RvrMoveKind::turnLeft, 90 * globals::MICROS_PER_deg);
        delay(50);
    }

    // wait for 5 minutes
    delay(5UL * 60 * 1000);
}

/****************** OTHER TESTING ******************************/

// **SERVO ANGLE TEST**

// buffer used for parsing ints
const int INT_BUF_SIZE = 7;
char intBuf[INT_BUF_SIZE];

void testServoAngle() {
    if (globals::RUN_START) {
        Serial.println(F("[___TEST] Testing servo angle PWM behaviour"));
        delay(1000);
        globals::RUN_START = false;
    }

    Serial.println(F("[___TEST] Enter servo angle/microseconds pulse length"));
    while (!Serial.available()) {
        ; // wait for a number to be sent on the Serial Monitor
    }
    size_t len = Serial.readBytesUntil(',', intBuf, (INT_BUF_SIZE - 1));
    size_t idx = min(len, (INT_BUF_SIZE - 1));
    intBuf[idx] = '\0'; // null terminate the string
    int input_num = atoi(intBuf);

    ALog.infoln("got angle/microseconds %d", input_num);

    //globals::THE_SERVO.writeMicroseconds(input_num);
    globals::THE_SERVO.write(input_num);
}

// **SONAR SWEEP TEST**

void testSonarSweep() {
    if (globals::RUN_START) {
        Serial.println(F("[___TEST] Testing sonar sweep"));
        globals::RUN_START = false;
    }
    Serial.println("");
    Serial.println(F("[___TEST] Press <Enter> to start a new sweep."));
    while (!Serial.available()) {
    }
    Serial.readBytesUntil('\n', serialClearBuf, CLEAR_BUF_SIZE);

    // create a variable to store the readings in
    SonarReadingSet readings;

    // perform the sonar sweep
    readings.doSonarSweep();

    // print out the measured values
    readings.printToSerialMonitor();
    Serial.println("");
}

// **SONAR RELIABILITY TEST**

// number of sonar measurements to take in each set
const int8_t SONAR_MEASUREMENTS = 3;

// routine for testing the reliability of the sonar subsystem
void testSonarReliability() {
    if (globals::RUN_START) {
        Serial.println(F("[___TEST] Testing sonar subsystem reliability"));

        // move the servo to test how securely the sonar module is attached
        setServoAngle(0);
        delay(500);
        setServoAngle(180);
        delay(500);
        setServoAngle(90);

        globals::RUN_START = false;
    }

    Serial.println("");
    Serial.println(F("[___TEST] Press <Enter> to start a new test."));
    while (!Serial.available()) {
    }
    Serial.readBytesUntil('\n', serialClearBuf, CLEAR_BUF_SIZE);

    for (int8_t idx = 0; idx < SONAR_MEASUREMENTS; idx += 1) {
        float dist = pollDistance();
        if (dist == 0.0) {
            Serial.println("[___TEST] failed measurement");
        } else {
            Serial.print("[___TEST] measurement: ");
            Serial.println(dist, 4);
        }
    }
}

// **CONSTANT MOTION TEST**

// Runs the two motors at the provided speeds for the provided time.
// Used for testing deviation when driving in a straight line and pivoting.
void testMovement(RvrMoveKind moveKind, unsigned long time) {
    if (globals::RUN_START) {
        Serial.println(F("[___TEST] Testing constant motion"));
        delay(2000);
    }

    // perform the specified movement
    doRvrMove(moveKind, time);

    // wait for 10 minutes
    delay(10UL * 60 * 1000);
}
