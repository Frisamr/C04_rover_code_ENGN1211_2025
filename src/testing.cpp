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

void demo_level_2() {
    if (globals::RUN_START) {
        ALog.infoln(F("Level 2"));
        delay(2000);
        globals::RUN_START = false;
    }

    RvrMoveWrapper move = basicCollisionAvoid();
    ALog.infoln("decided on move kind %s and time %u", getMoveName(move.moveKind), move.time);

    /*
    Serial.println(F("[___TEST] Press <Enter> to do next movement."));
    while (!Serial.available()) {
    }
    Serial.readBytesUntil('\n', serialClearBuf, CLEAR_BUF_SIZE);
    */

    doRvrMove(move.moveKind, move.time);
}

// **LEVEL 1**

void demo_level_1_part2() {
    if (globals::RUN_START) {
        ALog.infoln(F("Level 1, part 2"));
        delay(2000);
        globals::RUN_START = false;
    }

    for (int idx = 0; idx < 4; idx += 1) {
        doRvrMove(RvrMoveKind::driveFwd, 30 * constants::MILLIS_PER_CM);
        delay(50);
        doRvrMove(RvrMoveKind::turnLeft, 90 * constants::MILLIS_PER_DEG);
        delay(50);
    }

    // wait for 5 minutes
    delay(5UL * 60 * 1000);
}

/****************** OTHER TESTING ******************************/

// buffer used for parsing ints
const int INT_BUF_SIZE = 7;
char intBuf[INT_BUF_SIZE];

// **ANGLED SONAR TEST**

void testAngledSonar() {
    if (globals::RUN_START) {
        Serial.println(F("[___TEST] Testing angled sonar"));
        delay(300);
        globals::RUN_START = false;
    }

    Serial.println(F("[___TEST] Enter servo angle/microseconds pulse length"));
    while (!Serial.available()) {
        ; // wait for a number to be sent on the Serial Monitor
    }
    size_t len = Serial.readBytesUntil(',', intBuf, (INT_BUF_SIZE - 1));
    size_t idx = min(len, (INT_BUF_SIZE - 1));
    intBuf[idx] = '\0'; // null terminate the string
    int input_angle = atoi(intBuf);

    ALog.infoln("got angle %d degrees", input_angle);

    SonarReading reading = takeReadingAtAngle(input_angle);
    ALog.infoln("got reading {dist = %F, fail=%T}", reading.distance, reading.failed);
}

// **SERVO ANGLE TEST**

void testServoAngle() {
    if (globals::RUN_START) {
        Serial.println(F("[___TEST] Testing servo angle PWM behaviour"));
        delay(300);
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
