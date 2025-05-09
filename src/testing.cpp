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

void demoLevel_1_part1() {
    if (globals::RUN_START) {
        ALog.infoln(F("Level 1, part 1"));
        delay(3000);
    }

    driveRover(255, 255, 3000);   // drive forward in a straight line,
    delay(3000);                  // stop for 3 seconds
    driveRover(-255, -255, 3000); // reverse back to starting position

    // wait for 1 minute
    delay(1UL * 60 * 1000);
}

void demoLevel_1_part2() {
    if (globals::RUN_START) {
        ALog.infoln(F("Level 1, part 2"));
        delay(300);
    }

    for (int idx = 0; idx < 4; idx += 1) {
        driveRover(255, 247, 30 * globals::MILLIS_PER_CM);
        delay(50);
        driveRover(-255, 255, 90 * globals::MILLIS_PER_DEGREE);
        delay(50);
    }

    // wait for 1 minute
    delay(1UL * 60 * 1000);
}

/****************** OTHER TESTING ******************************/

// **COLLISION AVOIDANCE TEST**
// NOTE: this is a Proof-of-Concept, not a full maze-solving algorithm.

// variables for storing rover state
RoverAction nextAction = RoverAction::sweepScan;
bool stopped = false;
SonarReading currentReading;

// Proof-of-Concept collision avoiance.
// This algorithm will not actually navigate the maze. It just stops the rover and blinks the built-in LED on the
// Arduino if it detects walls closer than 3.5cm
void testCollisionAvoidance() {
    if (globals::RUN_START) {
        Serial.println(F("[___TEST] Testing collision avoidance"));
        pinMode(LED_BUILTIN, OUTPUT);

        // pause for 1s, to allow for moving into position
        // you can increase this delay if you need more time after pressing the reset button
        delay(1000);

        globals::RUN_START = false;
    }

    if (stopped) {
        // blink the built-in LED
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
    } else {
        switch (nextAction) {
        // short step foward
        case RoverAction::shortStepFoward: {
            driveRover(255, 255, globals::SHORT_STEP_TIME);
            nextAction = RoverAction::sweepScan;
            ALog.infoln(F("completed short step foward, switching to scan"));
            break;
        }

        // long step foward
        case RoverAction::longStepForward: {
            driveRover(255, 255, globals::LONG_STEP_TIME);
            nextAction = RoverAction::sweepScan;
            ALog.infoln(F("completed long step foward, switching to scan"));
            break;
        }

        // perform a scan, and decide what to do next
        case RoverAction::sweepScan: {
            // perform a sweeping scan, measuring at 45 degree angles
            // sonarSys.sonarSweep(&currentReading);
            float currentDist = pollDistance();

            // make the decision
            if (currentDist < constants::STOP_THRESHOLD) {
                ALog.infoln(F("walls too close, stopping"));
                stopped = true;
            } else if (currentDist < constants::SLOW_THRESHOLD) {
                ALog.infoln(F("walls close, short stepping"));
                nextAction = RoverAction::shortStepFoward;
            } else {
                ALog.infoln(F("no walls close, long stepping"));
                nextAction = RoverAction::longStepForward;
            }
            break;
        }

        // BAD CODE: this should never happen, but handle it anyway
        default: {
            ALog.errorln(F("PoC collision avoidance reaching invalid state."));
            nextAction = RoverAction::sweepScan;
            break;
        }
        }
    }
}

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
void testConstantMotion(int motor1Speed, int motor2Speed, unsigned long time) {
    if (globals::RUN_START) {
        Serial.println("[___TEST] Testing constant motion");
    }

    // pause for 1s, to allow for moving into position
    // you can increase this delay if you need more time after pressing the reset button
    delay(1000);

    // drive the rover forward with the provided values
    driveRover(motor1Speed, motor2Speed, time);

    // wait for 10 minutes
    delay(10UL * 60 * 1000);
}
