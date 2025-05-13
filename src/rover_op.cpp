#include <stddef.h>
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

constexpr int SonarReadingSet::readingAngles[NUM_READINGS];

/****************** DECLARATIONS ******************************/
void getMoveDirs(RoverMove move, bool& leftReverse, bool& rightReverse);
void getMoveSpeeds(RoverMove move, int& leftMotorSpeed, int& rightMotorSpeed);


/****************** MAZE-SOLVING FUNCTIONS ******************************/

void moveRover(RoverMove move, unsigned long time) {
    ALog.traceln("`driveRover` called with move %s, and time %u", getMoveName(move), time);

    // variables for motor settings
    bool leftReverse;
    bool rightReverse;
    getMoveDirs(move, leftReverse, rightReverse);
    int leftMotorSpeed;
    int rightMotorSpeed;
    getMoveSpeeds(move, leftMotorSpeed, rightMotorSpeed);

    // Spin both motors
    setMotorSpeed(constants::LEFT_MOTOR, leftMotorSpeed, leftReverse);
    setMotorSpeed(constants::RIGHT_MOTOR, rightMotorSpeed, rightReverse);

    // Drive forward for provided time
    delayMicroseconds(time);

    // Stop the motors. The motors brake at 0 speed, so reverse doesn't matter
    setMotorSpeed(constants::LEFT_MOTOR, 0, false);
    setMotorSpeed(constants::RIGHT_MOTOR, 0, false);
}



void SonarReadingSet::doSonarSweep() {
    ALog.traceln("`SonarReadingSet::doSonarSweep` called");

    for (size_t idx = 0; idx < NUM_READINGS; idx += 1) {
        int angle = SonarReadingSet::readingAngles[idx];
        setServoAngle(angle);

        float dist = pollDistance();
        this->distanceReadings[idx] = dist;
        if (dist == 0.0) {
            this->failedReadings[idx] = true;
        } else {
            this->failedReadings[idx] = false;
        }
    }
}

/****************** HELPER FUNCTIONS ******************************/

void SonarReadingSet::printToSerialMonitor() {
    Serial.print("SonarReadingSet { ");
    for (size_t idx = 0; idx < NUM_READINGS; idx += 1) {

        int angle = SonarReadingSet::readingAngles[idx];
        Serial.print(angle);


        float dist = this->distanceReadings[idx];
        Serial.print(": ");
        Serial.print(dist);
        Serial.print(", ");
    }
    Serial.print(" }");
}

/* int SonarReadingSet::idxToAngle(size_t idx) {
    ALog.verboseln("`SonarReadingSet::idxToAngle` called with idx %X", idx);
    if (idx >= NUM_READINGS) {
        return -1;
    }
    return SonarReadingSet::readingAngles[idx];
} */
size_t SonarReadingSet::angleToIdx(int angle) {
    ALog.verbose("`SonarReadingSet::angleToIdx` called with angle %d  ", angle);

    for (size_t idx = 0; idx < NUM_READINGS; idx += 1) {
        if (angle == SonarReadingSet::readingAngles[idx]) {
            ALog.verboseln("got idx %X", idx);
            return idx;
        }
    }
    ALog.verboseln("could not find angle, returning 0");
    return 0;
}

void getMoveDirs(RoverMove move, bool& leftReverse, bool& rightReverse) {
    // choose motor directions
    switch (move) {
    case RoverMove::turnLeft: {
        leftReverse = true;
        rightReverse = false;
        return;
    }
    case RoverMove::turnRight: {
        leftReverse = false;
        rightReverse = true;
        return;
    }
    case RoverMove::driveForward: {
        leftReverse = false;
        rightReverse = false;
        return;
    }
    case RoverMove::driveBack: {
        leftReverse = true;
        rightReverse = true;
        return;
    }
    }
}
void getMoveSpeeds(RoverMove move, int& leftMotorSpeed, int& rightMotorSpeed) {
    // choose motor speed
    if (move == RoverMove::turnLeft || move == RoverMove::turnRight) {
        leftMotorSpeed = globals::MOTOR_CONFIG.leftMotorTurn;
        rightMotorSpeed = globals::MOTOR_CONFIG.rightMotorTurn;
        return;
    }
    if (move == RoverMove::driveForward || move == RoverMove::driveBack) {
        leftMotorSpeed = globals::MOTOR_CONFIG.leftMotorDrive;
        rightMotorSpeed = globals::MOTOR_CONFIG.rightMotorDrive;
        return;
    }
}

namespace rover_move_names {
    const char* turnLeftStr = "turnLeft";
    const char* turnRightStr = "turnRight";
    const char* driveForwardStr = "driveForward";
    const char* driveBackStr = "driveBack";
} //namespace rover_move_names

// Get the name of a movement as a C-style string.
const char* getMoveName(RoverMove move) {
    switch (move) {
    case RoverMove::turnLeft: {
        return rover_move_names::turnLeftStr;
    }
    case RoverMove::turnRight: {
        return rover_move_names::turnRightStr;
    }
    case RoverMove::driveForward: {
        return rover_move_names::driveForwardStr;
    }
    case RoverMove::driveBack: {
        return rover_move_names::driveBackStr;
    }
    default: {
        // All cases are covered, so this should be impossible.
        return nullptr;
    }
    }
}
