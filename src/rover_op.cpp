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

/****************** FORWARD DECLARATIONS ******************************/
// we have to declare this here because static data members are janky in C++11
constexpr int SonarReadingSet::readingAngles[NUM_READINGS];

// helper functions for `doRvrMove`
void getMoveDirs(RvrMoveKind moveKind, bool& leftReverse, bool& rightReverse);
void getMoveSpeeds(RvrMoveKind moveKind, int& leftMotorSpeed, int& rightMotorSpeed);

/****************** MAZE-SOLVING FUNCTIONS ******************************/

void doRvrMove(RvrMoveKind moveKind, unsigned long time) {
    ALog.traceln("`doRvrMove` called with move %s, and time %u", getMoveName(moveKind), time);

    // variables for motor settings
    bool leftReverse;
    bool rightReverse;
    getMoveDirs(moveKind, leftReverse, rightReverse);
    int leftMotorSpeed;
    int rightMotorSpeed;
    getMoveSpeeds(moveKind, leftMotorSpeed, rightMotorSpeed);

    // Spin both motors
    setMotorSpeed(constants::LEFT_MOTOR, leftMotorSpeed, leftReverse);
    setMotorSpeed(constants::RIGHT_MOTOR, rightMotorSpeed, rightReverse);

    // Move for provided time
    delay(time);

    // Stop the motors. The motors brake when speed is 0, so the `reverse` setting doesn't matter
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

void getMoveDirs(RvrMoveKind moveKind, bool& leftReverse, bool& rightReverse) {
    // choose motor directions
    switch (moveKind) {
    case RvrMoveKind::turnLeft: {
        leftReverse = true;
        rightReverse = false;
        return;
    }
    case RvrMoveKind::turnRight: {
        leftReverse = false;
        rightReverse = true;
        return;
    }
    case RvrMoveKind::driveFwd: {
        leftReverse = false;
        rightReverse = false;
        return;
    }
    case RvrMoveKind::driveBack: {
        leftReverse = true;
        rightReverse = true;
        return;
    }
    }
}
void getMoveSpeeds(RvrMoveKind moveKind, int& leftMotorSpeed, int& rightMotorSpeed) {
    // choose motor speed
    if (moveKind == RvrMoveKind::turnLeft || moveKind == RvrMoveKind::turnRight) {
        leftMotorSpeed = globals::MOTOR_CONFIG.leftMotorTurn;
        rightMotorSpeed = globals::MOTOR_CONFIG.rightMotorTurn;
        return;
    }
    if (moveKind == RvrMoveKind::driveFwd || moveKind == RvrMoveKind::driveBack) {
        leftMotorSpeed = globals::MOTOR_CONFIG.leftMotorDrive;
        rightMotorSpeed = globals::MOTOR_CONFIG.rightMotorDrive;
        return;
    }
}

namespace rover_move_names {
    const char* turnLeftStr = "turnLeft";
    const char* turnRightStr = "turnRight";
    const char* driveFwdStr = "driveFwd";
    const char* driveBackStr = "driveBack";
} //namespace rover_move_names

// Get the name of a movement as a C-style string.
const char* getMoveName(RvrMoveKind moveKind) {
    switch (moveKind) {
    case RvrMoveKind::turnLeft: {
        return rover_move_names::turnLeftStr;
    }
    case RvrMoveKind::turnRight: {
        return rover_move_names::turnRightStr;
    }
    case RvrMoveKind::driveFwd: {
        return rover_move_names::driveFwdStr;
    }
    case RvrMoveKind::driveBack: {
        return rover_move_names::driveBackStr;
    }
    default: {
        // All cases are covered, so this should be impossible.
        return nullptr;
    }
    }
}
