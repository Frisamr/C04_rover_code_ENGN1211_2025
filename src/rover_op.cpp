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

// helper functions for `doRvrMove`

void getMoveDirs(RvrMoveKind moveKind, bool& leftReverse, bool& rightReverse);
void getMoveSpeeds(RvrMoveKind moveKind, int& leftMotorSpeed, int& rightMotorSpeed);

/****************** MAZE SOLVING ******************************/

// TODO
/*
RvrMoveWrapper turn90() {
    ALog.traceln("`turn90()` called");

    // create a `SonarReadingSet` and peform the sonar sweep to populate it with readings
    SonarReadingSet currentReadings;
    currentReadings.doSonarSweep();
    setServoAngle(90);
    currentReadings.printToSerialMonitor();

    SonarReading straight = currentReadings.getReadingAtAngle(90);
    SonarReading left = currentReadings.getReadingAtAngle(180);
    SonarReading right = currentReadings.getReadingAtAngle(0);

    if (straight.distance < constants::WALL_DIST) {
        if (left.distance > 23.0) {
            RvrMoveWrapper move = {
                RvrMoveKind::turnLeft,
                90 * constants::MICROS_PER_DEG,
            };
            return move;
        }
    }
}
*/

/*
int8_t alignToWalls() {
    ALog.traceln("`alignToWalls` called");

    float prevLeftDist = 0.0;
    while (true) {
        doRvrMove()
    }
}
*/

bool readingCloserThan(SonarReading& reading, float dist) {
    if (reading.failed) {
        // assume wall is too far away, so we're good
        return false;
    }
    if (reading.distance <= dist) {
        return true;
    }
    return false;
}

constexpr float BASIC_COLLISION_OFFSET = -1.6;

RvrMoveWrapper basicCollisionAvoid() {
    ALog.traceln("`basicCollisionAvoid()` called");

    // take readings
    SonarReading reading_45 = takeReadingAtAngle(45);
    SonarReading reading_90 = takeReadingAtAngle(90);
    SonarReading reading_135 = takeReadingAtAngle(135);

    // apply offset
    reading_45.offsetBy(BASIC_COLLISION_OFFSET);
    reading_90.offsetBy(BASIC_COLLISION_OFFSET);
    reading_135.offsetBy(BASIC_COLLISION_OFFSET);

    // set the servo module back to straight ahead.
    setServoAngle(90);

    // if walls are too close
    if (readingCloserThan(reading_45, constants::WALL_DIST) || readingCloserThan(reading_90, constants::WALL_DIST) ||
        readingCloserThan(reading_135, constants::WALL_DIST))
    {
        ALog.infoln("walls too close");
        RvrMoveWrapper move = {
            RvrMoveKind::driveFwd,
            0, // don't move
        };
        return move;
    }
    // if walls are kinda close
    if (readingCloserThan(reading_45, constants::WALL_DIST + constants::LONG_STEP_DIST) ||
        readingCloserThan(reading_90, constants::WALL_DIST + constants::LONG_STEP_DIST) ||
        readingCloserThan(reading_135, constants::WALL_DIST + constants::LONG_STEP_DIST))
    {
        ALog.infoln("walls kinda close");
        unsigned long stepTime = timeToDriveDist(constants::SHORT_STEP_DIST);
        RvrMoveWrapper move = {
            RvrMoveKind::driveFwd,
            stepTime,
        };
        return move;
    }

    // if walls are not close
    ALog.infoln("walls not close");
    unsigned long stepTime = timeToDriveDist(constants::LONG_STEP_DIST);
    RvrMoveWrapper move = {
        RvrMoveKind::driveFwd,
        stepTime,
    };
    return move;
}

/****************** BASIC ROVER OPERATIONS ******************************/

SonarReading takeReadingAtAngle(int angle) {
    ALog.traceln("`takeReadingAtAngle` called with angle %d", angle);

    setServoAngle(angle);
    float dist = pollDistance();
    bool fail = (dist == 0.0);
    SonarReading reading = {
        dist,
        fail,
    };
    return reading;
}

void doRvrMove(RvrMoveKind moveKind, unsigned long time) {
    ALog.traceln("`doRvrMove` called with move %s, and time %u", getMoveName(moveKind), time);

    // don't move if `time` is 0
    if (time == 0) {
        return;
    }

    // variables for motor settings
    bool leftReverse;
    bool rightReverse;
    getMoveDirs(moveKind, leftReverse, rightReverse);
    int leftMotorSpeed;
    int rightMotorSpeed;
    getMoveSpeeds(moveKind, leftMotorSpeed, rightMotorSpeed);

    unsigned long delay_ms;
    unsigned int delay_us;

    // time is in milliseconds for drive moves
    if (moveKind == RvrMoveKind::driveFwd || moveKind == RvrMoveKind::driveBack) {
        delay_ms = time;
    }
    // time is in microseconds for turn moves
    if (moveKind == RvrMoveKind::turnLeft || moveKind == RvrMoveKind::turnRight) {
        delay_ms = (time / 1000);
        delay_us = (time % 1000);
    }
    ALog.traceln("got delay_ms %u, delay_us %d", delay_ms, delay_us);

    // Spin both motors
    setMotorSpeed(constants::LEFT_MOTOR, leftMotorSpeed, leftReverse);
    setMotorSpeed(constants::RIGHT_MOTOR, rightMotorSpeed, rightReverse);

    // Move for provided time
    delay(delay_ms);
    delayMicroseconds(delay_us);

    // Stop the motors. The motors brake when speed is 0, so the `reverse` setting doesn't matter
    setMotorSpeed(constants::LEFT_MOTOR, 0, false);
    setMotorSpeed(constants::RIGHT_MOTOR, 0, false);
}

/****************** HELPER FUNCTIONS ******************************/

void SonarReading::offsetBy(float offset) {
    if (!(this->failed)) {
        this->distance += offset;
    }
}

unsigned long timeToDriveDist(float dist_cm) {
    //ALog.verboseln("WHAT ON EARTH: %F", static_cast<float>(constants::MILLIS_PER_CM));
    float exactTime = dist_cm * static_cast<float>(constants::MILLIS_PER_CM);
    unsigned long time = static_cast<unsigned long>(round(exactTime));
    ALog.verboseln("`timeToDriveDist` called with dist %F, got time %u", dist_cm, time);
    return time;
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
        leftMotorSpeed = constants::MOTOR_CONFIG.leftMotorTurn;
        rightMotorSpeed = constants::MOTOR_CONFIG.rightMotorTurn;
        return;
    }
    if (moveKind == RvrMoveKind::driveFwd || moveKind == RvrMoveKind::driveBack) {
        leftMotorSpeed = constants::MOTOR_CONFIG.leftMotorDrive;
        rightMotorSpeed = constants::MOTOR_CONFIG.rightMotorDrive;
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
