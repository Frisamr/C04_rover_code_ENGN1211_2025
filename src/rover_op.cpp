// import standard arduino functions, better int types, servo library
#include <Arduino.h>
#include <Servo.h>

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
void getMoveDirs(RoverMovement move, bool& leftReverse, bool& rightReverse);
void getMoveSpeeds(RoverMovement move, int& leftMotorSpeed, int& rightMotorSpeed);

/****************** MAZE-SOLVING HELPER FUNCTIONS ******************************/

// Sweeps the servo and measures at 45 degree increments, writing the results to
// the provided pointer. Returns -1 if any of the measurement fail. Failed
// measurements are set to 0.0
int sonarSweep(SonarReading* result) {
    ALog.traceln("`SonarSystem::sonarSweep` called");

    int startAngle = 0;
    int increment = 45;

    if (globals::SERVO_ANGLE > 90) {
        startAngle = 180;
        increment = -45;
    }

    int errno = 0;
    for (int angle = startAngle; (angle >= 0) && (angle <= 180); angle += increment) {
        setServoAngle(angle);
        float dist = pollDistance();
        result->setDistAtAngle(dist, angle);
        if (dist == 0.0) {
            errno = -1;
        }
    }

    return errno;
}

void moveRover(RoverMovement move, unsigned long time) {
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

/****************** HELPER FUNCTIONS ******************************/

void SonarReading::setDistAtAngle(float dist, int angle) {
    ALog.verboseln("`SonarSystem::setDistAtAngle` called with dist %F and angle %d", dist, angle);

    switch (angle) {
    case 0:
        this->dist_left = dist;
        break;
    case 45:
        this->dist_left_45 = dist;
        break;
    case 90:
        this->dist_front = dist;
        break;
    case 135:
        this->dist_right_45 = dist;
        break;
    case 180:
        this->dist_right = dist;
        break;
    default:
        ALog.error("`SonarReading::setDistAtAngle` called with invalid angle %d", angle);
    }
}

void SonarReading::printToSerialMonitor() {
    Serial.print("Sonar reading: { left = ");
    Serial.print(this->dist_left, 4);
    Serial.print(", left45 = ");
    Serial.print(this->dist_left_45, 4);
    Serial.print(", front = ");
    Serial.print(this->dist_front, 4);
    Serial.print(", right45 = ");
    Serial.print(this->dist_right_45, 4);
    Serial.print(", right = ");
    Serial.print(this->dist_right, 4);
    Serial.println(" }");
}

float SonarReading::minDist() {
    ALog.traceln("`SonarReading::minDist` called");

    // any real reading is guaranteed to be less than this
    float minimumDist = this->dist_left;

    if ((this->dist_left_45) < minimumDist) {
        minimumDist = this->dist_left_45;
    }
    if ((this->dist_front) < minimumDist) {
        minimumDist = this->dist_front;
    }
    if ((this->dist_right_45) < minimumDist) {
        minimumDist = this->dist_right_45;
    }
    if ((this->dist_right) < minimumDist) {
        minimumDist = this->dist_right;
    }

    return minimumDist;
}

namespace moveNames {
    const char* turnLeftStr = "turnLeft";
    const char* turnRightStr = "turnRight";
    const char* driveForwardStr = "driveForward";
    const char* driveBackStr = "driveBack";
} //namespace moveNames

// Get the name of a movement as a C-style string.
const char* getMoveName(RoverMovement move) {
    switch (move) {
    case RoverMovement::turnLeft: {
        return moveNames::turnLeftStr;
    }
    case RoverMovement::turnRight: {
        return moveNames::turnRightStr;
    }
    case RoverMovement::driveForward: {
        return moveNames::driveForwardStr;
    }
    case RoverMovement::driveBack: {
        return moveNames::driveBackStr;
    }
    default: {
        // All cases are covered, so this should be impossible.
        return nullptr;
    }
    }
}

void getMoveDirs(RoverMovement move, bool& leftReverse, bool& rightReverse) {
    // choose motor directions
    switch (move) {
    case RoverMovement::turnLeft: {
        leftReverse = true;
        rightReverse = false;
        return;
    }
    case RoverMovement::turnRight: {
        leftReverse = false;
        rightReverse = true;
        return;
    }
    case RoverMovement::driveForward: {
        leftReverse = false;
        rightReverse = false;
        return;
    }
    case RoverMovement::driveBack: {
        leftReverse = true;
        rightReverse = true;
        return;
    }
    }
}
void getMoveSpeeds(RoverMovement move, int& leftMotorSpeed, int& rightMotorSpeed) {
    // choose motor speed
    if (move == RoverMovement::turnLeft || move == RoverMovement::turnRight) {
        leftMotorSpeed = globals::MOTOR_CONFIG.leftMotorTurn;
        rightMotorSpeed = globals::MOTOR_CONFIG.rightMotorTurn;
        return;
    }
    if (move == RoverMovement::driveForward || move == RoverMovement::driveBack) {
        leftMotorSpeed = globals::MOTOR_CONFIG.leftMotorDrive;
        rightMotorSpeed = globals::MOTOR_CONFIG.rightMotorDrive;
        return;
    }
}
