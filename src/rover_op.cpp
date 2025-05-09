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

/****************** MAZE-SOLVING HELPER FUNCTIONS ******************************/

// Sweeps the servo and measures at 45 degree increments, writing the results to
// the provided pointer. Returns -1 if any of the measurement fail. Failed
// measurements are set to 0.0
int8_t sonarSweep(SonarReading *result_ptr) {
    ALog.traceln("`SonarSystem::sonarSweep` called");

    int startAngle = 0;
    int increment = 45;

    if (globals::SERVO_ANGLE > 90) {
        startAngle = 180;
        increment = -45;
    }

    int8_t errno = 0;
    for (int angle = startAngle; (angle >= 0) && (angle <= 180); angle += increment) {
        setServoAngle(angle);
        float dist = pollDistance();
        result_ptr->setDistAtAngle(dist, angle);
        if (dist == 0.0) {
            errno = -1;
        }
    }

    return errno;
}

void driveRover(int leftMotorSpeed, int rightMotorSpeed, unsigned long time) {
    ALog.traceln(
        "`driveRover` called with leftMotorSpeed %d, rightMotorSpeed %d, and time %u", leftMotorSpeed, rightMotorSpeed,
        time
    );

    // spin both motors forwards
    setMotorSpeed(constants::LEFT_MOTOR, leftMotorSpeed);
    setMotorSpeed(constants::RIGHT_MOTOR, rightMotorSpeed);

    // drive forward for provided time
    delay(time);

    // stop
    setMotorSpeed(constants::LEFT_MOTOR, 0);
    setMotorSpeed(constants::RIGHT_MOTOR, 0);
}

/****************** SONAR READING HELPER FUNCTIONS ******************************/

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

