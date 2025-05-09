// import Arduino libs
#include "HardwareSerial.h"
#include <Arduino.h>
#include <WString.h>
#include <avr/pgmspace.h>
// import better int types, servo library
#include <Servo.h>
#include <inttypes.h>

// import logging library
#include "ArduinoLog.h"

// the globals and constants of this program
#include "globals.h"
using namespace constants;
using namespace globals;

// import the data type and function declarations from the header file
#include "components.h"

/****************** STATIC CONFIG VALUES ******************************/

// there is a short delay (70ms) at the end of the polling function
// this prevents the sonar module from being polled too quickly and the previous
// pulse interfering with the next measurement
constexpr int POLLING_COOLDOWN_MS = 70;

// The speed of sound in air is 340m/s.
// We need to convert to centimetres/microsecond
//   (multiply by 100 to convert m to cm, divide by (1000 * 1000) to convert s
//   to us).
// Multiply by 1/2 so we don't have to in the loop.
constexpr float HALF_TIMES_VELOCITY = ((340.0 * 100.0) / (1000.0 * 1000.0)) * 0.5;

// The nominal maximum operating distance the HC-SR04 is 400cm. The full length of the
// maze is just under 80cm. So if we read a value of, say, 500cm or 900cm,
// something has clearly gone wrong. This variable sets the threshold for
// whether a distance is consider valid or not.
constexpr float MAX_VALID_SONAR_DIST = 900.0;
constexpr float MAX_VALID_SONAR_TIME = MAX_VALID_SONAR_DIST / HALF_TIMES_VELOCITY;

/****************** SERVO AND SONAR FUNCTIONS ******************************/

float pollSonarModuleRaw() {
    ALog.traceln("`pollSonarModuleRaw()` called");

    // this prevents the previous pulse interfering with the next measurement
    delay(POLLING_COOLDOWN_MS);

    // send a 10us pulse to trigger the sonar module
    digitalWrite(constants::TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(constants::TRIGGER_PIN, LOW);

    // measure the duration of the echo signal
    unsigned long duration = pulseIn(constants::ECHO_PIN, HIGH);
    ALog.verboseln("Measured duration (micros): %u", duration);

    // calculate the distance: (duration * velocity) / 2
    float distance = static_cast<float>(duration) * HALF_TIMES_VELOCITY;
    ALog.verboseln("Calculated distance (cm): %F", distance);

    return distance;
}

float pollDistance() {
    ALog.traceln("`pollDistance()` called");

    float finalDist = 0.0;
    int good_readings = 0;

    int poll_num = 0;
    for (poll_num = 0; poll_num < MAX_POLL_ATTEMPTS; poll_num += 1) {
        float rawDist = pollSonarModuleRaw();

        // failed reading
        if (rawDist > MAX_VALID_SONAR_DIST) {
            ALog.verboseln("bad reading: %F", rawDist);
            continue;
        }
        // successful reading
        good_readings += 1;
        finalDist += rawDist;
        if (good_readings >= MIN_POLL_ATTEMPTS) {
            poll_num += 1; // break skips the increment, so do it manually
            break;
        }
    }

    if (good_readings > 0) {
        finalDist = finalDist / static_cast<float>(good_readings);
    }

    ALog.traceln(
        "%d readings taken, %d were good readings. Final reading is %F", poll_num, good_readings, finalDist
    );

    return finalDist;
}

void setServoAngle(int angle) {
    ALog.traceln("`setServoAngle` called with angle %d, current ms is %d", angle, globals::SERVO_ANGLE);

    globals::THE_SERVO.write(angle);

    int angleDiff = abs(globals::SERVO_ANGLE - angle);
    int rotationTime = (angleDiff * globals::MILLIS_PER_DEGREE) + 50; // add an extra 30ms for safety

    ALog.verboseln("delay to allow for rotation: %d", rotationTime);
    delay(rotationTime);

    globals::SERVO_ANGLE = angle;
}

void initSonarSystem() {
    ALog.traceln("`initSonarSystem()` called");

    // the servo is controlled by the ouput of this pin
    pinMode(constants::SERVO_PIN, OUTPUT);

    // input from this pin is used to measure and calculate distances
    pinMode(constants::ECHO_PIN, INPUT);

    // the sonar module is triggered by the output of this pin
    pinMode(constants::TRIGGER_PIN, OUTPUT);
    digitalWrite(constants::TRIGGER_PIN, LOW); // start the trigger pin on low, ready to trigger the module

    globals::THE_SERVO.attach(constants::SERVO_PIN, 1000, 2000); // attach the servo to the servo pin

    // set a starting angle
    globals::THE_SERVO.writeMicroseconds(1500);
    globals::SERVO_ANGLE = 90;
}

/****************** MOTOR CONTROL FUNCTIONS ******************************/

void setMotorSpeed(const MotorPins &targetMotor, int speed) {
    if (speed >= 0) {
        // spin motor forwards
        // `pin1` HIGH and `pin2` LOW, as per MotorPins struct spec
        digitalWrite(targetMotor.pin2, LOW);
        digitalWrite(targetMotor.pin1, HIGH);

        // write PWM wave to enable pin to control motor speed
        // if `speed` is 0, then the enable pin duty cycle is zero and the other
        // pins don't matter
        analogWrite(targetMotor.enablePin, speed);
    } else if (speed < 0) {
        // spin motor backwards
        // `pin1` and `pin2` are swapped compared to forwards case
        // so voltage applied to motor is reversed
        digitalWrite(targetMotor.pin1, LOW);
        digitalWrite(targetMotor.pin2, HIGH);

        // analogWrite only takes values between `0` and `255`
        // `speed` is negative, so we have to negate it to make it a positive
        // value
        analogWrite(targetMotor.enablePin, -speed);
    }
}

void initMotor(const MotorPins &targetMotor) {
    // set the pins for controlling this motor to output mode
    pinMode(targetMotor.pin1, OUTPUT);
    pinMode(targetMotor.pin2, OUTPUT);
    pinMode(targetMotor.enablePin, OUTPUT);

    // set all the pins to low voltage, so no voltage is applied to the motor
    // until is is activated
    digitalWrite(targetMotor.pin1, LOW);
    digitalWrite(targetMotor.pin2, LOW);
    analogWrite(targetMotor.enablePin, 0);
}