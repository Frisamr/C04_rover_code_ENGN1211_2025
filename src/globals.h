// this is an [include guard](https://en.wikipedia.org/wiki/Include_guard)
#ifndef GLOBAL_H
#define GLOBAL_H

#include <inttypes.h>

// Servo library (for global servo object)
#include <Servo.h>

// components code (for pin structs)
#include "components.h"

/****************** CONSTANTS ******************************/

namespace constants {
    /****************** MAZE-SOLVING CONFIG ******************************/

    // speed settings for the motors for different movements
    constexpr MotorSettings MOTOR_CONFIG = {
        255, //leftMotorDrive
        250, //rightMotorDrive
        255, //leftMotorTurn
        255, //rightMotorTurn
    };

    // how close the rover will aim to drive to the left wall, in cm
    constexpr float WALL_DIST = 1.5;

    // how far the rover will move when it is far away from a wall, in cm
    constexpr float LONG_STEP_DIST = 4.0;

    // how far the rover will move when it is close to a wall, in cm
    constexpr float SHORT_STEP_DIST = 0.5;

    /****************** SONAR CONFIG ******************************/

    // There is a short delay at the start of the sonar polling function.
    // This prevents the sonar module from being polled too quickly and
    // the previous pulse interfering with the next measurement.
    // This variable controls the delay.
    constexpr int POLL_COOLDOWN_MS = 80;

    // Min number of times the sonar subsytem will poll the sonar module.
    // If the sonar is polled multiple times, the final calculated distance will be the
    // average of the readings.
    constexpr int8_t MIN_POLL_ATTEMPTS = 3;
    // Max number of times the sonar subsystem will re-try polling the sonar module when measurements fail.
    constexpr int8_t MAX_POLL_ATTEMPTS = 6;

    // The nominal maximum operating distance the HC-SR04 is 400cm. The full length of the maze is just under 80cm.
    // So if we read a value of, say, 500cm or 900cm, clearly something has gone wrong.
    // This variable sets the threshold for whether a distance is consider valid or not.
    constexpr float MAX_VALID_SONAR_DIST = 900.0;

    /****************** SERVO CONFIG ******************************/

    // Microseconds pulse width corresponding to the min (0 degrees) angle of the servo.
    constexpr int SERVO_MIN_MICROS = 560;
    // Microseconds pulse width corresponding to the max (180 degrees) angle of the servo.
    constexpr int SERVO_MAX_MICROS = 2500;

    // After setting the servo angle, we need to wait a bit for it to finishing turning.
    // According to the datasheet, the servo rotates at 500 degrees per second at 6V. This equates to 2ms per degree.
    // However, we are running it at 5V, so add a bit of extra time to compensate.
    constexpr unsigned long SERVO_MILLIS_PER_DEG = 3;

    /****************** ARDUINO PINS ******************************/

    // definition of motor 1 pins
    constexpr MotorPins LEFT_MOTOR{
        9,  // pin1
        8,  // pin2
        10, //enablePin
    };
    // definition of motor 2 pins
    constexpr MotorPins RIGHT_MOTOR{
        12, // pin1
        7,  // pin2
        11, //enablePin
    };

    // sin for servo control via PWM
    constexpr uint8_t SERVO_PIN = 3;

    // sonar module trigger pin
    constexpr uint8_t TRIGGER_PIN = 4;

    // sonar module echo pin
    constexpr uint8_t ECHO_PIN = 5;

} //namespace constants

/****************** NON-CONSTANT GLOBAL VARS ******************************/

// Because these variables are non-constant, they cannot be initialised in this header file.
// They must be initialised in the main file.
namespace globals {
    extern uint16_t MILLIS_PER_CM;

    // how long (in ms) it takes the rover to turn 1 degree
    extern unsigned long MILLIS_45_DEG;

    // global servo object
    extern Servo THE_SERVO;

    // stores the current angle the servo is at
    extern int SERVO_ANGLE;

    // used for printing a message about what test is being run on the first run (and running test setup)
    extern bool RUN_START;
} //namespace globals

#endif // GLOBAL_H
