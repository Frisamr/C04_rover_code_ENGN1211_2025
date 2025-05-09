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

    // how close the rover will drive to a wall before slowing down
    constexpr float SLOW_THRESHOLD = 4.5;

    // how close the rover will drive to a wall before stopping
    constexpr float STOP_THRESHOLD = 3.5;

    /****************** SONAR CONFIG ******************************/

    // There is a short delay at the start of the sonar polling function.
    // This prevents the sonar module from being polled too quickly and
    // the previous pulse interfering with the next measurement.
    // This variable controls the delay.
    constexpr int POLLING_COOLDOWN_MS = 80;

    // Min number of times the sonar subsytem will poll the sonar module.
    // If the sonar is polled multiple times, the final calculated distance will be the
    // average of the readings.
    constexpr int8_t MIN_POLL_ATTEMPTS = 3;
    // Max number of times the sonar subsystem will re-try polling the sonar module when measurements fail.
    constexpr int8_t MAX_POLL_ATTEMPTS = 10;

    /****************** SERVO CONFIG ******************************/

    // Microseconds pulse width corresponding to the min (0 degrees) angle of the servo.
    constexpr int SERVO_MIN_US = 590;
    // Microseconds pulse width corresponding to the max (180 degrees) angle of the servo.
    constexpr int SERVO_MAX_US = 2530;

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

// Because these vars are non-constant, they cannot be initialised in this header file.
// They must be initialised in the main file.
namespace globals {
    // global servo object
    extern Servo THE_SERVO;

    // stores the current angle the servo is at
    extern int SERVO_ANGLE;

    extern MotorSettings MOTOR_CONFIG;

    // how long (in ms) it takes the rover to drive 1cm forward at full speed
    extern unsigned long MILLIS_PER_CM;

    // how long (in ms) it takes the rover to rotate 1 degree at full speed
    extern unsigned long MILLIS_PER_DEGREE;

    // how long the rover will drive for when doing a short step
    // set so the rover will drive approx. 0.5cm
    extern unsigned long SHORT_STEP_TIME;

    // how long the rover will drive for when doing a long step
    // set so the rover will drive 3cm
    extern unsigned long LONG_STEP_TIME;

    // used for printing a message about what test is being run on the first run (and running test setup)
    extern bool RUN_START;
} //namespace globals

#endif // GLOBAL_H