#ifndef COMPONENTS_H
#define COMPONENTS_H

#include <inttypes.h>

// Pins for controlling a single motor.
// `pin1` and `pin2` should be defined such that
//    - a HIGH voltage on `pin1`, and
//    - a LOW voltage on `pin2`,
// will cause the motor to spin forwards.
// The enable pin must be a PWM capable pin.
struct MotorPins {
public:
    uint8_t pin1;
    uint8_t pin2;
    uint8_t enablePin;
};

// Motor settings. Speed valuse that will be used for moving the motor.
// The "Drive" values are used when moving straight forwards or backwards.
// The "Turn" values are used when turning.
struct MotorSettings {
    int leftMotorDrive;
    int rightMotorDrive;
    int leftMotorTurn;
    int rightMotorTurn;
};

// `delayMicroseconds()` doesn't work for large values, use this instead.
void delayMicrosLong(unsigned long us);

// Initialise a motor. Must be called before setting the motor speed.
void initMotor(const MotorPins& targetMotor);

// Initialise the servo. Must be called before setting the servo angle.
void initServo();

// Initialise the sonar module. Must be called before polling the sonar module.
void initSonarMod();

/**
 * Set the speed of a motor.
 *
 * \param targetMotor - which motor will have its speed set
 * \param speed - The speed to run the target motor at. Must be between 255 and 0. If `0`, the motor will brake.
 * \param reverse - Whether the motor should be run in reverse.
 * \return void
 */
void setMotorSpeed(const MotorPins& targetMotor, int speed, bool reverse);

// Measures the distance from the sonar module to an object in front of it,
// retrying failed attempts up to a certain number of attempts.
// Returns 0 if the distance cannot be found.
float pollDistance();

// Sets the angle that measurements will be taken at (relative to the rover).
// 0 is pointing to the left, 90 is pointing straight foward, 180 is pointing right.
void setServoAngle(int angle);

#endif // COMPONENTS_H