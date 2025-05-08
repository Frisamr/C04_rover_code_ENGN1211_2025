#ifndef COMPONENTS_H
#define COMPONENTS_H

#include <Servo.h>
#include <inttypes.h>

// this struct contains the values for a full sonar sweep reading
struct SonarReading; // forward declaration (full declaration at the bottom of this file)

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

void initMotor(const MotorPins &targetMotor);

void setMotorSpeed(const MotorPins &targetMotor, int speed);

// Sweeps the servo and measures at 45 degree increments, writing the
// results to the provided pointer.
// Returns -1 if any of the measurements fail. Failed measurements are set to 0.0
int8_t sonarSweep(SonarReading *result);

// Measures the distance from the sonar module to an object in front of it,
// retrying failed attempts up to a certain number of attempts.
// Returns 0 if the distance cannot be found.
float pollDistance();

// initialise the sonar module and servo
void initSonarSystem();

// Sets the angle that measurements will be taken at (relative to the rover).
// 0 is pointing to the left, 90 is pointing straight foward, 180 is pointing
// right.
void setServoAngle(int angle);

// get a single distance value from the sonar module
float pollSonarModule();

// Initialise a motor. Must be called before setting the motor speed.
void initMotor(const MotorPins &targetMotor);

/// Set the speed of a motor. The motor must be initialised first.
///
/// \param targetMotor - which motor will have its speed set
/// \param speed - the speed to run the target motor at. If `speed` is negative,
/// the motor will run backwards. If `speed` is `0`, the motor will brake.
/// \return void
///
void setMotorSpeed(const MotorPins &targetMotor, int speed);

// full declaration
struct SonarReading {
    float dist_left;
    float dist_left_45;
    float dist_front;
    float dist_right_45;
    float dist_right;

    // helper function for setting values in this struct
    void setDistAtAngle(float dist, int angle);

    // print the value of this reading to the serial monitor
    void printToSerialMonitor();

    // minimum distance out of all the 45 degree reading in a full reading
    float minDist();
};

#endif // COMPONENTS_H