// this is an [include guard](https://en.wikipedia.org/wiki/Include_guard)
#ifndef ROVER_OP_H
#define ROVER_OP_H

#include <Servo.h>
#include <inttypes.h>

/****************** OTHER ******************************/

// components control functions
#include "components.h"

enum struct RoverAction {
    turnLeft,
    turnRight,
    longStepForward,
    shortStepFoward,
    sweepScan,
};

#define NUM_DISTANCES 6

// this struct contains the values for a full sonar sweep reading
struct SonarReading {
    float distances[NUM_DISTANCES];

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

/**
 * Perform a rover movement.
 *
 * \param move - The type of movement to perform
 * \param time - How long the rover will do the movement. Measured in microseconds.
 * \return void
 */
void driveRover(int leftMotorSpeed, int rightMotorSpeed, unsigned long time);

// Sweeps the servo and measures at 45 degree increments, writing the
// results to the provided pointer.
// Returns -1 if any of the measurements fail. Failed measurements are set to 0.0
int8_t sonarSweep(SonarReading *result);


#endif // ROVER_OP_H