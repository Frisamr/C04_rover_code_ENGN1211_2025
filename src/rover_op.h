// this is an [include guard](https://en.wikipedia.org/wiki/Include_guard)
#ifndef ROVER_OP_H
#define ROVER_OP_H

/****************** OTHER ******************************/

// components control functions
#include "components.h"

enum struct RvrMoveKind {
    turnLeft,
    turnRight,
    driveFwd,
    driveBack,
};

#define NUM_READINGS 6

// this struct holds a set of readings for a full sonar sweep
struct SonarReadingSet {
public:
    // holds the actual measure distances
    float distanceReadings[NUM_READINGS];

    // used for tracking which readings failed and which succeeded
    bool failedReadings[NUM_READINGS];

    // Sweeps the servo and measures at various angles, setting the distance and failed values.
    void doSonarSweep();

    // prints out the values of the readings to the serial monitor
    void printToSerialMonitor();

    // ## Static members:

    // list of angles to take readings at
    constexpr static int readingAngles[NUM_READINGS] = {0, 10, 45, 90, 135, 180};

    // Get the angle associated with a particular distance reading index.
    // Returns `-1` if the index is out of bounds
    //static int idxToAngle(size_t idx);

    // Get the index associated with a particular distance reading angle.
    // Returns `0` if the angle is not found
    static size_t angleToIdx(int angle);
};

/**
 * Perform a rover movement.
 *
 * @param move The type of movement to perform.
 * @param time How long the rover will do the movement. Measured in milliseconds.
 * @return void
 */
void doRvrMove(RvrMoveKind moveKind, unsigned long time);

// Get the name of a movement as a string.
const char* getMoveName(RvrMoveKind moveKind);

#endif // ROVER_OP_H
