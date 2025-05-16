// this is an [include guard](https://en.wikipedia.org/wiki/Include_guard)
#ifndef ROVER_OP_H
#define ROVER_OP_H

/****************** DATA TYPES ******************************/

// components control functions
#include "components.h"

enum struct RvrMoveKind {
    turnLeft,
    turnRight,
    driveFwd,
    driveBack,
};

struct RvrMoveWrapper {
    RvrMoveKind moveKind;
    unsigned long time;
};

/****************** DECISION MAKERS ******************************/

RvrMoveWrapper basicCollisionAvoid();

/****************** UTILS ******************************/

// Calculate how long it will take to drive a certain (potentially non-integer!) distance in cm.
// Takes the distance in cm, and returns the time in milliseconds.
unsigned long timeToDriveDist(float dist_cm);

// Get the name of a movement as a string.
const char* getMoveName(RvrMoveKind moveKind);

/****************** BASIC ROVER OPERATIONS ******************************/

struct SonarReading {
    float distance;
    bool failed;

    void offsetBy(float offset);
};

SonarReading takeReadingAtAngle(int angle);

/**
 * Perform a rover movement.
 *
 * @param move The type of movement to perform.
 * @param time How long the rover will do the movement. Measured in milliseconds for drive moves and
 * microseconds for turn moves.
 * @return void
 */
void doRvrMove(RvrMoveKind moveKind, unsigned long time);

#endif // ROVER_OP_H
