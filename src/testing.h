// this is an [include guard](https://en.wikipedia.org/wiki/Include_guard)
#ifndef TESTING_H
#define TESTING_H

enum struct RoverMovement;

/****************** ASSESSMENT LEVELS ******************************/

void demoLevel_1_part1();
void demoLevel_1_part2();

/****************** OTHER TESTING ******************************/

// routine for testing the reliability of collision avoidance
void testCollisionAvoidance();

// routine for testing the reliability of the sonar subsystem
void testSonarReliability();

// routine for checking the angle of the servo
void testServoAngle();

// useful for testing movements in isolation
void testMovement(RoverMovement move, unsigned long time);

#endif // TESTING_H