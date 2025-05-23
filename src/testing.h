// this is an [include guard](https://en.wikipedia.org/wiki/Include_guard)
#ifndef TESTING_H
#define TESTING_H

enum struct RvrMoveKind;

/****************** ASSESSMENT LEVELS ******************************/

void demo_level_1_part2();
void demo_level_2();
void demo_level_345();

/****************** OTHER TESTING ******************************/

// routine for testing sonar behaviour at different angles
void testAngledSonar();

// routine for testing the reliability of the sonar subsystem
void testSonarReliability();

// routine for checking the angle of the servo
void testServoAngle();

// useful for testing movements in isolation
void testMovement(RvrMoveKind moveKind, unsigned long time, uint8_t reps);

#endif // TESTING_H
