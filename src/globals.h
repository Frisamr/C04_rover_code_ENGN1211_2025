// this is an [include guard](https://en.wikipedia.org/wiki/Include_guard)
#ifndef GLOBAL_H
#define GLOBAL_H

#include <inttypes.h>
#include "motor_ctl.h"
#include "sonar_system.h"

/****************** CONSTANTS ******************************/

namespace constants {
  /****************** PINS ******************************/

  // definition of motor 1 pins
  constexpr DCMotor LEFT_MOTOR{
    9,   // pin1
    8,   // pin2
    10,  //enablePin
  };
  // definition of motor 2 pins
  constexpr DCMotor RIGHT_MOTOR{
    12,  // pin1
    7,   // pin2
    11,  //enablePin
  };

  // define the sonar module pins
  constexpr SonarSystemPins SONAR_SYS_PINS{
    3,  // servoPin
    4,  // sonarTriggerPin
    5,  // sonarEchoPin
  };


  /****************** MAZE-SOLVING CONFIG ******************************/

  // how close the rover will drive to a wall before slowing down
  constexpr float SLOW_THRESHOLD = 4.5;

  // how close the rover will drive to a wall before stopping
  constexpr float STOP_THRESHOLD = 3.5;

}


/****************** NON-CONST GLOBALS ******************************/

// global sonar system object
extern SonarSystem sonarSys;

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
extern bool firstRun;


#endif