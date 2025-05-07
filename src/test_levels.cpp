/****************** IMPORTS AND BASE SETUP ******************************/
// import Arduino libs
#include <Arduino.h>
#include "HardwareSerial.h"
#include <avr/pgmspace.h>
#include <WString.h>
#include "pins_arduino.h"
// better int types
#include <inttypes.h>


// import logging library
#include "ArduinoLog.h"

// the globals and constants of this program
#include "globals.h"
// custom motor control code
#include "motor_ctl.h"
// custom sonar system code
#include "sonar_system.h"
// maze solving code
#include "rover_op.h"



/****************** LEVEL 2 ******************************/


/****************** LEVEL 1 ******************************/

void demoLevel_1_part2() {
  if (firstRun) {
    ALog.infoln(F("Level 1, part 2"));
    delay(300);
  }
  
  for (int idx = 0; idx < 4; idx += 1) {
    driveRover(255, 247, 30 * MILLIS_PER_CM);
    delay(50);
    driveRover(255, -255, 90 * MILLIS_PER_DEGREE);
    delay(50);
  }

  // wait for 1 minute
  delay(1UL * 60 * 1000);
}

void demoLevel_1_part1() {
  if (firstRun) {
    ALog.infoln(F("Level 1, part 1"));
    delay(3000);
  }
  
  driveRover(255, 255, 3000);   // drive forward in a straight line,
  delay(3000);                  // stop for 3 seconds
  driveRover(-255, -255, 3000); // reverse back to starting position

  // wait for 1 minute
  delay(1UL * 60 * 1000);
}

