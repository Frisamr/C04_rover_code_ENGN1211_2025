// this is an [include guard](https://en.wikipedia.org/wiki/Include_guard)
#ifndef SONAR_H
#define SONAR_H

#include <inttypes.h>
#include <Servo.h>

// this struct contains the values for a full sonar sweep reading
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

// this struct contains all the pin number required for operating the sonar subsystem.
struct SonarSystemPins {
public:
  uint8_t servoPin;
  uint8_t triggerPin;
  uint8_t echoPin;
};

// This class conceptually represents the sonar subsystem.
class SonarSystem {
public:
  const SonarSystemPins pins;
  Servo servo;
  int currentAngle;

  void init();
  
  // Sets the angle that measurements will be taken at (relative to the rover).
  // 0 is pointing to the left, 90 is pointing straight foward, 180 is pointing right.
  void setAngle(int angle);

  // Poll the sonar module, retrying failed attempts up to a certain number of attempts. 
  // Returns 0 if the distance cannot be found.
  float pollSonarDist();

  // Sweeps the servo and measures at 45 degree increments, writing the results to the provided pointer.
  // Returns -1 if any of the measurement fail. Failed measurements are set to 0.0
  int8_t sonarSweep(SonarReading* result);

private:
  float pollSonarModuleRaw();
};

void initSonarSystem(SonarSystem& sonar_sys);

// get a single distance value from the sonar module
float pollSonarModule(const SonarSystem& sonar_sys);

#endif