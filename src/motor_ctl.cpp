// import Arduino libs
#include <Arduino.h>
#include "HardwareSerial.h"
#include <avr/pgmspace.h>
#include <WString.h>
// import better int types
#include <inttypes.h>


// import logging library
#include "ArduinoLog.h"

// This struct conceptually represents a single motor.
// It contains the pins we will use to control the motor.
// `pin1` and `pin2` should be defined such that
//    - a HIGH voltage on `pin1`, and
//    - a LOW voltage on `pin2`,
// will cause the motor to spin forwards.
// The enable pin must be a PWM capable pin.
struct DCMotor {
public:
  uint8_t pin1;
  uint8_t pin2;
  uint8_t enablePin;
};

void setMotorSpeed(const DCMotor& targetMotor, int speed) {
  if (speed >= 0) {
    // spin motor forwards
    // `pin1` HIGH and `pin2` LOW, as per DCMotor struct spec
    digitalWrite(targetMotor.pin2, LOW);
    digitalWrite(targetMotor.pin1, HIGH);

    // write PWM wave to enable pin to control motor speed
    // if `speed` is 0, then the enable pin duty cycle is zero and the other pins don't matter
    analogWrite(targetMotor.enablePin, speed);
  } else if (speed < 0) {
    // spin motor backwards
    // `pin1` and `pin2` are swapped compared to forwards case
    // so voltage applied to motor is reversed
    digitalWrite(targetMotor.pin1, LOW);
    digitalWrite(targetMotor.pin2, HIGH);

    // analogWrite only takes values between `0` and `255`
    // `speed` is negative, so we have to negate it to make it a positive value
    analogWrite(targetMotor.enablePin, -speed);
  }
}

void initMotor(const DCMotor& targetMotor) {
  // set the pins for controlling this motor to output mode
  pinMode(targetMotor.pin1, OUTPUT);
  pinMode(targetMotor.pin2, OUTPUT);
  pinMode(targetMotor.enablePin, OUTPUT);

  // set all the pins to low voltage, so no voltage is applied to the motor until is is activated
  digitalWrite(targetMotor.pin1, LOW);
  digitalWrite(targetMotor.pin2, LOW);
  analogWrite(targetMotor.enablePin, 0);
}