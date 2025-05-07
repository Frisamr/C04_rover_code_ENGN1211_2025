// this is an [include guard](https://en.wikipedia.org/wiki/Include_guard)
#ifndef MOTOR_CTL_H
#define MOTOR_CTL_H

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

void initMotor(const DCMotor& targetMotor);

void setMotorSpeed(const DCMotor& targetMotor, int speed);

#endif