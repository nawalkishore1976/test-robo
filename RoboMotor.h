#ifndef ROBOMOTOR_H
#define ROBOMOTOR_H

#include <Arduino.h>

enum Direction : uint8_t {
  Forwards = 0,
  Backwards = 1
};

class RoboMotor {
public:
  RoboMotor(uint8_t enableFwd, uint8_t enableRev, uint8_t pwm);
  
  void spin(bool forwards, float speed);
  void spin(Direction direction, float speed);
  void stop();
  
  // Memory-efficient interpolation functions
  void interpolateMotorControl(float startSpeed, float endSpeed, 
                              uint8_t steps, uint8_t currentStep);
  void rampToSpeed(float targetSpeed, uint8_t rampSteps);

private:
  uint8_t enableFwd, enableRev, pwm;  // Use uint8_t to save memory
};

#endif
