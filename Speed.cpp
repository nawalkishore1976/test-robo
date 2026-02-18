#include "Speed.h"

// Simple sign function for Arduino
float sgn(float value) {
  if (value > 0.0) return 1.0;
  if (value < 0.0) return -1.0;
  return 0.0;
}

SpeedController::SpeedController(float Kp, float Ki, float Kd, float Ks,
                                 float Kv, float Ka)
    : Kp(Kp), Ki(Ki), Kd(Kd), Ks(Ks), Kv(Kv), Ka(Ka), 
      target(0), previousError(0), integral(0) {
}

void SpeedController::setTargetVelocity(float target) { 
  this->target = target; 
}

float SpeedController::update(float actual) {
  // calculate the error
  float error = target - actual;

  // --- PID stuff
  integral += error;
  float derivative = error - previousError;

  float kPOutput = Kp * error;
  float kIOutput = Ki * integral;
  float kDOutput = Kd * derivative;
  float output = kPOutput + kIOutput + kDOutput;
  // ---

  // --- FF stuff
  float expected = Ks * sgn(target) + Kv * target + Ka * derivative;
  // ---

  previousError = error;

  // combine FF and PID
  return output + expected;
}
