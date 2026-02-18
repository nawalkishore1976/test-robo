#include "PID.h"

PIDController::PIDController(float kP, float kI, float kD)
    : _kP(kP), _kI(kI), _kD(kD), debug(false) {}

PIDController::PIDController(float kP, float kI, float kD, bool debug)
    : _kP(kP), _kI(kI), _kD(kD), debug(debug) {}

float PIDController::update(float error) {
  _integral += error;
  float derivative = error - _previousError;

  float kPOutput = _kP * error;
  float kIOutput = _kI * _integral;
  float kDOutput = _kD * derivative;

  if (debug) {
    Serial.print("err: ");
    Serial.print(error);
    Serial.print(", kP: ");
    Serial.print(kPOutput);
    Serial.print(", kI: ");
    Serial.print(kIOutput);
    Serial.print(", kD: ");
    Serial.println(kDOutput);
  }

  float output = kPOutput + kIOutput + kDOutput;

  _previousError = error;
  return output;
}

void PIDController::reset() {
  _previousError = 0.0f;
  _integral = 0.0f;

  if (debug) {
    Serial.println("PID reset");
  }
}
