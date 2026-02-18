#ifndef PID_H
#define PID_H

class PIDController {
 public:
  PIDController(float kP, float kI, float kD);
  PIDController(float kP, float kI, float kD, bool debug);
  float update(float error);
  void reset();

 private:
  bool debug;
  float _kP, _kI, _kD;
  float _previousError = 0.0f;
  float _integral = 0.0f;
};

#endif // PID_H
