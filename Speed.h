/**
 * Manage output voltage for DC motors
 * Consists of a PID controller + feedforward
 *
 * Designed for multicore use:
 *  - 1 core sets output, other updates the PID/ff
 *
 * @author - Derock X (derock@derock.dev)
 * @license - MIT
 */

#ifndef SPEED_H
#define SPEED_H

class SpeedController {

public:
  SpeedController(float Kp, float Ki, float Kd, 
    float Ks = 0, float Kv = 0,
                  float Ka = 0);

  void setTargetVelocity(float target);
  float update(float actual);

private:
  // PID Constants
  float Kp, Ki, Kd;

  // Feedforward constants
  float Ks, Kv, Ka;

  // PID/FF runtime stuff
  float target;
  float previousError;
  float integral;
};

#endif
