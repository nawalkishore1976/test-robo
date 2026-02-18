#include "Motor.h"
#include "robot.h"
#include "config.h"
#include "imu.h"
#include "utils.h"

#include <cmath>
#include <cstdio>

// Arduino encoder library - you may need to install this
// Library: "Encoder" by Paul Stoffregen
#include <Encoder.h>

// Create encoder objects - adjust pin numbers as needed
Encoder leftEncoder(2, 3);   // pins 2 and 3 for left encoder
Encoder rightEncoder(4, 5);  // pins 4 and 5 for right encoder

SpeedController robot::driveLeftController =
    SpeedController(1.5, 0, 1.25, 15, 1.7, 0);

SpeedController robot::driveRightController =
    SpeedController(1.5, 0, 1.25, 15, 1.7, 0);

// Arduino doesn't have built-in mutex, using simple flag for basic protection
volatile bool odometryLock = false;

#define ODOM_DEBUG false

LRT resetValues = {0, 0, 0};
LRT prevSensors = {0, 0, 0};

Position *state = new Position({0, 0, 0});
LRT *robot::velocity = new LRT({0, 0, 0});

// Simple mutex replacement for Arduino
bool acquireLock(unsigned long timeoutMs) {
  unsigned long startTime = millis();
  while (odometryLock) {
    if (millis() - startTime > timeoutMs) {
      return false;
    }
    delay(1);
  }
  odometryLock = true;
  return true;
}

void releaseLock() {
  odometryLock = false;
}

/**
 * Normalizes the sensor data to account for factors such as
 * gear ratio, encoder ticks, etc.
 *
 * @note - marked inline to reduce overhead of function calls
 *       - marked static so only accessible in odom.cpp
 *
 * @param rawCount - raw encoder count
 */
inline static double readSensorData(long rawCount) {
  // count time by 14-counts-per-revolution and the approximate 20.4:1 gear
  // ratio (for extra credit, the exact ratio is 244904:12000 or 30613:1500).
  
  // Serial.print("[debug] [raw] encoder counts: ");
  // Serial.println(rawCount);

  // return (float)rawCount / 14 / (244984.0f / 12000) * 6.5f * M_PI;
  return (float)rawCount / 1060.5 * 6.5f * M_PI;
}

void robot::doOdometryTick() {
  // lock mutex
  if (!acquireLock(500)) {
    Serial.println("WARN ! Tracking failed to acquire mutex after 500ms.");
    return;
  };

  // 1. Store the current encoder values
  double left = readSensorData(leftEncoder.read());
  double right = readSensorData(rightEncoder.read());

  // 2. Calculate delta values
  double dL = left - prevSensors.left;
  double dR = right - prevSensors.right;

  // 3. Update the previous values
  prevSensors.left = left;
  prevSensors.right = right;

  // Calculate motor velocities
  velocity->left = dL / 0.01f;
  velocity->right = dR / 0.01f;

  // 5. Calculate new orientation
  double newTheta = getHeading() * M_PI / 180.0f;
  newTheta -= resetValues.theta;

  // flip
  newTheta = 2.0f * M_PI - newTheta;
  newTheta = utils::angleSquish(newTheta, true);

  // 6. Calculate change in orientation
  double dTheta = newTheta - state->theta;
  double d = (dL + dR) / 2;

  // 7. Update the state
  state->y += d * cos(state->theta + dTheta / 2);
  state->x += d * sin(state->theta + dTheta / 2);
  state->theta = newTheta;

  // unlock mutex
  releaseLock();
}

void robot::odometryTask() {
  int i = 0;

  while (true) {
    doOdometryTick();

    float leftSpeed =
        robot::driveLeftController.update(robot::velocity->left);
    float rightSpeed =
        robot::driveRightController.update(robot::velocity->right);

    robot::move(leftSpeed, rightSpeed);

    if (++i == 10) {
      i = 0;
      auto pos = robot::getPosition(true);
      Serial.print("[debug] x: "); Serial.print(pos.x);
      Serial.print(", y: "); Serial.print(pos.y);
      Serial.print(", h: "); Serial.println(pos.theta);
    }

    delay(10); // Arduino delay function
  }
}

void robot::setPose(const Position &newState, bool setTheta) {
  // Wait for lock
  while (!acquireLock(0)) {
    delay(1);
  }

  state->x = newState.x;
  state->y = newState.y;

  if (setTheta) {
    resetValues.theta = getHeading() * M_PI / 180;
  }

  Serial.print("[odom] Position reset to (");
  Serial.print(state->x); Serial.print(", ");
  Serial.print(state->y); Serial.print(", ");
  Serial.print(state->theta * 180 / M_PI); Serial.println(")");

  releaseLock();
}

void robot::initializeOdometry() {
  // Initialize encoders
  leftEncoder.write(0);
  rightEncoder.write(0);

  // Initialize heading calibration
  for (int i = 0; i < 100; i++) {
    getHeading();
    delay(10);
  }

  resetValues.theta = getHeading() * M_PI / 180;
  Serial.print("[debug] reset theta: ");
  Serial.println(resetValues.theta);
}

Position robot::getPosition(bool degrees, bool standardPos) {
  // Wait for lock
  while (!acquireLock(0)) {
    delay(1);
  }

  // get the state
  Position returnState =
      degrees ? Position({state->x, state->y,
                          static_cast<float>(state->theta * (180 / M_PI))})
              : *state;

  releaseLock();

  // bearing -> standard form
  if (standardPos) {
    returnState.theta = utils::angleSquish(M_PI_2 - returnState.theta);
  }

  return returnState;
}

void robot::moveVelocity(int left, int right) {
  robot::driveLeftController.setTargetVelocity(left);
  robot::driveRightController.setTargetVelocity(right);
}

void robot::move(int left, int right) {
  // get direction
  bool leftFwd = left > 0;
  bool rightFwd = right > 0;

  // now take abs
  left = abs(left);
  right = abs(right);

  // and convert from [0, 127] to [0, 100]
  float leftSpeed = (float)left / 127 * 100;
  float rightSpeed = (float)right / 127 * 100;

  // now move
  driveLeft.spin(leftFwd, leftSpeed > 100 ? 100 : leftSpeed);
  driveRight.spin(rightFwd, rightSpeed > 100 ? 100 : rightSpeed);
}
