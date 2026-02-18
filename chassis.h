#pragma once

#include "Motor.h"
#include "position.h"

struct LRT {
  float left, right, theta;
};

namespace robot {

extern SpeedController driveLeftController;
extern SpeedController driveRightController;
extern LRT *velocity;

/**
 * Handles one odometry tick
 */
void doOdometryTick();

/**
 * Runs odometry in a loop
 */
void odometryTask();

void initializeOdometry();

Position getPosition(bool degrees = false, bool standardPos = false);

void move(int left, int right);
void moveVelocity(int left, int right);

/**
 * Follow a path using fixed-size array instead of vector
 * @param pathPoints - pointer to array of Position points
 * @param pathSize - number of points in the path array
 * @param lookahead - lookahead distance
 * @param endTime - end time for path following
 * @param remainingDistance - remaining distance to travel
 */
void follow(Position* pathPoints, int pathSize, float lookahead, int endTime,
            float remainingDistance);

void turnTo(float angle);

void setPose(const Position &newState, bool setTheta = false);

} // namespace robot
