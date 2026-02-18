// The implementation below is mostly based off of
// the document written by Dawgma
// Here is a link to the original document
// https://www.chiefdelphi.com/uploads/default/original/3X/b/e/be0e06de00e07db66f97686505c3f4dde2e332dc.pdf

// Code inspired by Lemlib's implemenation
// slight modifications made to optimize performance and make it work with our
// custom lib

// set DEBUG to true to enable debug logging
#include "robot.h"
#include "utils.h"
#include <vector>
#include <cmath>
#include <algorithm>
#define PURE_PURSUIT_DEBUG true

#include "config.h"

// Arduino doesn't have M_PI_2 by default
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

/**
 * @brief find the closest point on the path to the robot
 *
 * @param pose the current pose of the robot
 * @param path the path to follow
 * @return int index to the closest point
 */
int findClosest(Position pose, std::vector<Position> &path, int prevIdx) {
  int closestPoint = 0;
  float closestDist = 1000000;
  float dist;

  // loop through all path points
  for (int i = 0; i < path.size(); i++) {
    dist = pose.distance(path.at(i));
    if (dist < closestDist) { // new closest point
      closestDist = dist;
      closestPoint = i;
    }
  }

  return closestPoint;
}

/**
 * @brief Function that finds the intersection point between a circle and a line
 *
 * @param p1 start point of the line
 * @param p2 end point of the line
 * @param pos position of the robot
 * @param path the path to follow
 * @return float how far along the line the
 */
float circleIntersect(const Position &p1, const Position &p2,
                      const Position &pose, float lookaheadDist) {
  // calculations
  // uses the quadratic formula to calculate intersection points
  Position d = p2 - p1;
  Position f = p1 - pose;
  float a = d * d;
  float b = 2 * (f * d);
  float c = (f * f) - lookaheadDist * lookaheadDist;
  float discriminant = b * b - 4 * a * c;

  // if a possible intersection was found
  if (discriminant >= 0) {
    discriminant = sqrt(discriminant);
    float t1 = (-b - discriminant) / (2 * a);
    float t2 = (-b + discriminant) / (2 * a);

    // prioritize further down the path
    if (t2 >= 0 && t2 <= 1)
      return t2;
    else if (t1 >= 0 && t1 <= 1)
      return t1;
  }

  // no intersection found
  return -1;
}

/**
 * @brief returns the lookahead point
 *
 * @param lastLookahead - the last lookahead point
 * @param pose - the current position of the robot
 * @param path - the path to follow
 * @param lookaheadDist - the lookahead distance of the algorithm
 */
Position lookaheadPoint(const Position &lastLookahead, const Position &pose,
                        std::vector<Position> &path, int closest,
                        float lookaheadDist) {
  // find the furthest lookahead point on the path

  // optimizations applied:
  // only consider intersections that have an index greater than or equal to the
  // point closest to the robot and intersections that have an index greater
  // than or equal to the index of the last lookahead point
  const int start = max(closest, int(lastLookahead.theta));
  for (int i = start; i < path.size() - 1; i++) {
    Position lastPathPose = path.at(i);
    Position currentPathPose = path.at(i + 1);

    float t =
        circleIntersect(lastPathPose, currentPathPose, pose, lookaheadDist);

    if (t != -1) {
      Position lookahead = lastPathPose.lerp(currentPathPose, t);
      lookahead.theta = i;
      return lookahead;
    }
  }

  // robot deviated from path, use last lookahead point
  return lastLookahead;
}

/**
 * @brief Get the curvature of a circle that intersects the robot and the
 * lookahead point
 *
 * @param pos the position of the robot
 * @param heading the heading of the robot
 * @param lookahead the lookahead point
 * @return float curvature
 */
float findLookaheadCurvature(const Position &pose, float heading,
                             const Position &lookahead) {
  // calculate whether the robot is on the left or right side of the circle
  float side = utils::sgn(sin(heading) * (lookahead.x - pose.x) -
                          cos(heading) * (lookahead.y - pose.y));
  // calculate center point and radius
  float a = -tan(heading);
  float c = tan(heading) * pose.x - pose.y;
  float x =
      abs(a * lookahead.x + lookahead.y + c) / sqrt((a * a) + 1);
  float d = sqrt(pow(lookahead.x - pose.x, 2) + pow(lookahead.y - pose.y, 2));

  // return curvature
  return side * ((2 * x) / (d * d));
}

/**
 * @brief Move the robot along a path
 *
 * @param pathPoints list of points to follow
 * @param lookahead the lookahead distance. Units in inches. Larger values will
 * make the robot move faster but will follow the path less accurately
 * @param timeout the maximum time the robot can spend moving in milliseconds.
 * @param forwards whether the robot should follow the path going forwards. true
 * by default
 * @param async whether the function should be run asynchronously. true by
 * default
 */
void robot::follow(std::vector<Position> &pathPoints, float lookahead,
                     int endTime, float remainingDistance) {
  bool forwards = true;
  Position pose = robot::getPosition();
  Position lastPose = pose;
  Position lookaheadPose(0, 0, 0);
  Position lastLookahead = pathPoints.at(0);
  lastLookahead.theta = 0;
  float curvature;
  float targetVel;
  float prevLeftVel = 0;
  float prevRightVel = 0;
  int closestPoint = 0;
  float leftInput = 0;
  float rightInput = 0;
  float prevVel = 0;
  
  unsigned long startTime = millis(); // Arduino timing

  // loop until the robot is within the end tolerance
  for (int i = 0; i < endTime / 10; i++) {
    // get the current position of the robot
    pose = getPosition();
    if (!forwards)
      pose.theta -= M_PI;

    // Serial.print("pose: "); Serial.print(pose.x); 
    // Serial.print(", "); Serial.print(pose.y);
    // Serial.print(", "); Serial.println(utils::radToDeg(pose.theta));

    // find the closest point on the path to the robot
    closestPoint = findClosest(pose, pathPoints, closestPoint);
    // if the robot is at the end of the path, then stop
    if (pathPoints.at(closestPoint).theta == 0)
      break;

    // find the lookahead point
    lookaheadPose = lookaheadPoint(lastLookahead, pose, pathPoints,
                                   closestPoint, lookahead);
    lastLookahead = lookaheadPose; // update last lookahead position

    // get the curvature of the arc between the robot and the lookahead point
    float curvatureHeading = M_PI_2 - pose.theta;
    curvature = findLookaheadCurvature(pose, curvatureHeading, lookaheadPose);

    // find target velocity
    float distanceTillEnd =
        pose.distance(pathPoints.at(closestPoint)) + remainingDistance;
    for (int i = closestPoint; i < pathPoints.size() - 1; i++) {
      distanceTillEnd += pathPoints.at(i).distance(pathPoints.at(i + 1));
    }

    float timeTillEnd = endTime - (millis() - startTime); // milliseconds
    timeTillEnd /= 1000.0f;                               // seconds
    timeTillEnd /= 60.0f;                                 // minutes

    targetVel = (distanceTillEnd / (6.5f * M_PI)) / timeTillEnd; // rpm

    // if < 0, max speed time
    if (timeTillEnd < 0)
      targetVel = 300;
    targetVel = utils::slew(targetVel, prevVel, 4);

    // prevent stalling
    if (targetVel < 10)
      targetVel = 10;

    prevVel = targetVel;

    // calculate target left and right velocities
    float targetLeftVel = targetVel * (2 + curvature * DRIVE_TRACK_WIDTH) / 2;
    float targetRightVel = targetVel * (2 - curvature * DRIVE_TRACK_WIDTH) / 2;

    // ratio the speeds to respect the max speed
    float ratio = max(abs(targetLeftVel), abs(targetRightVel)) / 127.0f;
    if (ratio > 1) {
      targetLeftVel /= ratio;
      targetRightVel /= ratio;
    }

    // update previous velocities
    prevLeftVel = targetLeftVel;
    prevRightVel = targetRightVel;

    // Serial.print("left: "); Serial.print(targetLeftVel);
    // Serial.print(", right: "); Serial.println(targetRightVel);

    // move the drivetrain
    if (forwards) {
      robot::moveVelocity(targetLeftVel, targetRightVel);
    } else {
      robot::moveVelocity(-targetRightVel, -targetLeftVel);
    }

    delay(20); // Arduino delay function
  }

  robot::moveVelocity(0, 0);
}
