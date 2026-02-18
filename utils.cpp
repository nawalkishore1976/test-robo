#include "utils.h"
#include <math.h>

/**
 * Calculates the error between two angles.
 * BY DEFAULT, EXPECTS ANGLES IN DEGREES
 */
float utils::angleError(float angle1, float angle2, bool radians) {
  float divisor = radians ? (2.0f * M_PI) : 360.0f;
  float difference = angle1 - angle2;
  
  // Arduino-compatible remainder calculation
  while (difference > divisor / 2.0f) {
    difference -= divisor;
  }
  while (difference < -divisor / 2.0f) {
    difference += divisor;
  }
  
  return difference;
}

/**
 * Returns the angle in the range [0, 2PI]
 */
float utils::angleSquish(float angle, bool radians) {
  float circle = radians ? (2.0f * M_PI) : 360.0f;

  while (angle < 0.0f)
    angle += circle;
  return fmod(angle, circle);
}

/**
 * Converts degrees to radians
 */
float utils::degToRad(float deg) { 
  return deg * M_PI / 180.0f; 
}

/**
 * Converts radians to degrees
 */
float utils::radToDeg(float rad) { 
  return rad * 180.0f / M_PI; 
}

/**
 * @brief Slew rate limiter
 *
 * @param target target value
 * @param current current value
 * @param maxChange maximum change. No maximum if set to 0
 * @return float - the limited value
 */
float utils::slew(float target, float current, float maxChange) {
  float change = target - current;
  if (maxChange == 0.0f)
    return target;
  if (change > maxChange)
    change = maxChange;
  else if (change < -maxChange)
    change = -maxChange;
  return current + change;
}

/**
 * @brief Get the signed curvature of a circle that intersects the first pose
 * and the second pose
 *
 * @note The circle will be tangent to the theta value of the first pose
 * @note The curvature is signed. Positive curvature means the circle is going
 * clockwise, negative means counter-clockwise
 * @note Theta has to be in radians and in standard form. That means 0 is right
 * and increases counter-clockwise
 *
 * @param pose the first pose
 * @param other the second pose
 * @return float curvature
 */
float utils::getCurvature(Position pose, Position other) {
  // calculate whether the pose is on the left or right side of the circle
  float side = utils::sgn(sin(pose.theta) * (other.x - pose.x) -
                          cos(pose.theta) * (other.y - pose.y));
  // calculate center point and radius
  float a = -tan(pose.theta);
  float c = tan(pose.theta) * pose.x - pose.y;
  float x = fabs(a * other.x + other.y + c) / sqrt((a * a) + 1.0f);
  float d = sqrt((other.x - pose.x) * (other.x - pose.x) + 
                 (other.y - pose.y) * (other.y - pose.y));

  // return curvature
  return side * ((2.0f * x) / (d * d));
}
