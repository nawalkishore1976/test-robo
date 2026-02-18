#pragma once

#include "robot.h"

namespace utils {

/**
 * Calculates the error between two angles.
 * BY DEFAULT EXPECTS ANGLES IN DEGREES
 */
float angleError(float angle1, float angle2, bool radians = false);

/**
 * Returns the angle in the range [0, 2PI]
 */
float angleSquish(float angle, bool radians = true);

/**
 * Converts degrees to radians
 */
float degToRad(float deg);

/**
 * Converts radians to degrees
 */
float radToDeg(float rad);

/**
 * @brief Slew rate limiter
 *
 * @param target target value
 * @param current current value
 * @param maxChange maximum change. No maximum if set to 0
 * @return float - the limited value
 */
float slew(float target, float current, float maxChange);

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
float getCurvature(Position pose, Position other);

template <typename T> constexpr T sgn(T value) { return value < 0 ? -1 : 1; }

}; // namespace utils
