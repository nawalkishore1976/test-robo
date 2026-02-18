#include "ExitCondition.h"
#include <math.h>

/**
 * @brief Create a new Exit Condition
 *
 * @param range the range where the countdown is allowed to start
 * @param time how much time to wait while in range before exiting (milliseconds)
 */
ExitCondition::ExitCondition(const float range, const unsigned long time)
    : range(range), time(time), startTime(0), done(false) {}

/**
 * @brief whether the exit condition has been met
 *
 * @return true exit condition met
 * @return false exit condition not met
 */
bool ExitCondition::getExit() const { 
  return done; 
}

/**
 * @brief update the exit condition
 *
 * @param input the input for the exit condition
 * @return true exit condition met
 * @return false exit condition not met
 */
bool ExitCondition::update(const float input) {
  const unsigned long curTime = millis();  // Arduino's millis() function
  
  if (fabs(input) > range) {
    startTime = 0;  // Reset timer
  } else if (startTime == 0) {
    startTime = curTime;  // Start timer
  } else if (curTime >= startTime + time) {
    done = true;  // Time elapsed, exit condition met
  }
  
  return done;
}

/**
 * @brief reset the exit condition timer
 *
 */
void ExitCondition::reset() {
  startTime = 0;
  done = false;
}

/**
 * @brief sets the exit conditions
 *
 * @param range the range where the countdown is allowed to start
 * @param time how much time to wait while in range before exiting (milliseconds)
 */
void ExitCondition::setExit(const float range, const unsigned long time) {
  this->range = range;
  this->time = time;
}
