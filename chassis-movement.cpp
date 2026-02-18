#include "ExitCondition.h"
#include "PID.h"
#include "robot.h"
#include "utils.h"

PIDController angularPID(4.0f, 0.0f, 20.0f);
ExitCondition angularLargeExit(4.0f, 300);
ExitCondition angularSmallExit(1.0f, 100);

/**
 * @brief Turns to a given angle
 */
void robot::turnTo(float degrees) {
  degrees = utils::angleSquish(degrees - 90.0f, false);

  // reset angular controllers/exit conditions
  angularPID.reset();
  angularLargeExit.reset();
  angularSmallExit.reset();

  while (!angularLargeExit.getExit() && !angularSmallExit.getExit()) {
    // calculate error in degrees
    // this is because degrees makes it easier to tune the PID
    // as errors are larger
    Position pose = getPosition(true);
    float error = utils::angleError(pose.theta, degrees);

    // calculate the output from the PID
    float power = angularPID.update(error);
    angularLargeExit.update(error);
    angularSmallExit.update(error);

    // constrain the output using Arduino's built-in function
    power = constrain(power, -99.0f, 99.0f);
    
    // Optional debug output
    /*
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(", Power: ");
    Serial.println(power);
    */
    
    robot::moveVelocity((int)(-power), (int)(power));

    delay(10);  // Arduino's delay function
  }

  // stop the drivetrain
  move(0, 0);
}
