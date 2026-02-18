https://www.reddit.com/user/Robobloqjerrywang/
https://github.com/slgrobotics/q-scout

C:\Program Files\MyQode\resources\remote\sources

https://github.com/aretche/recursos-robobloq/tree/main/software/android

ICM-20948 Sensor Module 9 Axis MEMS Motion Tracking Device Sensor Low Power Integrated Circuits ICM20948


SN74LV4051
MBR0520LT1G
DRV8833


struct error_tracking {
    float ideal_cardinal;    // 0, 90, 180, 270
    float heading_error;     
    
    long target_x;           // Ideal global X in steps
    long target_y;           // Ideal global Y in steps
    long x_error;            // Carryover for X
    long y_error;            // Carryover for Y
};


void executeSmartTurn(float requestedDegrees, int requestedDir) {
    // 1. Capture where we are vs where we should be
    float currentGyro = getGlobalZ();
    
    // 2. Determine the "Real" error from the last move
    // Example: If at 2.0 degrees, ideal is 0.0, error is -2.0
    float last_move_error = curr.error_tracking.ideal_cardinal - currentGyro;

    // 3. Update the global "Ideal" for the NEXT move
    // If we are at 0 and turn 90 Right (dir 1), next ideal is 90
    curr.error_tracking.ideal_cardinal += (requestedDegrees * requestedDir);
    
    // 4. Calculate the adjusted turn magnitude
    // Example: 90 degrees requested + (-2.0 error) = 88.0 actual degrees
    float adjustedDegrees = requestedDegrees + (last_move_error * requestedDir);

    // 5. Safety: If adjustment makes degrees negative, flip direction
    int finalDir = requestedDir;
    if (adjustedDegrees < 0) {
        adjustedDegrees = abs(adjustedDegrees);
        finalDir *= -1; // Reverse the turn direction
    }

    // 6. Execute using your specific function
    Turn(adjustedDegrees, finalDir);
}



void executeSmartMove(long steps, int dir) {
    long adjustment = 0;
    float angle = curr.error_tracking.ideal_cardinal;

    // A. Pick the error based on current axis
    if (angle == 0 || angle == 180) {
        adjustment = curr.error_tracking.y_error;
        curr.error_tracking.y_error = 0;
    } else {
        adjustment = curr.error_tracking.x_error;
        curr.error_tracking.x_error = 0;
    }

    // B. Apply error to the steps
    // If we overshot last time, adjustment will be negative, reducing finalSteps
    long finalSteps = steps + adjustment;
    if (finalSteps < 0) finalSteps = 0; // Prevent negative distance

    // C. Move the robot
    MovePulse(finalSteps, dir);

    // D. Update Global Position Tracker
    // We use the 'intended' steps to keep the grid perfect
    long moveDist = steps * dir; 
    if (angle == 0)         curr.error_tracking.target_y += moveDist;
    else if (angle == 90)   curr.error_tracking.target_x += moveDist;
    else if (angle == 180)  curr.error_tracking.target_y -= moveDist;
    else if (angle == 270)  curr.error_tracking.target_x -= moveDist;

    // E. Calculate new error for next time
    // Actual distance is where the encoders actually stopped
    long actualTotal = (curr.left_encoder + curr.right_encoder) / 2;
    
    if (angle == 0 || angle == 180) {
        curr.error_tracking.y_error = curr.error_tracking.target_y - (actualTotal * dir);
    } else {
        curr.error_tracking.x_error = curr.error_tracking.target_x - (actualTotal * dir);
    }
}

