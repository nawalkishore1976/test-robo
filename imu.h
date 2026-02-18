#ifndef IMU_H
#define IMU_H

#include <ICM20948_WE.h>

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif

enum CalibrationStatus {
  CALIBRATION_STATUS_UNCALIBRATED = 0,
  CALIBRATION_STATUS_LOW_ACCURACY = 1,
  CALIBRATION_STATUS_MEDIUM_ACCURACY = 2,
  CALIBRATION_STATUS_HIGH_ACCURACY = 3,
};

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

extern euler_t *ypr;

// Initialize the IMU
bool initIMU();

// Get calibration status (simplified for ICM20948)
CalibrationStatus getCalibrationStatus();

// Convert accelerometer and gyro data to euler angles
void calculateEulerAngles(xyzFloat gVal, xyzFloat aVal, euler_t *ypr, bool degrees = false);

// Get current heading (yaw) angle
float getHeading();

// Update IMU data
void updateIMU();

// Get raw sensor data
xyzFloat getAccelData();
xyzFloat getGyroData();
xyzFloat getMagData();

#endif // IMU_H
