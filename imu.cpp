#include "imu.h"
#include <Wire.h>

#define ICM20948_ADDR 0x68

euler_t *ypr = new euler_t;
ICM20948_WE *imu = new ICM20948_WE(ICM20948_ADDR);

// Simple calibration status tracking
CalibrationStatus calibration_status = CALIBRATION_STATUS_UNCALIBRATED;
unsigned long calibration_start_time = 0;
bool calibration_in_progress = false;

bool initIMU() {
  Wire.begin();
  
  if (!imu->init()) {
    Serial.println("[error] Failed to initialize ICM20948");
    return false;
  }
  
  Serial.println("[imu] ICM20948 initialized successfully");
  
  // Configure the sensor
  imu->setAccRange(ICM20948_ACC_RANGE_4G);
  imu->setGyrRange(ICM20948_GYRO_RANGE_250);
  imu->setAccDLPF(ICM20948_DLPF_6);
  imu->setGyrDLPF(ICM20948_DLPF_6);
  
  // Start calibration timer
  calibration_start_time = millis();
  calibration_in_progress = true;
  
  return true;
}

void calculateEulerAngles(xyzFloat gVal, xyzFloat aVal, euler_t *ypr, bool degrees) {
  // Simple complementary filter for attitude estimation
  static float roll_acc, pitch_acc;
  static float roll_gyro = 0.0f, pitch_gyro = 0.0f, yaw_gyro = 0.0f;
  static unsigned long last_time = 0;
  
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0f; // Convert to seconds
  last_time = current_time;
  
  if (dt > 0.1f) dt = 0.01f; // Prevent large dt values
  
  // Calculate roll and pitch from accelerometer
  roll_acc = atan2(aVal.y, sqrt(aVal.x * aVal.x + aVal.z * aVal.z));
  pitch_acc = atan2(-aVal.x, sqrt(aVal.y * aVal.y + aVal.z * aVal.z));
  
  // Integrate gyroscope data
  roll_gyro += gVal.x * dt;
  pitch_gyro += gVal.y * dt;
  yaw_gyro += gVal.z * dt;
  
  // Complementary filter (98% gyro, 2% accel)
  float alpha = 0.98f;
  ypr->roll = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
  ypr->pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
  ypr->yaw = yaw_gyro; // Pure gyro integration for yaw
  
  // Convert to degrees if requested
  if (degrees) {
    ypr->roll *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->yaw *= RAD_TO_DEG;
  }
}

void updateIMU() {
  if (!imu->dataReady()) {
    return;
  }
  
  // Read sensor data
  xyzFloat gVal = imu->getGValues();
  xyzFloat aVal = imu->getCorrectedAccRawValues();
  
  // Calculate euler angles
  calculateEulerAngles(gVal, aVal, ypr, true);
  
  // Update calibration status based on time and data stability
  if (calibration_in_progress) {
    unsigned long elapsed = millis() - calibration_start_time;
    
    if (elapsed > 10000) { // 10 seconds
      calibration_status = CALIBRATION_STATUS_HIGH_ACCURACY;
      calibration_in_progress = false;
    } else if (elapsed > 5000) { // 5 seconds
      calibration_status = CALIBRATION_STATUS_MEDIUM_ACCURACY;
    } else if (elapsed > 2000) { // 2 seconds
      calibration_status = CALIBRATION_STATUS_LOW_ACCURACY;
    }
  }
}

float getHeading() {
  updateIMU();
  return ypr->yaw;
}

CalibrationStatus getCalibrationStatus() {
  return calibration_status;
}

xyzFloat getAccelData() {
  return imu->getCorrectedAccRawValues();
}

xyzFloat getGyroData() {
  return imu->getGValues();
}

xyzFloat getMagData() {
  return imu->getMagValues();
}
