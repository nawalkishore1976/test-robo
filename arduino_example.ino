#include "imu.h"

void setup() {
  Serial.begin(9600);
  
  if (initIMU()) {
    Serial.println("IMU initialized successfully");
  } else {
    Serial.println("Failed to initialize IMU");
  }
}

void loop() {
  float heading = getHeading();
  
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print("°, Status: ");
  Serial.println(getCalibrationStatus());
  
  delay(100);
}