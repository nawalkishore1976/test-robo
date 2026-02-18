#include "RoboMotor.h"

// Create motor objects
// RoboMotor(forwardPin, backwardPin, pwmPin)
RoboMotor leftMotor(2, 3, 5);    // IN1=2, IN2=3, ENA=5
RoboMotor rightMotor(4, 7, 6);   // IN3=4, IN4=7, ENB=6

void setup() {
  Serial.begin(9600);
  Serial.println("RoboMotor Motor Driver Test");
}

void loop() {
  // Move forward at 50% speed
  leftMotor.spin(RoboMotor::Forwards, 50.0f);
  rightMotor.spin(RoboMotor::Forwards, 50.0f);
  delay(2000);
  
  // Stop motors
  leftMotor.stop();
  rightMotor.stop();
  delay(1000);
  
  // Move backward at 30% speed
  leftMotor.spin(RoboMotor::Backwards, 30.0f);
  rightMotor.spin(RoboMotor::Backwards, 30.0f);
  delay(2000);
  
  // Stop motors
  leftMotor.stop();
  rightMotor.stop();
  delay(1000);
}