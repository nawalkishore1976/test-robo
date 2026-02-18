#include "RoboMotor.h"

RoboMotor::RoboMotor(int enableFwd, int enableRev, int pwm)
    : enableFwd(enableFwd), enableRev(enableRev), pwm(pwm) {
  
  // Initialize direction pins as outputs
  pinMode(enableFwd, OUTPUT);
  pinMode(enableRev, OUTPUT);
  
  // Initialize PWM pin as output
  pinMode(pwm, OUTPUT);
  
  // Start with motor stopped
  digitalWrite(enableFwd, LOW);
  digitalWrite(enableRev, LOW);
  analogWrite(pwm, 0);
  
  // Debug output
  Serial.print("[RoboMotor] Initialized - Fwd pin: ");
  Serial.print(enableFwd);
  Serial.print(", Rev pin: ");
  Serial.print(enableRev);
  Serial.print(", PWM pin: ");
  Serial.println(pwm);
}

void RoboMotor::spin(bool forwards, float speed) {
  spin(forwards ? Forwards : Backwards, speed);
}

void RoboMotor::spin(Direction direction, float speed) {
  // Constrain speed to 0-100%
  speed = constrain(speed, 0.0f, 100.0f);
  
  // Convert percentage to PWM value (0-255)
  int pwmValue = (int)((speed * 255.0f) / 100.0f);
  
  // Set direction pins
  if (direction == Forwards) {
    digitalWrite(enableFwd, HIGH);
    digitalWrite(enableRev, LOW);
  } else {
    digitalWrite(enableFwd, LOW);
    digitalWrite(enableRev, HIGH);
  }
  
  // Set speed
  analogWrite(pwm, pwmValue);
  
  /* Debug output - uncomment if needed
  Serial.print("Motor: ");
  Serial.print(direction == Forwards ? "FWD" : "REV");
  Serial.print(", Speed: ");
  Serial.print(speed);
  Serial.print("%, PWM: ");
  Serial.println(pwmValue);
  */
}

void RoboMotor::stop() {
  // Turn off both direction pins
  digitalWrite(enableFwd, LOW);
  digitalWrite(enableRev, LOW);
  
  // Set PWM to 0
  analogWrite(pwm, 0);
}
