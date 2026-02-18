#include <Arduino.h>
#include "common_macro.h"
#include "RoboMotor.h"
#include "robot.h"
#include "config.h"

// Simple point structure - no need for packing
struct Point {
  float x, y;      // Just use floats
  int speed;
};

// Simple path array in regular memory
Point robotPath[] = {
  {0.0, 0.0, 50},
  {1.0, 0.0, 50},
  {0.0, 0.0, 50},
  {0.0, 1.0, 50},
  {1.0, 1.0, 50},
  {0.0, 1.0, 50},
  {0.0, 2.0, 50},
  {1.0, 2.0, 50},
  {1.0, 3.0, 50},
  {3.0, 3.0, 50},
  {3.0, 2.0, 50},
  {2.16, 2.0, 0}
};

RoboMotor driveRight(6, 5, 4);
RoboMotor driveLeft(7, 8, 9);

int currentPointIndex = 0;
int pathLength = sizeof(robotPath) / sizeof(Point);
bool missionActive = false;
uint32_t startTime;

void setup() {
  Serial.begin(115200);
  DEBUG_PRINTLN(F("=== Robot Tour Controller ==="));
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  
  robot::initialize();
  
  DEBUG_PRINTLN(F("Press button to start..."));
  
  while (digitalRead(2) == HIGH) {
    digitalWrite(LED_BUILTIN, millis() % 1000 < 500);
    delay(10);
  }
  
  delay(500);
  startTime = millis();
  missionActive = true;
  DEBUG_PRINTLN(F("Mission starting!"));
}

void loop() {
  if (!missionActive) {
    digitalWrite(LED_BUILTIN, millis() % 2000 < 1000);
    delay(100);
    return;
  }
  
  if (currentPointIndex < pathLength) {
    Point current = robotPath[currentPointIndex];
    
    DEBUG_PRINTF(F("Moving to point "), currentPointIndex);
    DEBUG_PRINTF(F(": ("), current.x);
    DEBUG_PRINTF(F(", "), current.y);
    DEBUG_PRINTLN(F(")"));
    
    robot::moveTo(current.x, current.y, current.speed);
    
    currentPointIndex++;
  } else {
    // Mission complete
    driveLeft.stop();
    driveRight.stop();
    
    DEBUG_PRINTF(F("Mission complete! Time: "), (millis() - startTime) / 1000.0f);
    DEBUG_PRINTLN(F(" seconds"));
    
    missionActive = false;
  }
  
  delay(100);
}