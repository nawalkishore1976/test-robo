#include <Arduino.h>

// Local includes
#include "common_macro.h"
#include "RoboMotor.h"
#include "robot.h"
#include "config.h"

// ============================================================================
// PRECISE DATA STRUCTURES (Using RAM and floats for accuracy)
// ============================================================================

struct Point {
  float x, y;           // Position in meters (high precision)
  float speed;          // Speed in m/s or percentage
  float heading;        // Direction in degrees (0-360)
  bool useCircular;     // Use circular interpolation
  float radius;         // Curve radius for circular movement
};

struct PathState {
  int currentIndex;           // Current point being processed
  int totalPoints;           // Total points in path
  bool isInterpolating;      // Are we between two points?
  int interpStep;            // Current interpolation step
  int interpSteps;           // Total interpolation steps
  Point currentTarget;       // Current interpolated target
  Point segmentStart;        // Start of current segment
  Point segmentEnd;          // End of current segment
  float segmentProgress;     // Progress through segment (0.0 to 1.0)
};

// ============================================================================
// PRECISE PATH DEFINITION (Stored in RAM for easy modification)
// ============================================================================

Point robotPath[] = {
  // x, y, speed, heading, useCircular, radius
  {0.0, 0.0, 0.5, 0.0, false, 0.0},      // Start position
  {1.0, 0.0, 0.5, 0.0, false, 0.0},      // Move right 1 meter
  {0.0, 0.0, 0.5, 180.0, true, 0.5},     // Return with curve
  {0.0, 1.0, 0.5, 90.0, false, 0.0},     // Move up 1 meter
  {1.0, 1.0, 0.5, 45.0, true, 0.3},      // Move to top-right with curve
  {0.0, 1.0, 0.5, 180.0, true, 0.5},     // Return with curve
  {0.0, 2.0, 0.5, 90.0, false, 0.0},     // Move up another meter
  {1.0, 2.0, 0.5, 0.0, false, 0.0},      // Move right
  {1.0, 3.0, 0.5, 90.0, false, 0.0},     // Move up
  {3.0, 3.0, 0.5, 0.0, true, 1.0},       // Move right 2 meters with wide curve
  {3.0, 2.0, 0.5, 270.0, false, 0.0},    // Move down
  {2.16, 2.0, 0.0, 225.0, false, 0.0}    // Final position with stop
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

RoboMotor driveRight(6, 5, 4);
RoboMotor driveLeft(7, 8, 9);

PathState pathState;
uint32_t startTime;
bool missionActive = false;

// Configuration
const float INTERPOLATION_RESOLUTION = 0.05; // 5cm between interpolated points
const int MAX_INTERPOLATION_STEPS = 100;     // Safety limit

// ============================================================================
// PRECISE INTERPOLATION FUNCTIONS
// ============================================================================

/**
 * @brief Calculate precise distance between two points
 */
float calculateDistance(const Point &a, const Point &b) {
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  return sqrt(dx * dx + dy * dy);
}

/**
 * @brief Calculate heading between two points
 */
float calculateHeading(const Point &from, const Point &to) {
  float dx = to.x - from.x;
  float dy = to.y - from.y;
  float heading = atan2(dy, dx) * 180.0 / PI;
  if (heading < 0) heading += 360.0;
  return heading;
}

/**
 * @brief Linear interpolation between two points
 */
Point linearInterpolation(const Point &start, const Point &end, float t) {
  Point result;
  result.x = start.x + (end.x - start.x) * t;
  result.y = start.y + (end.y - start.y) * t;
  result.speed = start.speed + (end.speed - start.speed) * t;
  result.heading = start.heading + (end.heading - start.heading) * t;
  result.useCircular = false;
  result.radius = 0.0;
  return result;
}

/**
 * @brief Circular interpolation for smooth curves
 */
Point circularInterpolation(const Point &start, const Point &end, float t, float radius) {
  Point result;
  
  // Calculate circle center and angles
  float distance = calculateDistance(start, end);
  float midX = (start.x + end.x) / 2.0;
  float midY = (start.y + end.y) / 2.0;
  
  // Calculate perpendicular offset for circle center
  float perpAngle = atan2(end.y - start.y, end.x - start.x) + PI/2.0;
  float centerOffset = sqrt(radius * radius - (distance/2.0) * (distance/2.0));
  
  float centerX = midX + cos(perpAngle) * centerOffset;
  float centerY = midY + sin(perpAngle) * centerOffset;
  
  // Calculate start and end angles
  float startAngle = atan2(start.y - centerY, start.x - centerX);
  float endAngle = atan2(end.y - centerY, end.x - centerX);
  
  // Ensure we take the shorter arc
  float angleDiff = endAngle - startAngle;
  if (angleDiff > PI) angleDiff -= 2.0 * PI;
  if (angleDiff < -PI) angleDiff += 2.0 * PI;
  
  // Interpolate along the arc
  float currentAngle = startAngle + angleDiff * t;
  
  result.x = centerX + cos(currentAngle) * radius;
  result.y = centerY + sin(currentAngle) * radius;
  result.speed = start.speed + (end.speed - start.speed) * t;
  result.heading = currentAngle * 180.0 / PI + 90.0; // Tangent to circle
  result.useCircular = true;
  result.radius = radius;
  
  return result;
}

/**
 * @brief Smart interpolation that chooses method based on path requirements
 */
Point interpolatePoint(const Point &start, const Point &end, float t) {
  if (start.useCircular && start.radius > 0.0) {
    return circularInterpolation(start, end, t, start.radius);
  } else {
    return linearInterpolation(start, end, t);
  }
}

// ============================================================================
// PATH PROCESSING FUNCTIONS
// ============================================================================

/**
 * @brief Initialize path processing state
 */
void initializePath() {
  pathState.currentIndex = 0;
  pathState.totalPoints = sizeof(robotPath) / sizeof(Point);
  pathState.isInterpolating = false;
  pathState.interpStep = 0;
  pathState.interpSteps = 0;
  pathState.segmentProgress = 0.0;
  
  DEBUG_PRINT(F("Path initialized with "));
  DEBUG_PRINT(pathState.totalPoints);
  DEBUG_PRINTLN(F(" points"));
}

/**
 * @brief Process next point in path with precise interpolation
 */
bool processNextPathSegment() {
  // Check if path is complete
  if (pathState.currentIndex >= pathState.totalPoints - 1) {
    return false;
  }
  
  // Start new segment if not interpolating
  if (!pathState.isInterpolating) {
    pathState.segmentStart = robotPath[pathState.currentIndex];
    pathState.segmentEnd = robotPath[pathState.currentIndex + 1];
    
    // Calculate interpolation steps based on distance and resolution
    float distance = calculateDistance(pathState.segmentStart, pathState.segmentEnd);
    pathState.interpSteps = min(MAX_INTERPOLATION_STEPS, 
                               max(1, (int)(distance / INTERPOLATION_RESOLUTION)));
    pathState.interpStep = 0;
    pathState.isInterpolating = true;
    pathState.segmentProgress = 0.0;
    
    DEBUG_PRINT(F("Segment "));
    DEBUG_PRINT(pathState.currentIndex);
    DEBUG_PRINT(F(" -> "));
    DEBUG_PRINT(pathState.currentIndex + 1);
    DEBUG_PRINT(F(", Distance: "));
    DEBUG_PRINT(distance);
    DEBUG_PRINT(F("m, Steps: "));
    DEBUG_PRINT(pathState.interpSteps);
    DEBUG_PRINT(F(", Method: "));
    DEBUG_PRINTLN(pathState.segmentStart.useCircular ? F("Circular") : F("Linear"));
  }
  
  // Calculate current interpolation progress
  pathState.segmentProgress = (float)pathState.interpStep / (float)pathState.interpSteps;
  
  // Generate interpolated point
  pathState.currentTarget = interpolatePoint(pathState.segmentStart, 
                                           pathState.segmentEnd, 
                                           pathState.segmentProgress);
  
  // Send precise movement command
  robot::moveTo(pathState.currentTarget.x, 
                 pathState.currentTarget.y,
                 pathState.currentTarget.speed * 100); // Convert to percentage
  
  // Advanced debug info
  DEBUG_PRINT(F("Target: ("));
  DEBUG_PRINT(pathState.currentTarget.x, 3);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(pathState.currentTarget.y, 3);
  DEBUG_PRINT(F("), Heading: "));
  DEBUG_PRINT(pathState.currentTarget.heading, 1);
  DEBUG_PRINT(F("°, Progress: "));
  DEBUG_PRINT(pathState.segmentProgress * 100.0, 1);
  DEBUG_PRINTLN(F("%"));
  
  // Advance interpolation
  pathState.interpStep++;
  
  // Check if segment complete
  if (pathState.interpStep >= pathState.interpSteps) {
    pathState.isInterpolating = false;
    pathState.currentIndex++;
    
    DEBUG_PRINT(F("Completed segment to point "));
    DEBUG_PRINTLN(pathState.currentIndex);
  }
  
  return true;
}

// ============================================================================
// ARDUINO SETUP AND MAIN LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  DEBUG_PRINTLN(F("=== Precision Robot Tour Controller ==="));
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  
  robot::initialize();
  initializePath();
  
  // Print detailed path information
  DEBUG_PRINTLN(F("Precision path with interpolation:"));
  for (int i = 0; i < pathState.totalPoints; i++) {
    Point p = robotPath[i];
    DEBUG_PRINT(F("  Point "));
    DEBUG_PRINT(i);
    DEBUG_PRINT(F(": ("));
    DEBUG_PRINT(p.x, 2);
    DEBUG_PRINT(F(", "));
    DEBUG_PRINT(p.y, 2);
    DEBUG_PRINT(F("), Speed: "));
    DEBUG_PRINT(p.speed, 2);
    DEBUG_PRINT(F(", Heading: "));
    DEBUG_PRINT(p.heading, 1);
    DEBUG_PRINT(F("°"));
    if (p.useCircular) {
      DEBUG_PRINT(F(", Curve R="));
      DEBUG_PRINT(p.radius, 2);
    }
    DEBUG_PRINTLN();
  }
  
  DEBUG_PRINTLN(F("Press button to start precision mission..."));
  
  while (digitalRead(2) == HIGH) {
    digitalWrite(LED_BUILTIN, millis() % 1000 < 500);
    delay(10);
  }
  
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  
  DEBUG_PRINTLN(F("Precision mission starting!"));
  startTime = millis();
  missionActive = true;
}

void loop() {
  if (!missionActive) {
    digitalWrite(LED_BUILTIN, millis() % 2000 < 1000);
    delay(100);
    return;
  }
  
  if (!processNextPathSegment()) {
    // Mission complete
    driveLeft.stop();
    driveRight.stop();
    
    uint32_t totalTime = millis() - startTime;
    
    DEBUG_PRINT(F("Precision mission complete! Time: "));
    DEBUG_PRINT(totalTime / 1000.0);
    DEBUG_PRINTLN(F(" seconds"));
    
    missionActive = false;
  }
  
  delay(20); // Higher frequency for precision
} 