#include <Arduino.h>
#include <avr/pgmspace.h>

// Local includes
#include "common_macro.h"  // Use debug macros from here
#include "RoboMotor.h"
#include "robot.h"
#include "config.h"

// ============================================================================
// MEMORY-EFFICIENT DATA STRUCTURES
// ============================================================================

// Compact position using 16-bit fixed point (divide by 100 for actual value)
struct CompactPoint {
  int16_t x, y;      // Position in cm * 100 (fixed point)
  uint8_t speed;     // Speed 0-100%
} __attribute__((packed));

// Minimal path state - only track current processing
struct PathState {
  uint8_t currentIndex;        // Current point being processed
  uint8_t totalPoints;         // Total points in path
  bool isInterpolating;        // Are we between two points?
  uint8_t interpStep;          // Current interpolation step
  uint8_t interpSteps;         // Total interpolation steps
  CompactPoint currentTarget;  // Current interpolated target
};

// ============================================================================
// PROGRAM MEMORY CONSTANTS (Stored in Flash, not RAM)
// ============================================================================

// Pre-defined path stored in program memory - only ~24 bytes
const PROGMEM CompactPoint ROBOT_PATH[] = {
  {0, 0, 50},        // Start position
  {100, 0, 50},      // Move right 1 meter
  {0, 0, 50},        // Return to start  
  {0, 100, 50},      // Move up 1 meter
  {100, 100, 50},    // Move to top-right
  {0, 100, 50},      // Return to top-left
  {0, 200, 50},      // Move up another meter
  {100, 200, 50},    // Move right
  {100, 300, 50},    // Move up
  {300, 300, 50},    // Move right 2 meters
  {300, 200, 50},    // Move down
  {216, 200, 0}      // Final position with stop
};

const PROGMEM uint8_t PATH_LENGTH = sizeof(ROBOT_PATH) / sizeof(CompactPoint);
const PROGMEM int16_t GRID_SIZE = 5000;     // 50.00 cm in fixed point
const PROGMEM int16_t GRID_OFFSET = 2500;   // 25.00 cm in fixed point  
const PROGMEM uint8_t INTERP_RESOLUTION = 5; // cm between interpolated points

// ============================================================================
// GLOBAL VARIABLES (Minimal RAM usage - ~20 bytes total)
// ============================================================================

RoboMotor driveRight(6, 5, 4);  // Right motor pins
RoboMotor driveLeft(7, 8, 9);   // Left motor pins

PathState pathState;             // Current path processing state
uint32_t startTime;              // Mission start time
bool missionActive = false;       // Mission status

// ============================================================================
// MEMORY-EFFICIENT HELPER FUNCTIONS  
// ============================================================================

/**
 * @brief Read a point from program memory (saves RAM)
 * @param index Point index in the path array
 * @return CompactPoint loaded from PROGMEM
 */
CompactPoint readPathPoint(uint8_t index) {
  CompactPoint point;
  memcpy_P(&point, &ROBOT_PATH[index], sizeof(CompactPoint));
  return point;
}

/**
 * @brief Convert grid coordinates to absolute centimeters
 * @param gridPoint Point in grid coordinates (0-3 typical)
 * @return Point in absolute coordinates (cm)
 */
CompactPoint toAbsoluteCoords(CompactPoint gridPoint) {
  int16_t gridSize = pgm_read_word(&GRID_SIZE);
  int16_t offset = pgm_read_word(&GRID_OFFSET);
  
  gridPoint.x = (gridPoint.x * gridSize) / 100 + offset;
  gridPoint.y = (gridPoint.y * gridSize) / 100 + offset;
  
  return gridPoint;
}

/**
 * @brief Calculate distance between two points (integer math)
 * @param a First point
 * @param b Second point  
 * @return Distance in cm (approximate)
 */
uint16_t calculateDistance(const CompactPoint &a, const CompactPoint &b) {
  int32_t dx = (int32_t)a.x - b.x;
  int32_t dy = (int32_t)a.y - b.y;
  
  // Fast integer square root approximation for distance
  uint32_t distSq = (dx * dx + dy * dy) / 10000; // Convert from fixed point
  
  // Simple integer square root
  uint16_t dist = 0;
  while (dist * dist < distSq) dist++;
  
  return dist;
}

/**
 * @brief Generate single interpolated point between two points
 * @param start Starting point
 * @param end Ending point
 * @param step Current interpolation step (0 to totalSteps-1)
 * @param totalSteps Total number of interpolation steps
 * @return Interpolated point
 */
CompactPoint interpolatePoint(const CompactPoint &start, const CompactPoint &end, 
                             uint8_t step, uint8_t totalSteps) {
  CompactPoint result;
  
  // Linear interpolation using integer math
  result.x = start.x + ((int32_t)(end.x - start.x) * step) / totalSteps;
  result.y = start.y + ((int32_t)(end.y - start.y) * step) / totalSteps;
  result.speed = start.speed; // Use start point speed
  
  return result;
}

// ============================================================================
// PATH PROCESSING FUNCTIONS
// ============================================================================

/**
 * @brief Initialize path processing state
 */
void initializePath() {
  pathState.currentIndex = 0;
  pathState.totalPoints = pgm_read_byte(&PATH_LENGTH);
  pathState.isInterpolating = false;
  pathState.interpStep = 0;
  pathState.interpSteps = 0;
  
  DEBUG_PRINTF(F("Path initialized with "), pathState.totalPoints);
  DEBUG_PRINTLN(F(" points"));
}

/**
 * @brief Process next point in path (pair-wise processing)
 * @return true if more points to process, false if complete
 */
bool processNextPathSegment() {
  // Check if path is complete
  if (pathState.currentIndex >= pathState.totalPoints - 1) {
    return false; // Path complete
  }
  
  // If not currently interpolating, start new segment
  if (!pathState.isInterpolating) {
    CompactPoint current = readPathPoint(pathState.currentIndex);
    CompactPoint next = readPathPoint(pathState.currentIndex + 1);
    
    // Convert to absolute coordinates
    current = toAbsoluteCoords(current);
    next = toAbsoluteCoords(next);
    
    // Calculate interpolation steps based on distance
    uint16_t distance = calculateDistance(current, next);
    uint8_t resolution = pgm_read_byte(&INTERP_RESOLUTION);
    pathState.interpSteps = max(1, distance / resolution);
    pathState.interpStep = 0;
    pathState.isInterpolating = true;
    
    // Multi-line debug output with proper checks
#if DEBUG_ENABLED
    DEBUG_PRINT(F("Segment "));
    DEBUG_PRINT(pathState.currentIndex);
    DEBUG_PRINT(F(" -> "));
    DEBUG_PRINT(pathState.currentIndex + 1);
    DEBUG_PRINT(F(", Distance: "));
    DEBUG_PRINT(distance);
    DEBUG_PRINTLN(F(" cm"));
#endif
  }
  
  // Process current interpolation step
  CompactPoint current = toAbsoluteCoords(readPathPoint(pathState.currentIndex));
  CompactPoint next = toAbsoluteCoords(readPathPoint(pathState.currentIndex + 1));
  
  pathState.currentTarget = interpolatePoint(current, next, 
                                           pathState.interpStep, 
                                           pathState.interpSteps);
  
  // Move to interpolated position
  robot::moveTo(pathState.currentTarget.x / 100.0f, 
                 pathState.currentTarget.y / 100.0f,
                 pathState.currentTarget.speed);
  
  // Advance interpolation
  pathState.interpStep++;
  
  // Check if segment complete
  if (pathState.interpStep >= pathState.interpSteps) {
    pathState.isInterpolating = false;
    pathState.currentIndex++;
    
    DEBUG_PRINTF(F("Completed segment to point "), pathState.currentIndex);
  }
  
  return true;
}

// ============================================================================
// ARDUINO SETUP AND MAIN LOOP
// ============================================================================

/**
 * @brief Arduino setup function - runs once at startup
 */
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial connection
  
  DEBUG_PRINTLN(F("=== Robot Tour Controller ==="));
  
  // Initialize hardware
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, INPUT_PULLUP); // Start button
  
  // Initialize robot and motors
  robot::initialize();
  
  // Initialize path processing
  initializePath();
  
  // Print path information (multi-line debug with proper checks)
#if DEBUG_ENABLED
  DEBUG_PRINTLN(F("Predefined path:"));
  for (uint8_t i = 0; i < pgm_read_byte(&PATH_LENGTH); i++) {
    CompactPoint point = readPathPoint(i);
    DEBUG_PRINT(F("  Point "));
    DEBUG_PRINT(i);
    DEBUG_PRINT(F(": ("));
    DEBUG_PRINT(point.x);
    DEBUG_PRINT(F(", "));
    DEBUG_PRINT(point.y);
    DEBUG_PRINT(F(") Speed: "));
    DEBUG_PRINTLN(point.speed);
  }
#endif
  
  DEBUG_PRINTLN(F("Press button to start..."));
  
  // Wait for start button
  while (digitalRead(2) == HIGH) {
    digitalWrite(LED_BUILTIN, millis() % 1000 < 500); // Blink LED
    delay(10);
  }
  
  delay(500); // Debounce
  digitalWrite(LED_BUILTIN, HIGH);
  
  DEBUG_PRINTLN(F("Mission starting!"));
  startTime = millis();
  missionActive = true;
}

/**
 * @brief Arduino main loop - runs continuously
 */
void loop() {
  if (!missionActive) {
    // Mission complete - blink LED slowly
    digitalWrite(LED_BUILTIN, millis() % 2000 < 1000);
    delay(100);
    return;
  }
  
  // Process next path segment (pair-wise)
  if (!processNextPathSegment()) {
    // Path complete
    driveLeft.stop();
    driveRight.stop();
    
    uint32_t totalTime = millis() - startTime;
    
    // Multi-line debug output for mission complete
#if DEBUG_ENABLED
    DEBUG_PRINT(F("Mission complete! Time: "));
    DEBUG_PRINT(totalTime / 1000.0f);
    DEBUG_PRINTLN(F(" seconds"));
#endif
    
    missionActive = false;
  }
  
  delay(50); // Small delay for processing
}
