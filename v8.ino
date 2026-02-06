#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "RB_ENCONDERMOTOR.h" 
#include "RB_BUZZER.h"
#include "RB_RGBLED.h"
#include <ICM20948_WE.h>
#include <avr/pgmspace.h>

// =============================================================================
// DEBUG AND SIMULATION CONFIGURATION
// =============================================================================
#define DEBUG_ENABLED 1
#define DRY_RUN 1        // Set to 1 for dry run testing
#define SIMULATE 1       // Set to 1 for simulation mode

#if DEBUG_ENABLED
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// =============================================================================
// MEMORY-EFFICIENT DATA STRUCTURES
// =============================================================================

// Anchor Point - 6 bytes (discrete grid positions from commands)
struct AnchorPoint {
  int16_t x, y;    // in 0.5cm units
  uint8_t dir;     // Direction: 0=North, 1=East, 2=South, 3=West
  uint8_t type;    // 0=move, 1=turn point
  
  inline float getX() const { return x * 0.5f; }
  inline float getY() const { return y * 0.5f; }
  inline float getHeading() const { return dir * PI * 0.5f; }
  inline void setAnchor(float px, float py, uint8_t direction, uint8_t ptype = 0) {
    x = (int16_t)(px * 2.0f);
    y = (int16_t)(py * 2.0f);
    dir = direction;
    type = ptype;
  }
};

// Smooth Waypoint - 6 bytes (interpolated points for continuous path)
struct WayPoint {
  int16_t x, y;    // in 0.5cm units
  uint16_t speed;  // in 0.1 cm/s units
  
  inline float getX() const { return x * 0.5f; }
  inline float getY() const { return y * 0.5f; }
  inline float getSpeed() const { return speed * 0.1f; }
  inline void setWaypoint(float px, float py, float spd) {
    x = (int16_t)(px * 2.0f);
    y = (int16_t)(py * 2.0f);
    speed = (uint16_t)(spd * 10.0f);
  }
  inline bool isStop() const { return speed < 2; }
};

// Path Segment for continuous motion - 6 bytes
struct PathSegment {
  uint8_t type;    // 0=continuous path, 1=stop
  uint8_t count;   // number of waypoints
  uint16_t offset; // offset in waypoints array
  int16_t data;    // reserved
};

// =============================================================================
// CONFIGURATION & CONSTANTS
// =============================================================================
#define MAX_ANCHORS 30           // Anchor points from commands
#define MAX_WAYPOINTS 80         // Interpolated waypoints
#define MAX_SEGMENTS 10          
#define UNIT_SIZE 5.0            // 5cm grid units
#define DEFAULT_SPEED 25.0       // cm/s
#define TURN_RADIUS 8.0          // cm - radius for smooth turns
#define INTERPOLATION_STEP 2.0   // cm between interpolated points

// Direction constants
#define DIR_NORTH 0  // Y+
#define DIR_EAST  1  // X+
#define DIR_SOUTH 2  // Y-
#define DIR_WEST  3  // X-

// =============================================================================
// COMMAND STRING AND DIRECTION DELTAS
// =============================================================================
const char PROGMEM testCommands[] = "FRFRFRF"; // Test: square path

const PROGMEM int8_t directionDelta[4][2] = {
  {0,  1}, // North: Y+
  {1,  0}, // East:  X+
  {0, -1}, // South: Y-
  {-1, 0}  // West:  X-
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
RB_EncoderMotor M1(1);
RB_EncoderMotor M2(2);
RB_Buzzer Buzzer(BUZZER_PIN);
RB_RGBLed RGBLED(RGB_LED_PIN, 2);

volatile long m1_count = 0;
volatile long m2_count = 0;

// Path data
AnchorPoint anchors[MAX_ANCHORS];
WayPoint waypoints[MAX_WAYPOINTS];
PathSegment segments[MAX_SEGMENTS];
uint8_t anchorCount = 0;
uint8_t waypointCount = 0;
uint8_t segmentCount = 0;

// Motor control
int16_t m1_pwm = 0, m2_pwm = 0;
float targetLeftSpeed = 0, targetRightSpeed = 0;

// =============================================================================
// SIMULATION AND ISRS
// =============================================================================
#if SIMULATE
void simulateEncoders() {
  static uint32_t lastSimUpdate = 0;
  uint32_t now = micros();
  if ((now - lastSimUpdate) < 20000) return;
  
  float dt = (now - lastSimUpdate) / 1000000.0f;
  float m1_pps = m1_pwm * 4.0f;
  float m2_pps = -m2_pwm * 4.0f; // Account for reversed wiring
  
  m1_count += (long)(m1_pps * dt);
  m2_count += (long)(m2_pps * dt);
  lastSimUpdate = now;
}
#endif

void m1EncoderISR() { 
  #if SIMULATE
    simulateEncoders(); 
  #else
    m1_count++; 
  #endif
}

void m2EncoderISR() { 
  #if SIMULATE
    simulateEncoders(); 
  #else
    m2_count++; 
  #endif
}

// =============================================================================
// STEP 1: CONVERT COMMANDS TO ANCHOR POINTS
// =============================================================================
uint8_t convertCommandsToAnchors(const char* commands) {
  float currentX = 0, currentY = 0;
  uint8_t currentDir = DIR_NORTH;
  anchorCount = 0;
  
  DEBUG_PRINTLN("=== STEP 1: COMMANDS TO ANCHOR POINTS ===");
  DEBUG_PRINT("Commands: ");
  DEBUG_PRINTLN(commands);
  
  // Add starting anchor
  anchors[anchorCount].setAnchor(currentX, currentY, currentDir, 0);
  anchorCount++;
  
  uint8_t len = strlen(commands);
  for (uint8_t i = 0; i < len && anchorCount < MAX_ANCHORS - 1; i++) {
    char cmd = commands[i];
    
    switch (cmd) {
      case 'L': // Turn left
        currentDir = (currentDir + 3) % 4;
        // Mark current position as turn point
        anchors[anchorCount - 1].type = 1;
        DEBUG_PRINT("L: Turn left at (");
        DEBUG_PRINT(currentX);
        DEBUG_PRINT(", ");
        DEBUG_PRINT(currentY);
        DEBUG_PRINTLN(")");
        break;
        
      case 'R': // Turn right
        currentDir = (currentDir + 1) % 4;
        // Mark current position as turn point
        anchors[anchorCount - 1].type = 1;
        DEBUG_PRINT("R: Turn right at (");
        DEBUG_PRINT(currentX);
        DEBUG_PRINT(", ");
        DEBUG_PRINT(currentY);
        DEBUG_PRINTLN(")");
        break;
        
      case 'F': // Forward
      case 'S':
        {
          int8_t dx, dy;
          memcpy_P(&dx, &directionDelta[currentDir][0], 1);
          memcpy_P(&dy, &directionDelta[currentDir][1], 1);
          
          currentX += dx * UNIT_SIZE;
          currentY += dy * UNIT_SIZE;
          
          anchors[anchorCount].setAnchor(currentX, currentY, currentDir, 0);
          
          DEBUG_PRINT("F: Move to (");
          DEBUG_PRINT(currentX);
          DEBUG_PRINT(", ");
          DEBUG_PRINT(currentY);
          DEBUG_PRINTLN(")");
          
          anchorCount++;
        }
        break;
        
      case 'B': // Backward
        {
          int8_t dx, dy;
          uint8_t oppositeDir = (currentDir + 2) % 4;
          memcpy_P(&dx, &directionDelta[oppositeDir][0], 1);
          memcpy_P(&dy, &directionDelta[oppositeDir][1], 1);
          
          currentX += dx * UNIT_SIZE;
          currentY += dy * UNIT_SIZE;
          
          anchors[anchorCount].setAnchor(currentX, currentY, currentDir, 0);
          
          DEBUG_PRINT("B: Move to (");
          DEBUG_PRINT(currentX);
          DEBUG_PRINT(", ");
          DEBUG_PRINT(currentY);
          DEBUG_PRINTLN(")");
          
          anchorCount++;
        }
        break;
    }
  }
  
  DEBUG_PRINT("Created ");
  DEBUG_PRINT(anchorCount);
  DEBUG_PRINTLN(" anchor points");
  DEBUG_PRINTLN("=========================================");
  
  return anchorCount;
}

// =============================================================================
// STEP 2: INTERPOLATE ANCHOR POINTS TO SMOOTH WAYPOINTS
// =============================================================================
uint8_t interpolateAnchorsToWaypoints() {
  waypointCount = 0;
  
  DEBUG_PRINTLN("=== STEP 2: ANCHOR INTERPOLATION ===");
  
  for (uint8_t i = 0; i < anchorCount - 1 && waypointCount < MAX_WAYPOINTS - 5; i++) {
    AnchorPoint* current = &anchors[i];
    AnchorPoint* next = &anchors[i + 1];
    
    DEBUG_PRINT("Interpolating from A");
    DEBUG_PRINT(i);
    DEBUG_PRINT(" to A");
    DEBUG_PRINT(i + 1);
    
    // If current point is a turn point, create smooth curve
    if (current->type == 1 && i > 0 && i < anchorCount - 1) {
      AnchorPoint* prev = &anchors[i - 1];
      
      // Create smooth turn using circular arc
      float prevX = prev->getX();
      float prevY = prev->getY();
      float currX = current->getX();
      float currY = current->getY();
      float nextX = next->getX();
      float nextY = next->getY();
      
      // Calculate turn angle
      float inAngle = atan2(currY - prevY, currX - prevX);
      float outAngle = atan2(nextY - currY, nextX - currX);
      float turnAngle = outAngle - inAngle;
      
      // Normalize turn angle
      while (turnAngle > PI) turnAngle -= 2 * PI;
      while (turnAngle < -PI) turnAngle += 2 * PI;
      
      DEBUG_PRINT(" [SMOOTH TURN ");
      DEBUG_PRINT(turnAngle * 180 / PI);
      DEBUG_PRINT(" deg]");
      
      // Create arc waypoints
      uint8_t arcPoints = (uint8_t)(abs(turnAngle) * TURN_RADIUS / INTERPOLATION_STEP);
      arcPoints = constrain(arcPoints, 3, 10);
      
      for (uint8_t j = 0; j <= arcPoints && waypointCount < MAX_WAYPOINTS - 1; j++) {
        float t = (float)j / arcPoints;
        float angle = inAngle + t * turnAngle;
        
        // Simple arc calculation - could be improved with proper curve fitting
        float arcX = currX + (t - 0.5f) * TURN_RADIUS * cos(angle + PI/2);
        float arcY = currY + (t - 0.5f) * TURN_RADIUS * sin(angle + PI/2);
        
        float speed = DEFAULT_SPEED * (1.0f - 0.3f * abs(turnAngle) / PI); // Slow in turns
        waypoints[waypointCount].setWaypoint(arcX, arcY, speed);
        waypointCount++;
      }
      
    } else {
      // Straight line interpolation
      float dx = next->getX() - current->getX();
      float dy = next->getY() - current->getY();
      float distance = sqrt(dx * dx + dy * dy);
      
      uint8_t steps = (uint8_t)(distance / INTERPOLATION_STEP);
      steps = max(steps, 1);
      
      DEBUG_PRINT(" [STRAIGHT ");
      DEBUG_PRINT(distance);
      DEBUG_PRINT("cm, ");
      DEBUG_PRINT(steps);
      DEBUG_PRINT(" steps]");
      
      for (uint8_t j = 0; j <= steps && waypointCount < MAX_WAYPOINTS - 1; j++) {
        float t = (float)j / steps;
        float x = current->getX() + t * dx;
        float y = current->getY() + t * dy;
        
        waypoints[waypointCount].setWaypoint(x, y, DEFAULT_SPEED);
        waypointCount++;
      }
    }
    
    DEBUG_PRINTLN();
  }
  
  // Add final waypoint and make it a stop
  if (waypointCount > 0) {
    waypoints[waypointCount - 1].speed = 0; // Stop at end
  }
  
  DEBUG_PRINT("Generated ");
  DEBUG_PRINT(waypointCount);
  DEBUG_PRINTLN(" smooth waypoints");
  DEBUG_PRINTLN("===================================");
  
  return waypointCount;
}

// =============================================================================
// STEP 3: CREATE CONTINUOUS PATH SEGMENTS
// =============================================================================
uint8_t generateContinuousSegments() {
  segmentCount = 0;
  
  DEBUG_PRINTLN("=== STEP 3: CONTINUOUS PATH SEGMENTS ===");
  
  // Create one continuous segment for all waypoints
  segments[0].type = 0; // Continuous path
  segments[0].count = waypointCount;
  segments[0].offset = 0;
  segments[0].data = 0;
  segmentCount = 1;
  
  DEBUG_PRINT("Created 1 continuous segment with ");
  DEBUG_PRINT(waypointCount);
  DEBUG_PRINTLN(" waypoints");
  DEBUG_PRINTLN("========================================");
  
  return segmentCount;
}

// =============================================================================
// PRINT FUNCTIONS
// =============================================================================
void printAnchors() {
  DEBUG_PRINTLN("=== ANCHOR POINTS ===");
  for (uint8_t i = 0; i < anchorCount; i++) {
    DEBUG_PRINT("A");
    DEBUG_PRINT(i);
    DEBUG_PRINT(": (");
    DEBUG_PRINT(anchors[i].getX());
    DEBUG_PRINT(", ");
    DEBUG_PRINT(anchors[i].getY());
    DEBUG_PRINT(") dir=");
    DEBUG_PRINT(anchors[i].dir);
    if (anchors[i].type == 1) {
      DEBUG_PRINT(" [TURN]");
    }
    DEBUG_PRINTLN();
  }
  DEBUG_PRINTLN("====================");
}

void printWaypoints() {
  DEBUG_PRINTLN("=== SMOOTH WAYPOINTS ===");
  for (uint8_t i = 0; i < waypointCount; i++) {
    DEBUG_PRINT("W");
    DEBUG_PRINT(i);
    DEBUG_PRINT(": (");
    DEBUG_PRINT(waypoints[i].getX());
    DEBUG_PRINT(", ");
    DEBUG_PRINT(waypoints[i].getY());
    DEBUG_PRINT(") @ ");
    DEBUG_PRINT(waypoints[i].getSpeed());
    if (waypoints[i].isStop()) {
      DEBUG_PRINT(" [STOP]");
    }
    DEBUG_PRINTLN();
  }
  DEBUG_PRINTLN("=======================");
}

// =============================================================================
// MOTOR CONTROL
// =============================================================================
void updateMotorControl() {
  static uint32_t lastUpdate = 0;
  uint32_t now = micros();
  if ((now - lastUpdate) < 50000) return;
  
  m1_pwm = (int16_t)(targetLeftSpeed * 2.0f);
  m2_pwm = (int16_t)(targetRightSpeed * 2.0f);
  m1_pwm = constrain(m1_pwm, -99, 99);
  m2_pwm = constrain(m2_pwm, -99, 99);
  
  if (!DRY_RUN) {
    M1.SetTarPWM(m1_pwm);
    M2.SetTarPWM(-m2_pwm); // M2 reversed wiring
  }
  
  lastUpdate = now;
}

// =============================================================================
// MAIN FUNCTIONS
// =============================================================================
void setup() {
  Serial.begin(115200);
  DEBUG_PRINTLN("Robot Tour V8 - Continuous Smooth Paths");
  DEBUG_PRINTLN("Commands -> Anchors -> Interpolated Waypoints -> Continuous Segments");
  DEBUG_PRINTLN();
  
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIGHT_PIN, OUTPUT);
  
  RGBLED.setColor(1, 50, 0, 0); // Red during init
  RGBLED.show();
  
  if (!DRY_RUN) {
    M1.SetMotionMode(PWM_MODE);
    M2.SetMotionMode(PWM_MODE);
    attachInterrupt(digitalPinToInterrupt(M1.GetInterruptNumA()), m1EncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2.GetInterruptNumA()), m2EncoderISR, CHANGE);
  }
  
  // Process commands through 3 steps
  char commands[32];
  strcpy_P(commands, testCommands);
  
  // Step 1: Commands to anchor points
  convertCommandsToAnchors(commands);
  printAnchors();
  
  // Step 2: Interpolate anchors to smooth waypoints
  interpolateAnchorsToWaypoints();
  printWaypoints();
  
  // Step 3: Create continuous path segments
  generateContinuousSegments();
  
  // Memory usage
  DEBUG_PRINTLN("=== MEMORY USAGE ===");
  DEBUG_PRINT("Anchors: ");
  DEBUG_PRINT(anchorCount * sizeof(AnchorPoint));
  DEBUG_PRINTLN(" bytes");
  DEBUG_PRINT("Waypoints: ");
  DEBUG_PRINT(waypointCount * sizeof(WayPoint));
  DEBUG_PRINTLN(" bytes");
  DEBUG_PRINT("Segments: ");
  DEBUG_PRINT(segmentCount * sizeof(PathSegment));
  DEBUG_PRINTLN(" bytes");
  DEBUG_PRINTLN("===================");
  
  RGBLED.setColor(1, 0, 50, 0); // Green ready
  RGBLED.show();
  DEBUG_PRINTLN("Ready! Press button for continuous path execution.");
}

void loop() {
  if (digitalRead(START_BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(START_BUTTON_PIN) == LOW) {
      RGBLED.setColor(1, 0, 0, 50); // Blue running
      RGBLED.show();
      
      DEBUG_PRINTLN("=== EXECUTING CONTINUOUS PATH ===");
      
      // Execute continuous path through all waypoints
      for (uint8_t i = 0; i < waypointCount; i++) {
        WayPoint* wp = &waypoints[i];
        
        targetLeftSpeed = wp->getSpeed();
        targetRightSpeed = wp->getSpeed();
        
        DEBUG_PRINT("-> W");
        DEBUG_PRINT(i);
        DEBUG_PRINT(": (");
        DEBUG_PRINT(wp->getX());
        DEBUG_PRINT(", ");
        DEBUG_PRINT(wp->getY());
        DEBUG_PRINT(") @ ");
        DEBUG_PRINT(wp->getSpeed());
        DEBUG_PRINTLN(" cm/s");
        
        updateMotorControl();
        
        if (wp->isStop()) {
          targetLeftSpeed = 0;
          targetRightSpeed = 0;
          updateMotorControl();
          DEBUG_PRINTLN("STOPPED at final waypoint");
          break;
        }
        
        delay(DRY_RUN ? 100 : 300);
      }
      
      DEBUG_PRINTLN("=== CONTINUOUS PATH COMPLETE ===");
      RGBLED.setColor(1, 0, 50, 0); // Green success
      RGBLED.show();
      delay(3000);
    }
  }
  
  #if SIMULATE
    simulateEncoders();
  #endif
  
  updateMotorControl();
  delay(10);
}
