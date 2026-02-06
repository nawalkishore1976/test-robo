#include <RobotBuilder.h>  // For RB_EncoderMotor

// =============================================================================
// CONFIGURATION & CONSTANTS
// =============================================================================
const float WHEEL_RADIUS = 3.0;           // cm
const int ENCODER_STEPS_PER_REV = 660;    // steps per revolution
const float WHEEL_BASE = 20.0;            // cm - distance between wheels
const float TURN_RADIUS = 25.0;           // cm
const float PI = 3.14159265359;

// Calculate steps per cm
const float STEPS_PER_CM = ENCODER_STEPS_PER_REV / (2 * PI * WHEEL_RADIUS); // ~35 steps/cm

// PPS Configuration
const float FORWARD_BASE_PPS = 1000.0;
const float TURN_BASE_PPS = 400.0;
const float PPS_FACTOR = 1.0;  // Adjustable factor

// Movement parameters
const int FORWARD_SUBSTEPS = 10;
const int TURN_SUBSTEPS = 20;

// =============================================================================
// DATA STRUCTURES
// =============================================================================
struct Position {
  float x;
  float y;
  float heading;  // radians
};

struct MotorState {
  long target_steps;
  long actual_steps;
  long steps_error;
  float target_pps;
  float current_pps;
  int current_pwm;
};

struct MovementCommand {
  char type;          // 'F', 'L', 'R'
  float distance;     // cm for forward, degrees for turn
  int substeps;
  bool completed;
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
RB_EncoderMotor M1(1);  // Left motor
RB_EncoderMotor M2(2);  // Right motor (Master for left turns)

MotorState m1_state = {0, 0, 0, 0, 0, 0};
MotorState m2_state = {0, 0, 0, 0, 0, 0};

Position current_pos = {0, 0, 0};
Position target_pos = {0, 0, 0};

float turn_angle_error = 0.0;  // Accumulated turn angle error

volatile long m1_encoder_count = 0;
volatile long m2_encoder_count = 0;

// Command sequence
String commands = "RRLLFF";
int command_index = 0;
MovementCommand current_command;
int current_substep = 0;

unsigned long last_update = 0;
const unsigned long UPDATE_INTERVAL = 50; // 50ms

// =============================================================================
// SETUP FUNCTION
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Advanced Robot Controller Starting...");
  
  M1.begin();
  M2.begin();
  
  // Initialize states
  resetMotorStates();
  
  Serial.print("Steps per cm: ");
  Serial.println(STEPS_PER_CM);
  
  // Start first command
  if (commands.length() > 0) {
    initializeNextCommand();
  }
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  unsigned long current_time = millis();
  
  if (current_time - last_update >= UPDATE_INTERVAL) {
    updateEncoderCounts();
    updateMotorControl();
    
    if (isMovementComplete()) {
      completeCurrentSubstep();
    }
    
    printStatus();
    last_update = current_time;
  }
  
  // Check if all commands completed
  if (command_index >= commands.length()) {
    stopMotors();
    Serial.println("All commands completed!");
    while(1) delay(1000);
  }
}

// =============================================================================
// MOTOR CONTROL FUNCTIONS
// =============================================================================
void updateMotorControl() {
  // Update actual steps
  m1_state.actual_steps = m1_encoder_count;
  m2_state.actual_steps = m2_encoder_count;
  
  // Calculate step errors
  long m1_error = m1_state.target_steps - m1_state.actual_steps;
  long m2_error = m2_state.target_steps - m2_state.actual_steps;
  
  // Calculate PWM based on PPS and error
  int m1_pwm = calculatePWM(m1_state, m1_error, false); // M1 is slave
  int m2_pwm = calculatePWM(m2_state, m2_error, true);  // M2 is master for left turns
  
  // Apply PWM with deadzone
  if (abs(m1_error) > 5) {
    M1.setPWM(m1_pwm);
  } else {
    M1.setPWM(0);
  }
  
  if (abs(m2_error) > 5) {
    M2.setPWM(m2_pwm);
  } else {
    M2.setPWM(0);
  }
}

int calculatePWM(MotorState &motor, long error, bool is_master) {
  // PPS to PWM conversion (simplified)
  float target_pwm = map(motor.target_pps, 0, 2000, 0, 255);
  
  // PID-like adjustment
  float kp = 0.15;
  float adjustment = kp * error;
  
  // For slave motor, sync with master
  if (!is_master && current_command.type == 'L') {
    // For left turns, M1 follows M2 with ratio
    float ratio = (float)m1_state.target_steps / (float)m2_state.target_steps;
    adjustment += (m2_state.current_pps * ratio - motor.current_pps) * 10;
  }
  
  int pwm = constrain(target_pwm + adjustment, 0, 255);
  
  // Set direction based on error
  if (error > 0) return pwm;
  else if (error < 0) return -pwm;
  else return 0;
}

// =============================================================================
// MOVEMENT COMMAND FUNCTIONS
// =============================================================================
void initializeNextCommand() {
  if (command_index >= commands.length()) return;
  
  char cmd = commands.charAt(command_index);
  Serial.print("Starting command: ");
  Serial.println(cmd);
  
  current_command.type = cmd;
  current_substep = 0;
  
  switch (cmd) {
    case 'F':
      current_command.distance = 25.0; // 25 cm forward
      current_command.substeps = FORWARD_SUBSTEPS;
      break;
    case 'L':
      current_command.distance = 90.0; // 90 degree left turn
      current_command.substeps = TURN_SUBSTEPS;
      break;
    case 'R':
      current_command.distance = 90.0; // 90 degree right turn
      current_command.substeps = TURN_SUBSTEPS;
      break;
  }
  
  current_command.completed = false;
  executeNextSubstep();
}

void executeNextSubstep() {
  if (current_substep >= current_command.substeps) {
    // Command completed, move to next
    command_index++;
    if (command_index < commands.length()) {
      initializeNextCommand();
    }
    return;
  }
  
  float progress = (float)current_substep / (float)current_command.substeps;
  
  if (current_command.type == 'F') {
    executeForwardSubstep(progress);
  } else {
    executeTurnSubstep(progress);
  }
  
  current_substep++;
}

void executeForwardSubstep(float progress) {
  // Distance for this substep
  float substep_distance = current_command.distance / current_command.substeps;
  long substep_steps = substep_distance * STEPS_PER_CM;
  
  // Apply error correction
  substep_steps -= (m1_state.steps_error + m2_state.steps_error) / 2;
  
  // Set targets (same for both wheels)
  m1_state.target_steps = m1_state.actual_steps + substep_steps;
  m2_state.target_steps = m2_state.actual_steps + substep_steps;
  
  // Calculate PPS using interpolation (1000 at 50% down to 400 at end)
  float target_pps = calculateForwardPPS(progress);
  m1_state.target_pps = target_pps * PPS_FACTOR;
  m2_state.target_pps = target_pps * PPS_FACTOR;
  
  Serial.print("Forward substep, PPS: ");
  Serial.println(target_pps);
}

void executeTurnSubstep(float progress) {
  // Calculate turn parameters
  float turn_angle_rad = radians(current_command.distance / current_command.substeps);
  
  // Apply turn angle error correction
  turn_angle_rad += turn_angle_error / current_command.substeps;
  
  // Calculate arc lengths for differential drive
  float outer_radius = TURN_RADIUS + (WHEEL_BASE / 2.0);
  float inner_radius = TURN_RADIUS - (WHEEL_BASE / 2.0);
  
  float outer_arc = outer_radius * turn_angle_rad;
  float inner_arc = inner_radius * turn_angle_rad;
  
  long outer_steps = outer_arc * STEPS_PER_CM;
  long inner_steps = inner_arc * STEPS_PER_CM;
  
  // Apply step error correction
  if (current_command.type == 'L') {
    // Left turn: M2 (right wheel) is outer, M1 (left wheel) is inner
    m2_state.target_steps = m2_state.actual_steps + outer_steps - m2_state.steps_error;
    m1_state.target_steps = m1_state.actual_steps + inner_steps - m1_state.steps_error;
  } else {
    // Right turn: M1 (left wheel) is outer, M2 (right wheel) is inner
    m1_state.target_steps = m1_state.actual_steps + outer_steps - m1_state.steps_error;
    m2_state.target_steps = m2_state.actual_steps + inner_steps - m2_state.steps_error;
  }
  
  // Calculate PPS using interpolation (300 -> 450 -> 300)
  float target_pps = calculateTurnPPS(progress);
  
  if (current_command.type == 'L') {
    // M2 is master for left turns
    m2_state.target_pps = target_pps * PPS_FACTOR;
    m1_state.target_pps = (target_pps * inner_radius / outer_radius) * PPS_FACTOR;
  } else {
    // M1 is master for right turns
    m1_state.target_pps = target_pps * PPS_FACTOR;
    m2_state.target_pps = (target_pps * inner_radius / outer_radius) * PPS_FACTOR;
  }
  
  Serial.print("Turn substep, Master PPS: ");
  Serial.println(target_pps);
}

// =============================================================================
// PPS CALCULATION FUNCTIONS
// =============================================================================
float calculateForwardPPS(float progress) {
  // Interpolation: 1000 at 50% down to 400 at end
  if (progress <= 0.5) {
    return FORWARD_BASE_PPS; // 1000 PPS for first 50%
  } else {
    // Linear interpolation from 1000 to 400 for remaining 50%
    float local_progress = (progress - 0.5) / 0.5;
    return FORWARD_BASE_PPS * (1.0 - 0.6 * local_progress); // 1000 to 400
  }
}

float calculateTurnPPS(float progress) {
  // Interpolation: 300 -> 450 -> 300 (smooth bell curve)
  float mid_progress = 0.5;
  float max_pps = 450.0;
  float min_pps = 300.0;
  
  if (progress <= mid_progress) {
    // 300 to 450
    float local_progress = progress / mid_progress;
    return min_pps + (max_pps - min_pps) * local_progress;
  } else {
    // 450 to 300
    float local_progress = (progress - mid_progress) / mid_progress;
    return max_pps - (max_pps - min_pps) * local_progress;
  }
}

// =============================================================================
// MOVEMENT COMPLETION AND ERROR TRACKING
// =============================================================================
bool isMovementComplete() {
  long m1_error = abs(m1_state.target_steps - m1_state.actual_steps);
  long m2_error = abs(m2_state.target_steps - m2_state.actual_steps);
  
  return (m1_error < 10 && m2_error < 10); // 10 steps tolerance
}

void completeCurrentSubstep() {
  // Update step errors for next correction
  m1_state.steps_error += (m1_state.actual_steps - m1_state.target_steps);
  m2_state.steps_error += (m2_state.actual_steps - m2_state.target_steps);
  
  // Update turn angle error if this was a turn
  if (current_command.type != 'F') {
    float expected_angle = radians(current_command.distance / current_command.substeps);
    float actual_angle = calculateActualTurnAngle();
    turn_angle_error += (expected_angle - actual_angle);
  }
  
  // Update position
  updateCurrentPosition();
  
  // Execute next substep
  executeNextSubstep();
}

float calculateActualTurnAngle() {
  // Calculate actual turn angle from wheel movements
  long left_steps = m1_state.actual_steps - m1_encoder_count;
  long right_steps = m2_state.actual_steps - m2_encoder_count;
  
  float left_dist = left_steps / STEPS_PER_CM;
  float right_dist = right_steps / STEPS_PER_CM;
  
  return (left_dist - right_dist) / WHEEL_BASE; // radians
}

void updateCurrentPosition() {
  // Update robot position based on wheel movements (simplified)
  long m1_delta = m1_state.actual_steps - m1_encoder_count;
  long m2_delta = m2_state.actual_steps - m2_encoder_count;
  
  float left_dist = m1_delta / STEPS_PER_CM;
  float right_dist = m2_delta / STEPS_PER_CM;
  float avg_dist = (left_dist + right_dist) / 2.0;
  
  current_pos.x += avg_dist * sin(current_pos.heading);
  current_pos.y += avg_dist * cos(current_pos.heading);
  current_pos.heading += (left_dist - right_dist) / WHEEL_BASE;
  
  // Normalize heading
  while (current_pos.heading > 2 * PI) current_pos.heading -= 2 * PI;
  while (current_pos.heading < 0) current_pos.heading += 2 * PI;
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================
void updateEncoderCounts() {
  m1_encoder_count = M1.getEncoderCount();
  m2_encoder_count = M2.getEncoderCount();
}

void resetMotorStates() {
  m1_state = {0, 0, 0, 0, 0, 0};
  m2_state = {0, 0, 0, 0, 0, 0};
  m1_encoder_count = 0;
  m2_encoder_count = 0;
}

void stopMotors() {
  M1.setPWM(0);
  M2.setPWM(0);
}

void printStatus() {
  Serial.print("Cmd:");
  Serial.print(command_index);
  Serial.print("/");
  Serial.print(commands.length());
  Serial.print(" Sub:");
  Serial.print(current_substep);
  Serial.print(" M1:");
  Serial.print(m1_state.actual_steps);
  Serial.print("/");
  Serial.print(m1_state.target_steps);
  Serial.print(" M2:");
  Serial.print(m2_state.actual_steps);
  Serial.print("/");
  Serial.print(m2_state.target_steps);
  Serial.print(" Pos:(");
  Serial.print(current_pos.x, 1);
  Serial.print(",");
  Serial.print(current_pos.y, 1);
  Serial.print(") H:");
  Serial.print(degrees(current_pos.heading), 1);
  Serial.print("° Err:");
  Serial.print(m1_state.steps_error);
  Serial.print(",");
  Serial.print(m2_state.steps_error);
  Serial.print(",");
  Serial.println(degrees(turn_angle_error), 2);
}


a2.py - Can we make M2 as master for left tunn. My Wheel Radius is 3 cm and encoder steps is 660 per RMP. 
Also I want to maintain the target PPS for F as 1000 * facor. For Turn I want to Maintain the Master PPS as 400 * factor  .
 for slave mortoe the PPS should shold . Make sure Step per cm is calculated propery. 

The target PPS should be different for turn and forward. 

For Straignt L use interpolation to break into 10 sub steps movement. the target PPS should be 1000 for 50%. then it should be reached to 400 in the end. 

For Turn. use interpolation to break into 20 steps .  Start with 300 in the middle reach to 450 and then come back to 300 for smoot movement and stable direction. 


Create simple small and resuableble functions. Keep track of targey and actal co-ordinates and current heading and target heading.

Make sure the ratio of left and right PPS is good for 25 cm radius turn for 90 degree. if the turn angle is less or more, add errors to turn angle error. 
for next  steps make adjustment in steps for steps error and turn angle error. keep seperate variables for steps error on m1, m2 and turn angle error.

