#include <math.h>

const float EPSILON = 1e-4;
const float CM_PER_SQUARE = 50.0;
const float PI = 3.14159265359;

// Assume these functions exist
extern void setMotor1(int pwm);    // -255 to 255
extern void setMotor2(int pwm);    // -255 to 255  
extern float getGyroZ();           // Angular velocity in deg/s
extern volatile long encoder1Count; // Left encoder count
extern volatile long encoder2Count; // Right encoder count

struct Command {
  uint8_t command_type;
  float turn;      // Turn angle in radians
  int32_t ticks;   // Movement ticks
  float tw_off;    // Track width offset
};

struct Config {
  int ticks_per_cm;
  float kp_move;
  float kp_hold;
  float kp_straight;
  float kp_velocity;
  float turn_accel_time;
  float straight_accel_time;
  float friction;
  float dowel_off;
  bool reverse;
  bool reverse_enc;
  bool reverse_enc2;
  float imu_weight;
};

struct ConfigCommand {
  float kp_move, kp_hold, kp_straight, kp_velocity;
  float turn_accel_time, straight_accel_time;
  float friction, dowel_off;
  int reverse, reverse_enc, reverse_enc2;
  float imu_weight;
  float velocity, velocity_twoff, time, vtime;
};

struct PlanningResult {
  Command* commands;
  int command_count;
  ConfigCommand config;
};

enum TokenType {
  TOKEN_UP,
  TOKEN_DOWN, 
  TOKEN_LEFT,
  TOKEN_RIGHT
};

struct Token {
  TokenType type;
  float value;
};

// Global configuration
Config config = {
  .ticks_per_cm = 100,
  .kp_move = 1.0,
  .kp_hold = 0.5,
  .kp_straight = 2.0,
  .kp_velocity = 1.2,
  .turn_accel_time = 0.2,
  .straight_accel_time = 0.3,
  .friction = 0.1,
  .dowel_off = 2.0,
  .reverse = false,
  .reverse_enc = false,
  .reverse_enc2 = false,
  .imu_weight = 0.7
};

// Robot state tracking
float current_angle = 0.0;           // Current heading in radians
float gyro_bias = 0.0;               // Gyro bias compensation
unsigned long last_time = 0;         // For gyro integration
float robot_x = 0.0, robot_y = 0.0;  // Position tracking

float mod_floats(float a, float b) {
  return a - round(a / (b + 2.0 * EPSILON)) * b;
}

float target_angle_for_token(TokenType type) {
  switch (type) {
    case TOKEN_UP: return PI / 2.0;     // 90 degrees
    case TOKEN_DOWN: return -PI / 2.0;  // -90 degrees  
    case TOKEN_LEFT: return PI;         // 180 degrees
    case TOKEN_RIGHT: return 0.0;       // 0 degrees
    default: return 0.0;
  }
}

void update_gyro_angle() {
  unsigned long current_time = millis();
  if (last_time == 0) {
    last_time = current_time;
    return;
  }
  
  float dt = (current_time - last_time) / 1000.0;
  float gyro_rate = (getGyroZ() - gyro_bias) * PI / 180.0; // Convert to rad/s
  
  current_angle += gyro_rate * dt;
  current_angle = mod_floats(current_angle, 2.0 * PI);
  
  last_time = current_time;
}

Command plan_token(Token* tok, float* angle, Config* cfg) {
  float dist = tok->value;
  float target_ang = target_angle_for_token(tok->type);
  float dang = target_ang - *angle;
  
  if (dang > PI + EPSILON) {
    dang -= 2.0 * PI;
  } else if (dang < -PI - EPSILON) {
    dang += 2.0 * PI;
  }
  
  dang = mod_floats(dang, PI); // Go backwards instead of 180deg turn
  if (abs(dang) < EPSILON) {
    dang = 0.0;
  }
  
  *angle += dang;
  *angle = mod_floats(*angle, 2.0 * PI);
  
  // Backwards driving check
  if (abs(abs(*angle - target_ang) - PI) < EPSILON) {
    dist = -dist;
  }
  
  Command cmd;
  cmd.command_type = 0; // TurnMove
  cmd.turn = dang;
  cmd.ticks = (int32_t)(dist * cfg->ticks_per_cm * CM_PER_SQUARE);
  cmd.tw_off = 0.0;
  
  return cmd;
}

PlanningResult plan_path(Token* tokens, int token_count, float total_time) {
  static Command commands[50]; // Static allocation for Arduino
  int cmd_count = 0;
  
  // Calculate final position (not used but matches original)
  float xfin = 0.0, yfin = 0.0;
  for (int i = 0; i < token_count; i++) {
    switch (tokens[i].type) {
      case TOKEN_UP: yfin += tokens[i].value; break;
      case TOKEN_DOWN: yfin -= tokens[i].value; break;
      case TOKEN_LEFT: xfin -= tokens[i].value; break;
      case TOKEN_RIGHT: xfin += tokens[i].value; break;
    }
  }
  
  // Plan commands
  float angle = target_angle_for_token(tokens[0].type);
  for (int i = 0; i < token_count; i++) {
    commands[cmd_count++] = plan_token(&tokens[i], &angle, &config);
  }
  
  // Fix ending angle
  float ediff = target_angle_for_token(tokens[token_count-1].type) - angle;
  if (abs(ediff) > EPSILON) {
    // Backtrack to last turn and recalculate
    for (int i = cmd_count - 1; i >= 0; i--) {
      if (abs(commands[i].turn) > EPSILON) {
        commands[i].turn += ediff;
        if (commands[i].turn > PI) {
          commands[i].turn -= 2.0 * PI;
        } else if (commands[i].turn < -PI) {
          commands[i].turn += 2.0 * PI;
        }
        angle = target_angle_for_token(tokens[token_count-1].type);
        
        // Re-calculate commands after this one
        int new_count = i + 1;
        for (int j = i; j < token_count; j++) {
          Command res = plan_token(&tokens[j], &angle, &config);
          if (j == i) {
            commands[i].ticks = res.ticks;
            commands[i].tw_off = res.tw_off;
          } else {
            commands[new_count++] = res;
          }
        }
        cmd_count = new_count;
        break;
      }
    }
  }
  
  // Add dowel offset
  commands[0].ticks += (int32_t)(config.dowel_off * config.ticks_per_cm) * 
                       (commands[0].ticks >= 0 ? 1 : -1);
  commands[cmd_count-1].ticks -= (int32_t)(config.dowel_off * config.ticks_per_cm) * 
                                 (commands[cmd_count-1].ticks >= 0 ? 1 : -1);
  
  // Calculate velocity and tw_off
  float velocity = 0.0;
  float velocity_twoff = 0.0;
  float vtime = total_time;
  
  for (int i = 0; i < cmd_count - 1; i++) {
    velocity += abs(commands[i].ticks);
    velocity_twoff += abs(commands[i].tw_off);
    vtime -= config.straight_accel_time * 2.0;
    
    if (abs(commands[i + 1].turn) > EPSILON) {
      velocity_twoff += abs(commands[i + 1].turn);
      vtime -= 2.0 * config.turn_accel_time;
    }
    
    if (i == cmd_count - 2) {
      velocity += abs(commands[i + 1].ticks);
      velocity_twoff += abs(commands[i + 1].tw_off);
      vtime -= config.straight_accel_time * 2.0;
    }
    
    // Calculate tw_off
    if (abs(commands[i + 1].turn) < EPSILON) continue;
    commands[i].tw_off -= 0.5;
    commands[i + 1].tw_off -= 0.5;
  }
  
  // Print results
  Serial.println("=== PLANNED PATH ===");
  for (int i = 0; i < cmd_count; i++) {
    if (abs(commands[i].turn) > EPSILON) {
      Serial.print("Turn: ");
      Serial.print(commands[i].turn * 180.0 / PI);
      Serial.println(" degrees");
    }
    Serial.print("Move: ");
    Serial.print(commands[i].ticks);
    Serial.print(" ticks ");
    Serial.print(commands[i].tw_off < 0 ? "-" : "+");
    Serial.print(abs(commands[i].tw_off));
    Serial.println(" track width");
  }
  Serial.print("Distance: ");
  Serial.print(velocity / config.ticks_per_cm);
  Serial.println(" cm");
  Serial.print("Velocity: ");
  Serial.print(velocity / vtime);
  Serial.println(" ticks/sec");
  
  PlanningResult result;
  result.commands = commands;
  result.command_count = cmd_count;
  // Fill config command (simplified)
  result.config.velocity = velocity;
  result.config.time = total_time;
  result.config.vtime = vtime;
  
  return result;
}
  // IMPROVED TURNING with PID control
  void execute_turn_improved(float target_turn) {
    float target_angle = normalize_angle(current_angle + target_turn);
    float last_error = 0;
    float integral = 0;
  
    while (true) {
      update_gyro_angle();
      float error = normalize_angle(target_angle - current_angle);
    
      if (abs(error) < 0.02) break;  // Tighter tolerance
    
      // PID Control
      integral += error * 0.01;  // I term
      float derivative = (error - last_error) / 0.01;  // D term
    
      float control = (error * 2.0) + (integral * 0.1) + (derivative * 0.05);
    
      // Deceleration profile
      if (abs(error) < 0.2) control *= 0.5;  // Slow down near target
    
      setMotor1(-constrain(control, -255, 255));
      setMotor2(constrain(control, -255, 255));
    
      last_error = error;
      delay(10);
    }
  }

  // IMPROVED MOVEMENT with encoder prediction
  void execute_movement_improved(int target_ticks) {
    encoder1Count = 0;
    encoder2Count = 0;
  
    while (abs(encoder1Count) < abs(target_ticks)) {
      int remaining1 = abs(target_ticks) - abs(encoder1Count);
      int remaining2 = abs(target_ticks) - abs(encoder2Count);
    
      // Deceleration profile
      int max_speed = 200;
      if (remaining1 < 100) max_speed = max_speed * remaining1 / 100;
    
      // Multi-sensor fusion
      float gyro_error = normalize_angle(current_angle - target_heading);
      int encoder_diff = encoder1Count - encoder2Count;
    
      // Advanced control
      int speed1 = max_speed - (gyro_error * 50) - (encoder_diff * 2);
      int speed2 = max_speed + (gyro_error * 50) + (encoder_diff * 2);
    
      setMotor1(constrain(speed1, -255, 255));
      setMotor2(constrain(speed2, -255, 255));
    
      delay(5);  // Faster update rate
    }
  }

  void execute_turn(float turn_radians) {
    if (abs(turn_radians) < EPSILON) return;
  
    float target_angle = current_angle + turn_radians;
    target_angle = mod_floats(target_angle, 2.0 * PI);
  
    Serial.print("Turning ");
    Serial.print(turn_radians * 180.0 / PI);
    Serial.println(" degrees");
  
    int turn_speed = 120;
    if (turn_radians > 0) {
      // Turn left (counter-clockwise)
      setMotor1(-turn_speed);
      setMotor2(turn_speed);
    } else {
      // Turn right (clockwise)  
      setMotor1(turn_speed);
      setMotor2(-turn_speed);
    }
  
    while (true) {
      update_gyro_angle();
      float remaining = target_angle - current_angle;
    
      // Normalize to [-PI, PI]
      if (remaining > PI) remaining -= 2.0 * PI;
      if (remaining < -PI) remaining += 2.0 * PI;
    
      if (abs(remaining) < 0.05) break; // ~3 degree tolerance
    
      // Proportional control
      float control = constrain(remaining * 100, -turn_speed, turn_speed);
      setMotor1(-control);
      setMotor2(control);
    
      delay(10);
    }
  
    setMotor1(0);
    setMotor2(0);
    delay(100);
  }

  void execute_movement(int32_t ticks, float tw_off) {
    if (ticks == 0) return;
  
    Serial.print("Moving ");
    Serial.print(ticks);
    Serial.println(" ticks");
  
    // Reset encoders
    encoder1Count = 0;
    encoder2Count = 0;
  
    int base_speed = (ticks > 0) ? 150 : -150;
    int speed_offset = (int)(tw_off * 20); // Adjust multiplier as needed
  
    int target_ticks = abs(ticks);
  
    while (abs(encoder1Count) < target_ticks || abs(encoder2Count) < target_ticks) {
      update_gyro_angle();
    
      // Calculate remaining distance for each motor
      int remaining1 = target_ticks - abs(encoder1Count);
      int remaining2 = target_ticks - abs(encoder2Count);
    
      // Gyro-based straight line correction
      float angle_error = current_angle - target_angle_for_token(TOKEN_RIGHT); // Assume moving straight
      int gyro_correction = (int)(angle_error * config.kp_straight * 100);
    
      // Encoder-based correction
      int encoder_diff = encoder1Count - encoder2Count;
      int encoder_correction = (int)(encoder_diff * 0.5);
    
      // Calculate motor speeds
      int speed1 = base_speed + speed_offset - gyro_correction - encoder_correction;
      int speed2 = base_speed - speed_offset + gyro_correction + encoder_correction;
    
      // Slow down near end
      if (remaining1 < 100) speed1 = speed1 * remaining1 / 100;
      if (remaining2 < 100) speed2 = speed2 * remaining2 / 100;
    
      setMotor1(constrain(speed1, -255, 255));
      setMotor2(constrain(speed2, -255, 255));
    
      delay(10);
    }
  
    setMotor1(0);
    setMotor2(0);
    delay(100);
  }
void execute_command(Command* cmd) {
  execute_turn(cmd->turn);
  execute_movement(cmd->ticks, cmd->tw_off);
}

void calibrate_gyro() {
  Serial.println("Calibrating gyro... keep robot still");
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += getGyroZ();
    delay(50);
  }
  gyro_bias = sum / 100.0;
  Serial.print("Gyro bias: ");
  Serial.println(gyro_bias);
}

void parse_and_execute_path(String path_data) {
  static Token tokens[20];
  int token_count = 0;
  float total_time = 0.0;
  
  // Parse the path data
  int start = 0;
  while (start < path_data.length()) {
    int end = path_data.indexOf('\n', start);
    if (end == -1) end = path_data.length();
    
    String line = path_data.substring(start, end);
    line.trim();
    start = end + 1;
    
    if (line.length() == 0 || line.startsWith("#")) continue;
    
    int space = line.indexOf(' ');
    if (space == -1) continue;
    
    String command = line.substring(0, space);
    float value = line.substring(space + 1).toFloat();
    
    command.toLowerCase();
    
    if (command == "up") {
      tokens[token_count++] = {TOKEN_UP, value};
    } else if (command == "down") {
      tokens[token_count++] = {TOKEN_DOWN, value};
    } else if (command == "left") {
      tokens[token_count++] = {TOKEN_LEFT, value};
    } else if (command == "right") {
      tokens[token_count++] = {TOKEN_RIGHT, value};
    } else if (command == "time") {
      total_time = value;
    }
  }
  
  if (total_time <= 0.0) {
    Serial.println("Error: No time specified");
    return;
  }
  
  Serial.print("Parsed ");
  Serial.print(token_count);
  Serial.println(" commands");
  
  // Plan the path
  PlanningResult result = plan_path(tokens, token_count, total_time);
  
  Serial.println("Starting execution in 3 seconds...");
  delay(3000);
  
  // Execute all commands
  for (int i = 0; i < result.command_count; i++) {
    Serial.print("Command ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.println(result.command_count);
    execute_command(&result.commands[i]);
    delay(200); // Small pause between commands
  }
  
  Serial.println("Path execution complete!");
}

void setup() {
  Serial.begin(9600);
  Serial.println("Robot Path Planner Ready");
  Serial.println("Send path data or use commands:");
  Serial.println("CAL - Calibrate gyro");  
  Serial.println("RESET - Reset position");
  Serial.println("Example path format:");
  Serial.println("time 10.0");
  Serial.println("up 2");  
  Serial.println("right 3");
  Serial.println("down 1");
  
  calibrate_gyro();
  last_time = millis();
}

void loop() {
  update_gyro_angle();
  
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.equalsIgnoreCase("CAL")) {
      calibrate_gyro();
    } else if (input.equalsIgnoreCase("RESET")) {
      current_angle = 0.0;
      robot_x = robot_y = 0.0;
      encoder1Count = encoder2Count = 0;
      Serial.println("Position and angle reset");
    } else if (input.length() > 0) {
      // Assume it's path data
      String full_path = input + "\n";
      
      // Read additional lines if they come quickly
      delay(100);
      while (Serial.available()) {
        full_path += Serial.readStringUntil('\n') + "\n";
        delay(50);
      }
      
      parse_and_execute_path(full_path);
    }
  }
}