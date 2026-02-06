#include <RobotBuilder.h>  // For RB_EncoderMotor
// #include <IMU_Library.h>  // Replace with your actual IMU library

// =============================================================================
// CONFIGURATION & CONSTANTS
// =============================================================================
const float WHEEL_BASE = 20.0;        // cm - distance between wheels
const int STEPS_PER_CM = 100;         // 100 encoder steps = 1 cm
const float TURN_RADIUS = 25.0;       // cm
const float FORWARD_DISTANCE = 25.0;  // cm
const int BASE_PWM = 150;             // Base PWM value
const int MAX_PWM = 255;              // Maximum PWM value
const int MIN_PWM = 50;               // Minimum PWM value

// =============================================================================
// MOTOR AND SENSOR OBJECTS
// =============================================================================
RB_EncoderMotor M1(1);  // Master motor
RB_EncoderMotor M2(2);  // Slave motor

// IMU object (replace with your actual IMU library)
// IMU myImu;

// =============================================================================
// DATA STRUCTURES
// =============================================================================
struct MotorCommand {
  int left_steps;
  int right_steps;
  char command_type;  // 'F', 'R', 'L'
};

struct MotorState {
  long target_steps;
  long actual_steps;
  long accumulated_error;
  int current_pwm;
  float current_pps;  // Pulses per second
  float target_pps;
};

struct RobotState {
  float x;
  float y;
  float heading;          // radians
  float imu_heading;      // IMU heading for comparison
  int command_index;
  bool movement_complete;
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
MotorState m1_state = {0, 0, 0, BASE_PWM, 0, 0};
MotorState m2_state = {0, 0, 0, BASE_PWM, 0, 0};
RobotState robot_state = {0, 0, 0, 0, 0, true};

volatile long m1_count = 0;
volatile long m2_count = 0;

// Command mapping and step definitions
const String commands = "RRLLFF";
MotorCommand command_map[3] = {
  {8000, 8000, 'F'},   // Forward: F-1 (smooth steps for 25cm)
  {10000, 8000, 'R'},  // Right: R-2 (left wheel more steps)
  {8000, 10000, 'L'}   // Left: L-3 (right wheel more steps)
};

unsigned long last_update_time = 0;
unsigned long last_imu_update = 0;
const unsigned long UPDATE_INTERVAL = 50;   // 50ms update interval
const unsigned long IMU_INTERVAL = 100;     // 100ms IMU update interval

// =============================================================================
// SETUP FUNCTION
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Robot Controller Starting...");
  
  // Initialize motors
  M1.begin();
  M2.begin();
  
  // Initialize IMU (placeholder)
  // myImu.begin();
  
  // Setup encoder interrupts (example - adjust pins as needed)
  // attachInterrupt(digitalPinToInterrupt(2), m1EncoderISR, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(3), m2EncoderISR, CHANGE);
  
  Serial.println("System initialized. Starting command execution...");
  
  // Reset states
  resetMotorStates();
  robot_state.command_index = 0;
  robot_state.movement_complete = false;
  
  // Start first command
  if (commands.length() > 0) {
    executeNextCommand();
  }
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  unsigned long current_time = millis();
  
  // Update motor control at regular intervals
  if (current_time - last_update_time >= UPDATE_INTERVAL) {
    updateEncoderCounts();
    updateMotorControl();
    checkMovementComplete();
    printStatus();
    last_update_time = current_time;
  }
  
  // Update IMU at regular intervals
  if (current_time - last_imu_update >= IMU_INTERVAL) {
    updateImu();
    compareHeadings();
    last_imu_update = current_time;
  }
  
  // Execute next command if current movement is complete
  if (robot_state.movement_complete && robot_state.command_index < commands.length()) {
    delay(500);  // Brief pause between commands
    executeNextCommand();
  }
  
  // Check if all commands completed
  if (robot_state.command_index >= commands.length()) {
    Serial.println("All commands completed!");
    stopMotors();
    while(1) {
      delay(1000);
      Serial.println("Mission complete. Reset to restart.");
    }
  }
}

// =============================================================================
// MOTOR CONTROL FUNCTIONS
// =============================================================================
void updateMotorControl() {
  // Update actual steps from encoders
  m1_state.actual_steps = m1_count;
  m2_state.actual_steps = m2_count;
  
  // Calculate current errors
  long m1_error = m1_state.target_steps - m1_state.actual_steps;
  long m2_error = m2_state.target_steps - m2_state.actual_steps;
  
  // Update accumulated errors
  m1_state.accumulated_error += m1_error;
  m2_state.accumulated_error += m2_error;
  
  // Calculate PWM adjustments
  int m1_pwm = calculatePWM(m1_state, m1_error, true);   // M1 is master
  int m2_pwm = calculatePWM(m2_state, m2_error, false);  // M2 is slave
  
  // Apply PWM values
  if (abs(m1_error) > 10) {  // Dead zone to prevent oscillation
    M1.setPWM(m1_pwm);
    m1_state.current_pwm = m1_pwm;
  } else {
    M1.setPWM(0);
    m1_state.current_pwm = 0;
  }
  
  if (abs(m2_error) > 10) {
    M2.setPWM(m2_pwm);
    m2_state.current_pwm = m2_pwm;
  } else {
    M2.setPWM(0);
    m2_state.current_pwm = 0;
  }
}

int calculatePWM(MotorState &motor, long error, bool is_master) {
  // PID-like control for PWM calculation
  float kp = 0.1;    // Proportional gain
  float ki = 0.01;   // Integral gain (for accumulated error)
  float kd = 0.05;   // Derivative gain
  
  static long last_error_m1 = 0;
  static long last_error_m2 = 0;
  
  long* last_error = is_master ? &last_error_m1 : &last_error_m2;
  long derivative = error - *last_error;
  *last_error = error;
  
  // Calculate PWM adjustment
  float pwm_adjustment = kp * error + ki * motor.accumulated_error + kd * derivative;
  
  // For slave motor (M2), add ratio adjustment to follow master
  if (!is_master) {
    float speed_ratio = (float)m2_state.target_steps / (float)m1_state.target_steps;
    float master_speed = m1_state.current_pps;
    pwm_adjustment += (master_speed * speed_ratio - motor.current_pps) * 10;
  }
  
  int new_pwm = constrain(motor.current_pwm + pwm_adjustment, MIN_PWM, MAX_PWM);
  
  // Set direction based on error
  if (error > 0) {
    return new_pwm;  // Forward
  } else if (error < 0) {
    return -new_pwm; // Reverse
  } else {
    return 0;        // Stop
  }
}

// =============================================================================
// COMMAND EXECUTION FUNCTIONS
// =============================================================================
void executeNextCommand() {
  if (robot_state.command_index >= commands.length()) return;
  
  char cmd = commands.charAt(robot_state.command_index);
  Serial.print("Executing command: ");
  Serial.println(cmd);
  
  MotorCommand motor_cmd;
  
  // Map command to motor steps
  switch (cmd) {
    case 'F':
      motor_cmd = command_map[0];  // F-1
      break;
    case 'R':
      motor_cmd = command_map[1];  // R-2
      break;
    case 'L':
      motor_cmd = command_map[2];  // L-3
      break;
    default:
      Serial.println("Unknown command!");
      robot_state.command_index++;
      return;
  }
  
  // Apply error correction from previous movement
  motor_cmd.left_steps -= m1_state.accumulated_error;
  motor_cmd.right_steps -= m2_state.accumulated_error;
  
  // Set new targets
  m1_state.target_steps = m1_state.actual_steps + motor_cmd.left_steps;
  m2_state.target_steps = m2_state.actual_steps + motor_cmd.right_steps;
  
  // Reset movement state
  robot_state.movement_complete = false;
  robot_state.command_index++;
  
  Serial.print("Target steps - M1: ");
  Serial.print(m1_state.target_steps);
  Serial.print(", M2: ");
  Serial.println(m2_state.target_steps);
}

void checkMovementComplete() {
  long m1_error = abs(m1_state.target_steps - m1_state.actual_steps);
  long m2_error = abs(m2_state.target_steps - m2_state.actual_steps);
  
  // Movement complete when both motors are within tolerance
  if (m1_error < 20 && m2_error < 20) {  // 20 steps tolerance
    robot_state.movement_complete = true;
    stopMotors();
    
    Serial.println("Movement complete!");
    Serial.print("Final errors - M1: ");
    Serial.print(m1_error);
    Serial.print(", M2: ");
    Serial.println(m2_error);
  }
}

// =============================================================================
// IMU FUNCTIONS (Placeholder)
// =============================================================================
void updateImu() {
  // Placeholder function - replace with actual IMU update code
  // robot_state.imu_heading = myImu.getHeading();
  
  // For simulation, calculate expected heading from motor steps
  static long last_m1_steps = 0;
  static long last_m2_steps = 0;
  
  long m1_delta = m1_state.actual_steps - last_m1_steps;
  long m2_delta = m2_state.actual_steps - last_m2_steps;
  
  float left_dist = m1_delta / (float)STEPS_PER_CM;
  float right_dist = m2_delta / (float)STEPS_PER_CM;
  
  float delta_heading = (left_dist - right_dist) / WHEEL_BASE;
  robot_state.heading += delta_heading;
  
  // Normalize heading
  while (robot_state.heading > 2 * PI) robot_state.heading -= 2 * PI;
  while (robot_state.heading < 0) robot_state.heading += 2 * PI;
  
  last_m1_steps = m1_state.actual_steps;
  last_m2_steps = m2_state.actual_steps;
  
  // Simulate IMU heading (add some noise/drift for testing)
  robot_state.imu_heading = robot_state.heading + random(-5, 5) * 0.01;
}

float getImuHead() {
  // Placeholder function - replace with actual IMU read
  return robot_state.imu_heading;
}

void compareHeadings() {
  float calc_heading_deg = robot_state.heading * 180.0 / PI;
  float imu_heading_deg = robot_state.imu_heading * 180.0 / PI;
  float heading_error = calc_heading_deg - imu_heading_deg;
  
  Serial.print("Heading - Calculated: ");
  Serial.print(calc_heading_deg, 1);
  Serial.print("°, IMU: ");
  Serial.print(imu_heading_deg, 1);
  Serial.print("°, Error: ");
  Serial.print(heading_error, 1);
  Serial.println("°");
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================
void updateEncoderCounts() {
  // Read encoder counts from motors
  m1_count = M1.getEncoderCount();
  m2_count = M2.getEncoderCount();
}

void resetMotorStates() {
  m1_state = {0, 0, 0, BASE_PWM, 0, 0};
  m2_state = {0, 0, 0, BASE_PWM, 0, 0};
  m1_count = 0;
  m2_count = 0;
}

void stopMotors() {
  M1.setPWM(0);
  M2.setPWM(0);
  m1_state.current_pwm = 0;
  m2_state.current_pwm = 0;
}

void printStatus() {
  Serial.print("Cmd: ");
  Serial.print(robot_state.command_index);
  Serial.print("/");
  Serial.print(commands.length());
  Serial.print(" | M1: ");
  Serial.print(m1_state.actual_steps);
  Serial.print("/");
  Serial.print(m1_state.target_steps);
  Serial.print(" (");
  Serial.print(m1_state.current_pwm);
  Serial.print(") | M2: ");
  Serial.print(m2_state.actual_steps);
  Serial.print("/");
  Serial.print(m2_state.target_steps);
  Serial.print(" (");
  Serial.print(m2_state.current_pwm);
  Serial.print(") | Errors: ");
  Serial.print(m1_state.accumulated_error);
  Serial.print(",");
  Serial.println(m2_state.accumulated_error);
}

// =============================================================================
// INTERRUPT SERVICE ROUTINES (Example)
// =============================================================================
/*
void m1EncoderISR() {
  // Encoder interrupt for M1
  // Implement based on your encoder setup
  if (digitalRead(M1_ENCODER_A) == digitalRead(M1_ENCODER_B)) {
    m1_count++;
  } else {
    m1_count--;
  }
}

void m2EncoderISR() {
  // Encoder interrupt for M2
  // Implement based on your encoder setup
  if (digitalRead(M2_ENCODER_A) == digitalRead(M2_ENCODER_B)) {
    m2_count++;
  } else {
    m2_count--;
  }
}
*/








Convert the following code to Ardnui Sketch ino.

1. I have two Motoes RB_EncoderMotor M1(1) , M(2) with encoders m1_count and m2_count 
2. Make a data structires which maps command F-1 , R-2 , L- 3 and calculate or stoes the steps based on encoder dim and wheel dimension

imu myImu 

My turn radius is 25 CM and my F is 25 CM in current direction

R_Steps = (1000,800) * 10 or a better smoot steps 
L_Steps = (800,1000) * 10 or a better smoot steps
F_Steps = (800,800) * 10 or a better smoot steps

M1 is master motor -- will try to maintain the PPS speed - the pwn should be truented by current PPP/Speed, target PPS Speed and current PWN . 
M2 is slave motor -- will try to follow M1 spped - including adjustment for M1/M2 target speed ratio, current accumulated error . 

Keep track of current heading and compate and print with imu heading. updateImu() getImuHead() placeholder functions to update and get imu .

commands ="RRLLFF"

Accumulate and ajust Errors in M1 and M2 Tager and actuak steps and ajust in next steps. So if in prevois step if there was an overshoot of 50 steps the current step should reduct the target by 50 steps. 






import math
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# Configure matplotlib
import matplotlib
matplotlib.use('TkAgg')
plt.ion()

# =============================================================================
# CONFIGURATION & CONSTANTS
# =============================================================================
GRID_SIZE = 500.0        # cm - 500x500 grid
WHEEL_BASE = 20.0        # cm - distance between wheels
STEPS_PER_CM = 100       # 100 encoder steps = 1 cm
ROBOT_SIZE = 10.0        # cm - robot visualization size

# =============================================================================
# DATA STRUCTURES
# =============================================================================
@dataclass
class RobotState:
    """Robot position and orientation state"""
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians (North = 0°)
    left_distance: float = 0.0   # cumulative left wheel distance
    right_distance: float = 0.0  # cumulative right wheel distance
    
    def get_heading_deg(self) -> float:
        return math.degrees(self.heading)

@dataclass
class StepCommand:
    """Left and right motor step command"""
    left_steps: int
    right_steps: int
    
    def get_distances(self) -> Tuple[float, float]:
        """Convert steps to distances in cm"""
        left_dist = self.left_steps / STEPS_PER_CM
        right_dist = self.right_steps / STEPS_PER_CM
        return left_dist, right_dist

# =============================================================================
# ROBOT STEP SIMULATOR
# =============================================================================
class RobotStepSimulator:
    def __init__(self, start_x: float = 0.0, start_y: float = 0.0, start_heading: float = 0.0):
        self.start_x = start_x
        self.start_y = start_y
        self.start_heading = start_heading
        self.robot_states: List[RobotState] = []
        self.step_commands: List[StepCommand] = []
        
    def process_step_array(self, steps_array: List[Tuple[int, int]]):
        """Process array of (left_steps, right_steps) and simulate robot movement"""
        print(f"Processing {len(steps_array)} step commands...")
        if True:
            # Initialize robot state
            current_state = RobotState(self.start_x, self.start_y, self.start_heading)
            self.robot_states = [current_state]
            self.step_commands = []
            
            for i, (left_steps, right_steps) in enumerate(steps_array):
                step_cmd = StepCommand(left_steps, right_steps)
                self.step_commands.append(step_cmd)
                
                # Calculate new robot state
                new_state = self.calculate_next_state(current_state, step_cmd)
                self.robot_states.append(new_state)
                
                print(f"Step {i+1}: L={left_steps}, R={right_steps} -> "
                    f"Pos=({new_state.x:.1f}, {new_state.y:.1f}), "
                    f"Heading={new_state.get_heading_deg():.1f}°")
                
                current_state = new_state
                
        print(f"Simulation complete. Generated {len(self.robot_states)} robot states.")
    def calculate_next_state(self, current_state: RobotState, step_cmd: StepCommand) -> RobotState:
        """Calculate next robot state using differential drive kinematics"""
        left_dist, right_dist = step_cmd.get_distances()

        # Calculate movement parameters
        distance = (left_dist + right_dist) / 2.0  # Average distance moved
        delta_heading = (left_dist - right_dist) / WHEEL_BASE  # Fixed: left - right for proper turn direction

        # Create new state
        new_state = RobotState()

        # Update cumulative wheel distances
        new_state.left_distance = current_state.left_distance + left_dist
        new_state.right_distance = current_state.right_distance + right_dist

        if abs(delta_heading) < 1e-6:
            # Straight line movement
            new_state.heading = current_state.heading
            new_state.x = current_state.x + distance * math.sin(current_state.heading)
            new_state.y = current_state.y + distance * math.cos(current_state.heading)
        else:
            # Curved movement (differential drive)
            # Update heading first
            new_state.heading = current_state.heading + delta_heading
            
            # Normalize heading to [0, 2π]
            new_state.heading = new_state.heading % (2 * math.pi)
            if new_state.heading < 0:
                new_state.heading += 2 * math.pi
                
            # For curved movement, use the average heading for position calculation
            avg_heading = current_state.heading + delta_heading / 2.0
            
            # Calculate new position using average heading
            new_state.x = current_state.x + distance * math.sin(avg_heading)
            new_state.y = current_state.y + distance * math.cos(avg_heading)

        return new_state
    def display_path(self):
        """Display the robot path simulation"""
        if len(self.robot_states) < 2:
            print("No path to display. Need at least 2 states.")
            return
            
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Main path plot
        ax1.set_xlim(0, GRID_SIZE)
        ax1.set_ylim(0, GRID_SIZE)
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlabel('X (cm)')
        ax1.set_ylabel('Y (cm)')
        ax1.set_title('Robot Path from Encoder Steps')
        
        # Extract path coordinates
        path_x = [state.x for state in self.robot_states]
        path_y = [state.y for state in self.robot_states]
        
        # Plot path
        ax1.plot(path_x, path_y, 'b-', linewidth=3, alpha=0.7, label='Robot Path')
        ax1.plot(path_x[0], path_y[0], 'go', markersize=12, label='Start')
        ax1.plot(path_x[-1], path_y[-1], 'ro', markersize=12, label='End')
        
        # Add direction arrows
        arrow_step = max(1, len(self.robot_states) // 10)
        for i in range(0, len(self.robot_states)-1, arrow_step):
            state = self.robot_states[i]
            dx = 15 * math.sin(state.heading)
            dy = 15 * math.cos(state.heading)
            ax1.arrow(state.x, state.y, dx, dy, head_width=8, head_length=8, 
                     fc='red', ec='red', alpha=0.6)
        
        # Add North arrow at start
        start_state = self.robot_states[0]
        ax1.arrow(start_state.x, start_state.y, 0, 25, head_width=5, head_length=5, 
                 fc='green', ec='green', alpha=0.8)
        ax1.text(start_state.x + 10, start_state.y + 30, 'N (0°)', fontsize=10, 
                fontweight='bold', color='green')
        
        # Plot step points
        for i, state in enumerate(self.robot_states[1:], 1):
            ax1.plot(state.x, state.y, 'ko', markersize=4, alpha=0.6)
            if i <= 10:  # Label first 10 points
                ax1.text(state.x + 5, state.y + 5, f'{i}', fontsize=8, alpha=0.7)
        
        ax1.legend()
        
        # Step analysis plot
        ax2.set_title('Step Commands Analysis')
        steps = range(1, len(self.step_commands) + 1)
        left_steps = [cmd.left_steps for cmd in self.step_commands]
        right_steps = [cmd.right_steps for cmd in self.step_commands]
        
        ax2.bar([s - 0.2 for s in steps], left_steps, 0.4, label='Left Steps', alpha=0.7)
        ax2.bar([s + 0.2 for s in steps], right_steps, 0.4, label='Right Steps', alpha=0.7)
        ax2.set_xlabel('Step Command #')
        ax2.set_ylabel('Encoder Steps')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Add info text
        info_text = f"Robot Parameters:\n"
        info_text += f"Wheel Base: {WHEEL_BASE}cm\n"
        info_text += f"Steps/cm: {STEPS_PER_CM}\n"
        info_text += f"Grid: (0,0) to ({GRID_SIZE:.0f},{GRID_SIZE:.0f})\n"
        info_text += f"Start: ({self.start_x:.1f},{self.start_y:.1f})\n"
        info_text += f"Total Commands: {len(self.step_commands)}\n"
        
        final_state = self.robot_states[-1]
        info_text += f"\nFinal State:\n"
        info_text += f"Position: ({final_state.x:.1f}, {final_state.y:.1f})\n"
        info_text += f"Heading: {final_state.get_heading_deg():.1f}°\n"
        info_text += f"Left Dist: {final_state.left_distance:.1f}cm\n"
        info_text += f"Right Dist: {final_state.right_distance:.1f}cm"
        
        ax1.text(0.02, 0.98, info_text, transform=ax1.transAxes, 
                verticalalignment='top', bbox=dict(boxstyle="round,pad=0.3", 
                facecolor="lightgray", alpha=0.8), fontfamily='monospace', fontsize=9)
        
        plt.tight_layout()
        plt.show()
    
    def print_simulation_data(self):
        """Print detailed simulation data"""
        print("\n" + "="*100)
        print("ROBOT STEP SIMULATION DATA")
        print("="*100)
        print(f"{'Step':<4} {'L_Steps':<8} {'R_Steps':<8} {'L_Dist':<8} {'R_Dist':<8} "
              f"{'X':<8} {'Y':<8} {'Heading':<8} {'ΔHeading':<10}")
        print("-"*100)
        
        prev_heading = self.start_heading
        for i, (state, cmd) in enumerate(zip(self.robot_states[1:], self.step_commands)):
            left_dist, right_dist = cmd.get_distances()
            delta_heading = math.degrees(state.heading - prev_heading)
            
            # Normalize delta heading
            while delta_heading > 180:
                delta_heading -= 360
            while delta_heading < -180:
                delta_heading += 360
                
            print(f"{i+1:<4} {cmd.left_steps:<8} {cmd.right_steps:<8} "
                  f"{left_dist:<8.1f} {right_dist:<8.1f} "
                  f"{state.x:<8.1f} {state.y:<8.1f} {state.get_heading_deg():<8.1f} "
                  f"{delta_heading:<10.1f}")
            
            prev_heading = state.heading
            
        print("-"*100)
        print(f"Total steps: {len(self.step_commands)}")

# =============================================================================
# MAIN FUNCTION WITH TEST EXAMPLES
# =============================================================================
def main():
    print("Robot Step Simulator - Differential Drive")
    print("Commands: Array of (left_steps, right_steps)")
    print("100 steps = 1 cm, Wheel base = 20 cm")
    print("="*50)
    
    # Test examples
    test_cases = [
        
        {
            "name": "Right Turn [(1000, 990), ] *100",
            "description": "Left wheel more steps - right turn",
            "steps":  [(1000, 900) ] * 25  +  [(1000, 1100) ] * 5 + [ (1000, 1000)] * 5 
        }
        
        

    ]
    
    for i, test_case in enumerate(test_cases):
        print(f"\n{'='*20} Test {i+1}: {test_case['name']} {'='*20}")
        print(f"Description: {test_case['description']}")
        print(f"Steps: {test_case['steps']}")
        
        simulator = RobotStepSimulator(start_x=50.0, start_y=50.0)
        simulator.process_step_array(test_case['steps'])
        
        input(f"Press Enter to display simulation for '{test_case['name']}'...")
        simulator.display_path()
        simulator.print_simulation_data()
        
        if input("Continue to next test? (y/n): ").lower() != 'y':
            break

# Custom test function
def test_custom_steps():
    """Test with custom step array"""
    print("\n" + "="*50)
    print("Custom Step Test")
    print("="*50)
    
    # Define your custom steps here
    custom_steps = [
        (1000, 1000),  # Straight 10cm
        (1500, 500),   # Right curve  
        (800, 800),    # Straight 8cm
        (300, 1200),   # Left curve
        (1000, 1000)   # Straight 10cm
    ]
    
    simulator = RobotStepSimulator(start_x=100.0, start_y=100.0)
    simulator.process_step_array(custom_steps)
    simulator.display_path()
    simulator.print_simulation_data()

if __name__ == "__main__":
    main()
    
    # Uncomment to run custom test
    # test_custom_steps()
