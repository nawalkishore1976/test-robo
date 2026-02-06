import math
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import time

# Configure matplotlib for Windows
import matplotlib
matplotlib.use('TkAgg')
plt.ion()

# =============================================================================
# REAL HARDWARE CONFIGURATION & CONSTANTS - FIXED
# =============================================================================
UNIT_SIZE = 25.0         # 25cm grid units (1 unit = 25cm)
DEFAULT_SPEED = 15.0     # cm/s - conservative for real hardware
CURVE_SPEED = 10.0       # cm/s for curves (slower for stability)
TURN_RADIUS = 25.0       # cm - radius for turns (1 unit = 25cm)
MAX_WAYPOINTS_PER_SEGMENT = 5  # Maximum waypoints per path segment

# Robot physical parameters
WHEEL_BASE = 20.0        # cm - distance between wheels
MAX_WHEEL_SPEED = 20.0   # cm/s - conservative for real hardware
MIN_WHEEL_SPEED = 2.0    # cm/s - minimum wheel speed

# PID Controller parameters - VERY STRONG FOR REAL HARDWARE
KP_HEADING = 5.0         # Very strong proportional gain for heading
KI_HEADING = 0.1         # Integral gain for heading
KD_HEADING = 0.3         # Derivative gain for heading
KP_POSITION = 3.0        # Strong proportional gain for position
KI_POSITION = 0.05       # Integral gain for position
KD_POSITION = 0.15       # Derivative gain for position

# Conversion factor
PULSES_PER_CM = 100.0    # pulses per cm

# Hardware noise levels (realistic but manageable)
MOTOR_NOISE = 0.05       # 5% motor speed variation
SLIP_FACTOR = 0.02       # 2% wheel slippage
IMU_NOISE = 0.015        # 1.5% IMU heading noise

# Direction constants - NORTH=0°
DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST = 0, 1, 2, 3
DIRECTION_NAMES = ["NORTH", "EAST", "SOUTH", "WEST"]
DIRECTION_DELTA = [[0, 1], [1, 0], [0, -1], [-1, 0]]
DIRECTION_TO_RADIANS = [0.0, math.pi/2, math.pi, 3*math.pi/2]

# =============================================================================
# SIMPLIFIED DATA STRUCTURES
# =============================================================================
@dataclass
class WayPoint:
    """Simplified waypoints for real hardware"""
    x: float
    y: float
    speed: float
    heading: float  # radians
    target_left_speed: float = 0.0   # cm/s
    target_right_speed: float = 0.0  # cm/s
    
    def get_heading_deg(self) -> float:
        return math.degrees(self.heading) % 360
    
    def is_stop(self) -> bool:
        return self.speed < 0.5

@dataclass
class RobotState:
    """Real robot state"""
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians
    speed: float = 0.0
    left_wheel_speed: float = 0.0
    right_wheel_speed: float = 0.0
    time: float = 0.0

@dataclass
class PIDController:
    """PID controller optimized for real hardware"""
    kp: float
    ki: float
    kd: float
    prev_error: float = 0.0
    integral: float = 0.0
    max_integral: float = 20.0
    
    def update(self, error: float, dt: float) -> float:
        """Update PID with integral windup protection"""
        # Integral with windup protection
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        
        # Derivative
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        
        return output
    
    def reset(self):
        """Reset PID controller"""
        self.prev_error = 0.0
        self.integral = 0.0

# =============================================================================
# FIXED REAL HARDWARE PATH PLANNER
# =============================================================================
class RealRobotPathPlanner:
    def __init__(self):
        self.waypoints: List[WayPoint] = []
        
    def process_commands(self, commands: str):
        """Process robot commands - ONE MANEUVER AT A TIME"""
        self.waypoints.clear()
        
        current_x, current_y = 0.0, 0.0
        current_dir = DIR_NORTH
        
        print(f"Processing {len(commands)} commands: {commands}")
        
        # Add starting waypoint
        self.waypoints.append(WayPoint(current_x, current_y, 0, DIRECTION_TO_RADIANS[current_dir],
                                     DEFAULT_SPEED, DEFAULT_SPEED))
        
        for i, cmd in enumerate(commands):
            print(f"Command {i+1}: {cmd} at position ({current_x:.1f}, {current_y:.1f}) facing {DIRECTION_NAMES[current_dir]}")
            
            if cmd in ['L', 'R']:
                # Execute turn
                current_x, current_y, current_dir = self.create_turn_waypoints(
                    current_x, current_y, current_dir, cmd)
                    
            elif cmd in ['F', 'S']:
                # Execute forward move
                current_x, current_y = self.create_forward_waypoints(
                    current_x, current_y, current_dir)
        
        # Set final waypoint as stop
        if self.waypoints:
            self.waypoints[-1].speed = 0
            self.waypoints[-1].target_left_speed = 0
            self.waypoints[-1].target_right_speed = 0
            
        print(f"Generated {len(self.waypoints)} total waypoints")
    
    def create_turn_waypoints(self, start_x: float, start_y: float, start_dir: int, turn_cmd: str) -> Tuple[float, float, int]:
        """Create waypoints for a single turn with strong differential"""
        
        # Calculate turn parameters
        if turn_cmd == 'R':  # Right turn
            new_dir = (start_dir + 1) % 4
            # Strong differential for right turn
            left_speed = CURVE_SPEED * 1.8   # Left wheel much faster
            right_speed = CURVE_SPEED * 0.2  # Right wheel much slower
        else:  # Left turn
            new_dir = (start_dir + 3) % 4
            # Strong differential for left turn
            left_speed = CURVE_SPEED * 0.2   # Left wheel much slower
            right_speed = CURVE_SPEED * 1.8  # Right wheel much faster
        
        # Ensure minimum speeds
        left_speed = max(MIN_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left_speed))
        right_speed = max(MIN_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right_speed))
        
        # Calculate end position (simplified - stay in same place for turn)
        end_x, end_y = start_x, start_y  # In-place turn for simplicity
        
        # Create turn waypoints
        start_heading = DIRECTION_TO_RADIANS[start_dir]
        end_heading = DIRECTION_TO_RADIANS[new_dir]
        
        # Handle heading wrap-around
        heading_diff = end_heading - start_heading
        if turn_cmd == 'R':
            if heading_diff <= 0:
                heading_diff += 2 * math.pi
        else:
            if heading_diff >= 0:
                heading_diff -= 2 * math.pi
        
        # Create exactly MAX_WAYPOINTS_PER_SEGMENT waypoints for the turn
        for j in range(MAX_WAYPOINTS_PER_SEGMENT):
            t = j / (MAX_WAYPOINTS_PER_SEGMENT - 1)
            
            # Interpolate heading
            heading = start_heading + t * heading_diff
            while heading < 0:
                heading += 2 * math.pi
            while heading >= 2 * math.pi:
                heading -= 2 * math.pi
            
            # Position stays roughly the same for in-place turn
            wp_x = start_x + np.random.normal(0, 0.5)  # Small random movement
            wp_y = start_y + np.random.normal(0, 0.5)
            
            self.waypoints.append(WayPoint(wp_x, wp_y, CURVE_SPEED, heading,
                                         left_speed, right_speed))
        
        print(f"  Created turn: {DIRECTION_NAMES[start_dir]} -> {DIRECTION_NAMES[new_dir]} "
              f"(L={left_speed:.1f}, R={right_speed:.1f})")
        
        return end_x, end_y, new_dir
    
    def create_forward_waypoints(self, start_x: float, start_y: float, current_dir: int) -> Tuple[float, float]:
        """Create waypoints for forward movement"""
        
        dx, dy = DIRECTION_DELTA[current_dir]
        end_x = start_x + dx * UNIT_SIZE
        end_y = start_y + dy * UNIT_SIZE
        
        # Create waypoints along the straight path
        for j in range(MAX_WAYPOINTS_PER_SEGMENT):
            t = j / (MAX_WAYPOINTS_PER_SEGMENT - 1)
            
            wp_x = start_x + t * (end_x - start_x)
            wp_y = start_y + t * (end_y - start_y)
            heading = DIRECTION_TO_RADIANS[current_dir]
            
            self.waypoints.append(WayPoint(wp_x, wp_y, DEFAULT_SPEED, heading,
                                         DEFAULT_SPEED, DEFAULT_SPEED))
        
        print(f"  Created forward move: ({start_x:.1f},{start_y:.1f}) -> ({end_x:.1f},{end_y:.1f})")
        
        return end_x, end_y

# =============================================================================
# FIXED REAL HARDWARE ROBOT SIMULATION
# =============================================================================
class RealRobotSimulation:
    def __init__(self, planner: RealRobotPathPlanner):
        self.planner = planner
        self.robot_state = RobotState()
        self.pid_heading = PIDController(KP_HEADING, KI_HEADING, KD_HEADING)
        self.pid_position_x = PIDController(KP_POSITION, KI_POSITION, KD_POSITION)
        self.pid_position_y = PIDController(KP_POSITION, KI_POSITION, KD_POSITION)
        self.actual_path = []
        
    def add_hardware_noise(self, left_speed: float, right_speed: float) -> Tuple[float, float]:
        """Add realistic hardware noise"""
        # Motor speed variation
        left_noise = np.random.normal(1.0, MOTOR_NOISE)
        right_noise = np.random.normal(1.0, MOTOR_NOISE)
        
        # Wheel slippage
        slip_left = np.random.normal(1.0, SLIP_FACTOR)
        slip_right = np.random.normal(1.0, SLIP_FACTOR)
        
        left_speed *= left_noise * slip_left
        right_speed *= right_noise * slip_right
        
        return left_speed, right_speed
    
    def calculate_control_with_pid(self, target_wp: WayPoint, dt: float) -> Tuple[float, float]:
        """Calculate wheel speeds using VERY STRONG PID control"""
        
        # Position errors
        error_x = target_wp.x - self.robot_state.x
        error_y = target_wp.y - self.robot_state.y
        
        # Heading error (most important for turns)
        heading_error = target_wp.heading - self.robot_state.heading
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # STRONG PID corrections
        heading_correction = self.pid_heading.update(heading_error, dt)
        pos_x_correction = self.pid_position_x.update(error_x, dt)
        pos_y_correction = self.pid_position_y.update(error_y, dt)
        
        # Start with target speeds
        left_speed = target_wp.target_left_speed
        right_speed = target_wp.target_right_speed
        
        # Apply STRONG heading correction (most important)
        left_speed += heading_correction * -1  # Reverse for left wheel
        right_speed += heading_correction * 1   # Normal for right wheel
        
        # Apply position corrections (smaller effect)
        position_correction = (pos_x_correction + pos_y_correction) * 0.3
        left_speed += position_correction
        right_speed += position_correction
        
        # Ensure reasonable limits
        left_speed = max(MIN_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left_speed))
        right_speed = max(MIN_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right_speed))
        
        return left_speed, right_speed
    
    def simulate_step(self, target_wp: WayPoint, dt: float) -> RobotState:
        """Simulate one step with realistic hardware behavior"""
        
        # Calculate control with STRONG PID
        left_speed, right_speed = self.calculate_control_with_pid(target_wp, dt)
        
        # Add hardware noise
        left_speed, right_speed = self.add_hardware_noise(left_speed, right_speed)
        
        # Calculate robot kinematics
        linear_velocity = (left_speed + right_speed) / 2.0
        angular_velocity = (left_speed - right_speed) / WHEEL_BASE
        
        # Add IMU noise to heading
        angular_velocity += np.random.normal(0, IMU_NOISE)
        
        # Update robot state
        new_state = RobotState()
        new_state.time = self.robot_state.time + dt
        
        # Update heading
        new_state.heading = self.robot_state.heading + angular_velocity * dt
        while new_state.heading < 0:
            new_state.heading += 2 * math.pi
        while new_state.heading >= 2 * math.pi:
            new_state.heading -= 2 * math.pi
        
        # Update position
        avg_heading = (self.robot_state.heading + new_state.heading) / 2.0
        new_state.x = self.robot_state.x + linear_velocity * math.sin(avg_heading) * dt
        new_state.y = self.robot_state.y + linear_velocity * math.cos(avg_heading) * dt
        
        new_state.speed = linear_velocity
        new_state.left_wheel_speed = left_speed
        new_state.right_wheel_speed = right_speed
        
        return new_state
    
    def run_simulation(self):
        """Run complete simulation with realistic hardware behavior"""
        if not self.planner.waypoints:
            print("No waypoints to follow!")
            return
        
        print(f"\n=== REAL HARDWARE ROBOT SIMULATION ===")
        print(f"Following {len(self.planner.waypoints)} waypoints")
        
        # Initialize robot
        self.robot_state = RobotState(0.0, 0.0, 0.0, 0.0, 0.0)
        self.actual_path = []
        
        # Simulation parameters
        dt = 0.1  # 100ms time step
        
        # Follow waypoints
        for i, target_wp in enumerate(self.planner.waypoints):
            self.robot_state = self.simulate_step(target_wp, dt)
            self.actual_path.append((self.robot_state.x, self.robot_state.y))
            
            # Calculate errors
            error_x = self.robot_state.x - target_wp.x
            error_y = self.robot_state.y - target_wp.y
            error_distance = math.sqrt(error_x**2 + error_y**2)
            error_heading = math.degrees(self.robot_state.heading - target_wp.heading)
            while error_heading > 180:
                error_heading -= 360
            while error_heading < -180:
                error_heading += 360
            
            print(f"WP{i:2}: Target=({target_wp.x:5.1f},{target_wp.y:5.1f},{target_wp.get_heading_deg():6.1f}°) "
                  f"Actual=({self.robot_state.x:5.1f},{self.robot_state.y:5.1f},{math.degrees(self.robot_state.heading):6.1f}°) "
                  f"Error=({error_distance:4.1f}cm,{error_heading:5.1f}°)")
            
            if target_wp.is_stop():
                print("Robot stopped at final waypoint")
                break
        
        # Print final results
        if self.planner.waypoints:
            final_target = self.planner.waypoints[-1]
            final_error_x = self.robot_state.x - final_target.x
            final_error_y = self.robot_state.y - final_target.y
            final_error_distance = math.sqrt(final_error_x**2 + final_error_y**2)
            final_error_heading = math.degrees(self.robot_state.heading - final_target.heading)
            while final_error_heading > 180:
                final_error_heading -= 360
            while final_error_heading < -180:
                final_error_heading += 360
            
            print(f"\n=== FINAL RESULTS ===")
            print(f"Target: ({final_target.x:.1f}, {final_target.y:.1f}) @ {final_target.get_heading_deg():.1f}°")
            print(f"Actual: ({self.robot_state.x:.1f}, {self.robot_state.y:.1f}) @ {math.degrees(self.robot_state.heading):.1f}°")
            print(f"Position Error: {final_error_distance:.1f}cm")
            print(f"Heading Error: {final_error_heading:.1f}°")
            
            # Success criteria for real hardware
            position_ok = final_error_distance < 5.0  # Within 5cm
            heading_ok = abs(final_error_heading) < 10.0  # Within 10 degrees
            
            if position_ok and heading_ok:
                print("✅ SUCCESS: Robot reached target within acceptable limits!")
            else:
                print("❌ NEEDS TUNING: Adjust PID parameters or reduce noise")

# =============================================================================
# MAIN FUNCTION FOR REAL HARDWARE TESTING
# =============================================================================
def main():
    print("Real Hardware Robot Control System v2")
    print("Commands: F=Forward, L=Left, R=Right")
    print("Max 5 waypoints per maneuver, Strong PID Control")
    print("=" * 55)
    
    # Test command
    command = input("Enter command (default 'R'): ").strip().upper() or 'R'
    
    # Limit command length for practical testing
    if len(command) > 10:
        print(f"Warning: Command too long ({len(command)} chars). Truncating to 10.")
        command = command[:10]
    
    # Create path planner and simulation
    planner = RealRobotPathPlanner()
    planner.process_commands(command)
    
    simulation = RealRobotSimulation(planner)
    simulation.run_simulation()

if __name__ == "__main__":
    main()
