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
# REAL HARDWARE CONFIGURATION & CONSTANTS
# =============================================================================
UNIT_SIZE = 25.0         # 25cm grid units (1 unit = 25cm)
DEFAULT_SPEED = 20.0     # cm/s - reduced for real hardware
CURVE_SPEED = 12.0       # cm/s for curves (slower for stability)
TURN_RADIUS = 25.0       # cm - radius for turns (1 unit = 25cm)
MAX_WAYPOINTS = 5        # Maximum waypoints for interpolation

# Robot physical parameters
WHEEL_BASE = 20.0        # cm - distance between wheels
MAX_WHEEL_SPEED = 25.0   # cm/s - maximum wheel speed for real hardware
MIN_WHEEL_SPEED = 3.0    # cm/s - minimum wheel speed

# PID Controller parameters - STRONG FOR REAL HARDWARE
KP_HEADING = 2.5         # Strong proportional gain for heading
KI_HEADING = 0.05        # Integral gain for heading
KD_HEADING = 0.15        # Derivative gain for heading
KP_POSITION = 1.5        # Proportional gain for position
KI_POSITION = 0.02       # Integral gain for position
KD_POSITION = 0.08       # Derivative gain for position

# Conversion factor: assume 100 pulses per cm for wheel encoders
PULSES_PER_CM = 100.0    # pulses per cm

# Hardware noise levels (realistic)
ENCODER_NOISE = 0.05     # 5% encoder noise
MOTOR_NOISE = 0.08       # 8% motor speed variation
SLIP_FACTOR = 0.03       # 3% wheel slippage
IMU_NOISE = 0.02         # 2% IMU heading noise

# Direction constants - NORTH=0°
DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST = 0, 1, 2, 3
DIRECTION_NAMES = ["NORTH", "EAST", "SOUTH", "WEST"]
DIRECTION_DELTA = [[0, 1], [1, 0], [0, -1], [-1, 0]]
DIRECTION_TO_RADIANS = [0.0, math.pi/2, math.pi, 3*math.pi/2]

# =============================================================================
# DATA STRUCTURES FOR REAL HARDWARE
# =============================================================================
@dataclass
class AnchorPoint:
    """Key waypoints for real robot navigation"""
    x: float
    y: float
    dir: int
    type: str = "move"
    turn_center_x: float = 0.0
    turn_center_y: float = 0.0
    
    def get_heading_rad(self) -> float:
        return DIRECTION_TO_RADIANS[self.dir]

@dataclass
class WayPoint:
    """Simplified waypoints for real hardware"""
    x: float
    y: float
    speed: float
    heading: float  # radians
    target_m1_pps: float = 0.0
    target_m2_pps: float = 0.0
    
    def get_heading_deg(self) -> float:
        return math.degrees(self.heading)
    
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
    max_integral: float = 10.0
    
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
# REAL HARDWARE PATH PLANNER
# =============================================================================
class RealRobotPathPlanner:
    def __init__(self):
        self.anchors: List[AnchorPoint] = []
        self.waypoints: List[WayPoint] = []
        
    def process_commands(self, commands: str):
        """Process robot commands for real hardware"""
        self.create_anchors(commands)
        self.create_simplified_waypoints()
    
    def create_anchors(self, commands: str):
        """Create anchor points from commands"""
        current_x, current_y = 0.0, 0.0
        current_dir = DIR_NORTH
        self.anchors.clear()
        
        # Add starting anchor
        self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "start"))
        
        for cmd in commands:
            if cmd in ['L', 'R']:
                # Create turn
                self.create_turn(current_x, current_y, current_dir, cmd)
                current_dir = (current_dir + (-1 if cmd == 'L' else 1)) % 4
                current_x, current_y = self.anchors[-1].x, self.anchors[-1].y
                
            elif cmd in ['F', 'S']:
                # Move forward
                dx, dy = DIRECTION_DELTA[current_dir]
                current_x += dx * UNIT_SIZE
                current_y += dy * UNIT_SIZE
                self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "move"))
    
    def create_turn(self, start_x: float, start_y: float, start_dir: int, turn_cmd: str):
        """Create simplified turn with center calculation"""
        # Calculate turn center and end position
        if turn_cmd == 'R':  # Right turn
            if start_dir == DIR_NORTH:
                center_x, center_y = start_x + TURN_RADIUS, start_y
                end_x, end_y = start_x + TURN_RADIUS, start_y + TURN_RADIUS
                end_dir = DIR_EAST
            elif start_dir == DIR_EAST:
                center_x, center_y = start_x, start_y - TURN_RADIUS
                end_x, end_y = start_x + TURN_RADIUS, start_y - TURN_RADIUS
                end_dir = DIR_SOUTH
            elif start_dir == DIR_SOUTH:
                center_x, center_y = start_x - TURN_RADIUS, start_y
                end_x, end_y = start_x - TURN_RADIUS, start_y - TURN_RADIUS
                end_dir = DIR_WEST
            else:  # DIR_WEST
                center_x, center_y = start_x, start_y + TURN_RADIUS
                end_x, end_y = start_x - TURN_RADIUS, start_y + TURN_RADIUS
                end_dir = DIR_NORTH
        else:  # Left turn
            if start_dir == DIR_NORTH:
                center_x, center_y = start_x - TURN_RADIUS, start_y
                end_x, end_y = start_x - TURN_RADIUS, start_y + TURN_RADIUS
                end_dir = DIR_WEST
            elif start_dir == DIR_EAST:
                center_x, center_y = start_x, start_y + TURN_RADIUS
                end_x, end_y = start_x + TURN_RADIUS, start_y + TURN_RADIUS
                end_dir = DIR_NORTH
            elif start_dir == DIR_SOUTH:
                center_x, center_y = start_x + TURN_RADIUS, start_y
                end_x, end_y = start_x + TURN_RADIUS, start_y - TURN_RADIUS
                end_dir = DIR_EAST
            else:  # DIR_WEST
                center_x, center_y = start_x, start_y - TURN_RADIUS
                end_x, end_y = start_x - TURN_RADIUS, start_y - TURN_RADIUS
                end_dir = DIR_SOUTH
        
        self.anchors.append(AnchorPoint(start_x, start_y, start_dir, "turn_start", center_x, center_y))
        self.anchors.append(AnchorPoint(end_x, end_y, end_dir, "turn_end", center_x, center_y))
    
    def create_simplified_waypoints(self):
        """Create maximum 5 waypoints between anchors"""
        self.waypoints.clear()
        
        for i in range(len(self.anchors) - 1):
            current = self.anchors[i]
            next_anchor = self.anchors[i + 1]
            
            if current.type == "turn_start" and next_anchor.type == "turn_end":
                # Create turn waypoints (max 5)
                self.create_turn_waypoints(current, next_anchor)
            else:
                # Create straight waypoints (max 5)
                self.create_straight_waypoints(current, next_anchor)
        
        # Set final waypoint as stop
        if self.waypoints:
            self.waypoints[-1].speed = 0
    
    def create_turn_waypoints(self, start: AnchorPoint, end: AnchorPoint):
        """Create maximum 5 waypoints for turn"""
        center_x, center_y = start.turn_center_x, start.turn_center_y
        
        # Calculate turn direction
        start_angle = math.atan2(start.y - center_y, start.x - center_x)
        end_angle = math.atan2(end.y - center_y, end.x - center_x)
        
        # Determine turn direction and angle
        start_dir = start.dir
        end_dir = end.dir
        
        if (start_dir + 1) % 4 == end_dir:  # Right turn
            angle_diff = -math.pi/2
            heading_start = DIRECTION_TO_RADIANS[start_dir]
            heading_diff = math.pi/2
        else:  # Left turn
            angle_diff = math.pi/2
            heading_start = DIRECTION_TO_RADIANS[start_dir]
            heading_diff = -math.pi/2
        
        # Create exactly MAX_WAYPOINTS waypoints
        for j in range(MAX_WAYPOINTS):
            t = j / (MAX_WAYPOINTS - 1)
            
            # Position along arc
            angle = start_angle + t * angle_diff
            wp_x = center_x + TURN_RADIUS * math.cos(angle)
            wp_y = center_y + TURN_RADIUS * math.sin(angle)
            
            # Heading progression
            heading = heading_start + t * heading_diff
            while heading < 0:
                heading += 2 * math.pi
            while heading >= 2 * math.pi:
                heading -= 2 * math.pi
            
            # Calculate differential wheel speeds
            left_speed, right_speed = self.calculate_turn_speeds(angle_diff < 0)
            
            self.waypoints.append(WayPoint(wp_x, wp_y, CURVE_SPEED, heading, 
                                         left_speed * PULSES_PER_CM, right_speed * PULSES_PER_CM))
    
    def create_straight_waypoints(self, start: AnchorPoint, end: AnchorPoint):
        """Create maximum 5 waypoints for straight line"""
        distance = math.sqrt((end.x - start.x)**2 + (end.y - start.y)**2)
        if distance < 0.1:
            return
        
        # Create exactly MAX_WAYPOINTS waypoints
        for j in range(MAX_WAYPOINTS):
            t = j / (MAX_WAYPOINTS - 1)
            
            wp_x = start.x + t * (end.x - start.x)
            wp_y = start.y + t * (end.y - start.y)
            heading = start.get_heading_rad()
            
            self.waypoints.append(WayPoint(wp_x, wp_y, DEFAULT_SPEED, heading,
                                         DEFAULT_SPEED * PULSES_PER_CM, DEFAULT_SPEED * PULSES_PER_CM))
    
    def calculate_turn_speeds(self, is_right_turn: bool) -> Tuple[float, float]:
        """Calculate wheel speeds for turns"""
        # For right turn: left wheel faster
        if is_right_turn:
            left_speed = CURVE_SPEED * 1.4  # 40% faster
            right_speed = CURVE_SPEED * 0.6  # 40% slower
        else:
            left_speed = CURVE_SPEED * 0.6  # 40% slower
            right_speed = CURVE_SPEED * 1.4  # 40% faster
        
        # Ensure within limits
        left_speed = max(MIN_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left_speed))
        right_speed = max(MIN_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right_speed))
        
        return left_speed, right_speed

# =============================================================================
# REAL HARDWARE ROBOT SIMULATION WITH STRONG PID
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
        """Calculate wheel speeds using strong PID control"""
        # Get base target speeds
        target_left = target_wp.target_m1_pps / PULSES_PER_CM
        target_right = target_wp.target_m2_pps / PULSES_PER_CM
        
        # Position errors
        error_x = target_wp.x - self.robot_state.x
        error_y = target_wp.y - self.robot_state.y
        distance_error = math.sqrt(error_x**2 + error_y**2)
        
        # Heading error
        heading_error = target_wp.heading - self.robot_state.heading
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # PID corrections
        heading_correction = self.pid_heading.update(heading_error, dt)
        pos_x_correction = self.pid_position_x.update(error_x, dt)
        pos_y_correction = self.pid_position_y.update(error_y, dt)
        
        # Combine corrections
        speed_correction = (pos_x_correction + pos_y_correction) * 0.5
        
        # Apply corrections
        left_speed = target_left + speed_correction - heading_correction
        right_speed = target_right + speed_correction + heading_correction
        
        # Limit speeds
        left_speed = max(MIN_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left_speed))
        right_speed = max(MIN_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right_speed))
        
        return left_speed, right_speed
    
    def simulate_step(self, target_wp: WayPoint, dt: float) -> RobotState:
        """Simulate one step with realistic hardware behavior"""
        # Calculate control with PID
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
        
        # Update position with encoder noise
        encoder_factor_left = np.random.normal(1.0, ENCODER_NOISE)
        encoder_factor_right = np.random.normal(1.0, ENCODER_NOISE)
        actual_linear = linear_velocity * (encoder_factor_left + encoder_factor_right) / 2.0
        
        avg_heading = (self.robot_state.heading + new_state.heading) / 2.0
        new_state.x = self.robot_state.x + actual_linear * math.sin(avg_heading) * dt
        new_state.y = self.robot_state.y + actual_linear * math.cos(avg_heading) * dt
        
        new_state.speed = linear_velocity
        new_state.left_wheel_speed = left_speed
        new_state.right_wheel_speed = right_speed
        
        return new_state
    
    def run_simulation(self):
        """Run complete simulation with realistic hardware behavior"""
        if not self.planner.waypoints:
            print("No waypoints to follow!")
            return
        
        print("=== REAL HARDWARE ROBOT SIMULATION ===")
        print(f"Command processed with {len(self.planner.waypoints)} waypoints")
        
        # Create visualization
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
        
        # Plot planned path
        if self.planner.waypoints:
            wp_x = [wp.x for wp in self.planner.waypoints]
            wp_y = [wp.y for wp in self.planner.waypoints]
            ax1.plot(wp_x, wp_y, 'b-o', linewidth=2, label='Planned Path')
        
        # Plot anchors
        for anchor in self.planner.anchors:
            if anchor.type == 'start':
                ax1.plot(anchor.x, anchor.y, 'gs', markersize=12, label='Start')
            elif 'turn' in anchor.type:
                ax1.plot(anchor.x, anchor.y, 'ro', markersize=8)
        
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlabel('X (cm)')
        ax1.set_ylabel('Y (cm)')
        ax1.set_title('Real Hardware Robot Path')
        ax1.legend()
        
        # Simulate robot following waypoints
        dt = 0.1  # 100ms time step
        self.actual_path = []
        
        # Initialize robot at first waypoint
        if self.planner.waypoints:
            first_wp = self.planner.waypoints[0]
            self.robot_state = RobotState(first_wp.x, first_wp.y, 0.0, 0.0, 0.0)
        
        # Follow waypoints
        for i, target_wp in enumerate(self.planner.waypoints):
            self.robot_state = self.simulate_step(target_wp, dt)
            self.actual_path.append((self.robot_state.x, self.robot_state.y))
            
            # Print progress
            error_x = self.robot_state.x - target_wp.x
            error_y = self.robot_state.y - target_wp.y
            error_h = math.degrees(self.robot_state.heading - target_wp.heading)
            print(f"WP{i}: Target=({target_wp.x:.1f},{target_wp.y:.1f},{target_wp.get_heading_deg():.1f}°) "
                  f"Actual=({self.robot_state.x:.1f},{self.robot_state.y:.1f},{math.degrees(self.robot_state.heading):.1f}°) "
                  f"Error=({error_x:.1f},{error_y:.1f},{error_h:.1f}°)")
            
            if target_wp.is_stop():
                print("Robot stopped at final waypoint")
                break
        
        # Plot actual path
        if len(self.actual_path) > 1:
            actual_x = [p[0] for p in self.actual_path]
            actual_y = [p[1] for p in self.actual_path]
            ax1.plot(actual_x, actual_y, 'r--', linewidth=2, alpha=0.8, label='Actual Path')
            ax1.plot(self.robot_state.x, self.robot_state.y, 'ro', markersize=10, label='Final Position')
        
        ax1.legend()
        
        # Plot errors over time
        if len(self.actual_path) >= len(self.planner.waypoints):
            errors = []
            for i, wp in enumerate(self.planner.waypoints):
                if i < len(self.actual_path):
                    error = math.sqrt((self.actual_path[i][0] - wp.x)**2 + (self.actual_path[i][1] - wp.y)**2)
                    errors.append(error)
            
            ax2.plot(range(len(errors)), errors, 'g-o', linewidth=2)
            ax2.set_xlabel('Waypoint')
            ax2.set_ylabel('Position Error (cm)')
            ax2.set_title('Position Error vs Waypoint')
            ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show(block=True)
        
        # Print final results
        if self.planner.waypoints:
            final_target = self.planner.waypoints[-1]
            final_error_x = self.robot_state.x - final_target.x
            final_error_y = self.robot_state.y - final_target.y
            final_error_h = math.degrees(self.robot_state.heading - final_target.heading)
            final_distance_error = math.sqrt(final_error_x**2 + final_error_y**2)
            
            print(f"\n=== FINAL RESULTS ===")
            print(f"Target: ({final_target.x:.1f}, {final_target.y:.1f}) @ {final_target.get_heading_deg():.1f}°")
            print(f"Actual: ({self.robot_state.x:.1f}, {self.robot_state.y:.1f}) @ {math.degrees(self.robot_state.heading):.1f}°")
            print(f"Position Error: {final_distance_error:.1f}cm")
            print(f"Heading Error: {final_error_h:.1f}°")

# =============================================================================
# MAIN FUNCTION FOR REAL HARDWARE TESTING
# =============================================================================
def main():
    print("Real Hardware Robot Control System")
    print("Commands: F=Forward, L=Left, R=Right")
    print("PID Control with Hardware Noise Simulation")
    print("=" * 50)
    
    # Test command
    command = input("Enter command (default 'R'): ").strip().upper() or 'R'
    
    # Create path planner and simulation
    planner = RealRobotPathPlanner()
    planner.process_commands(command)
    
    simulation = RealRobotSimulation(planner)
    simulation.run_simulation()

print ("sss")
main()