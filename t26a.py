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
# CONFIGURATION & CONSTANTS
# =============================================================================
UNIT_SIZE = 25.0         # 25cm grid units (1 unit = 25cm)
DEFAULT_SPEED = 25.0     # cm/s
CURVE_SPEED = 15.0       # cm/s for curves (slower)
TURN_RADIUS = 25.0       # cm - radius for turns (1 unit = 25cm)
INTERPOLATION_STEP = 2.0 # cm between interpolated points
DEBUG_ENABLED = True

# Robot physical parameters
WHEEL_BASE = 20.0        # cm - distance between wheels
MAX_WHEEL_SPEED = 30.0   # cm/s - maximum wheel speed
MIN_WHEEL_SPEED = 5.0    # cm/s - minimum wheel speed

# PID Controller parameters - IMPROVED
KP = 1.0                 # Proportional gain (reduced)
KI = 0.01                # Integral gain (reduced)
KD = 0.05                # Derivative gain (reduced)

# Conversion factor: assume 100 pulses per cm for wheel encoders
PULSES_PER_CM = 100.0    # pulses per cm

# Direction constants - FIXED FOR PROPER NORTH=0°
DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST = 0, 1, 2, 3
DIRECTION_NAMES = ["NORTH", "EAST", "SOUTH", "WEST"]
DIRECTION_ARROWS = ["↑", "→", "↓", "←"]

# Direction deltas [dx, dy] - CONSISTENT WITH NORTH=0°
DIRECTION_DELTA = [
    [0,  1],  # North: Y+ (0°)
    [1,  0],  # East:  X+ (90°)
    [0, -1],  # South: Y- (180°)
    [-1, 0]   # West:  X- (270°)
]

# FIXED: Direction to radians mapping - North = 0°
DIRECTION_TO_RADIANS = [
    0.0,           # North: 0° = 0 radians
    math.pi/2,     # East:  90° = π/2 radians  
    math.pi,       # South: 180° = π radians
    3*math.pi/2    # West:  270° = 3π/2 radians
]

# =============================================================================
# DATA STRUCTURES
# =============================================================================
@dataclass
class AnchorPoint:
    """Key waypoints including circular turn points"""
    x: float
    y: float
    dir: int        # Direction: 0=North, 1=East, 2=South, 3=clsWest
    type: str = "move"   # "start", "move", "turn_start", "turn_end"
    turn_center_x: float = 0.0  # Center of circular turn
    turn_center_y: float = 0.0
    
    def get_heading_rad(self) -> float:
        """FIXED: Return heading in radians where North = 0°"""
        return DIRECTION_TO_RADIANS[self.dir]
        
    def get_heading_deg(self) -> float:
        """FIXED: Return heading in degrees where North = 0°"""
        return self.dir * 90.0
  
@dataclass
class WayPoint:
      """Interpolated points for smooth robot motion"""
      x: float
      y: float
      speed: float
      heading: float  # radians - FIXED: North = 0°
      time: float = 0.0
      curvature: float = 0.0
      target_m1_pps: float = 0.0
      target_m2_pps: float = 0.0
      target_turn: float = 0.0
      expected_total_turn: float = 0.0
      target_m1_steps: int = 0
      target_m2_steps: int = 0
      # ADD THESE NEW FIELDS:
      expected_m1_steps: int = 0      # expected M1 steps for this timestep
      expected_m2_steps: int = 0      # expected M2 steps for this timestep  
      expected_turn: float = 0.0      # expected turn for this timestep
      expected_total_turn_step: float = 0.0  # expected total turn at this step

      def get_heading_deg(self) -> float:
          """Convert heading from radians to degrees"""
          return math.degrees(self.heading)

      def is_stop(self) -> bool:
          return self.speed < 0.2
    
      def calculate_differential_wheel_speeds(self, linear_speed: float, curvature: float) -> Tuple[float, float]:
              """Calculate left and right wheel speeds - FIXED for correct right turn"""
              if abs(curvature) < 1e-6:  # Straight line
                  return linear_speed, linear_speed
        
              radius = 1.0 / abs(curvature)
              angular_velocity = linear_speed / radius
        
              # FIXED: For right turn (positive curvature):
              # - Left wheel (M1) should be faster (outer arc)
              # - Right wheel (M2) should be slower (inner arc)
        
              if curvature > 0:  # Right turn - LEFT wheel faster
                  left_radius = radius + WHEEL_BASE / 2   # Outer arc - longer path
                  right_radius = radius - WHEEL_BASE / 2  # Inner arc - shorter path
                  left_speed = angular_velocity * left_radius   # FASTER
                  right_speed = angular_velocity * right_radius # SLOWER
            
                  # Ensure minimum speed for inner wheel
                  if right_radius < WHEEL_BASE / 2:
                      right_speed = MIN_WHEEL_SPEED
                      left_speed = right_speed * left_radius / right_radius
                
              else:  # Left turn - RIGHT wheel faster
                  left_radius = radius - WHEEL_BASE / 2   # Inner arc - shorter path  
                  right_radius = radius + WHEEL_BASE / 2  # Outer arc - longer path
                  left_speed = angular_velocity * left_radius  # SLOWER
                  right_speed = angular_velocity * right_radius # FASTER
            
                  # Ensure minimum speed for inner wheel
                  if left_radius < WHEEL_BASE / 2:
                      left_speed = MIN_WHEEL_SPEED
                      right_speed = left_speed * right_radius / left_radius
        
              return left_speed, right_speed
@dataclass
class RobotState:
    """Current robot state for simulation"""
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians - FIXED: North = 0°
    speed: float = 0.0
    time: float = 0.0
    left_wheel_speed: float = 0.0
    right_wheel_speed: float = 0.0
    actual_m1_pps: float = 0.0  # actual left wheel pulses per second
    actual_m2_pps: float = 0.0  # actual right wheel pulses per second
    actual_turn: float = 0.0    # actual turn angle in degrees
    actual_total_turn: float = 0.0  # actual cumulative turn from start
    actual_m1_steps: int = 0    # actual cumulative left wheel steps
    actual_m2_steps: int = 0    # actual cumulative right wheel steps

@dataclass
class PIDController:
    """PID controller for robot following"""
    kp: float = KP
    ki: float = KI
    kd: float = KD
    prev_error: float = 0.0
    integral: float = 0.0
    
    def update(self, error: float, dt: float) -> float:
        # Limit integral windup
        if abs(self.integral) > 5.0:
            self.integral = 5.0 if self.integral > 0 else -5.0
            
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# =============================================================================
# FIXED ROBOT PATH PLANNER V14 - NORTH = 0° WITH PRECOMPUTED WHEEL SPEEDS
# =============================================================================
class RobotPathPlannerV14:
    def __init__(self):
        self.anchors: List[AnchorPoint] = []
        self.waypoints: List[WayPoint] = []
        self.total_distance = 0.0
        self.total_time = 0.0
        self.current_command = ""
        
    def debug_print(self, msg: str):
        if DEBUG_ENABLED:
            print(msg)
    
    def convert_commands_to_anchors_v14(self, commands: str) -> int:
        """V14: FIXED circular turn paths with North = 0°"""
        current_x, current_y = 0.0, 0.0
        current_dir = DIR_NORTH  # Start facing North (0°)
        self.anchors.clear()
        self.current_command = commands
        
        self.debug_print("=== V14: FIXED COMMANDS - NORTH = 0° ===")
        self.debug_print(f"Commands: {commands}")
        self.debug_print(f"Turn radius: {TURN_RADIUS}cm (1 unit)")
        self.debug_print(f"Initial direction: {DIRECTION_NAMES[current_dir]} = {DIRECTION_TO_RADIANS[current_dir]:.3f} rad = {current_dir * 90}°")
        
        # Add starting anchor
        self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "start"))
        self.debug_print(f"START: ({current_x}, {current_y}) facing {DIRECTION_NAMES[current_dir]} (0°)")
        
        for i, cmd in enumerate(commands):
            if cmd in ['L', 'R']:
                # Create FIXED circular turn
                self.create_circular_turn_fixed(current_x, current_y, current_dir, cmd)
                
                # Update direction after turn
                if cmd == 'L':
                    current_dir = (current_dir + 3) % 4  # Turn left: -1 = +3 (mod 4)
                    self.debug_print(f"L: Turn left -> {DIRECTION_NAMES[current_dir]} ({current_dir * 90}°)")
                else:
                    current_dir = (current_dir + 1) % 4  # Turn right: +1
                    self.debug_print(f"R: Turn right -> {DIRECTION_NAMES[current_dir]} ({current_dir * 90}°)")
                
                # Update position to end of turn
                current_x, current_y = self.anchors[-1].x, self.anchors[-1].y
                
            elif cmd in ['F', 'S']:
                # Move forward one unit
                dx, dy = DIRECTION_DELTA[current_dir]
                current_x += dx * UNIT_SIZE
                current_y += dy * UNIT_SIZE
                
                self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "move"))
                self.debug_print(f"F: Move to ({current_x}, {current_y}) facing {DIRECTION_NAMES[current_dir]} ({current_dir * 90}°)")
                
            elif cmd == 'B':
                # Move backward one unit
                opposite_dir = (current_dir + 2) % 4
                dx, dy = DIRECTION_DELTA[opposite_dir]
                current_x += dx * UNIT_SIZE
                current_y += dy * UNIT_SIZE
                
                self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "move"))
                self.debug_print(f"B: Move to ({current_x}, {current_y}) facing {DIRECTION_NAMES[current_dir]} ({current_dir * 90}°)")
        
        self.debug_print(f"Created {len(self.anchors)} anchor points")
        self.debug_print("=" * 50)
        
        return len(self.anchors)
    
    def create_circular_turn_fixed(self, start_x: float, start_y: float, start_dir: int, turn_cmd: str):
        """FIXED: Create circular turn path with correct directions and North = 0°"""
        self.debug_print(f"  Creating FIXED {turn_cmd} turn from ({start_x}, {start_y}) facing {DIRECTION_NAMES[start_dir]} ({start_dir * 90}°)")
        
        if turn_cmd == 'R':  # Right turn - CLOCKWISE
            if start_dir == DIR_NORTH:      # North(0°) -> East(90°) CLOCKWISE
                center_x = start_x + TURN_RADIUS
                center_y = start_y
                end_x = start_x + TURN_RADIUS
                end_y = start_y + TURN_RADIUS  
                end_dir = DIR_EAST
            elif start_dir == DIR_EAST:     # East(90°) -> South(180°) CLOCKWISE
                center_x = start_x
                center_y = start_y - TURN_RADIUS
                end_x = start_x + TURN_RADIUS
                end_y = start_y - TURN_RADIUS
                end_dir = DIR_SOUTH
            elif start_dir == DIR_SOUTH:    # South(180°) -> West(270°) CLOCKWISE
                center_x = start_x - TURN_RADIUS
                center_y = start_y
                end_x = start_x - TURN_RADIUS  
                end_y = start_y - TURN_RADIUS
                end_dir = DIR_WEST
            else:  # DIR_WEST               # West(270°) -> North(0°) CLOCKWISE
                center_x = start_x
                center_y = start_y + TURN_RADIUS
                end_x = start_x - TURN_RADIUS
                end_y = start_y + TURN_RADIUS
                end_dir = DIR_NORTH
                
        else:  # Left turn - COUNTER-CLOCKWISE
            if start_dir == DIR_NORTH:      # North(0°) -> West(270°) CCW
                center_x = start_x - TURN_RADIUS
                center_y = start_y
                end_x = start_x - TURN_RADIUS
                end_y = start_y + TURN_RADIUS
                end_dir = DIR_WEST
            elif start_dir == DIR_EAST:     # East(90°) -> North(0°) CCW
                center_x = start_x
                center_y = start_y + TURN_RADIUS
                end_x = start_x + TURN_RADIUS
                end_y = start_y + TURN_RADIUS
                end_dir = DIR_NORTH
            elif start_dir == DIR_SOUTH:    # South(180°) -> East(90°) CCW
                center_x = start_x + TURN_RADIUS
                center_y = start_y
                end_x = start_x + TURN_RADIUS
                end_y = start_y - TURN_RADIUS
                end_dir = DIR_EAST
            else:  # DIR_WEST              # West(270°) -> South(180°) CCW
                center_x = start_x
                center_y = start_y - TURN_RADIUS
                end_x = start_x - TURN_RADIUS
                end_y = start_y - TURN_RADIUS
                end_dir = DIR_SOUTH
        
        # Add anchors
        self.anchors.append(AnchorPoint(start_x, start_y, start_dir, "turn_start", center_x, center_y))
        self.anchors.append(AnchorPoint(end_x, end_y, end_dir, "turn_end", center_x, center_y))
        
        self.debug_print(f"    Turn center: ({center_x}, {center_y})")
        self.debug_print(f"    Turn end: ({end_x}, {end_y}) facing {DIRECTION_NAMES[end_dir]} ({end_dir * 90}°)")
    
    def calculate_differential_wheel_speeds(self, linear_speed: float, curvature: float) -> Tuple[float, float]:
        """Calculate left and right wheel speeds for differential drive"""
        if abs(curvature) < 1e-6:  # Straight line
            return linear_speed, linear_speed
        
        # For circular motion: R = 1/curvature
        radius = 1.0 / curvature
        
        # Angular velocity
        angular_velocity = linear_speed / abs(radius)
        
        # Wheel speeds for differential drive
        # Left wheel radius = R - wheelbase/2
        # Right wheel radius = R + wheelbase/2
        if curvature > 0:  # Right turn (clockwise) - LEFT wheel faster
            left_radius = abs(radius) + WHEEL_BASE / 2   # Outer arc
            right_radius = abs(radius) - WHEEL_BASE / 2  # Inner arc
            left_speed = angular_velocity * left_radius  # FASTER
            right_speed = angular_velocity * right_radius # SLOWER
        else:  # Left turn (counter-clockwise) - RIGHT wheel faster
            left_radius = abs(radius) - WHEEL_BASE / 2   # Inner arc  
            right_radius = abs(radius) + WHEEL_BASE / 2  # Outer arc
            left_speed = angular_velocity * left_radius  # SLOWER
            right_speed = angular_velocity * right_radius # FASTER
            
        return left_speed, right_speed
    
    def interpolate_waypoints_v14(self) -> int:
        """Create waypoints including FIXED circular arc interpolation with precomputed wheel speeds"""
        self.waypoints.clear()
        current_time = 0.0
        prev_heading = 0.0
        cumulative_total_turn = 0.0
        cumulative_m1_steps = 0
        cumulative_m2_steps = 0
        
        self.debug_print("=== V14: FIXED WAYPOINT INTERPOLATION WITH PRECOMPUTED WHEEL SPEEDS ===")
        
        for i in range(len(self.anchors) - 1):
            current = self.anchors[i]
            next_anchor = self.anchors[i + 1]
            
            if current.type == "turn_start" and next_anchor.type == "turn_end":
                # FIXED circular arc interpolation
                self.interpolate_circular_arc_fixed(current, next_anchor, current_time, prev_heading, 
                                                  cumulative_total_turn, cumulative_m1_steps, cumulative_m2_steps)
                arc_length = 0.25 * 2 * math.pi * TURN_RADIUS  # Quarter circle
                segment_time = arc_length / CURVE_SPEED
                current_time += segment_time
                if self.waypoints:
                    prev_heading = self.waypoints[-1].heading
                    cumulative_total_turn = self.waypoints[-1].expected_total_turn
                    cumulative_m1_steps = self.waypoints[-1].target_m1_steps
                    cumulative_m2_steps = self.waypoints[-1].target_m2_steps
                
            else:
                # Straight line interpolation
                start_x, start_y = current.x, current.y
                end_x, end_y = next_anchor.x, next_anchor.y
                
                distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
                speed = DEFAULT_SPEED
                segment_time = distance / speed if distance > 0 else 0
                
                num_points = max(3, int(distance / INTERPOLATION_STEP))
                
                for j in range(num_points + 1):
                    t = j / num_points
                    wp_x = start_x + t * (end_x - start_x)
                    wp_y = start_y + t * (end_y - start_y)
                    
                    # FIXED heading calculation - use anchor direction
                    heading = current.get_heading_rad()
                    waypoint_time = current_time + t * segment_time
                    
                    # Calculate target turn (small incremental change)
                    target_turn = math.degrees(heading - prev_heading)
                    # Normalize to [-5, 5] degrees
                    while target_turn > 180:
                        target_turn -= 360
                    while target_turn < -180:
                        target_turn += 360
                    target_turn = max(-5.0, min(5.0, target_turn))
                    
                    # Update cumulative total turn
                    cumulative_total_turn += target_turn
                    
                    # Calculate wheel speeds for straight line (same speed)
                    left_speed, right_speed = speed, speed
                    
                    # Convert to pulses per second
                    target_m1_pps = left_speed * PULSES_PER_CM
                    target_m2_pps = right_speed * PULSES_PER_CM
                    
                    # Calculate cumulative steps (assuming 0.1s time step)
                    dt = 0.1
                    cumulative_m1_steps += int(target_m1_pps * dt)
                    cumulative_m2_steps += int(target_m2_pps * dt)
                    
                    self.waypoints.append(WayPoint(wp_x, wp_y, speed, heading, waypoint_time, 0.0, 
                                                 target_m1_pps, target_m2_pps, target_turn, cumulative_total_turn,
                                                 cumulative_m1_steps, cumulative_m2_steps))
                    prev_heading = heading
                
                current_time += segment_time
        
        # Set final waypoint as stop
        if self.waypoints:
            self.waypoints[-1].speed = 0
            self.waypoints[-1].target_m1_pps = 0
            self.waypoints[-1].target_m2_pps = 0
            
        self.total_time = current_time
        self.total_distance = sum(math.sqrt((self.waypoints[i+1].x - self.waypoints[i].x)**2 + 
                                          (self.waypoints[i+1].y - self.waypoints[i].y)**2) 
                                for i in range(len(self.waypoints)-1))
        
        self.debug_print(f"Generated {len(self.waypoints)} waypoints")
        self.debug_print(f"Total distance: {self.total_distance:.1f}cm")
        self.debug_print(f"Total time: {self.total_time:.1f}s")
        
        return len(self.waypoints)
    def interpolate_circular_arc_fixed(self, start_anchor: AnchorPoint, end_anchor: AnchorPoint, start_time: float, 
                                   prev_heading: float, cumulative_total_turn: float, cumulative_m1_steps: int, cumulative_m2_steps: int):
        """FIXED: Interpolate points along circular arc with correct heading progression"""
        center_x = start_anchor.turn_center_x
        center_y = start_anchor.turn_center_y
    
        # Calculate start and end angles
        start_angle = math.atan2(start_anchor.y - center_y, start_anchor.x - center_x)
        end_angle = math.atan2(end_anchor.y - center_y, end_anchor.x - center_x)
    
        # FIXED: For right turn, heading should INCREASE from 0° to 90°
        start_dir = start_anchor.dir
        end_dir = end_anchor.dir
        
        if (start_dir + 1) % 4 == end_dir:  # Right turn (clockwise)
            # For North(0°) → East(90°): heading should increase
            angle_diff = -math.pi/2  # Geometric angle (counterclockwise in standard coords)
            heading_start = 0.0  # North = 0°
            heading_end = math.pi/2  # East = 90°
            heading_diff = math.pi/2  # +90° for right turn
        else:  # Left turn (counter-clockwise)
            angle_diff = math.pi/2
            heading_start = 0.0  # North = 0°
            heading_end = 3*math.pi/2  # West = 270°
            heading_diff = -math.pi/2  # -90° for left turn (or +270°)

        arc_length = abs(angle_diff) * TURN_RADIUS
        segment_time = arc_length / CURVE_SPEED
        num_points = max(8, int(arc_length / INTERPOLATION_STEP))  # More points for smoother curves
                # Calculate curvature (1/radius) with correct sign
        if angle_diff < 0:  # Right turn (clockwise) 
            curvature = 1.0 / TURN_RADIUS   # POSITIVE for right turn
        else:  # Left turn (counter-clockwise)
            curvature = -1.0 / TURN_RADIUS  # NEGATIVE for left turn

        self.debug_print(f"  FIXED Arc: center=({center_x:.1f},{center_y:.1f})")

        self.debug_print(f"  Start angle={math.degrees(start_angle):.1f}° End angle={math.degrees(end_angle):.1f}°")
        self.debug_print(f"  Angle diff: {math.degrees(angle_diff):.1f}° Arc length: {arc_length:.1f}cm Points: {num_points}")
        self.debug_print(f"  Creating {'RIGHT' if curvature > 0 else 'LEFT'} turn")
        self.debug_print(f"  Curvature: {curvature:.4f} (positive=right, negative=left)")
    
        for j in range(num_points + 1):
            t = j / num_points
            angle = start_angle + t * angle_diff
        
            wp_x = center_x + TURN_RADIUS * math.cos(angle)
            wp_y = center_y + TURN_RADIUS * math.sin(angle)
        
            # Correct heading progression
            heading = heading_start + t * heading_diff
            
            # Normalize heading to [0, 2π]
            while heading < 0:
                heading += 2 * math.pi
            while heading >= 2 * math.pi:
                heading -= 2 * math.pi
        
            # Calculate target turn (small incremental change)
            target_turn = math.degrees(heading - prev_heading)
            # Normalize to small angle
            while target_turn > 180:
                target_turn -= 360
            while target_turn < -180:
                target_turn += 360
            # Limit to small increments
            angle_step = math.degrees(angle_diff) / num_points
            target_turn = max(-abs(angle_step), min(abs(angle_step), target_turn))
        
            # Update cumulative total turn
            cumulative_total_turn += target_turn
        
            waypoint_time = start_time + t * segment_time
                          # Calculate differential wheel speeds for this curve point
            left_speed, right_speed = self.calculate_differential_wheel_speeds(CURVE_SPEED, curvature)

            if j == 0:  # First point only
                self.debug_print(f"  Turn verification: curvature={curvature:.3f}, M1={left_speed*PULSES_PER_CM:.0f}, M2={right_speed*PULSES_PER_CM:.0f}")
                self.debug_print(f"  For RIGHT turn: curvature>0={curvature>0}, M1>M2={left_speed>right_speed}")

            if j == 0:  # Debug first point
                self.debug_print(f"  First point wheel speeds: L={left_speed:.1f} R={right_speed:.1f}")
                self.debug_print(f"  For RIGHT turn: L should be > R: {left_speed > right_speed}")

            # Convert to pulses per second
            target_m1_pps = left_speed * PULSES_PER_CM
            target_m2_pps = right_speed * PULSES_PER_CM
        
            # Calculate cumulative steps (assuming 0.1s time step)
            dt = 0.1
            cumulative_m1_steps += int(target_m1_pps * dt)
            cumulative_m2_steps += int(target_m2_pps * dt)
        
            self.waypoints.append(WayPoint(wp_x, wp_y, CURVE_SPEED, heading, waypoint_time, 
                                   curvature, target_m1_pps, target_m2_pps, target_turn, cumulative_total_turn,
                                   cumulative_m1_steps, cumulative_m2_steps))
            prev_heading = heading
    def process_commands(self, commands: str):
        """Process commands with V14 fixes and North = 0°"""
        self.convert_commands_to_anchors_v14(commands)
        self.interpolate_waypoints_v14()

# =============================================================================
# FIXED ROBOT SIMULATION V14 - NORTH = 0° WITH PRECOMPUTED CONTROL
# =============================================================================
class RobotSimulationV14a:
    def __init__(self, planner: RobotPathPlannerV14):
        self.planner = planner
        self.robot_state = RobotState()
        self.target_path = []
        self.actual_path = []
        self.pid_heading = PIDController()
        self.pid_position = PIDController()
        self.robot_artists = []
        self.debug_data = []
        self.prev_robot_heading = 0.0
        self.actual_cumulative_total_turn = 0.0
        self.actual_cumulative_m1_steps = 0
        self.actual_cumulative_m2_steps = 0
        
    def calculate_wheel_speeds_precomputed(self, target_wp: WayPoint) -> Tuple[float, float]:
        """Use precomputed wheel speeds with minor corrections"""
        
        # Start with precomputed target speeds
        target_left = target_wp.target_m1_pps / PULSES_PER_CM  # Convert back to cm/s
        target_right = target_wp.target_m2_pps / PULSES_PER_CM
        
        # Small position-based correction for accuracy
        pos_error_x = target_wp.x - self.robot_state.x
        pos_error_y = target_wp.y - self.robot_state.y
        distance_error = math.sqrt(pos_error_x**2 + pos_error_y**2)
        
        # Small heading correction
        heading_error = target_wp.heading - self.robot_state.heading
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Apply small corrections (much smaller than before)
        correction = self.pid_heading.update(heading_error, 0.1) * 0.5  # Reduced correction
        
        # Apply corrections
        left_speed = target_left - correction
        right_speed = target_right + correction
        
        # If we're far from target, slow down both wheels proportionally
        if distance_error > 3.0:
            slowdown = max(0.3, min(1.0, 3.0 / distance_error))
            left_speed *= slowdown
            right_speed *= slowdown
        
        # Limit wheel speeds
        left_speed = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left_speed))
        right_speed = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right_speed))
        
        return left_speed, right_speed
    def simulate_robot_step_fixed(self, target_wp: WayPoint, dt: float) -> RobotState:
            """FIXED: Correct angular velocity and position updates"""
            
            # Calculate desired wheel speeds
            left_speed, right_speed = self.calculate_wheel_speeds_precomputed(target_wp)
            
            # Add minimal noise
            left_speed += np.random.normal(0, 0.02)
            right_speed += np.random.normal(0, 0.001)
            
            # FIXED: Correct differential drive kinematics for North=0° system
            linear_velocity = (left_speed + right_speed) / 2.0
            
            # FIXED: For North=0° coordinate system where clockwise is positive:
            # Right turn: left_speed > right_speed, angular_velocity should be positive
            angular_velocity = (left_speed - right_speed) / WHEEL_BASE
            
            # Update robot state
            new_state = RobotState()
            new_state.time = self.robot_state.time + dt
            
            # Update heading
            new_state.heading = self.robot_state.heading + angular_velocity * dt
            
            # Normalize heading to [0, 2π]
            while new_state.heading < 0:
                new_state.heading += 2 * math.pi
            while new_state.heading >= 2 * math.pi:
                new_state.heading -= 2 * math.pi
                
            # FIXED: Position update for North=0° system
            # North (0°) = +Y direction, East (90°) = +X direction
            new_state.x = self.robot_state.x + linear_velocity * math.sin(new_state.heading) * dt
            new_state.y = self.robot_state.y + linear_velocity * math.cos(new_state.heading) * dt
            
            new_state.speed = linear_velocity
            new_state.left_wheel_speed = left_speed
            new_state.right_wheel_speed = right_speed
        
            # Calculate actual values
            new_state.actual_m1_pps = left_speed * PULSES_PER_CM
            new_state.actual_m2_pps = right_speed * PULSES_PER_CM
        
            # Calculate actual turn angle change
            actual_turn = math.degrees(new_state.heading - self.prev_robot_heading)
            while actual_turn > 180:
                actual_turn -= 360
            while actual_turn < -180:
                actual_turn += 360
            new_state.actual_turn = actual_turn
        
            # Update cumulative values
            self.actual_cumulative_total_turn += actual_turn
            new_state.actual_total_turn = self.actual_cumulative_total_turn
        
            self.actual_cumulative_m1_steps += int(new_state.actual_m1_pps * dt)
            self.actual_cumulative_m2_steps += int(new_state.actual_m2_pps * dt)
            new_state.actual_m1_steps = self.actual_cumulative_m1_steps
            new_state.actual_m2_steps = self.actual_cumulative_m2_steps
        
            self.prev_robot_heading = new_state.heading
        
            return new_state
    def print_debug_comparison(self):
        """Print detailed comparison of target vs actual with expected step values"""
        print("\n" + "="*200)
        print("TARGET vs ACTUAL COMPARISON WITH EXPECTED STEP VALUES (NORTH = 0°)")
        print("="*200)
        print(f"{'Step':<4} {'Target Position & Heading':<35} {'Actual Position & Heading':<35} "
                f"{'Expected M1/M2 PPS':<18} {'Actual M1/M2 PPS':<16} "
                f"{'Expected Turn':<12} {'Actual Turn':<12} {'Step Errors':<25}")
        print("-" * 200)
    
        for i, data in enumerate(self.debug_data):
            target = data['target']
            actual = data['actual']
        
            # Get expected values for this step
            expected_m1_pps = target.target_m1_pps
            expected_m2_pps = target.target_m2_pps
            expected_turn = getattr(target, 'expected_turn', 0.0)
            expected_m1_steps = getattr(target, 'expected_m1_steps', 0)
            expected_m2_steps = getattr(target, 'expected_m2_steps', 0)
        
            # Calculate errors
            error_x = actual.x - target.x
            error_y = actual.y - target.y
            error_hdg = math.degrees(actual.heading - target.heading)
            while error_hdg > 180: error_hdg -= 360
            while error_hdg < -180: error_hdg += 360
        
            m1_pps_error = actual.actual_m1_pps - expected_m1_pps
            m2_pps_error = actual.actual_m2_pps - expected_m2_pps
            turn_error = actual.actual_turn - expected_turn
        
            # Calculate actual step increments for this timestep
            if i > 0:
                prev_actual = self.debug_data[i-1]['actual']
                actual_m1_step_increment = actual.actual_m1_steps - prev_actual.actual_m1_steps
                actual_m2_step_increment = actual.actual_m2_steps - prev_actual.actual_m2_steps
            else:
                actual_m1_step_increment = actual.actual_m1_steps
                actual_m2_step_increment = actual.actual_m2_steps
        
            m1_step_error = actual_m1_step_increment - expected_m1_steps
            m2_step_error = actual_m2_step_increment - expected_m2_steps
        
            print(f"{i:<4} Tgt=({target.x:5.1f},{target.y:5.1f},{target.get_heading_deg():5.1f}°) "
                    f"Act=({actual.x:5.1f},{actual.y:5.1f},{math.degrees(actual.heading):5.1f}°) "
                    f"ExpM1/M2={expected_m1_pps:4.0f}/{expected_m2_pps:4.0f} "
                    f"ActM1/M2={actual.actual_m1_pps:4.0f}/{actual.actual_m2_pps:4.0f} "
                    f"ExpTurn={expected_turn:5.1f}° ActTurn={actual.actual_turn:5.1f}° "
                    f"StepErr: M1={m1_step_error:+3} M2={m2_step_error:+3} Trn={turn_error:+4.1f}°")
    # Print summary statistics
        if self.debug_data:
            errors_x = [data['actual'].x - data['target'].x for data in self.debug_data]
            errors_y = [data['actual'].y - data['target'].y for data in self.debug_data]
            errors_hdg = []
            errors_m1 = []
            errors_m2 = []
            errors_turn = []
            errors_total_turn = []
            errors_m1_steps = []
            errors_m2_steps = []
            
            for data in self.debug_data:
                error_hdg = math.degrees(data['actual'].heading - data['target'].heading)
                while error_hdg > 180:
                    error_hdg -= 360
                while error_hdg < -180:
                    error_hdg += 360
                errors_hdg.append(error_hdg)
                
                errors_m1.append(data['actual'].actual_m1_pps - data['target'].target_m1_pps)
                errors_m2.append(data['actual'].actual_m2_pps - data['target'].target_m2_pps)
                errors_turn.append(data['actual'].actual_turn - data['target'].target_turn)
                errors_total_turn.append(data['actual'].actual_total_turn - data['target'].expected_total_turn)
                errors_m1_steps.append(data['actual'].actual_m1_steps - data['target'].target_m1_steps)
                errors_m2_steps.append(data['actual'].actual_m2_steps - data['target'].target_m2_steps)
            
            print(f"\nSUMMARY STATISTICS:")
            print(f"Average Error X: {np.mean(errors_x):.2f}cm")
            print(f"Average Error Y: {np.mean(errors_y):.2f}cm") 
            print(f"Average Error Heading: {np.mean(errors_hdg):.1f}°")
            print(f"Average Error M1 PPS: {np.mean(errors_m1):.0f}")
            print(f"Average Error M2 PPS: {np.mean(errors_m2):.0f}")
            print(f"Average Error Turn: {np.mean(errors_turn):.1f}°")
            print(f"Average Error Total Turn: {np.mean(errors_total_turn):.1f}°")
            print(f"Average Error M1 Steps: {np.mean(errors_m1_steps):.0f}")
            print(f"Average Error M2 Steps: {np.mean(errors_m2_steps):.0f}")
            print(f"Max Error X: {np.max(np.abs(errors_x)):.2f}cm")
            print(f"Max Error Y: {np.max(np.abs(errors_y)):.2f}cm")
            print(f"Max Error Heading: {np.max(np.abs(errors_hdg)):.1f}°")
            print(f"Max Error M1 PPS: {np.max(np.abs(errors_m1)):.0f}")
            print(f"Max Error M2 PPS: {np.max(np.abs(errors_m2)):.0f}")
            print(f"Max Error Turn: {np.max(np.abs(errors_turn)):.1f}°")
            print(f"Max Error Total Turn: {np.max(np.abs(errors_total_turn)):.1f}°")
            print(f"Max Error M1 Steps: {np.max(np.abs(errors_m1_steps)):.0f}")
            print(f"Max Error M2 Steps: {np.max(np.abs(errors_m2_steps)):.0f}")
    
    def create_comprehensive_plot(self):
        """Create comprehensive plot with all data - North = 0°"""
        fig = plt.figure(figsize=(20, 12))
        
        # Create subplots
        ax_main = plt.subplot(2, 3, (1, 4))  # Main path plot (spans 2 rows)
        ax_commands = plt.subplot(2, 3, 2)   # Commands info
        ax_anchors = plt.subplot(2, 3, 3)    # Anchor data
        ax_waypoints = plt.subplot(2, 3, 5)  # Waypoint data  
        ax_simulation = plt.subplot(2, 3, 6) # Simulation data
        
        # Main path plot
        ax_main.set_aspect('equal')
        ax_main.grid(True, alpha=0.3)
        ax_main.set_xlabel('X (cm)')
        ax_main.set_ylabel('Y (cm)')
        ax_main.set_title(f'FIXED Robot Path V14 (North=0°) - Command: {self.planner.current_command}')
        
        # Plot anchor points
        for i, anchor in enumerate(self.planner.anchors):
            if anchor.type == 'start':
                ax_main.plot(anchor.x, anchor.y, 'gs', markersize=15, label='Start')
                # Add North arrow at start
                ax_main.arrow(anchor.x, anchor.y, 0, 8, head_width=2, head_length=2, 
                            fc='green', ec='green', alpha=0.8)
                ax_main.text(anchor.x + 3, anchor.y + 10, 'N (0°)', fontsize=10, fontweight='bold', color='green')
            elif 'turn' in anchor.type:
                ax_main.plot(anchor.x, anchor.y, 'ro', markersize=10, label='Turn Points')
                # Show turn center
                ax_main.plot(anchor.turn_center_x, anchor.turn_center_y, 'r+', markersize=8, label='Turn Centers')
            else:
                ax_main.plot(anchor.x, anchor.y, 'bo', markersize=8, label='Move Points')
        
        # Plot stop point (final waypoint)
        if self.planner.waypoints:
            final_wp = self.planner.waypoints[-1]
            ax_main.plot(final_wp.x, final_wp.y, 'rs', markersize=20, markerfacecolor='red', 
                        markeredgecolor='darkred', markeredgewidth=2, label='Stop Point')
        
        # Plot target path
        if self.planner.waypoints:
            wp_x = [wp.x for wp in self.planner.waypoints]
            wp_y = [wp.y for wp in self.planner.waypoints]
            ax_main.plot(wp_x, wp_y, 'b-', linewidth=2, alpha=0.7, label='Target Path')
            
            # Add direction arrows
            arrow_step = max(1, len(self.planner.waypoints) // 15)
            for i in range(0, len(self.planner.waypoints)-1, arrow_step):
                wp = self.planner.waypoints[i]
                dx = 3 * math.cos(wp.heading)
                dy = 3 * math.sin(wp.heading)
                ax_main.arrow(wp.x, wp.y, dx, dy, head_width=1.5, head_length=1.5, 
                            fc='blue', ec='blue', alpha=0.6)
        
        # Remove duplicate labels
        handles, labels = ax_main.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax_main.legend(by_label.values(), by_label.keys(), loc='upper right')
        
        # Commands info
        ax_commands.axis('off')
        ax_commands.set_title('Command Analysis V14 (North=0°)')
        cmd_text = f"Commands: {self.planner.current_command}\n"
        cmd_text += f"Length: {len(self.planner.current_command)}\n"
        cmd_text += f"Turns: {self.planner.current_command.count('L') + self.planner.current_command.count('R')}\n"
        cmd_text += f"Moves: {self.planner.current_command.count('F') + self.planner.current_command.count('S')}\n"
        cmd_text += f"Unit Size: {UNIT_SIZE}cm\n"
        cmd_text += f"Turn Radius: {TURN_RADIUS}cm\n"
        cmd_text += f"Wheelbase: {WHEEL_BASE}cm\n"
        #@@cmd_text += f"PPS Factor: {PULSES_PER
class RobotSimulationV14:
    def __init__(self, planner: RobotPathPlannerV14):
        self.planner = planner
        self.robot_state = RobotState()
        self.target_path = []
        self.actual_path = []
        self.pid_heading = PIDController()
        self.pid_position = PIDController()
        self.robot_artists = []
        self.debug_data = []
        self.prev_robot_heading = 0.0
        self.actual_cumulative_total_turn = 0.0
        self.actual_cumulative_m1_steps = 0
        self.actual_cumulative_m2_steps = 0
    def calculate_wheel_speeds_precomputed(self, target_wp: WayPoint) -> Tuple[float, float]:
        """Use precomputed wheel speeds with NO corrections - trust the path planner"""
    
        # Use precomputed target speeds directly (these are already correct!)
        target_left = target_wp.target_m1_pps / PULSES_PER_CM
        target_right = target_wp.target_m2_pps / PULSES_PER_CM
    
        # NO PID corrections - trust the precomputed values!
    
        return target_left, target_right

    def simulate_robot_step_fixed2(self, target_wp: WayPoint, dt: float) -> RobotState:
        pass
            # ... rest of your existing method
    def simulate_robot_step_fixed(self, target_wp: WayPoint, dt: float) -> RobotState:
        """FIXED: Simulate one step with proper differential drive kinematics"""
        
        # Get target wheel speeds
        left_speed, right_speed = self.calculate_wheel_speeds_precomputed(target_wp)
        
        # Add minimal noise
        left_speed += np.random.normal(0, 0.005)  # Even less noise
        right_speed += np.random.normal(0, 0.005)
        
        
        # CALCULATE EXPECTED VALUES FIRST (before any modifications)
        dt_step = dt  # 0.1 seconds
        expected_left_speed = target_wp.target_m1_pps / PULSES_PER_CM   # cm/s
        expected_right_speed = target_wp.target_m2_pps / PULSES_PER_CM  # cm/s
        expected_linear_vel = (expected_left_speed + expected_right_speed) / 2.0
        expected_angular_vel = (expected_left_speed - expected_right_speed) / WHEEL_BASE
        expected_turn_step = math.degrees(expected_angular_vel * dt_step)
        
        # Expected step increments for this 0.1s timestep
        expected_m1_steps_increment = int(expected_left_speed * PULSES_PER_CM * dt_step)
        expected_m2_steps_increment = int(expected_right_speed * PULSES_PER_CM * dt_step)
        
        # Store expected values in waypoint for debugging
        target_wp.expected_turn = expected_turn_step
        target_wp.expected_m1_steps = expected_m1_steps_increment  
        target_wp.expected_m2_steps = expected_m2_steps_increment
        
        # CORRECT differential drive kinematics
        linear_velocity = (left_speed + right_speed) / 2.0
        angular_velocity = (left_speed - right_speed) / WHEEL_BASE
        
        # Update robot state
        new_state = RobotState()
        new_state.time = self.robot_state.time + dt
        
        # Update heading
        new_state.heading = self.robot_state.heading + angular_velocity * dt
        
        # Normalize heading to [0, 2π]
        while new_state.heading < 0:
            new_state.heading += 2 * math.pi
        while new_state.heading >= 2 * math.pi:
            new_state.heading -= 2 * math.pi
        
        # FIXED: Position update using average heading for accuracy
        avg_heading = (self.robot_state.heading + new_state.heading) / 2.0
        new_state.x = self.robot_state.x + linear_velocity * math.sin(avg_heading) * dt
        new_state.y = self.robot_state.y + linear_velocity * math.cos(avg_heading) * dt
        
        # Store other values
        new_state.speed = linear_velocity
        new_state.left_wheel_speed = left_speed
        new_state.right_wheel_speed = right_speed
        new_state.actual_m1_pps = left_speed * PULSES_PER_CM
        new_state.actual_m2_pps = right_speed * PULSES_PER_CM
        
        # Calculate turn change
        actual_turn = math.degrees(new_state.heading - self.prev_robot_heading)
        while actual_turn > 180:
            actual_turn -= 360
        while actual_turn < -180:
            actual_turn += 360
        new_state.actual_turn = actual_turn
        
        # Update cumulative values
        self.actual_cumulative_total_turn += actual_turn
        new_state.actual_total_turn = self.actual_cumulative_total_turn
        
        self.actual_cumulative_m1_steps += int(new_state.actual_m1_pps * dt)
        self.actual_cumulative_m2_steps += int(new_state.actual_m2_pps * dt)
        new_state.actual_m1_steps = self.actual_cumulative_m1_steps
        new_state.actual_m2_steps = self.actual_cumulative_m2_steps
        
        self.prev_robot_heading = new_state.heading
        
        return new_state

    def print_debug_comparison(self):
        """Print detailed comparison of target vs actual with North = 0° including motor speeds and cumulative values"""
        print("\n" + "="*180)
        print("TARGET vs ACTUAL COMPARISON (NORTH = 0°) - WITH MOTOR SPEEDS AND CUMULATIVE VALUES")
        print("="*180)
        print(f"{'Step':<4} {'TgtX':<8} {'TgtY':<8} {'TgtHdg':<8} {'ActX':<8} {'ActY':<8} {'ActHdg':<8} {'ErrX':<6} {'ErrY':<6} {'ErrHdg':<7} {'TgtM1':<8} {'TgtM2':<8} {'ActM1':<8} {'ActM2':<8} {'TgtTrn':<7} {'ActTrn':<7} {'TgtTot':<8} {'ActTot':<8} {'TgtSt1':<8} {'TgtSt2':<8} {'ActSt1':<8} {'ActSt2':<8} {'DifSt1':<7} {'DifSt2':<7}")
        print("-"*180)
        
        for i, data in enumerate(self.debug_data):
            target = data['target']
            actual = data['actual']
            error_x = actual.x - target.x
            error_y = actual.y - target.y
            error_hdg = math.degrees(actual.heading - target.heading)
            
            # Normalize heading error
            while error_hdg > 180:
                error_hdg -= 360
            while error_hdg < -180:
                error_hdg += 360
            
            diff_m1_steps = actual.actual_m1_steps - target.target_m1_steps
            diff_m2_steps = actual.actual_m2_steps - target.target_m2_steps
            
            print(f"{i:<4} {target.x:<8.2f} {target.y:<8.2f} {target.get_heading_deg():<8.1f} "
                  f"{actual.x:<8.2f} {actual.y:<8.2f} {math.degrees(actual.heading):<8.1f} "
                  f"{error_x:<6.2f} {error_y:<6.2f} {error_hdg:<7.1f} "
                  f"{target.target_m1_pps:<8.0f} {target.target_m2_pps:<8.0f} "
                  f"{actual.actual_m1_pps:<8.0f} {actual.actual_m2_pps:<8.0f} "
                  f"{target.target_turn:<7.1f} {actual.actual_turn:<7.1f} "
                  f"{target.expected_total_turn:<8.1f} {actual.actual_total_turn:<8.1f} "
                  f"{target.target_m1_steps:<8} {target.target_m2_steps:<8} "
                  f"{actual.actual_m1_steps:<8} {actual.actual_m2_steps:<8} "
                  f"{diff_m1_steps:<7} {diff_m2_steps:<7}")
        
        print("="*180)
        
        # Print summary statistics
        if self.debug_data:
            errors_x = [data['actual'].x - data['target'].x for data in self.debug_data]
            errors_y = [data['actual'].y - data['target'].y for data in self.debug_data]
            errors_hdg = []
            errors_m1 = []
            errors_m2 = []
            errors_turn = []
            errors_total_turn = []
            errors_m1_steps = []
            errors_m2_steps = []
            
            for data in self.debug_data:
                error_hdg = math.degrees(data['actual'].heading - data['target'].heading)
                while error_hdg > 180:
                    error_hdg -= 360
                while error_hdg < -180:
                    error_hdg += 360
                errors_hdg.append(error_hdg)
                
                errors_m1.append(data['actual'].actual_m1_pps - data['target'].target_m1_pps)
                errors_m2.append(data['actual'].actual_m2_pps - data['target'].target_m2_pps)
                errors_turn.append(data['actual'].actual_turn - data['target'].target_turn)
                errors_total_turn.append(data['actual'].actual_total_turn - data['target'].expected_total_turn)
                errors_m1_steps.append(data['actual'].actual_m1_steps - data['target'].target_m1_steps)
                errors_m2_steps.append(data['actual'].actual_m2_steps - data['target'].target_m2_steps)
            
            print(f"\nSUMMARY STATISTICS:")
            print(f"Average Error X: {np.mean(errors_x):.2f}cm")
            print(f"Average Error Y: {np.mean(errors_y):.2f}cm") 
            print(f"Average Error Heading: {np.mean(errors_hdg):.1f}°")
            print(f"Average Error M1 PPS: {np.mean(errors_m1):.0f}")
            print(f"Average Error M2 PPS: {np.mean(errors_m2):.0f}")
            print(f"Average Error Turn: {np.mean(errors_turn):.1f}°")
            print(f"Average Error Total Turn: {np.mean(errors_total_turn):.1f}°")
            print(f"Average Error M1 Steps: {np.mean(errors_m1_steps):.0f}")
            print(f"Average Error M2 Steps: {np.mean(errors_m2_steps):.0f}")
            print(f"Max Error X: {np.max(np.abs(errors_x)):.2f}cm")
            print(f"Max Error Y: {np.max(np.abs(errors_y)):.2f}cm")
            print(f"Max Error Heading: {np.max(np.abs(errors_hdg)):.1f}°")
            print(f"Max Error M1 PPS: {np.max(np.abs(errors_m1)):.0f}")
            print(f"Max Error M2 PPS: {np.max(np.abs(errors_m2)):.0f}")
            print(f"Max Error Turn: {np.max(np.abs(errors_turn)):.1f}°")
            print(f"Max Error Total Turn: {np.max(np.abs(errors_total_turn)):.1f}°")
            print(f"Max Error M1 Steps: {np.max(np.abs(errors_m1_steps)):.0f}")
            print(f"Max Error M2 Steps: {np.max(np.abs(errors_m2_steps)):.0f}")
    
    def create_comprehensive_plot(self):
        """Create comprehensive plot with all data - North = 0°"""
        fig = plt.figure(figsize=(20, 12))
        
        # Create subplots
        ax_main = plt.subplot(2, 3, (1, 4))  # Main path plot (spans 2 rows)
        ax_commands = plt.subplot(2, 3, 2)   # Commands info
        ax_anchors = plt.subplot(2, 3, 3)    # Anchor data
        ax_waypoints = plt.subplot(2, 3, 5)  # Waypoint data  
        ax_simulation = plt.subplot(2, 3, 6) # Simulation data
        
        # Main path plot
        ax_main.set_aspect('equal')
        ax_main.grid(True, alpha=0.3)
        ax_main.set_xlabel('X (cm)')
        ax_main.set_ylabel('Y (cm)')
        ax_main.set_title(f'FIXED Robot Path V14 (North=0°) - Command: {self.planner.current_command}')
        
        # Plot anchor points
        for i, anchor in enumerate(self.planner.anchors):
            if anchor.type == 'start':
                ax_main.plot(anchor.x, anchor.y, 'gs', markersize=15, label='Start')
                # Add North arrow at start
                ax_main.arrow(anchor.x, anchor.y, 0, 8, head_width=2, head_length=2, 
                            fc='green', ec='green', alpha=0.8)
                ax_main.text(anchor.x + 3, anchor.y + 10, 'N (0°)', fontsize=10, fontweight='bold', color='green')
            elif 'turn' in anchor.type:
                ax_main.plot(anchor.x, anchor.y, 'ro', markersize=10, label='Turn Points')
                # Show turn center
                ax_main.plot(anchor.turn_center_x, anchor.turn_center_y, 'r+', markersize=8, label='Turn Centers')
            else:
                ax_main.plot(anchor.x, anchor.y, 'bo', markersize=8, label='Move Points')
        
        # Plot stop point (final waypoint)
        if self.planner.waypoints:
            final_wp = self.planner.waypoints[-1]
            ax_main.plot(final_wp.x, final_wp.y, 'rs', markersize=20, markerfacecolor='red', 
                        markeredgecolor='darkred', markeredgewidth=2, label='Stop Point')
        
        # Plot target path
        if self.planner.waypoints:
            wp_x = [wp.x for wp in self.planner.waypoints]
            wp_y = [wp.y for wp in self.planner.waypoints]
            ax_main.plot(wp_x, wp_y, 'b-', linewidth=2, alpha=0.7, label='Target Path')
            
            # Add direction arrows
            arrow_step = max(1, len(self.planner.waypoints) // 15)
            for i in range(0, len(self.planner.waypoints)-1, arrow_step):
                wp = self.planner.waypoints[i]
                dx = 3 * math.cos(wp.heading)
                dy = 3 * math.sin(wp.heading)
                ax_main.arrow(wp.x, wp.y, dx, dy, head_width=1.5, head_length=1.5, 
                            fc='blue', ec='blue', alpha=0.6)
        
        # Remove duplicate labels
        handles, labels = ax_main.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax_main.legend(by_label.values(), by_label.keys(), loc='upper right')
        
        # Commands info
        ax_commands.axis('off')
        ax_commands.set_title('Command Analysis V14 (North=0°)')
        cmd_text = f"Commands: {self.planner.current_command}\n"
        cmd_text += f"Length: {len(self.planner.current_command)}\n"
        cmd_text += f"Turns: {self.planner.current_command.count('L') + self.planner.current_command.count('R')}\n"
        cmd_text += f"Moves: {self.planner.current_command.count('F') + self.planner.current_command.count('S')}\n"
        cmd_text += f"Unit Size: {UNIT_SIZE}cm\n"
        cmd_text += f"Turn Radius: {TURN_RADIUS}cm\n"
        cmd_text += f"Wheelbase: {WHEEL_BASE}cm\n"
        cmd_text += f"PPS Factor: {PULSES_PER_CM}\n"
        cmd_text += f"Initial: North = 0°"
        ax_commands.text(0.05, 0.95, cmd_text, transform=ax_commands.transAxes, 
                        verticalalignment='top', fontfamily='monospace', fontsize=10)
        
        # Anchor data
        ax_anchors.axis('off')
        ax_anchors.set_title('Anchor Points')
        anchor_text = "ID  X      Y     Dir  Type\n"
        anchor_text += "-" * 30 + "\n"
        for i, anchor in enumerate(self.planner.anchors):
            dir_arrow = DIRECTION_ARROWS[anchor.dir]
            anchor_text += f"A{i:2} {anchor.x:6.1f} {anchor.y:6.1f} {dir_arrow}  {anchor.type}\n"
        ax_anchors.text(0.05, 0.95, anchor_text, transform=ax_anchors.transAxes, 
                       verticalalignment='top', fontfamily='monospace', fontsize=8)
        
        # Waypoint data (first 10)
        ax_waypoints.axis('off')
        ax_waypoints.set_title('Waypoints (First 10)')
        wp_text = "ID   X      Y    Spd Hdg   M1pps M2pps Turn\n"
        wp_text += "-" * 45 + "\n"
        for i, wp in enumerate(self.planner.waypoints[:10]):
            wp_text += f"W{i:2} {wp.x:6.1f} {wp.y:5.1f} {wp.speed:3.0f} {wp.get_heading_deg():5.1f}° {wp.target_m1_pps:5.0f} {wp.target_m2_pps:5.0f} {wp.target_turn:5.1f}°\n"
        if len(self.planner.waypoints) > 10:
            wp_text += f"... and {len(self.planner.waypoints)-10} more"
        ax_waypoints.text(0.05, 0.95, wp_text, transform=ax_waypoints.transAxes, 
                         verticalalignment='top', fontfamily='monospace', fontsize=8)
        
        # Simulation info
        ax_simulation.axis('off')
        ax_simulation.set_title('Simulation Data V14')
        sim_text = f"Total Distance: {self.planner.total_distance:.1f}cm\n"
        sim_text += f"Total Time: {self.planner.total_time:.1f}s\n"
        sim_text += f"Avg Speed: {self.planner.total_distance/self.planner.total_time:.1f}cm/s\n\n"
        sim_text += "Robot Parameters:\n"
        sim_text += f"Wheel Base: {WHEEL_BASE}cm\n"
        sim_text += f"Max Wheel Speed: {MAX_WHEEL_SPEED}cm/s\n"
        sim_text += f"Turn Radius: {TURN_RADIUS}cm\n"
        sim_text += f"Pulses/cm: {PULSES_PER_CM}\n\n"
        sim_text += "PID Parameters (Reduced):\n"
        sim_text += f"Kp: {KP}, Ki: {KI}, Kd: {KD}\n\n"
        sim_text += "Coordinate System:\n"
        sim_text += f"North: 0° (+Y direction)\n"
        sim_text += f"East: 90° (+X direction)"
        ax_simulation.text(0.05, 0.95, sim_text, transform=ax_simulation.transAxes, 
                          verticalalignment='top', fontfamily='monospace', fontsize=10)
        
        plt.tight_layout()
        return fig, ax_main
    
    def animate_robot_motion(self):
        """Animate FIXED robot following the path with North = 0°"""
        if not self.planner.waypoints:
            return
            
        print("=== ANIMATING FIXED ROBOT MOTION V14 (NORTH = 0°) ===")
        
        fig, ax_main = self.create_comprehensive_plot()
        
        self.actual_path = []
        self.debug_data = []
        dt = 0.1  # 100ms time step
        
        # Reset robot state - START FACING NORTH (0°)
        if self.planner.waypoints:
            first_wp = self.planner.waypoints[0]
            self.robot_state = RobotState(first_wp.x, first_wp.y, 0.0, 0.0, 0.0)  # heading = 0° = North
            print(f"Robot initialized at ({first_wp.x:.1f}, {first_wp.y:.1f}) facing North (0°)")
        
        for i, target_wp in enumerate(self.planner.waypoints):
            # Simulate robot step
            self.robot_state = self.simulate_robot_step_fixed(target_wp, dt)
            self.actual_path.append((self.robot_state.x, self.robot_state.y))
            
            # Store debug data
            self.debug_data.append({
                'target': target_wp,
                'actual': self.robot_state
            })
            
            # Clear previous robot visualization
            for artist in self.robot_artists:
                try:
                    artist.remove()
                except:
                    pass
            self.robot_artists.clear()
            
            # Plot actual traveled path
            if len(self.actual_path) > 1:
                actual_x = [p[0] for p in self.actual_path]
                actual_y = [p[1] for p in self.actual_path]
                actual_line = ax_main.plot(actual_x, actual_y, 'g-', linewidth=3, alpha=0.8, label='Actual Path')[0]
                self.robot_artists.append(actual_line)
            
            # Plot current robot position
            robot_dot = ax_main.plot(self.robot_state.x, self.robot_state.y, 'ro', markersize=12)[0]
            self.robot_artists.append(robot_dot)
            
            # Plot robot heading arrow (North = 0°)
            dx = 4 * math.cos(self.robot_state.heading)
            dy = 4 * math.sin(self.robot_state.heading)
            heading_arrow = ax_main.arrow(self.robot_state.x, self.robot_state.y, dx, dy, 
                                        head_width=2, head_length=2, fc='red', ec='red', linewidth=2)
            self.robot_artists.append(heading_arrow)
            
            # Update title with current state
            title = f'Robot at W{i}: Pos=({self.robot_state.x:.1f},{self.robot_state.y:.1f}) '
            title += f'Hdg={math.degrees(self.robot_state.heading):.1f}° Speed={self.robot_state.speed:.1f}cm/s '
            title += f'L/R={self.robot_state.left_wheel_speed:.1f}/{self.robot_state.right_wheel_speed:.1f}cm/s '
            title += f'Turn={self.robot_state.actual_turn:.1f}° TotalTurn={self.robot_state.actual_total_turn:.1f}°'
            ax_main.set_title(title)
                          # Get expected values for this step
            expected_turn = getattr(target_wp, 'expected_turn', 0.0)
            expected_m1_steps = getattr(target_wp, 'expected_m1_steps', 0)
            expected_m2_steps = getattr(target_wp, 'expected_m2_steps', 0)

            print(f"W{i:2}: Target=({target_wp.x:5.1f},{target_wp.y:5.1f},{target_wp.get_heading_deg():6.1f}°) "
                f"Actual=({self.robot_state.x:5.1f},{self.robot_state.y:5.1f},{math.degrees(self.robot_state.heading):6.1f}°) "
                f"ExpM1/M2={target_wp.target_m1_pps:.0f}/{target_wp.target_m2_pps:.0f}pps "
                f"ActM1/M2={self.robot_state.actual_m1_pps:.0f}/{self.robot_state.actual_m2_pps:.0f}pps "
                f"ExpTurn={expected_turn:.1f}° ActTurn={self.robot_state.actual_turn:.1f}° "
                f"ExpSteps={expected_m1_steps}/{expected_m2_steps} TotalTurn={self.robot_state.actual_total_turn:.1f}°")

            if target_wp.is_stop():
                ax_main.set_title('ROBOT STOPPED - Path Complete (North = 0°)')
                print("ROBOT STOPPED at final waypoint")
                break
            
            plt.draw()
            plt.pause(0.1)  # Faster animation
        
        # Print detailed comparison at the end
        self.print_debug_comparison()
        
        print("=== ROBOT ANIMATION COMPLETE ===")
        plt.show(block=True)

# =============================================================================
# MAIN SIMULATION - NORTH = 0° - TEST ONLY "R"
# =============================================================================
def main():
    print("Robot Path Planner V14 - FIXED with North = 0°")
    print("F = Move 25cm forward")
    print("R = Turn right 90° (CLOCKWISE): North->East->South->West->North")
    print("L = Turn left 90° (CCW): North->West->South->East->North")  
    print("Initial direction: North = 0° (pointing +Y)")
    print("Coordinate system: North=0°, East=90°, South=180°, West=270°")
    print("=" * 70)
    
    # Test only "R" command
    cmd_set = "R"
    print(f"\n{'='*20} TESTING: {cmd_set} {'='*20}")
    
    planner = RobotPathPlannerV14()
    planner.process_commands(cmd_set)
    
    sim = RobotSimulationV14(planner)
    
    input(f"Press Enter to simulate FIXED path for '{cmd_set}' (North=0°)...")
    sim.animate_robot_motion()

# main()

def testStraight():
    """Test function to verify robot moves straight north when both motors have same speed"""
    print("=== TESTING STRAIGHT MOTION (North = 0°) ===")
    print("Both motors same speed -> Robot should move in +Y direction only")
    print("=" * 60)
    
    # Initialize robot at origin facing North (0°)
    robot = RobotState(0.0, 0.0, 0.0, 0.0, 0.0)  # x, y, heading=0° (North)
    dt = 0.1  # time step
    
    # Test parameters
    test_speed = 10.0  # cm/s for both wheels
    test_steps = 10
    
    print(f"Initial: Robot at ({robot.x:.2f}, {robot.y:.2f}) heading {math.degrees(robot.heading):.1f}° (North)")
    print(f"Motor speeds: Left={test_speed} cm/s, Right={test_speed} cm/s")
    print(f"Expected: Robot moves in +Y direction only (North)")
    print("-" * 60)
    
    for step in range(test_steps):
        # Same speed for both wheels = straight motion
        left_speed = test_speed
        right_speed = test_speed
        
        # Calculate kinematics
        linear_velocity = (left_speed + right_speed) / 2.0
        angular_velocity = (right_speed - left_speed) / WHEEL_BASE  # Should be 0
        
        # Update robot state
        new_robot = RobotState()
        new_robot.time = robot.time + dt
        
        # Update heading (should not change for straight motion)
        new_robot.heading = robot.heading + angular_velocity * dt
        
        # FIXED: For North=0° coordinate system, use correct trigonometry
        # North (0°) should move in +Y direction
        # East (90°) should move in +X direction
        new_robot.x = robot.x + linear_velocity * math.sin(new_robot.heading) * dt  # FIXED: use sin for X
        new_robot.y = robot.y + linear_velocity * math.cos(new_robot.heading) * dt  # FIXED: use cos for Y
        
        new_robot.speed = linear_velocity
        new_robot.left_wheel_speed = left_speed
        new_robot.right_wheel_speed = right_speed
        
        # Calculate PPS values
        new_robot.actual_m1_pps = left_speed * PULSES_PER_CM
        new_robot.actual_m2_pps = right_speed * PULSES_PER_CM
        
        # Print results
        dx = new_robot.x - robot.x
        dy = new_robot.y - robot.y
        print(f"Step {step+1:2}: Pos=({new_robot.x:6.2f}, {new_robot.y:6.2f}) "
              f"Hdg={math.degrees(new_robot.heading):6.1f}° "
              f"dX={dx:6.2f} dY={dy:6.2f} "
              f"M1/M2={new_robot.actual_m1_pps:.0f}/{new_robot.actual_m2_pps:.0f}pps")
        
        robot = new_robot
    
    print("-" * 60)
    print(f"Final: Robot at ({robot.x:.2f}, {robot.y:.2f}) heading {math.degrees(robot.heading):.1f}°")
    print(f"Total movement: dX={robot.x:.2f}cm (should be ≈0), dY={robot.y:.2f}cm (should be >0)")
    
    # Verify results
    if abs(robot.x) < 0.1 and robot.y > 0:
        print("✓ TEST PASSED: Robot moved straight north as expected")
    else:
        print("✗ TEST FAILED: Robot did not move straight north")
        if abs(robot.x) > 0.1:
            print(f"  - Unexpected X movement: {robot.x:.2f}cm")
        if robot.y <= 0:
            print(f"  - No Y movement or wrong direction: {robot.y:.2f}cm")

#@@def main():
    # Test straight motion only
#testStraight()

def testR():
    """Test function to verify robot turns right when right motor has less PPS"""
    print("=== TESTING RIGHT TURN (R) ===")
    print("Left motor faster, Right motor slower -> Robot should turn right (clockwise)")
    print("=" * 70)
    
    # Initialize robot at origin facing North (0°)
    robot = RobotState(0.0, 0.0, 0.0, 0.0, 0.0)  # x, y, heading=0° (North)
    dt = 0.1  # time step
    
    # Test parameters for right turn
    left_speed = 18.50   # cm/s - faster (left wheel)
    right_speed = 6.0  # cm/s - slower (right wheel) as requested
    test_steps = int(1.00* 50*2 * ( 90/ 71.0))    # more steps to see the turn
    
    print(f"Initial: Robot at ({robot.x:.2f}, {robot.y:.2f}) heading {math.degrees(robot.heading):.1f}° (North)")
    print(f"Motor speeds: Left={left_speed} cm/s, Right={right_speed} cm/s")
    print(f"Expected: Robot turns right (clockwise) from North toward East")
    print("-" * 70)
    
    for step in range(test_steps):
        # Different speeds for right turn
        # Left faster than right = right turn (clockwise)
        
        # Calculate kinematics
        linear_velocity = (left_speed + right_speed) / 2.0
        angular_velocity = -(right_speed - left_speed) / WHEEL_BASE  # Negative = clockwise
        
        # Update robot state
        new_robot = RobotState()
        new_robot.time = robot.time + dt
        
        # Update heading (should increase for right turn)
        new_robot.heading = robot.heading + angular_velocity * dt
        
        # Normalize heading to [0, 2π]
        while new_robot.heading < 0:
            new_robot.heading += 2 * math.pi
        while new_robot.heading >= 2 * math.pi:
            new_robot.heading -= 2 * math.pi
        
        # FIXED: For North=0° coordinate system
        new_robot.x = robot.x + linear_velocity * math.sin(new_robot.heading) * dt  # sin for X
        new_robot.y = robot.y + linear_velocity * math.cos(new_robot.heading) * dt  # cos for Y
        
        new_robot.speed = linear_velocity
        new_robot.left_wheel_speed = left_speed
        new_robot.right_wheel_speed = right_speed
        
        # Calculate PPS values
        new_robot.actual_m1_pps = left_speed * PULSES_PER_CM
        new_robot.actual_m2_pps = right_speed * PULSES_PER_CM
        
        # Calculate turn change
        heading_change = math.degrees(new_robot.heading - robot.heading)
        while heading_change > 180:
            heading_change -= 360
        while heading_change < -180:
            heading_change += 360
        
        # Print results every few steps
        if step % 3 == 0 or step < 5:
            print(f"Step {step+1:2}: Pos=({new_robot.x:6.2f}, {new_robot.y:6.2f}) "
                  f"Hdg={math.degrees(new_robot.heading):6.1f}° "
                  f"ΔHdg={heading_change:5.1f}° "
                  f"M1/M2={new_robot.actual_m1_pps:.0f}/{new_robot.actual_m2_pps:.0f}pps "
                  f"AngVel={math.degrees(angular_velocity):5.1f}°/s")
        
        robot = new_robot
    
    print("-" * 70)
    final_heading_deg = math.degrees(robot.heading)
    total_turn = final_heading_deg  # Since we started at 0°
    
    print(f"Final: Robot at ({robot.x:.2f}, {robot.y:.2f}) heading {final_heading_deg:.1f}°")
    print(f"Total turn: {total_turn:.1f}° (positive = clockwise right turn)")
    print(f"Expected turn: ~90° for quarter circle right turn")
    
    # Verify results
    if 70 <= total_turn <= 110:  # Allow some tolerance around 90°
        print("✓ TEST PASSED: Robot turned right as expected")
    else:
        print("✗ TEST FAILED: Robot turn not as expected")
        if total_turn < 70:
            print(f"  - Turn too small: {total_turn:.1f}° (expected ~90°)")
        elif total_turn > 110:
            print(f"  - Turn too large: {total_turn:.1f}° (expected ~90°)")
    
    # Show the differential effect
    speed_diff = left_speed - right_speed
    expected_angular_vel = speed_diff / WHEEL_BASE
    print(f"\nDifferential Analysis:")
    print(f"Speed difference: {speed_diff:.1f} cm/s")
    print(f"Wheelbase: {WHEEL_BASE:.1f} cm")
    print(f"Expected angular velocity: {math.degrees(expected_angular_vel):.1f}°/s")
    print(f"Expected turn in {test_steps*dt:.1f}s: {math.degrees(expected_angular_vel * test_steps * dt):.1f}°")

    # Test right turn
#@testR()
main()

