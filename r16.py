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
UNIT_SIZE = 25.0         # 25cm grid units
DEFAULT_SPEED = 15.0     # cm/s
CURVE_SPEED = 10.0       # cm/s for curves
TURN_RADIUS = 25.0       # cm - radius for turns
INTERPOLATION_STEP = 1.0 # cm between points
DEBUG_ENABLED = True

# Robot physical parameters
WHEEL_BASE = 20.0        # cm
MAX_WHEEL_SPEED = 30.0   # cm/s
MIN_WHEEL_SPEED = 2.0    # cm/s

# Direction constants - NORTH=0°
DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST = 0, 1, 2, 3
DIRECTION_NAMES = ["NORTH", "EAST", "SOUTH", "WEST"] 
DIRECTION_ARROWS = ["↑", "→", "↓", "←"]

# Direction deltas [dx, dy]
DIRECTION_DELTA = [
    [0,  1],  # North: Y+
    [1,  0],  # East:  X+ 
    [0, -1],  # South: Y-
    [-1, 0]   # West:  X-
]

# Direction to radians - NORTH = 0°
DIRECTION_TO_RADIANS = [
    0.0,           # North: 0°
    math.pi/2,     # East:  90°
    math.pi,       # South: 180°
    3*math.pi/2    # West:  270°
]

# =============================================================================
# DATA STRUCTURES
# =============================================================================
@dataclass
class AnchorPoint:
    x: float
    y: float
    dir: int
    type: str = "move"
    turn_center_x: float = 0.0
    turn_center_y: float = 0.0
    
    def get_heading_rad(self) -> float:
        return DIRECTION_TO_RADIANS[self.dir]
        
    def get_heading_deg(self) -> float:
        return self.dir * 90.0

@dataclass
class WayPoint:
    x: float
    y: float
    speed: float
    heading: float  # radians
    time: float = 0.0
    curvature: float = 0.0
    
    def get_heading_deg(self) -> float:
        return math.degrees(self.heading)
    
    def is_stop(self) -> bool:
        return self.speed < 0.2

@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians
    speed: float = 0.0
    time: float = 0.0
    left_wheel_speed: float = 0.0
    right_wheel_speed: float = 0.0

@dataclass
class PIDController:
    kp: float = 1.0
    ki: float = 0.01
    kd: float = 0.05
    prev_error: float = 0.0
    integral: float = 0.0
    
    def update(self, error: float, dt: float) -> float:
        if abs(self.integral) > 10.0:
            self.integral = 10.0 if self.integral > 0 else -10.0
            
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# =============================================================================
# FIXED ROBOT PATH PLANNER
# =============================================================================
class RobotPathPlanner:
    def __init__(self):
        self.anchors: List[AnchorPoint] = []
        self.waypoints: List[WayPoint] = []
        self.total_distance = 0.0
        self.total_time = 0.0
        self.current_command = ""
        
    def debug_print(self, msg: str):
        if DEBUG_ENABLED:
            print(msg)
    
    def convert_commands_to_anchors(self, commands: str) -> int:
        """FIXED: Convert commands to anchors"""
        current_x, current_y = 0.0, 0.0
        current_dir = DIR_NORTH
        self.anchors.clear()
        self.current_command = commands
        
        self.debug_print("=== FIXED ROBOT PATH PLANNER ===")
        self.debug_print(f"Commands: {commands}")
        
        # Add starting anchor
        self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "start"))
        self.debug_print(f"START: ({current_x}, {current_y}) facing {DIRECTION_NAMES[current_dir]}")
        
        for i, cmd in enumerate(commands):
            if cmd in ['L', 'R']:
                # Create turn
                self.create_circular_turn_fixed(current_x, current_y, current_dir, cmd)
                
                # Update direction
                if cmd == 'L':
                    current_dir = (current_dir + 3) % 4  # Left turn
                else:
                    current_dir = (current_dir + 1) % 4  # Right turn
                
                # Update position to end of turn  
                current_x, current_y = self.anchors[-1].x, self.anchors[-1].y
                self.debug_print(f"{cmd}: Turn -> {DIRECTION_NAMES[current_dir]} at ({current_x}, {current_y})")
                
            elif cmd == 'F':
                # Move forward
                dx, dy = DIRECTION_DELTA[current_dir]
                current_x += dx * UNIT_SIZE
                current_y += dy * UNIT_SIZE
                
                self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "move"))
                self.debug_print(f"F: Move to ({current_x}, {current_y})")
        
        self.debug_print(f"Created {len(self.anchors)} anchor points")
        return len(self.anchors)
    
    def create_circular_turn_fixed(self, start_x: float, start_y: float, start_dir: int, turn_cmd: str):
        """FIXED: Create proper circular turn"""
        self.debug_print(f"  Creating {turn_cmd} turn from ({start_x}, {start_y}) facing {DIRECTION_NAMES[start_dir]}")
        
        if turn_cmd == 'R':  # Right turn (CLOCKWISE)
            if start_dir == DIR_NORTH:      # North -> East
                center_x = start_x + TURN_RADIUS
                center_y = start_y  
                end_x = center_x
                end_y = center_y + TURN_RADIUS
                end_dir = DIR_EAST
            elif start_dir == DIR_EAST:     # East -> South  
                center_x = start_x
                center_y = start_y - TURN_RADIUS
                end_x = center_x + TURN_RADIUS
                end_y = center_y
                end_dir = DIR_SOUTH
            elif start_dir == DIR_SOUTH:    # South -> West
                center_x = start_x - TURN_RADIUS
                center_y = start_y
                end_x = center_x
                end_y = center_y - TURN_RADIUS
                end_dir = DIR_WEST
            else:  # DIR_WEST               # West -> North
                center_x = start_x
                center_y = start_y + TURN_RADIUS
                end_x = center_x - TURN_RADIUS
                end_y = center_y
                end_dir = DIR_NORTH
                
        else:  # Left turn (COUNTER-CLOCKWISE)
            if start_dir == DIR_NORTH:      # North -> West
                center_x = start_x - TURN_RADIUS
                center_y = start_y
                end_x = center_x
                end_y = center_y + TURN_RADIUS
                end_dir = DIR_WEST
            elif start_dir == DIR_EAST:     # East -> North
                center_x = start_x
                center_y = start_y + TURN_RADIUS
                end_x = center_x + TURN_RADIUS
                end_y = center_y
                end_dir = DIR_NORTH
            elif start_dir == DIR_SOUTH:    # South -> East
                center_x = start_x + TURN_RADIUS
                center_y = start_y
                end_x = center_x
                end_y = center_y - TURN_RADIUS
                end_dir = DIR_EAST
            else:  # DIR_WEST              # West -> South
                center_x = start_x
                center_y = start_y - TURN_RADIUS
                end_x = center_x - TURN_RADIUS
                end_y = center_y
                end_dir = DIR_SOUTH
        
        # Add turn anchors
        self.anchors.append(AnchorPoint(start_x, start_y, start_dir, "turn_start", center_x, center_y))
        self.anchors.append(AnchorPoint(end_x, end_y, end_dir, "turn_end", center_x, center_y))
        
        self.debug_print(f"    Center: ({center_x}, {center_y})")
        self.debug_print(f"    End: ({end_x}, {end_y}) facing {DIRECTION_NAMES[end_dir]}")
    
    def interpolate_waypoints_fixed(self) -> int:
        """FIXED: Create proper waypoints"""
        self.waypoints.clear()
        current_time = 0.0
        
        self.debug_print("=== FIXED WAYPOINT INTERPOLATION ===")
        
        for i in range(len(self.anchors) - 1):
            current = self.anchors[i]
            next_anchor = self.anchors[i + 1]
            
            if current.type == "turn_start" and next_anchor.type == "turn_end":
                # FIXED: Circular arc interpolation
                self.interpolate_circular_arc_fixed(current, next_anchor, current_time)
                arc_length = 0.25 * 2 * math.pi * TURN_RADIUS
                segment_time = arc_length / CURVE_SPEED
                current_time += segment_time
                
            else:
                # Straight line interpolation
                distance = math.sqrt((next_anchor.x - current.x)**2 + (next_anchor.y - current.y)**2)
                if distance > 0:
                    speed = DEFAULT_SPEED
                    segment_time = distance / speed
                    num_points = max(3, int(distance / INTERPOLATION_STEP))
                    
                    for j in range(num_points + 1):
                        t = j / num_points
                        wp_x = current.x + t * (next_anchor.x - current.x)
                        wp_y = current.y + t * (next_anchor.y - current.y)
                        heading = current.get_heading_rad()
                        waypoint_time = current_time + t * segment_time
                        
                        self.waypoints.append(WayPoint(wp_x, wp_y, speed, heading, waypoint_time))
                    
                    current_time += segment_time
        
        # Set final waypoint as stop
        if self.waypoints:
            self.waypoints[-1].speed = 0
            
        self.total_time = current_time
        self.total_distance = sum(math.sqrt((self.waypoints[i+1].x - self.waypoints[i].x)**2 + 
                                          (self.waypoints[i+1].y - self.waypoints[i].y)**2) 
                                for i in range(len(self.waypoints)-1))
        
        self.debug_print(f"Generated {len(self.waypoints)} waypoints")
        self.debug_print(f"First few waypoints:")
        for i, wp in enumerate(self.waypoints[:5]):
            self.debug_print(f"  W{i}: ({wp.x:.1f}, {wp.y:.1f}) hdg={wp.get_heading_deg():.1f}° speed={wp.speed:.1f}")
        
        return len(self.waypoints)
    
    def interpolate_circular_arc_fixed(self, start_anchor: AnchorPoint, end_anchor: AnchorPoint, start_time: float):
        """FIXED: Interpolate circular arc correctly"""
        center_x = start_anchor.turn_center_x
        center_y = start_anchor.turn_center_y
        
        # Calculate angles from center
        start_angle = math.atan2(start_anchor.y - center_y, start_anchor.x - center_x)
        end_angle = math.atan2(end_anchor.y - center_y, end_anchor.x - center_x)
        
        # Determine turn direction
        start_dir = start_anchor.dir
        end_dir = end_anchor.dir
        
        # FIXED: Correct angle difference calculation
        if (start_dir + 1) % 4 == end_dir:  # Right turn (clockwise)
            # For clockwise, we want to go from start_angle to end_angle in negative direction
            if end_angle > start_angle:
                end_angle -= 2 * math.pi
            angle_diff = end_angle - start_angle  # This will be negative
        else:  # Left turn (counter-clockwise)
            # For counter-clockwise, we want to go from start_angle to end_angle in positive direction  
            if end_angle < start_angle:
                end_angle += 2 * math.pi
            angle_diff = end_angle - start_angle  # This will be positive
        
        arc_length = abs(angle_diff) * TURN_RADIUS
        segment_time = arc_length / CURVE_SPEED
        num_points = max(10, int(arc_length / INTERPOLATION_STEP))
        
        self.debug_print(f"  FIXED Arc: center=({center_x:.1f}, {center_y:.1f})")
        self.debug_print(f"  Start angle={math.degrees(start_angle):.1f}° End angle={math.degrees(end_angle):.1f}°")
        self.debug_print(f"  Angle diff={math.degrees(angle_diff):.1f}° Arc length={arc_length:.1f}cm Points={num_points}")
        
        for j in range(num_points + 1):
            t = j / num_points
            angle = start_angle + t * angle_diff
            
            # Calculate waypoint position
            wp_x = center_x + TURN_RADIUS * math.cos(angle)
            wp_y = center_y + TURN_RADIUS * math.sin(angle)
            
            # FIXED: Calculate heading as tangent to circle
            # Tangent angle is perpendicular to radius
            if angle_diff > 0:  # Counter-clockwise
                heading = angle + math.pi/2
            else:  # Clockwise
                heading = angle - math.pi/2
                
            # Normalize heading
            while heading < 0:
                heading += 2 * math.pi
            while heading >= 2 * math.pi:
                heading -= 2 * math.pi
            
            curvature = 1.0 / TURN_RADIUS
            waypoint_time = start_time + t * segment_time
            
            self.waypoints.append(WayPoint(wp_x, wp_y, CURVE_SPEED, heading, waypoint_time, curvature))
    
    def process_commands(self, commands: str):
        """Process commands"""
        self.convert_commands_to_anchors(commands)
        self.interpolate_waypoints_fixed()

# =============================================================================  
# FIXED ROBOT SIMULATION
# =============================================================================
class RobotSimulation:
    def __init__(self, planner: RobotPathPlanner = None):
        self.planner = planner
        self.robot_state = RobotState()
        self.actual_path = []
        self.waypoint_index = 0
        self.look_ahead_distance = 2.0
        self.arrival_threshold = 1.5  # REDUCED - advance waypoints sooner
        self.robot_artists = []
        self.debug_data = []
        
        # Gentler PID
        self.pid_heading = PIDController(kp=0.3, ki=0.005, kd=0.01)
        
    def find_target_waypoint(self) -> int:
        """FIXED: Find target waypoint and advance properly"""
        if not self.planner.waypoints:
            return 0
            
        current_pos = (self.robot_state.x, self.robot_state.y)
        
        # Check if we should advance to next waypoint
        while self.waypoint_index < len(self.planner.waypoints):
            wp = self.planner.waypoints[self.waypoint_index]
            distance = math.sqrt((wp.x - current_pos[0])**2 + (wp.y - current_pos[1])**2)
            
            # If close enough, advance to next waypoint
            if distance < self.arrival_threshold and self.waypoint_index < len(self.planner.waypoints) - 1:
                self.waypoint_index += 1
                self.debug_print(f"    Advanced to waypoint {self.waypoint_index}")
            else:
                break
                
        return min(self.waypoint_index, len(self.planner.waypoints) - 1)
    
    def calculate_wheel_speeds(self, target_wp: WayPoint) -> Tuple[float, float]:
        """FIXED: Calculate wheel speeds"""
        # Calculate distance and direction to target
        dx = target_wp.x - self.robot_state.x
        dy = target_wp.y - self.robot_state.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.3:  # Very close - stop
            return 0.0, 0.0
        
        # Calculate desired heading
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - self.robot_state.heading
        
        # Normalize heading error
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Calculate speeds
        base_speed = min(target_wp.speed, 8.0)  # Reduced max speed
        
        # Reduce speed when turning
        speed_factor = max(0.2, 1.0 - abs(heading_error) * 0.5)
        base_speed *= speed_factor
        
        # Reduce speed when close to target
        if distance < 3.0:
            base_speed = min(base_speed, distance * 2.0)
        
        # Calculate turn rate
        turn_rate = self.pid_heading.update(heading_error, 0.1)
        turn_rate = max(-1.0, min(1.0, turn_rate))  # Limit turn rate
        
        # Differential steering
        left_speed = base_speed - turn_rate * WHEEL_BASE / 2
        right_speed = base_speed + turn_rate * WHEEL_BASE / 2
        
        # Limit speeds
        left_speed = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left_speed))
        right_speed = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right_speed))
        
        return left_speed, right_speed
    
    def simulate_robot_step(self, dt: float) -> RobotState:
        """Simulate robot step"""
        wp_idx = self.find_target_waypoint()
        if wp_idx >= len(self.planner.waypoints):
            return self.robot_state
            
        target_wp = self.planner.waypoints[wp_idx]
        left_speed, right_speed = self.calculate_wheel_speeds(target_wp)
        
        # Add small noise
        left_speed += np.random.normal(0, 0.05)
        right_speed += np.random.normal(0, 0.05)
        
        # Update robot state
        new_state = RobotState()
        new_state.time = self.robot_state.time + dt
        new_state.left_wheel_speed = left_speed
        new_state.right_wheel_speed = right_speed
        
        # Calculate motion
        linear_velocity = (left_speed + right_speed) / 2.0
        angular_velocity = (right_speed - left_speed) / WHEEL_BASE
        
        # Update heading
        new_state.heading = self.robot_state.heading + angular_velocity * dt
        while new_state.heading < 0:
            new_state.heading += 2 * math.pi
        while new_state.heading >= 2 * math.pi:
            new_state.heading -= 2 * math.pi
        
        # Update position
        new_state.x = self.robot_state.x + linear_velocity * math.cos(new_state.heading) * dt
        new_state.y = self.robot_state.y + linear_velocity * math.sin(new_state.heading) * dt
        new_state.speed = linear_velocity
        
        return new_state
    
    def debug_print(self, msg: str):
        if DEBUG_ENABLED:
            print(msg)
    
    def animate_robot_motion(self):
        """FIXED: Animate robot motion"""
        if not self.planner.waypoints:
            return
            
        print("=== ANIMATING FIXED ROBOT MOTION ===")
        
        # Initialize robot at first waypoint
        if self.planner.waypoints:
            first_wp = self.planner.waypoints[0]
            self.robot_state = RobotState(first_wp.x, first_wp.y, 0.0, 0.0, 0.0)
            self.waypoint_index = 0
            print(f"Robot initialized at ({first_wp.x:.1f}, {first_wp.y:.1f}) facing North (0°)")
        
        self.actual_path = []
        self.debug_data = []
        dt = 0.1
        max_steps = 200  # Reduced max steps
        step = 0
        
        while step < max_steps:
            wp_idx = min(self.waypoint_index, len(self.planner.waypoints) - 1)
            target_wp = self.planner.waypoints[wp_idx]
            
            # Simulate step
            self.robot_state = self.simulate_robot_step(dt)
            self.actual_path.append((self.robot_state.x, self.robot_state.y))
            
            # Store debug data
            self.debug_data.append({
                'target': target_wp,
                'actual': self.robot_state,
                'waypoint_idx': wp_idx
            })
            
            distance_to_target = math.sqrt((target_wp.x - self.robot_state.x)**2 + (target_wp.y - self.robot_state.y)**2)
            
            print(f"Step {step:3}: WP{wp_idx:2} Target=({target_wp.x:5.1f},{target_wp.y:5.1f}) "
                  f"Actual=({self.robot_state.x:5.1f},{self.robot_state.y:5.1f}) "
                  f"Hdg={math.degrees(self.robot_state.heading):6.1f}° "
                  f"Dist={distance_to_target:4.1f}")
            
            # Check if finished
            if (wp_idx >= len(self.planner.waypoints) - 1 and 
                distance_to_target < 0.5 and 
                abs(self.robot_state.speed) < 1.0):
                print("ROBOT REACHED TARGET!")
                break
            
            step += 1
        
        if step >= max_steps:
            print("Simulation stopped - max steps reached")
        
        # Print final results
        if self.debug_data:
            final = self.debug_data[-1]
            target = final['target']
            actual = final['actual']
            error_x = actual.x - target.x
            error_y = actual.y - target.y
            error_dist = math.sqrt(error_x**2 + error_y**2)
            
            print(f"\nFINAL RESULTS:")
            print(f"Target: ({target.x:.1f}, {target.y:.1f})")
            print(f"Actual: ({actual.x:.1f}, {actual.y:.1f})")
            print(f"Error: ({error_x:.1f}, {error_y:.1f}) = {error_dist:.1f}cm")
        
        print("=== ANIMATION COMPLETE ===")

# =============================================================================
# MAIN
# =============================================================================
def main():
    test_commands = [
        "R",       # Single right turn
       # "RRRR",    # Complete circle
    ]
    
    print("FIXED Robot Path Planner")
    print("R = Turn right 90°")
    print("=" * 50)
    
    for cmd_set in test_commands:
        print(f"\n{'='*20} TESTING: {cmd_set} {'='*20}")
        
        planner = RobotPathPlanner()
        planner.process_commands(cmd_set)
        
        sim = RobotSimulation(planner)
        
        input(f"Press Enter to simulate FIXED path for '{cmd_set}'...")
        sim.animate_robot_motion()

#if __name__ == "__main__":
main()
