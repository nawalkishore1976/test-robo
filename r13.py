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
KP = 2.0                 # Proportional gain (increased)
KI = 0.05                # Integral gain (reduced)
KD = 0.1                 # Derivative gain (increased)

# Direction constants
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

# =============================================================================
# DATA STRUCTURES
# =============================================================================
@dataclass
class AnchorPoint:
    """Key waypoints including circular turn points"""
    x: float
    y: float
    dir: int        # Direction: 0=North, 1=East, 2=South, 3=West
    type: str = "move"   # "start", "move", "turn_start", "turn_end"
    turn_center_x: float = 0.0  # Center of circular turn
    turn_center_y: float = 0.0
    
    def get_heading_rad(self) -> float:
        return self.dir * math.pi * 0.5
        
    def get_heading_deg(self) -> float:
        return self.dir * 90.0

@dataclass
class WayPoint:
    """Interpolated points for smooth robot motion"""
    x: float
    y: float
    speed: float
    heading: float  # radians
    time: float = 0.0
    curvature: float = 0.0  # 1/radius for turning
    
    def get_heading_deg(self) -> float:
        return math.degrees(self.heading)
    
    def is_stop(self) -> bool:
        return self.speed < 0.2

@dataclass
class RobotState:
    """Current robot state for simulation"""
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians
    speed: float = 0.0
    time: float = 0.0
    left_wheel_speed: float = 0.0
    right_wheel_speed: float = 0.0

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
        if abs(self.integral) > 10.0:
            self.integral = 10.0 if self.integral > 0 else -10.0
            
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# =============================================================================
# FIXED ROBOT PATH PLANNER V13
# =============================================================================
class RobotPathPlannerV13:
    def __init__(self):
        self.anchors: List[AnchorPoint] = []
        self.waypoints: List[WayPoint] = []
        self.total_distance = 0.0
        self.total_time = 0.0
        self.current_command = ""
        
    def debug_print(self, msg: str):
        if DEBUG_ENABLED:
            print(msg)
    
    def convert_commands_to_anchors_v13(self, commands: str) -> int:
        """V13: FIXED circular turn paths with correct directions"""
        current_x, current_y = 0.0, 0.0
        current_dir = DIR_NORTH
        self.anchors.clear()
        self.current_command = commands
        
        self.debug_print("=== V13: FIXED COMMANDS TO CIRCULAR TURN ANCHORS ===")
        self.debug_print(f"Commands: {commands}")
        self.debug_print(f"Turn radius: {TURN_RADIUS}cm (1 unit)")
        
        # Add starting anchor
        self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "start"))
        self.debug_print(f"START: ({current_x}, {current_y}) facing {DIRECTION_NAMES[current_dir]}")
        
        for i, cmd in enumerate(commands):
            if cmd in ['L', 'R']:
                # Create FIXED circular turn
                self.create_circular_turn_fixed(current_x, current_y, current_dir, cmd)
                
                # Update direction after turn
                if cmd == 'L':
                    current_dir = (current_dir + 3) % 4  # Turn left: -1 = +3 (mod 4)
                    self.debug_print(f"L: Turn left -> {DIRECTION_NAMES[current_dir]}")
                else:
                    current_dir = (current_dir + 1) % 4  # Turn right: +1
                    self.debug_print(f"R: Turn right -> {DIRECTION_NAMES[current_dir]}")
                
                # Update position to end of turn
                current_x, current_y = self.anchors[-1].x, self.anchors[-1].y
                
            elif cmd in ['F', 'S']:
                # Move forward one unit
                dx, dy = DIRECTION_DELTA[current_dir]
                current_x += dx * UNIT_SIZE
                current_y += dy * UNIT_SIZE
                
                self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "move"))
                self.debug_print(f"F: Move to ({current_x}, {current_y})")
                
            elif cmd == 'B':
                # Move backward one unit
                opposite_dir = (current_dir + 2) % 4
                dx, dy = DIRECTION_DELTA[opposite_dir]
                current_x += dx * UNIT_SIZE
                current_y += dy * UNIT_SIZE
                
                self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "move"))
                self.debug_print(f"B: Move to ({current_x}, {current_y})")
        
        self.debug_print(f"Created {len(self.anchors)} anchor points")
        self.debug_print("=" * 50)
        
        return len(self.anchors)
    
    def create_circular_turn_fixed(self, start_x: float, start_y: float, start_dir: int, turn_cmd: str):
        """FIXED: Create circular turn path with correct directions"""
        self.debug_print(f"  Creating FIXED {turn_cmd} turn from ({start_x}, {start_y}) facing {DIRECTION_NAMES[start_dir]}")
        
        if turn_cmd == 'R':  # Right turn - CLOCKWISE
            if start_dir == DIR_NORTH:      # North -> East (CLOCKWISE)
                center_x = start_x + TURN_RADIUS
                center_y = start_y
                end_x = start_x + TURN_RADIUS
                end_y = start_y + TURN_RADIUS  
                end_dir = DIR_EAST
            elif start_dir == DIR_EAST:     # East -> South (CLOCKWISE)
                center_x = start_x
                center_y = start_y - TURN_RADIUS
                end_x = start_x + TURN_RADIUS
                end_y = start_y - TURN_RADIUS
                end_dir = DIR_SOUTH
            elif start_dir == DIR_SOUTH:    # South -> West (CLOCKWISE)
                center_x = start_x - TURN_RADIUS
                center_y = start_y
                end_x = start_x - TURN_RADIUS  
                end_y = start_y - TURN_RADIUS
                end_dir = DIR_WEST
            else:  # DIR_WEST               # West -> North (CLOCKWISE)
                center_x = start_x
                center_y = start_y + TURN_RADIUS
                end_x = start_x - TURN_RADIUS
                end_y = start_y + TURN_RADIUS
                end_dir = DIR_NORTH
                
        else:  # Left turn - COUNTER-CLOCKWISE
            if start_dir == DIR_NORTH:      # North -> West (CCW)
                center_x = start_x - TURN_RADIUS
                center_y = start_y
                end_x = start_x - TURN_RADIUS
                end_y = start_y + TURN_RADIUS
                end_dir = DIR_WEST
            elif start_dir == DIR_EAST:     # East -> North (CCW)
                center_x = start_x
                center_y = start_y + TURN_RADIUS
                end_x = start_x + TURN_RADIUS
                end_y = start_y + TURN_RADIUS
                end_dir = DIR_NORTH
            elif start_dir == DIR_SOUTH:    # South -> East (CCW)
                center_x = start_x + TURN_RADIUS
                center_y = start_y
                end_x = start_x + TURN_RADIUS
                end_y = start_y - TURN_RADIUS
                end_dir = DIR_EAST
            else:  # DIR_WEST              # West -> South (CCW)
                center_x = start_x
                center_y = start_y - TURN_RADIUS
                end_x = start_x - TURN_RADIUS
                end_y = start_y - TURN_RADIUS
                end_dir = DIR_SOUTH
        
        # Add anchors
        self.anchors.append(AnchorPoint(start_x, start_y, start_dir, "turn_start", center_x, center_y))
        self.anchors.append(AnchorPoint(end_x, end_y, end_dir, "turn_end", center_x, center_y))
        
        self.debug_print(f"    Turn center: ({center_x}, {center_y})")
        self.debug_print(f"    Turn end: ({end_x}, {end_y}) facing {DIRECTION_NAMES[end_dir]}")
    
    def interpolate_waypoints_v13(self) -> int:
        """Create waypoints including FIXED circular arc interpolation"""
        self.waypoints.clear()
        current_time = 0.0
        
        self.debug_print("=== V13: FIXED WAYPOINT INTERPOLATION WITH CIRCULAR ARCS ===")
        
        for i in range(len(self.anchors) - 1):
            current = self.anchors[i]
            next_anchor = self.anchors[i + 1]
            
            if current.type == "turn_start" and next_anchor.type == "turn_end":
                # FIXED circular arc interpolation
                self.interpolate_circular_arc_fixed(current, next_anchor, current_time)
                arc_length = 0.25 * 2 * math.pi * TURN_RADIUS  # Quarter circle
                segment_time = arc_length / CURVE_SPEED
                current_time += segment_time
                
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
                    
                    # FIXED heading calculation
                    if distance > 0:
                        heading = math.atan2(end_y - start_y, end_x - start_x)
                    else:
                        heading = current.get_heading_rad()
                        
                    waypoint_time = current_time + t * segment_time
                    
                    self.waypoints.append(WayPoint(wp_x, wp_y, speed, heading, waypoint_time, 0.0))
                
                current_time += segment_time
        
        # Set final waypoint as stop
        if self.waypoints:
            self.waypoints[-1].speed = 0
            
        self.total_time = current_time
        self.total_distance = sum(math.sqrt((self.waypoints[i+1].x - self.waypoints[i].x)**2 + 
                                          (self.waypoints[i+1].y - self.waypoints[i].y)**2) 
                                for i in range(len(self.waypoints)-1))
        
        self.debug_print(f"Generated {len(self.waypoints)} waypoints")
        self.debug_print(f"Total distance: {self.total_distance:.1f}cm")
        self.debug_print(f"Total time: {self.total_time:.1f}s")
        
        return len(self.waypoints)
    
    def interpolate_circular_arc_fixed(self, start_anchor: AnchorPoint, end_anchor: AnchorPoint, start_time: float):
        """FIXED: Interpolate points along circular arc with correct direction"""
        center_x = start_anchor.turn_center_x
        center_y = start_anchor.turn_center_y
        
        # Calculate start and end angles
        start_angle = math.atan2(start_anchor.y - center_y, start_anchor.x - center_x)
        end_angle = math.atan2(end_anchor.y - center_y, end_anchor.x - center_x)
        
        # FIXED: Determine turn direction based on start/end directions
        start_dir = start_anchor.dir
        end_dir = end_anchor.dir
        
        # Calculate the correct angle difference
        if (start_dir + 1) % 4 == end_dir:  # Right turn (clockwise)
            angle_diff = -math.pi/2  # Always -90 degrees for right turns
            if end_angle > start_angle:
                end_angle -= 2 * math.pi
        else:  # Left turn (counter-clockwise)
            angle_diff = math.pi/2   # Always +90 degrees for left turns
            if end_angle < start_angle:
                end_angle += 2 * math.pi
        
        arc_length = abs(angle_diff) * TURN_RADIUS
        segment_time = arc_length / CURVE_SPEED
        num_points = max(8, int(arc_length / INTERPOLATION_STEP))  # More points for smoother curves
        
        self.debug_print(f"  FIXED Arc: center=({center_x:.1f},{center_y:.1f}) start_angle={math.degrees(start_angle):.1f}° end_angle={math.degrees(end_angle):.1f}°")
        self.debug_print(f"  Arc angle_diff: {math.degrees(angle_diff):.1f}° length: {arc_length:.1f}cm, points: {num_points}")
        
        for j in range(num_points + 1):
            t = j / num_points
            angle = start_angle + t * angle_diff
            
            wp_x = center_x + TURN_RADIUS * math.cos(angle)
            wp_y = center_y + TURN_RADIUS * math.sin(angle)
            
            # FIXED: Heading is tangent to the circle (perpendicular to radius)
            heading = angle + (math.pi/2 if angle_diff > 0 else -math.pi/2)
            
            curvature = 1.0 / TURN_RADIUS  # Curvature = 1/radius
            waypoint_time = start_time + t * segment_time
            
            self.waypoints.append(WayPoint(wp_x, wp_y, CURVE_SPEED, heading, waypoint_time, curvature))
    
    def process_commands(self, commands: str):
        """Process commands with V13 fixes"""
        self.convert_commands_to_anchors_v13(commands)
        self.interpolate_waypoints_v13()

# =============================================================================
# FIXED ROBOT SIMULATION V13
# =============================================================================
class RobotSimulationV13:
    def __init__(self, planner: RobotPathPlannerV13):
        self.planner = planner
        self.robot_state = RobotState()
        self.target_path = []
        self.actual_path = []
        self.pid_heading = PIDController()
        self.pid_position = PIDController()  # Additional position PID
        self.robot_artists = []
        self.debug_data = []
        
    def calculate_wheel_speeds_fixed(self, target_wp: WayPoint) -> Tuple[float, float]:
        """FIXED: Calculate differential wheel speeds with proper control"""
        
        # Calculate position error
        pos_error_x = target_wp.x - self.robot_state.x
        pos_error_y = target_wp.y - self.robot_state.y
        distance_error = math.sqrt(pos_error_x**2 + pos_error_y**2)
        
        # Calculate heading error
        desired_heading = math.atan2(pos_error_y, pos_error_x)
        heading_error = desired_heading - self.robot_state.heading
        
        # Normalize heading error to [-π, π]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Base speed based on distance to target
        base_speed = min(target_wp.speed, distance_error * 2.0)  # Proportional to distance
        base_speed = max(MIN_WHEEL_SPEED, min(MAX_WHEEL_SPEED, base_speed))
        
        # Calculate turning component
        turn_speed = self.pid_heading.update(heading_error, 0.1) * 5.0  # Scale turning
        
        # Apply differential steering
        left_speed = base_speed - turn_speed
        right_speed = base_speed + turn_speed
        
        # Limit wheel speeds
        left_speed = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left_speed))
        right_speed = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right_speed))
        
        return left_speed, right_speed
    
    def simulate_robot_step_fixed(self, target_wp: WayPoint, dt: float) -> RobotState:
        """FIXED: Simulate one step with proper differential drive kinematics"""
        
        # Calculate desired wheel speeds
        left_speed, right_speed = self.calculate_wheel_speeds_fixed(target_wp)
        
        # Add minimal noise
        left_speed += np.random.normal(0, 0.1)
        right_speed += np.random.normal(0, 0.1)
        
        # FIXED: Proper differential drive kinematics
        # Linear velocity (average of wheel speeds)
        linear_velocity = (left_speed + right_speed) / 2.0
        
        # Angular velocity (difference of wheel speeds / wheelbase)
        angular_velocity = (right_speed - left_speed) / WHEEL_BASE
        
        # Update robot state using proper kinematics
        new_state = RobotState()
        new_state.time = self.robot_state.time + dt
        
        # Update heading first
        new_state.heading = self.robot_state.heading + angular_velocity * dt
        
        # Normalize heading
        while new_state.heading > 2 * math.pi:
            new_state.heading -= 2 * math.pi
        while new_state.heading < 0:
            new_state.heading += 2 * math.pi
            
        # Update position based on current heading and linear velocity
        new_state.x = self.robot_state.x + linear_velocity * math.cos(new_state.heading) * dt
        new_state.y = self.robot_state.y + linear_velocity * math.sin(new_state.heading) * dt
        
        new_state.speed = linear_velocity
        new_state.left_wheel_speed = left_speed
        new_state.right_wheel_speed = right_speed
        
        return new_state
    
    def print_debug_comparison(self):
        """Print detailed comparison of target vs actual"""
        print("\n" + "="*100)
        print("TARGET vs ACTUAL COMPARISON")
        print("="*100)
        print(f"{'Step':<4} {'Target X':<10} {'Target Y':<10} {'Target Hdg':<12} {'Actual X':<10} {'Actual Y':<10} {'Actual Hdg':<12} {'Error X':<8} {'Error Y':<8} {'Error Hdg':<10}")
        print("-"*100)
        
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
            
            print(f"{i:<4} {target.x:<10.2f} {target.y:<10.2f} {target.get_heading_deg():<12.1f} "
                  f"{actual.x:<10.2f} {actual.y:<10.2f} {math.degrees(actual.heading):<12.1f} "
                  f"{error_x:<8.2f} {error_y:<8.2f} {error_hdg:<10.1f}")
        
        print("="*100)
        
        # Print summary statistics
        if self.debug_data:
            errors_x = [data['actual'].x - data['target'].x for data in self.debug_data]
            errors_y = [data['actual'].y - data['target'].y for data in self.debug_data]
            errors_hdg = []
            for data in self.debug_data:
                error_hdg = math.degrees(data['actual'].heading - data['target'].heading)
                while error_hdg > 180:
                    error_hdg -= 360
                while error_hdg < -180:
                    error_hdg += 360
                errors_hdg.append(error_hdg)
            
            print(f"\nSUMMARY STATISTICS:")
            print(f"Average Error X: {np.mean(errors_x):.2f}cm")
            print(f"Average Error Y: {np.mean(errors_y):.2f}cm") 
            print(f"Average Error Heading: {np.mean(errors_hdg):.1f}°")
            print(f"Max Error X: {np.max(np.abs(errors_x)):.2f}cm")
            print(f"Max Error Y: {np.max(np.abs(errors_y)):.2f}cm")
            print(f"Max Error Heading: {np.max(np.abs(errors_hdg)):.1f}°")
    
    def create_comprehensive_plot(self):
        """Create comprehensive plot with all data"""
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
        ax_main.set_title(f'FIXED Robot Path V13 - Command: {self.planner.current_command}')
        
        # Plot anchor points
        for i, anchor in enumerate(self.planner.anchors):
            if anchor.type == 'start':
                ax_main.plot(anchor.x, anchor.y, 'gs', markersize=15, label='Start')
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
        ax_commands.set_title('Command Analysis')
        cmd_text = f"Commands: {self.planner.current_command}\n"
        cmd_text += f"Length: {len(self.planner.current_command)}\n"
        cmd_text += f"Turns: {self.planner.current_command.count('L') + self.planner.current_command.count('R')}\n"
        cmd_text += f"Moves: {self.planner.current_command.count('F') + self.planner.current_command.count('S')}\n"
        cmd_text += f"Unit Size: {UNIT_SIZE}cm\n"
        cmd_text += f"Turn Radius: {TURN_RADIUS}cm"
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
        
        # Waypoint data (first 15)
        ax_waypoints.axis('off')
        ax_waypoints.set_title('Waypoints (First 15)')
        wp_text = "ID   X      Y    Speed Heading  Curv\n"
        wp_text += "-" * 40 + "\n"
        for i, wp in enumerate(self.planner.waypoints[:15]):
            wp_text += f"W{i:2} {wp.x:6.1f} {wp.y:5.1f} {wp.speed:5.1f} {wp.get_heading_deg():7.1f}° {wp.curvature:.3f}\n"
        if len(self.planner.waypoints) > 15:
            wp_text += f"... and {len(self.planner.waypoints)-15} more"
        ax_waypoints.text(0.05, 0.95, wp_text, transform=ax_waypoints.transAxes, 
                         verticalalignment='top', fontfamily='monospace', fontsize=8)
        
        # Simulation info
        ax_simulation.axis('off')
        ax_simulation.set_title('Simulation Data')
        sim_text = f"Total Distance: {self.planner.total_distance:.1f}cm\n"
        sim_text += f"Total Time: {self.planner.total_time:.1f}s\n"
        sim_text += f"Avg Speed: {self.planner.total_distance/self.planner.total_time:.1f}cm/s\n\n"
        sim_text += "Robot Parameters:\n"
        sim_text += f"Wheel Base: {WHEEL_BASE}cm\n"
        sim_text += f"Max Wheel Speed: {MAX_WHEEL_SPEED}cm/s\n"
        sim_text += f"Turn Radius: {TURN_RADIUS}cm\n\n"
        sim_text += "PID Parameters:\n"
        sim_text += f"Kp: {KP}, Ki: {KI}, Kd: {KD}"
        ax_simulation.text(0.05, 0.95, sim_text, transform=ax_simulation.transAxes, 
                          verticalalignment='top', fontfamily='monospace', fontsize=10)
        
        plt.tight_layout()
        return fig, ax_main
    
    def animate_robot_motion(self):
        """Animate FIXED robot following the path"""
        if not self.planner.waypoints:
            return
            
        print("=== ANIMATING FIXED ROBOT MOTION V13 ===")
        
        fig, ax_main = self.create_comprehensive_plot()
        
        self.actual_path = []
        self.debug_data = []
        dt = 0.1  # 100ms time step
        
        # Reset robot state
        if self.planner.waypoints:
            first_wp = self.planner.waypoints[0]
            self.robot_state = RobotState(first_wp.x, first_wp.y, first_wp.heading, 0.0, 0.0)
        
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
            
            # Plot robot heading arrow  
            dx = 4 * math.cos(self.robot_state.heading)
            dy = 4 * math.sin(self.robot_state.heading)
            heading_arrow = ax_main.arrow(self.robot_state.x, self.robot_state.y, dx, dy, 
                                        head_width=2, head_length=2, fc='red', ec='red', linewidth=2)
            self.robot_artists.append(heading_arrow)
            
            # Update title with current state
            title = f'FIXED Robot at W{i}: Pos=({self.robot_state.x:.1f},{self.robot_state.y:.1f}) '
            title += f'Hdg={math.degrees(self.robot_state.heading):.1f}° Speed={self.robot_state.speed:.1f}cm/s '
            title += f'L/R Wheels={self.robot_state.left_wheel_speed:.1f}/{self.robot_state.right_wheel_speed:.1f}cm/s'
            ax_main.set_title(title)
            
            print(f"W{i:2}: Target=({target_wp.x:5.1f},{target_wp.y:5.1f},{target_wp.get_heading_deg():6.1f}°) "
                  f"Actual=({self.robot_state.x:5.1f},{self.robot_state.y:5.1f},{math.degrees(self.robot_state.heading):6.1f}°) "
                  f"Wheels=L{self.robot_state.left_wheel_speed:.1f}/R{self.robot_state.right_wheel_speed:.1f}")
            
            if target_wp.is_stop():
                ax_main.set_title('ROBOT STOPPED - Path Complete')
                print("ROBOT STOPPED at final waypoint")
                break
            
            plt.draw()
            plt.pause(0.1)  # Faster animation
        
        # Print detailed comparison at the end
        self.print_debug_comparison()
        
        print("=== ROBOT ANIMATION COMPLETE ===")
        plt.show(block=True)

# =============================================================================
# MAIN SIMULATION
# =============================================================================
def main():
    test_commands = [
        "R",            # Simple right turn
        "L",            # Simple left turn
        "RRRR",         # Complete circle (should return to origin)
        "FRF",          # Simple 90-degree turn
    ]
    
    print("Robot Path Planner V13 - FIXED Circular Differential Drive")
    print("F = Move 25cm forward")
    print("R = Turn right in 25cm radius arc (CLOCKWISE)")
    print("L = Turn left in 25cm radius arc (COUNTER-CLOCKWISE)")  
    print("RRRR should create a complete circle")
    print("=" * 60)
    
    for cmd_set in test_commands:
        print(f"\n{'='*20} TESTING: {cmd_set} {'='*20}")
        
        planner = RobotPathPlannerV13()
        planner.process_commands(cmd_set)
        
        sim = RobotSimulationV13(planner)
        
        input(f"Press Enter to visualize and simulate FIXED path for '{cmd_set}'...")
        sim.animate_robot_motion()

#if __name__ == "__main__":

main()