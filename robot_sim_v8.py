import math
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import time


# =============================================================================
# CONFIGURATION & CONSTANTS
# =============================================================================
UNIT_SIZE = 5.0          # 5cm grid units
DEFAULT_SPEED = 25.0     # cm/s
TURN_RADIUS = 8.0        # cm - radius for smooth turns
INTERPOLATION_STEP = 2.0 # cm between interpolated points
DEBUG_ENABLED = True

# Direction constants
DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST = 0, 1, 2, 3
DIRECTION_NAMES = ["NORTH", "EAST", "SOUTH", "WEST"]

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
    # " ""Discrete grid positions from commands " " "
    x: float
    y: float
    dir: int        # Direction: 0=North, 1=East, 2=South, 3=West
    type: int = 0   # 0=move, 1=turn point
    
    def get_heading(self) -> float:
        return self.dir * math.pi * 0.5

@dataclass
class WayPoint:
    # " " "Interpolated points for continuous path " " "
    x: float
    y: float
    speed: float
    
    def is_stop(self) -> bool:
        return self.speed < 0.2

@dataclass
class PathSegment:
    #" " " Path segment for continuous motion " " "
    type: int       # 0=continuous path, 1=stop
    count: int      # number of waypoints
    offset: int     # offset in waypoints array
    data: int = 0   # reserved

# =============================================================================
# ROBOT PATH PLANNER CLASS
# =============================================================================
class RobotPathPlanner:
    def __init__(self):
        self.anchors: List[AnchorPoint] = []
        self.waypoints: List[WayPoint] = []
        self.segments: List[PathSegment] = []
        
    def debug_print(self, msg: str):
        if DEBUG_ENABLED:
            print(msg)
    
    def convert_commands_to_anchors(self, commands: str) -> int:
        #" ""Step 1: Convert command string to anchor points"" "
        current_x, current_y = 0.0, 0.0
        current_dir = DIR_NORTH
        self.anchors.clear()
        
        self.debug_print("=== STEP 1: COMMANDS TO ANCHOR POINTS ===")
        self.debug_print(f"Commands: {commands}")
        
        # Add starting anchor
        self.anchors.append(AnchorPoint(current_x, current_y, current_dir, 0))
        
        for i, cmd in enumerate(commands):
            if cmd == 'L':  # Turn left
                current_dir = (current_dir + 3) % 4
                # Mark current position as turn point
                if self.anchors:
                    self.anchors[-1].type = 1
                self.debug_print(f"L: Turn left at ({current_x}, {current_y}) -> {DIRECTION_NAMES[current_dir]}")
                
            elif cmd == 'R':  # Turn right
                current_dir = (current_dir + 1) % 4
                # Mark current position as turn point
                if self.anchors:
                    self.anchors[-1].type = 1
                self.debug_print(f"R: Turn right at ({current_x}, {current_y}) -> {DIRECTION_NAMES[current_dir]}")
                
            elif cmd in ['F', 'S']:  # Forward
                dx, dy = DIRECTION_DELTA[current_dir]
                current_x += dx * UNIT_SIZE
                current_y += dy * UNIT_SIZE
                
                self.anchors.append(AnchorPoint(current_x, current_y, current_dir, 0))
                self.debug_print(f"F: Move to ({current_x}, {current_y})")
                
            elif cmd == 'B':  # Backward
                opposite_dir = (current_dir + 2) % 4
                dx, dy = DIRECTION_DELTA[opposite_dir]
                current_x += dx * UNIT_SIZE
                current_y += dy * UNIT_SIZE
                
                self.anchors.append(AnchorPoint(current_x, current_y, current_dir, 0))
                self.debug_print(f"B: Move to ({current_x}, {current_y})")
        
        self.debug_print(f"Created {len(self.anchors)} anchor points")
        self.debug_print("=========================================")
        
        return len(self.anchors)
    
    def interpolate_anchors_to_waypoints(self) -> int:
        #" ""Step 2: Interpolate anchor points to smooth waypoints" ""
        self.waypoints.clear()
        
        self.debug_print("=== STEP 2: ANCHOR INTERPOLATION ===")
        
        for i in range(len(self.anchors) - 1):
            current = self.anchors[i]
            next_anchor = self.anchors[i + 1]
            
            self.debug_print(f"Interpolating from A{i} to A{i+1}")
            
            # If current point is a turn point, create smooth curve
            if current.type == 1 and i > 0 and i < len(self.anchors) - 1:
                prev = self.anchors[i - 1]
                
                # Calculate turn angle
                in_angle = math.atan2(current.y - prev.y, current.x - prev.x)
                out_angle = math.atan2(next_anchor.y - current.y, next_anchor.x - current.x)
                turn_angle = out_angle - in_angle
                
                # Normalize turn angle
                while turn_angle > math.pi:
                    turn_angle -= 2 * math.pi
                while turn_angle < -math.pi:
                    turn_angle += 2 * math.pi
                
                self.debug_print(f" [SMOOTH TURN {math.degrees(turn_angle):.1f} deg]")
                
                # Create arc waypoints
                arc_points = max(3, min(10, int(abs(turn_angle) * TURN_RADIUS / INTERPOLATION_STEP)))
                
                for j in range(arc_points + 1):
                    t = j / arc_points
                    angle = in_angle + t * turn_angle
                    
                    # Simple arc calculation
                    arc_x = current.x + (t - 0.5) * TURN_RADIUS * math.cos(angle + math.pi/2)
                    arc_y = current.y + (t - 0.5) * TURN_RADIUS * math.sin(angle + math.pi/2)
                    
                    speed = DEFAULT_SPEED * (1.0 - 0.3 * abs(turn_angle) / math.pi)
                    self.waypoints.append(WayPoint(arc_x, arc_y, speed))
            else:
                # Straight line interpolation
                dx = next_anchor.x - current.x
                dy = next_anchor.y - current.y
                distance = math.sqrt(dx * dx + dy * dy)
                
                steps = max(1, int(distance / INTERPOLATION_STEP))
                
                self.debug_print(f" [STRAIGHT {distance:.1f}cm, {steps} steps]")
                
                for j in range(steps + 1):
                    t = j / steps
                    x = current.x + t * dx
                    y = current.y + t * dy
                    
                    self.waypoints.append(WayPoint(x, y, DEFAULT_SPEED))
        
        # Make final waypoint a stop
        if self.waypoints:
            self.waypoints[-1].speed = 0
        
        self.debug_print(f"Generated {len(self.waypoints)} smooth waypoints")
        self.debug_print("===================================")
        
        return len(self.waypoints)
    
    def generate_continuous_segments(self) -> int:
        #" ""Step 3: Create continuous path segments"" "
        self.segments.clear()
        
        self.debug_print("=== STEP 3: CONTINUOUS PATH SEGMENTS ===")
        
        # Create one continuous segment for all waypoints
        self.segments.append(PathSegment(0, len(self.waypoints), 0, 0))
        
        self.debug_print(f"Created 1 continuous segment with {len(self.waypoints)} waypoints")
        self.debug_print("========================================")
        
        return len(self.segments)
    
    def print_anchors(self):
        #" ""Print anchor points"" "
        print("=== ANCHOR POINTS ===")
        for i, anchor in enumerate(self.anchors):
            turn_str = " [TURN]" if anchor.type == 1 else ""
            print(f"A{i}: ({anchor.x:.1f}, {anchor.y:.1f}) dir={DIRECTION_NAMES[anchor.dir]}{turn_str}")
        print("====================")
    
    def print_waypoints(self):
        # Print waypoints# 
        print("=== SMOOTH WAYPOINTS ===")
        for i, wp in enumerate(self.waypoints):
            stop_str = " [STOP]" if wp.is_stop() else ""
            print(f"W{i}: ({wp.x:.1f}, {wp.y:.1f}) @ {wp.speed:.1f} cm/s{stop_str}")
        print("=======================")
    
    def process_commands(self, commands: str):
        # Process commands through all 3 steps# 
        self.convert_commands_to_anchors(commands)
        self.print_anchors()
        
        self.interpolate_anchors_to_waypoints()
        self.print_waypoints()
        
        self.generate_continuous_segments()

# =============================================================================
# VISUALIZATION CLASS
# =============================================================================
class RobotSimulation:
    def __init__(self, planner: RobotPathPlanner):
        self.planner = planner
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.robot_pos = [0, 0]
        self.robot_trail = []
        
    def plot_path(self):
        # Plot the generated path# 
        self.ax.clear()
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_title('Robot Path Planning V8 - Continuous Smooth Paths')
        
        # Plot anchor points (red squares)
        if self.planner.anchors:
            anchor_x = [a.x for a in self.planner.anchors]
            anchor_y = [a.y for a in self.planner.anchors]
            self.ax.plot(anchor_x, anchor_y, 'rs-', markersize=8, linewidth=1, 
                        alpha=0.7, label='Anchor Points')
            
            # Mark turn points
            turn_x = [a.x for a in self.planner.anchors if a.type == 1]
            turn_y = [a.y for a in self.planner.anchors if a.type == 1]
            if turn_x:
                self.ax.plot(turn_x, turn_y, 'ro', markersize=12, fillstyle='none', 
                            markeredgewidth=2, label='Turn Points')
        
        # Plot smooth waypoints (blue circles connected)
        if self.planner.waypoints:
            wp_x = [wp.x for wp in self.planner.waypoints]
            wp_y = [wp.y for wp in self.planner.waypoints]
            self.ax.plot(wp_x, wp_y, 'b.-', markersize=4, linewidth=2, 
                        alpha=0.8, label='Smooth Path')
            
            # Mark stop point
            if self.planner.waypoints[-1].is_stop():
                self.ax.plot(wp_x[-1], wp_y[-1], 'go', markersize=10, 
                            label='Stop Point')
        
        # Add grid lines at unit intervals
        if self.planner.waypoints:
            max_coord = max(max(wp_x), max(wp_y)) if wp_x and wp_y else UNIT_SIZE
            min_coord = min(min(wp_x), min(wp_y)) if wp_x and wp_y else -UNIT_SIZE
            
            # Grid lines every UNIT_SIZE
            grid_range = range(int(min_coord - UNIT_SIZE), int(max_coord + UNIT_SIZE * 2), int(UNIT_SIZE))
            for coord in grid_range:
                self.ax.axhline(y=coord, color='gray', alpha=0.2, linewidth=0.5)
                self.ax.axvline(x=coord, color='gray', alpha=0.2, linewidth=0.5)
        
        self.ax.legend()
        
    def animate_robot(self, speed_multiplier: float = 5.0):
        # Animate robot following the path# 
        if not self.planner.waypoints:
            return
            
        print("=== ANIMATING ROBOT MOVEMENT ===")
        
        self.robot_trail = [[0, 0]]  # Start at origin
        
        for i, wp in enumerate(self.planner.waypoints):
            self.robot_pos = [wp.x, wp.y]
            self.robot_trail.append([wp.x, wp.y])
            
            # Update plot
            self.plot_path()
            
            # Plot robot trail
            trail_x = [pos[0] for pos in self.robot_trail]
            trail_y = [pos[1] for pos in self.robot_trail]
            self.ax.plot(trail_x, trail_y, 'g-', linewidth=3, alpha=0.6, label='Robot Trail')
            
            # Plot robot position
            self.ax.plot(wp.x, wp.y, 'ro', markersize=15, label='Robot')
            
            # Add direction arrow (simple)
            if i < len(self.planner.waypoints) - 1:
                next_wp = self.planner.waypoints[i + 1]
                dx = next_wp.x - wp.x
                dy = next_wp.y - wp.y
                if abs(dx) > 0.1 or abs(dy) > 0.1:
                    length = math.sqrt(dx*dx + dy*dy)
                    dx, dy = dx/length * 3, dy/length * 3  # 3cm arrow
                    self.ax.arrow(wp.x, wp.y, dx, dy, head_width=1, 
                                head_length=1, fc='red', ec='red')
            
            self.ax.legend()
            
            print(f"Robot at W{i}: ({wp.x:.1f}, {wp.y:.1f}) @ {wp.speed:.1f} cm/s")
            
            if wp.is_stop():
                print("Robot STOPPED at final waypoint")
                self.ax.set_title('Robot Path Complete - STOPPED')
                break
            
            plt.pause(0.5 / speed_multiplier)  # Adjust animation speed
        
        print("=== ROBOT ANIMATION COMPLETE ===")

# =============================================================================
# MAIN SIMULATION
# =============================================================================
def main():
    # Test commands
    test_commands = [
        "FRFRFRF",      # Square path
        "FFFRF",        # L-shape
        "FRFLFRFR",     # Complex path
        "FFFRFFLFFF",   # Rectangle with turn
    ]
    
    print("Robot Tour V8 - Python Simulation")
    print("Commands: L=Left Turn, R=Right Turn, F=Forward, B=Backward")
    print("=" * 50)
    
    for cmd_set in test_commands:
        print(f"\n{'='*20} TESTING: {cmd_set} {'='*20}")
        
        # Create planner and process commands
        planner = RobotPathPlanner()
        planner.process_commands(cmd_set)
        
        # Create simulation and visualize
        sim = RobotSimulation(planner)
        
        # Static plot
        sim.plot_path()
        plt.show(block=False)
        
        # Wait for user input to continue
        input(f"Press Enter to animate robot following path for '{cmd_set}'...")
        
        # Animate robot
        sim.animate_robot(speed_multiplier=2.0)
        
        plt.show(block=False)
        input("Press Enter to continue to next test...")
        plt.close()


print("sss") 
main()
  