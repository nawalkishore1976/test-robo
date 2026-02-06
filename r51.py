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
TURN_RADIUS = 100.0      # cm - radius for 90° turns
MOVE_DISTANCE = 100.0    # cm - distance for F/B commands
INTERPOLATION_POINTS = 5 # Maximum points per turn
GRID_SIZE = 500.0        # cm - 500x500 grid
WHEEL_BASE = 20.0        # cm - distance between wheels

# Motor speeds (you can adjust these)
MOTOR_SPEED_HIGH = 100   # High speed
MOTOR_SPEED_LOW = 50     # Low speed for turns
MOTOR_SPEED_REVERSE = -100 # Reverse speed

# Direction constants (North = 0°)
DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST = 0, 1, 2, 3
DIRECTION_NAMES = ["NORTH", "EAST", "SOUTH", "WEST"]
DIRECTION_ARROWS = ["↑", "→", "↓", "←"]

# Direction deltas [dx, dy] for movement
DIRECTION_DELTA = [
    [0,  1],  # North: Y+
    [1,  0],  # East:  X+
    [0, -1],  # South: Y-
    [-1, 0]   # West:  X-
]

# Direction to radians
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
    """Key waypoints for robot path"""
    x: float
    y: float
    dir: int
    type: str = "move"  # "start", "move", "turn_start", "turn_end"
    turn_center_x: float = 0.0
    turn_center_y: float = 0.0
    left_motor: int = 0   # Left motor speed
    right_motor: int = 0  # Right motor speed
    
    def get_heading_rad(self) -> float:
        return DIRECTION_TO_RADIANS[self.dir]
        
    def get_heading_deg(self) -> float:
        return self.dir * 90.0

@dataclass
class WayPoint:
    """Interpolated points for smooth robot motion"""
    x: float
    y: float
    heading: float  # radians
    left_motor: int = 0
    right_motor: int = 0
    curvature: float = 0.0  # 1/radius for curves, 0 for straight
    
    def get_heading_deg(self) -> float:
        return math.degrees(self.heading)

# =============================================================================
# ROBOT SIMULATOR CLASS
# =============================================================================
class RobotSimulator:
    def __init__(self, start_x: float = 0.0, start_y: float = 0.0):
        self.anchors: List[AnchorPoint] = []
        self.waypoints: List[WayPoint] = []
        self.start_x = start_x
        self.start_y = start_y
        self.current_x = start_x
        self.current_y = start_y
        self.current_dir = DIR_NORTH
        self.commands = ""
        
    def process_commands(self, commands: str):
        """Process command string and generate anchor points"""
        self.commands = commands
        self.anchors.clear()
        self.current_x = self.start_x  # Start at specified position
        self.current_y = self.start_y
        self.current_dir = DIR_NORTH
        
        print(f"Processing commands: {commands}")
        print(f"Starting position: ({self.current_x}, {self.current_y})")
        
        # Add starting anchor
        self.anchors.append(AnchorPoint(
            self.current_x, self.current_y, self.current_dir, 
            "start", 0, 0, 0, 0
        ))
        
        for i, cmd in enumerate(commands):
            if cmd == 'R':
                self.process_right_turn()
            elif cmd == 'L':
                self.process_left_turn()
            elif cmd == 'F':
                self.process_forward()
            elif cmd == 'B':
                self.process_backward()
                
        print(f"Generated {len(self.anchors)} anchor points")
        
    def process_right_turn(self):
        """Process right turn (90° clockwise)"""
        print(f"Right turn from {DIRECTION_NAMES[self.current_dir]}")
        
        # Calculate turn center and end position
        center_x, center_y, end_x, end_y = self.calculate_turn_geometry('R')
        
        # Add turn start anchor
        self.anchors.append(AnchorPoint(
            self.current_x, self.current_y, self.current_dir,
            "turn_start", center_x, center_y, 
            MOTOR_SPEED_HIGH, MOTOR_SPEED_LOW  # Left faster for right turn
        ))
        
        # Update direction and position
        self.current_dir = (self.current_dir + 1) % 4
        self.current_x, self.current_y = end_x, end_y
        
        # Add turn end anchor
        self.anchors.append(AnchorPoint(
            self.current_x, self.current_y, self.current_dir,
            "turn_end", center_x, center_y,
            MOTOR_SPEED_HIGH, MOTOR_SPEED_LOW
        ))
        
    def process_left_turn(self):
        """Process left turn (90° counter-clockwise)"""
        print(f"Left turn from {DIRECTION_NAMES[self.current_dir]}")
        
        # Calculate turn center and end position
        center_x, center_y, end_x, end_y = self.calculate_turn_geometry('L')
        
        # Add turn start anchor
        self.anchors.append(AnchorPoint(
            self.current_x, self.current_y, self.current_dir,
            "turn_start", center_x, center_y,
            MOTOR_SPEED_REVERSE, MOTOR_SPEED_HIGH  # As specified: -100, +100
        ))
        
        # Update direction and position
        self.current_dir = (self.current_dir + 3) % 4  # -1 = +3 (mod 4)
        self.current_x, self.current_y = end_x, end_y
        
        # Add turn end anchor
        self.anchors.append(AnchorPoint(
            self.current_x, self.current_y, self.current_dir,
            "turn_end", center_x, center_y,
            MOTOR_SPEED_REVERSE, MOTOR_SPEED_HIGH
        ))
        
    def process_forward(self):
        """Process forward movement"""
        print(f"Forward from {DIRECTION_NAMES[self.current_dir]}")
        
        # Move in current direction
        dx, dy = DIRECTION_DELTA[self.current_dir]
        self.current_x += dx * MOVE_DISTANCE
        self.current_y += dy * MOVE_DISTANCE
        
        # Add move anchor
        self.anchors.append(AnchorPoint(
            self.current_x, self.current_y, self.current_dir,
            "move", 0, 0,
            MOTOR_SPEED_HIGH, MOTOR_SPEED_HIGH  # Both motors same speed
        ))
        
    def process_backward(self):
        """Process backward movement"""
        print(f"Backward from {DIRECTION_NAMES[self.current_dir]}")
        
        # Move opposite to current direction
        opposite_dir = (self.current_dir + 2) % 4
        dx, dy = DIRECTION_DELTA[opposite_dir]
        self.current_x += dx * MOVE_DISTANCE
        self.current_y += dy * MOVE_DISTANCE
        
        # Add move anchor
        self.anchors.append(AnchorPoint(
            self.current_x, self.current_y, self.current_dir,
            "move", 0, 0,
            MOTOR_SPEED_REVERSE, MOTOR_SPEED_REVERSE  # Both motors reverse
        ))
        
    def calculate_turn_geometry(self, turn_direction: str) -> Tuple[float, float, float, float]:
        """Calculate turn center and end position for 90° turn"""
        if turn_direction == 'R':  # Right turn (clockwise)
            if self.current_dir == DIR_NORTH:
                center_x = self.current_x + TURN_RADIUS
                center_y = self.current_y
                end_x = self.current_x + TURN_RADIUS
                end_y = self.current_y + TURN_RADIUS
            elif self.current_dir == DIR_EAST:
                center_x = self.current_x
                center_y = self.current_y - TURN_RADIUS
                end_x = self.current_x + TURN_RADIUS
                end_y = self.current_y - TURN_RADIUS
            elif self.current_dir == DIR_SOUTH:
                center_x = self.current_x - TURN_RADIUS
                center_y = self.current_y
                end_x = self.current_x - TURN_RADIUS
                end_y = self.current_y - TURN_RADIUS
            else:  # DIR_WEST
                center_x = self.current_x
                center_y = self.current_y + TURN_RADIUS
                end_x = self.current_x - TURN_RADIUS
                end_y = self.current_y + TURN_RADIUS
                
        else:  # Left turn (counter-clockwise)
            if self.current_dir == DIR_NORTH:
                center_x = self.current_x - TURN_RADIUS
                center_y = self.current_y
                end_x = self.current_x - TURN_RADIUS
                end_y = self.current_y + TURN_RADIUS
            elif self.current_dir == DIR_EAST:
                center_x = self.current_x
                center_y = self.current_y + TURN_RADIUS
                end_x = self.current_x + TURN_RADIUS
                end_y = self.current_y + TURN_RADIUS
            elif self.current_dir == DIR_SOUTH:
                center_x = self.current_x + TURN_RADIUS
                center_y = self.current_y
                end_x = self.current_x + TURN_RADIUS
                end_y = self.current_y - TURN_RADIUS
            else:  # DIR_WEST
                center_x = self.current_x
                center_y = self.current_y - TURN_RADIUS
                end_x = self.current_x - TURN_RADIUS
                end_y = self.current_y - TURN_RADIUS
                
        return center_x, center_y, end_x, end_y
        
    def interpolate_waypoints(self):
        """Generate waypoints with interpolation for smooth paths"""
        self.waypoints.clear()
        
        print("Interpolating waypoints...")
        
        for i in range(len(self.anchors) - 1):
            current = self.anchors[i]
            next_anchor = self.anchors[i + 1]
            
            if current.type == "turn_start" and next_anchor.type == "turn_end":
                # Interpolate circular arc
                self.interpolate_turn(current, next_anchor)
            else:
                # Interpolate straight line
                self.interpolate_straight(current, next_anchor)
                
        print(f"Generated {len(self.waypoints)} waypoints")
        
    def interpolate_turn(self, start_anchor: AnchorPoint, end_anchor: AnchorPoint):
        """Interpolate points along circular arc"""
        center_x = start_anchor.turn_center_x
        center_y = start_anchor.turn_center_y
        
        # Calculate start and end angles
        start_angle = math.atan2(start_anchor.y - center_y, start_anchor.x - center_x)
        end_angle = math.atan2(end_anchor.y - center_y, end_anchor.x - center_x)
        
        # Determine angle difference (90° turn)
        if start_anchor.dir == DIR_NORTH and end_anchor.dir == DIR_EAST:  # Right turn
            angle_diff = -math.pi/2
        elif start_anchor.dir == DIR_EAST and end_anchor.dir == DIR_SOUTH:  # Right turn
            angle_diff = -math.pi/2
        elif start_anchor.dir == DIR_SOUTH and end_anchor.dir == DIR_WEST:  # Right turn
            angle_diff = -math.pi/2
        elif start_anchor.dir == DIR_WEST and end_anchor.dir == DIR_NORTH:  # Right turn
            angle_diff = -math.pi/2
        else:  # Left turn
            angle_diff = math.pi/2
            
        # Create interpolated points
        for j in range(INTERPOLATION_POINTS + 1):
            t = j / INTERPOLATION_POINTS
            angle = start_angle + t * angle_diff
            
            x = center_x + TURN_RADIUS * math.cos(angle)
            y = center_y + TURN_RADIUS * math.sin(angle)
            
            # Calculate heading
            heading_start = start_anchor.get_heading_rad()
            heading_end = end_anchor.get_heading_rad()
            
            # Handle angle wrap-around for heading interpolation
            if abs(heading_end - heading_start) > math.pi:
                if heading_end > heading_start:
                    heading_start += 2 * math.pi
                else:
                    heading_end += 2 * math.pi
                    
            heading = heading_start + t * (heading_end - heading_start)
            
            # Normalize heading
            while heading < 0:
                heading += 2 * math.pi
            while heading >= 2 * math.pi:
                heading -= 2 * math.pi
                
            # Calculate curvature (1/radius)
            curvature = 1.0 / TURN_RADIUS if angle_diff < 0 else -1.0 / TURN_RADIUS
            
            waypoint = WayPoint(
                x, y, heading,
                start_anchor.left_motor, start_anchor.right_motor,
                curvature
            )
            self.waypoints.append(waypoint)
            
    def interpolate_straight(self, start_anchor: AnchorPoint, end_anchor: AnchorPoint):
        """Interpolate points along straight line"""
        # For straight segments, just add start and end points
        start_wp = WayPoint(
            start_anchor.x, start_anchor.y, start_anchor.get_heading_rad(),
            start_anchor.left_motor, start_anchor.right_motor, 0.0
        )
        
        end_wp = WayPoint(
            end_anchor.x, end_anchor.y, end_anchor.get_heading_rad(),
            end_anchor.left_motor, end_anchor.right_motor, 0.0
        )
        
        self.waypoints.extend([start_wp, end_wp])
        
    def display_path(self):
        """Display the robot path with grid from (0,0) to (500,500)"""
        fig, ax = plt.subplots(figsize=(12, 12))
        
        # Set up grid from (0,0) to (500,500)
        ax.set_xlim(0, GRID_SIZE)
        ax.set_ylim(0, GRID_SIZE)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_title(f'Robot Path Simulation - Commands: {self.commands}')
        
        # Plot anchor points
        for i, anchor in enumerate(self.anchors):
            if anchor.type == 'start':
                ax.plot(anchor.x, anchor.y, 'go', markersize=12, label='Start')
                # Add North arrow
                ax.arrow(anchor.x, anchor.y, 0, 20, head_width=5, head_length=5, 
                        fc='green', ec='green', alpha=0.8)
                ax.text(anchor.x + 10, anchor.y + 25, 'N (0°)', fontsize=10, 
                       fontweight='bold', color='green')
            elif 'turn' in anchor.type:
                color = 'red' if 'start' in anchor.type else 'orange'
                ax.plot(anchor.x, anchor.y, 'o', color=color, markersize=8, 
                       label='Turn Points' if i == 1 else "")
                # Show turn center
                ax.plot(anchor.turn_center_x, anchor.turn_center_y, 'r+', 
                       markersize=10, label='Turn Centers' if i == 1 else "")
            else:
                ax.plot(anchor.x, anchor.y, 'bo', markersize=8, 
                       label='Move Points' if anchor.type == 'move' else "")
        
        # Plot waypoints path
        if self.waypoints:
            wp_x = [wp.x for wp in self.waypoints]
            wp_y = [wp.y for wp in self.waypoints]
            ax.plot(wp_x, wp_y, 'b-', linewidth=3, alpha=0.7, label='Robot Path')
            
            # Add direction arrows along path
            arrow_step = max(1, len(self.waypoints) // 10)
            for i in range(0, len(self.waypoints), arrow_step):
                wp = self.waypoints[i]
                dx = 15 * math.sin(wp.heading)  # For North=0° system
                dy = 15 * math.cos(wp.heading)
                ax.arrow(wp.x, wp.y, dx, dy, head_width=8, head_length=8, 
                        fc='blue', ec='blue', alpha=0.6)
        
        # Add motor speed annotations
        info_text = "Motor Speeds:\n"
        info_text += "R: Left=100, Right=50\n"
        info_text += "L: Left=-100, Right=100\n" 
        info_text += "F: Left=100, Right=100\n"
        info_text += "B: Left=-100, Right=-100\n"
        info_text += f"\nTurn Radius: {TURN_RADIUS}cm\n"
        info_text += f"Move Distance: {MOVE_DISTANCE}cm\n"
        info_text += f"Grid: (0,0) to ({GRID_SIZE:.0f},{GRID_SIZE:.0f})\n"
        info_text += f"Start: ({self.start_x:.0f},{self.start_y:.0f})"
        
        ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
               verticalalignment='top', bbox=dict(boxstyle="round,pad=0.3", 
               facecolor="lightgray", alpha=0.8), fontfamily='monospace')
        
        # Remove duplicate legend entries
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc='upper right')
        
        plt.tight_layout()
        plt.show()
        
        # Print waypoint data
        self.print_waypoint_data()
        
    def print_waypoint_data(self):
        """Print detailed waypoint information"""
        print("\n" + "="*80)
        print("WAYPOINT DATA")
        print("="*80)
        print(f"{'ID':<3} {'X':<8} {'Y':<8} {'Heading':<8} {'L_Motor':<8} {'R_Motor':<8} {'Curvature':<10}")
        print("-"*80)
        
        for i, wp in enumerate(self.waypoints):
            print(f"{i:<3} {wp.x:<8.1f} {wp.y:<8.1f} {wp.get_heading_deg():<8.1f} "
                  f"{wp.left_motor:<8} {wp.right_motor:<8} {wp.curvature:<10.4f}")
                  
        print("-"*80)
        print(f"Total waypoints: {len(self.waypoints)}")

# =============================================================================
# MAIN FUNCTION
# =============================================================================
def main():
    print("Robot Simulator - Differential Drive with Waypoint Generation")
    print("Commands: R (Right), L (Left), F (Forward), B (Backward)")
    print("Grid: (0,0) to (500,500), Turn Radius: 100cm, Move Distance: 100cm")
    print("Starting Position: (0,0) - Bottom left corner")
    print("="*70)
    
    # Test commands
    test_commands = [
        "R",
        "RF",
        "RFR", 
        "RFRFRF",
        "LFRFR"
    ]
    
    for commands in test_commands:
        print(f"\n{'='*20} Testing: {commands} {'='*20}")
        
        # Create simulator with start point at (0,0)
        simulator = RobotSimulator(start_x=0.0, start_y=0.0)
        simulator.process_commands(commands)
        simulator.interpolate_waypoints()
        
        input(f"Press Enter to display path for '{commands}'...")
        simulator.display_path()
        
        if input("Continue to next test? (y/n): ").lower() != 'y':
            break

# Example of how to use with custom start point
def test_custom_start():
    print("\n" + "="*50)
    print("Testing with custom start point (50, 100)")
    print("="*50)
    
    simulator = RobotSimulator(start_x=50.0, start_y=100.0)
    simulator.process_commands("RFR")
    simulator.interpolate_waypoints()
    simulator.display_path()

if __name__ == "__main__":
    main()
    
    # Uncomment to test custom start point
    # test_custom_start()
