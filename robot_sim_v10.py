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
UNIT_SIZE = 5.0          # 5cm grid units
DEFAULT_SPEED = 25.0     # cm/s
CURVE_SPEED = 15.0       # cm/s for curves (slower)
TURN_RADIUS = 8.0        # cm - radius for smooth turns
INTERPOLATION_STEP = 1.0 # cm between interpolated points (smaller for smoother)
DEBUG_ENABLED = True

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
    """Key waypoints with smooth curves between direction changes"""
    x: float
    y: float
    dir: int        # Direction: 0=North, 1=East, 2=South, 3=West
    type: str = "move"   # "start", "move", "curve"
    
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
    time: float = 0.0  # time since start
    
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

# =============================================================================
# ENHANCED ROBOT PATH PLANNER
# =============================================================================
class RobotPathPlannerV10:
    def __init__(self):
        self.anchors: List[AnchorPoint] = []
        self.waypoints: List[WayPoint] = []
        self.total_distance = 0.0
        self.total_time = 0.0
        
    def debug_print(self, msg: str):
        if DEBUG_ENABLED:
            print(msg)
    
    def convert_commands_to_anchors_v10(self, commands: str) -> int:
        """V10: Remove last anchor on turns for smooth curves"""
        current_x, current_y = 0.0, 0.0
        current_dir = DIR_NORTH
        self.anchors.clear()
        
        self.debug_print("=== V10: COMMANDS TO SMOOTH ANCHOR POINTS ===")
        self.debug_print(f"Commands: {commands}")
        self.debug_print("Strategy: Remove last point on turns for smooth curves")
        
        # Add starting anchor
        self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "start"))
        self.debug_print(f"START: ({current_x}, {current_y}) facing {DIRECTION_NAMES[current_dir]}")
        
        i = 0
        while i < len(commands):
            cmd = commands[i]
            
            if cmd in ['L', 'R']:
                # REMOVE last anchor point for smooth turn
                if len(self.anchors) > 1 and self.anchors[-1].type == "move":
                    removed = self.anchors.pop()
                    self.debug_print(f"REMOVED anchor at ({removed.x}, {removed.y}) for smooth turn")
                
                # Process turn
                if cmd == 'L':
                    current_dir = (current_dir + 3) % 4
                    self.debug_print(f"L: Turn left -> {DIRECTION_NAMES[current_dir]}")
                else:
                    current_dir = (current_dir + 1) % 4
                    self.debug_print(f"R: Turn right -> {DIRECTION_NAMES[current_dir]}")
                
                # Look ahead for next move command to create curve endpoint
                j = i + 1
                while j < len(commands) and commands[j] in ['L', 'R']:
                    if commands[j] == 'L':
                        current_dir = (current_dir + 3) % 4
                    else:
                        current_dir = (current_dir + 1) % 4
                    j += 1
                
                # Process the next move command if it exists
                if j < len(commands) and commands[j] in ['F', 'S', 'B']:
                    next_cmd = commands[j]
                    
                    if next_cmd in ['F', 'S']:
                        dx, dy = DIRECTION_DELTA[current_dir]
                        current_x += dx * UNIT_SIZE
                        current_y += dy * UNIT_SIZE
                    elif next_cmd == 'B':
                        opposite_dir = (current_dir + 2) % 4
                        dx, dy = DIRECTION_DELTA[opposite_dir]
                        current_x += dx * UNIT_SIZE
                        current_y += dy * UNIT_SIZE
                    
                    # Add curve endpoint
                    self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "curve"))
                    self.debug_print(f"CURVE: to ({current_x}, {current_y}) after turn sequence")
                    
                    i = j  # Skip the processed move command
                
            elif cmd in ['F', 'S']:
                dx, dy = DIRECTION_DELTA[current_dir]
                current_x += dx * UNIT_SIZE
                current_y += dy * UNIT_SIZE
                
                self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "move"))
                self.debug_print(f"F: Move to ({current_x}, {current_y})")
                
            elif cmd == 'B':
                opposite_dir = (current_dir + 2) % 4
                dx, dy = DIRECTION_DELTA[opposite_dir]
                current_x += dx * UNIT_SIZE
                current_y += dy * UNIT_SIZE
                
                self.anchors.append(AnchorPoint(current_x, current_y, current_dir, "move"))
                self.debug_print(f"B: Move to ({current_x}, {current_y})")
            
            i += 1
        
        self.debug_print(f"Created {len(self.anchors)} smooth anchor points")
        self.debug_print("=" * 50)
        
        return len(self.anchors)
    
    def interpolate_smooth_waypoints(self) -> int:
        """Create smooth waypoints with enhanced curve interpolation"""
        self.waypoints.clear()
        current_time = 0.0
        
        self.debug_print("=== V10: SMOOTH WAYPOINT INTERPOLATION ===")
        
        for i in range(len(self.anchors) - 1):
            current = self.anchors[i]
            next_anchor = self.anchors[i + 1]
            
            start_x, start_y = current.x, current.y
            end_x, end_y = next_anchor.x, next_anchor.y
            
            # Calculate distance
            distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
            
            # Determine if this is a curve segment
            is_curve = (next_anchor.type == "curve" or 
                       (i > 0 and current.type == "curve"))
            
            speed = CURVE_SPEED if is_curve else DEFAULT_SPEED
            segment_time = distance / speed if distance > 0 else 0
            
            # Number of interpolation points
            num_points = max(3, int(distance / INTERPOLATION_STEP))
            
            self.debug_print(f"Segment {i}: ({start_x:.1f},{start_y:.1f}) -> ({end_x:.1f},{end_y:.1f})")
            self.debug_print(f"  Distance: {distance:.1f}cm, Speed: {speed:.1f}cm/s, Time: {segment_time:.2f}s")
            self.debug_print(f"  Type: {'CURVE' if is_curve else 'STRAIGHT'}, Points: {num_points}")
            
            if is_curve and i > 0:
                # Create smooth curve using bezier-like interpolation
                prev_anchor = self.anchors[i-1] if i > 0 else current
                
                # Control points for smooth curve
                mid_x = (start_x + end_x) / 2
                mid_y = (start_y + end_y) / 2
                
                # Create curved path
                for j in range(num_points + 1):
                    t = j / num_points
                    
                    # Smooth curve interpolation (quadratic bezier)
                    curve_x = (1-t)**2 * start_x + 2*(1-t)*t * mid_x + t**2 * end_x
                    curve_y = (1-t)**2 * start_y + 2*(1-t)*t * mid_y + t**2 * end_y
                    
                    # Calculate heading based on curve direction
                    if j < num_points:
                        next_t = (j + 1) / num_points
                        next_x = (1-next_t)**2 * start_x + 2*(1-next_t)*next_t * mid_x + next_t**2 * end_x
                        next_y = (1-next_t)**2 * start_y + 2*(1-next_t)*next_t * mid_y + next_t**2 * end_y
                        heading = math.atan2(next_y - curve_y, next_x - curve_x)
                    else:
                        heading = next_anchor.get_heading_rad()
                    
                    waypoint_time = current_time + t * segment_time
                    
                    self.waypoints.append(WayPoint(curve_x, curve_y, speed, heading, waypoint_time))
            else:
                # Straight line interpolation
                for j in range(num_points + 1):
                    t = j / num_points
                    
                    wp_x = start_x + t * (end_x - start_x)
                    wp_y = start_y + t * (end_y - start_y)
                    heading = math.atan2(end_y - start_y, end_x - start_x)
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
        self.debug_print(f"Total distance: {self.total_distance:.1f}cm")
        self.debug_print(f"Total time: {self.total_time:.1f}s")
        self.debug_print("=" * 50)
        
        return len(self.waypoints)
    
    def process_commands(self, commands: str):
        """Process commands with V10 improvements"""
        self.convert_commands_to_anchors_v10(commands)
        self.interpolate_smooth_waypoints()

# =============================================================================
# ENHANCED ROBOT SIMULATION WITH DETAILED VISUALIZATION
# =============================================================================
class RobotSimulationV10:
    def __init__(self, planner: RobotPathPlannerV10):
        self.planner = planner
        self.robot_state = RobotState()
        self.traveled_path = []
        self.robot_artists = []  # Store robot visual elements for removal
        
    def create_enhanced_plot(self):
        """Create detailed plot with all information"""
        fig, (ax_main, ax_info) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Main path plot
        ax_main.set_aspect('equal')
        ax_main.grid(True, alpha=0.3)
        ax_main.set_xlabel('X (cm)')
        ax_main.set_ylabel('Y (cm)')
        ax_main.set_title('Robot Path V10 - Smooth Curve Turns')
        
        # Plot anchor points
        if self.planner.anchors:
            # Different colors for different anchor types
            for anchor in self.planner.anchors:
                if anchor.type == 'start':
                    ax_main.plot(anchor.x, anchor.y, 'gs', markersize=15, label='Start Point')
                elif anchor.type == 'curve':
                    ax_main.plot(anchor.x, anchor.y, 'ro', markersize=10, label='Curve Points')
                else:
                    ax_main.plot(anchor.x, anchor.y, 'bo', markersize=8, label='Move Points')
        
        # Plot target path with arrows
        if self.planner.waypoints:
            wp_x = [wp.x for wp in self.planner.waypoints]
            wp_y = [wp.y for wp in self.planner.waypoints]
            
            # Target path
            ax_main.plot(wp_x, wp_y, 'b-', linewidth=2, alpha=0.7, label='Target Path')
            
            # Add direction arrows along path
            arrow_step = max(1, len(self.planner.waypoints) // 10)
            for i in range(0, len(self.planner.waypoints)-1, arrow_step):
                wp = self.planner.waypoints[i]
                dx = 2 * math.cos(wp.heading)
                dy = 2 * math.sin(wp.heading)
                ax_main.arrow(wp.x, wp.y, dx, dy, head_width=0.8, head_length=0.8, 
                            fc='blue', ec='blue', alpha=0.6)
        
        # Mark start point clearly
        if self.planner.anchors:
            start = self.planner.anchors[0]
            ax_main.text(start.x + 1, start.y + 1, 'START', fontsize=12, fontweight='bold', color='green')
            
            # Mark end point
            end = self.planner.anchors[-1]
            ax_main.text(end.x + 1, end.y + 1, 'END', fontsize=12, fontweight='bold', color='red')
        
        # Add grid lines
        if self.planner.waypoints:
            wp_x = [wp.x for wp in self.planner.waypoints]
            wp_y = [wp.y for wp in self.planner.waypoints]
            max_coord = max(max(wp_x), max(wp_y)) + 5
            min_coord = min(min(wp_x), min(wp_y)) - 5
            
            grid_range = range(int(min_coord), int(max_coord) + 5, int(UNIT_SIZE))
            for coord in grid_range:
                ax_main.axhline(y=coord, color='gray', alpha=0.2, linewidth=0.5)
                ax_main.axvline(x=coord, color='gray', alpha=0.2, linewidth=0.5)
        
        # Remove duplicate labels
        handles, labels = ax_main.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax_main.legend(by_label.values(), by_label.keys())
        
        # Info panel
        ax_info.axis('off')
        info_text = self.create_info_text()
        ax_info.text(0.05, 0.95, info_text, transform=ax_info.transAxes, 
                    verticalalignment='top', fontfamily='monospace', fontsize=10)
        
        plt.tight_layout()
        return fig, ax_main, ax_info
    
    def create_info_text(self):
        """Create detailed information text"""
        info = []
        info.append("=== ROBOT PATH INFORMATION V10 ===")
        info.append("")
        
        # Anchor points
        info.append("ANCHOR POINTS:")
        for i, anchor in enumerate(self.planner.anchors):
            dir_arrow = DIRECTION_ARROWS[anchor.dir]
            info.append(f"A{i}: ({anchor.x:5.1f}, {anchor.y:5.1f}) {dir_arrow} {anchor.type}")
        
        info.append("")
        info.append("PATH STATISTICS:")
        info.append(f"Total Waypoints: {len(self.planner.waypoints)}")
        info.append(f"Total Distance:  {self.planner.total_distance:.1f} cm")
        info.append(f"Estimated Time:  {self.planner.total_time:.1f} s")
        if self.planner.total_time > 0:
            info.append(f"Average Speed:   {self.planner.total_distance/self.planner.total_time:.1f} cm/s")
        
        info.append("")
        info.append("WAYPOINT DETAILS (First 10):")
        for i, wp in enumerate(self.planner.waypoints[:10]):
            dir_deg = wp.get_heading_deg()
            info.append(f"W{i:2}: ({wp.x:5.1f},{wp.y:5.1f}) {wp.speed:4.1f}cm/s {dir_deg:6.1f}° t={wp.time:.2f}s")
        
        if len(self.planner.waypoints) > 10:
            info.append(f"... and {len(self.planner.waypoints)-10} more waypoints")
        
        return "\n".join(info)
    
    def clear_robot_artists(self):
        """Remove previous robot visual elements"""
        for artist in self.robot_artists:
            try:
                artist.remove()
            except:
                pass
        self.robot_artists.clear()
    
    def animate_robot_motion(self):
        """Animate robot following the path"""
        if not self.planner.waypoints:
            return
            
        print("=== ANIMATING ROBOT MOTION V10 ===")
        
        fig, ax_main, ax_info = self.create_enhanced_plot()
        
        self.traveled_path = []
        
        for i, wp in enumerate(self.planner.waypoints):
            # Update robot state
            self.robot_state.x = wp.x
            self.robot_state.y = wp.y
            self.robot_state.heading = wp.heading
            self.robot_state.speed = wp.speed
            self.robot_state.time = wp.time
            
            self.traveled_path.append((wp.x, wp.y))
            
            # Clear previous robot position
            self.clear_robot_artists()
            
            # Plot traveled path
            if len(self.traveled_path) > 1:
                trav_x = [p[0] for p in self.traveled_path]
                trav_y = [p[1] for p in self.traveled_path]
                traveled_line = ax_main.plot(trav_x, trav_y, 'g-', linewidth=3, alpha=0.8, label='Traveled Path')[0]
                self.robot_artists.append(traveled_line)
            
            # Plot robot position
            robot_marker = ax_main.plot(wp.x, wp.y, 'ro', markersize=12)[0]
            self.robot_artists.append(robot_marker)
            
            # Plot robot heading arrow
            dx = 3 * math.cos(wp.heading)
            dy = 3 * math.sin(wp.heading)
            arrow = ax_main.arrow(wp.x, wp.y, dx, dy, head_width=1.5, head_length=1.5, 
                                fc='red', ec='red', linewidth=2)
            self.robot_artists.append(arrow)
            
            # Update robot info
            ax_main.set_title(f'Robot at W{i}: ({wp.x:.1f},{wp.y:.1f}) Speed:{wp.speed:.1f}cm/s Heading:{wp.get_heading_deg():.1f}° Time:{wp.time:.1f}s')
            
            print(f"Robot W{i:2}: pos=({wp.x:5.1f},{wp.y:5.1f}) spd={wp.speed:4.1f}cm/s hdg={wp.get_heading_deg():6.1f}° t={wp.time:.2f}s")
            
            if wp.is_stop():
                ax_main.set_title('ROBOT STOPPED - Path Complete')
                print("ROBOT STOPPED at final waypoint")
                break
            
            plt.draw()
            plt.pause(0.3)
        
        print("=== ROBOT ANIMATION COMPLETE ===")
        plt.show(block=True)

# =============================================================================
# MAIN SIMULATION
# =============================================================================
def main():
    test_commands = [
        "FRF",          # Simple curve test
        "FRFRF",        # Two curves  
        "FRFRFRF",      # Square with curves
        "FFFRF",        # Long straight then curve
        "FRFLF",        # Multiple direction changes
    ]
    
    print("Robot Path Planner V10 - Smooth Curve Turns")
    print("L/R removes last anchor for smooth curves")
    print("Enhanced visualization with arrows and coordinates")
    print("=" * 60)
    
    for cmd_set in test_commands:
        print(f"\n{'='*20} TESTING: {cmd_set} {'='*20}")
        
        planner = RobotPathPlannerV10()
        planner.process_commands(cmd_set)
        
        sim = RobotSimulationV10(planner)
        
        input(f"Press Enter to visualize and animate path for '{cmd_set}'...")
        sim.animate_robot_motion()

if __name__ == "__main__":
    main()
