import math
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
from typing import List

# Force matplotlib backend
import matplotlib
matplotlib.use('TkAgg')

# =============================================================================
# SIMPLIFIED VERSION - IMMEDIATE DISPLAY
# =============================================================================

UNIT_SIZE = 25.0
TURN_RADIUS = 25.0
DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST = 0, 1, 2, 3

@dataclass
class Point:
    x: float
    y: float
    type: str = ""

def create_right_turn_path():
    """Create a simple right turn path"""
    points = []
    
    # Start point
    points.append(Point(0, 0, "start"))
    
    # Right turn: North -> East
    # Center at (25, 0), turn from (0,0) to (25, 25)
    center_x, center_y = 25, 0
    
    # Create arc points
    start_angle = math.pi  # 180 degrees (point (0,0) relative to center (25,0))
    end_angle = math.pi/2  # 90 degrees (point (25,25) relative to center (25,0))
    
    # Go clockwise (negative direction)
    num_points = 20
    for i in range(num_points + 1):
        t = i / num_points
        angle = start_angle + t * (end_angle - start_angle)  # -90 degrees total
        
        x = center_x + TURN_RADIUS * math.cos(angle)
        y = center_y + TURN_RADIUS * math.sin(angle)
        points.append(Point(x, y, "turn"))
    
    points.append(Point(25, 25, "end"))
    
    return points

def plot_path():
    """Force display path immediately"""
    print("Creating right turn path...")
    points = create_right_turn_path()
    
    print(f"Created {len(points)} points")
    for i, p in enumerate(points[:5]):  # Show first 5
        print(f"  P{i}: ({p.x:.1f}, {p.y:.1f}) {p.type}")
    
    print("Creating plot...")
    
    # Create plot
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Extract coordinates
    x_coords = [p.x for p in points]
    y_coords = [p.y for p in points]
    
    # Plot path
    ax.plot(x_coords, y_coords, 'b-', linewidth=3, label='Robot Path')
    ax.scatter(x_coords[::4], y_coords[::4], c='red', s=50, alpha=0.7, label='Waypoints')
    
    # Mark start and end
    ax.scatter(x_coords[0], y_coords[0], c='green', s=200, marker='s', label='Start')
    ax.scatter(x_coords[-1], y_coords[-1], c='red', s=200, marker='*', label='End')
    
    # Mark turn center
    ax.scatter(25, 0, c='orange', s=100, marker='x', label='Turn Center')
    
    # Formatting
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title('Robot Right Turn Path (R command)')
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    
    # Set limits
    ax.set_xlim(-10, 35)
    ax.set_ylim(-10, 35)
    
    print("Displaying plot...")
    
    # FORCE DISPLAY - try multiple methods
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.show(block=False)
    plt.draw()
    
    # Keep window open
    print("Plot should be visible now!")
    print("Close the plot window when you're done viewing it.")
    
    # Wait for window to be closed or user input
    try:
        plt.show()  # This will block until window is closed
    except KeyboardInterrupt:
        print("Interrupted")
    
    print("Done!")

if __name__ == "__main__":
    print("Simple Robot Path Visualization Test")
    print("====================================")
    
    # Test basic matplotlib first
    print("Testing basic matplotlib...")
    test_fig, test_ax = plt.subplots()
    test_ax.plot([0, 1, 2], [0, 1, 0], 'r-o')
    test_ax.set_title('Matplotlib Test - If you see this, matplotlib works!')
    plt.show(block=False)
    plt.pause(2)  # Show for 2 seconds
    plt.close(test_fig)
    
    print("Basic test complete. Now showing robot path...")
    
    # Show the robot path
    plot_path()
