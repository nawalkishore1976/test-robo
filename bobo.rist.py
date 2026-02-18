import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, FancyArrowPatch
import time

class DifferentialDriveRobot:
    def __init__(self, wheel_radius=0.03, pulses_per_rotation=600, wheel_base=0.15):
        """
        Initialize robot parameters
        
        Args:
            wheel_radius: Wheel radius in meters (default 3cm = 0.03m)
            pulses_per_rotation: Encoder pulses per wheel rotation (default 600)
            wheel_base: Distance between wheels in meters (default 15cm = 0.15m)
        """
        self.wheel_radius = wheel_radius
        self.pulses_per_rotation = pulses_per_rotation
        self.wheel_base = wheel_base
        self.wheel_circumference = 2 * np.pi * wheel_radius
        
        # Robot state
        self.x = 0.0  # X position
        self.y = 0.0  # Y position  
        self.theta = 0.0  # Heading in radians
        
        # Tracking
        self.path_x = [0.0]
        self.path_y = [0.0]
        self.time_stamps = [0.0]
        self.headings = [0.0]
        
        # Current step info
        self.current_step = 0
        self.step_start_time = 0
        self.current_left_speed = 0
        self.current_right_speed = 0
        
    def pps_to_linear_speed(self, pps):
        """Convert pulses per second to linear wheel speed in m/s"""
        rotations_per_second = pps / self.pulses_per_rotation
        return rotations_per_second * self.wheel_circumference
    
    def update_position(self, left_pps, right_pps, dt):
        """
        Update robot position using differential drive kinematics
        
        Args:
            left_pps: Left wheel speed in pulses per second
            right_pps: Right wheel speed in pulses per second  
            dt: Time step in seconds
        """
        # Convert PPS to linear speeds
        v_left = self.pps_to_linear_speed(left_pps)
        v_right = self.pps_to_linear_speed(right_pps)
        
        # Calculate robot velocities
        v = (v_left + v_right) / 2.0  # Linear velocity
        omega = (v_right - v_left) / self.wheel_base  # Angular velocity
        
        # Update position and heading
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt
        
        # Normalize heading to [-π, π]
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        
        return v, omega
    
    def get_robot_corners(self):
        """Get robot body corners for visualization"""
        robot_length = 0.08  # 8cm
        robot_width = 0.06   # 6cm
        
        # Robot corners in local coordinates
        corners_local = np.array([
            [-robot_length/2, -robot_width/2],
            [robot_length/2, -robot_width/2], 
            [robot_length/2, robot_width/2],
            [-robot_length/2, robot_width/2],
            [-robot_length/2, -robot_width/2]  # Close the shape
        ])
        
        # Rotation matrix
        cos_theta = np.cos(self.theta)
        sin_theta = np.sin(self.theta)
        rotation_matrix = np.array([
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta]
        ])
        
        # Transform to global coordinates
        corners_global = np.dot(corners_local, rotation_matrix.T)
        corners_global[:, 0] += self.x
        corners_global[:, 1] += self.y
        
        return corners_global

class RobotSimulator:
    def __init__(self, robot, steps):
        """
        Initialize the simulator
        
        Args:
            robot: DifferentialDriveRobot instance
            steps: List of [left_pps, right_pps, duration] commands
        """
        self.robot = robot
        self.steps = steps
        self.dt = 0.05  # 50ms update rate
        
        # Simulation state
        self.current_step = 0
        self.step_time = 0
        self.simulation_time = 0
        self.is_running = True
        
        # Setup the plot
        self.fig, (self.ax_main, self.ax_info) = plt.subplots(1, 2, figsize=(15, 8))
        
        # Main plot setup
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_title('Robot Movement Simulation', fontsize=14)
        self.ax_main.set_xlabel('X Position (m)')
        self.ax_main.set_ylabel('Y Position (m)')
        
        # Info plot setup  
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.05, 0.95, '', transform=self.ax_info.transAxes,
                                         fontsize=12, verticalalignment='top',
                                         fontfamily='monospace')
        
        # Plot elements
        self.path_line, = self.ax_main.plot([], [], 'b-', linewidth=2, alpha=0.7, label='Path')
        self.robot_body, = self.ax_main.plot([], [], 'r-', linewidth=3, label='Robot')
        self.robot_center = Circle((0, 0), 0.01, color='red', zorder=5)
        self.ax_main.add_patch(self.robot_center)
        
        # Heading arrow
        self.heading_arrow = FancyArrowPatch((0, 0), (0, 0), 
                                           arrowstyle='->', mutation_scale=20,
                                           color='green', linewidth=2, zorder=4)
        self.ax_main.add_patch(self.heading_arrow)
        
        self.ax_main.legend()
        
        print("🤖 Robot Simulator Started!")
        print("Commands:")
        for i, step in enumerate(steps):
            print(f"  Step {i+1}: Left={step[0]:+4.0f}PPS, Right={step[1]:+4.0f}PPS, Time={step[2]:.1f}s")
    
    def update_simulation(self, frame):
        """Animation update function"""
        if not self.is_running:
            return
            
        # Check if we need to move to next step
        if self.current_step < len(self.steps):
            current_command = self.steps[self.current_step]
            left_pps, right_pps, duration = current_command
            
            if self.step_time < duration:
                # Execute current step
                v, omega = self.robot.update_position(left_pps, right_pps, self.dt)
                
                # Record position
                self.robot.path_x.append(self.robot.x)
                self.robot.path_y.append(self.robot.y)
                self.robot.time_stamps.append(self.simulation_time)
                self.robot.headings.append(self.robot.theta)
                
                self.step_time += self.dt
                self.simulation_time += self.dt
                
                # Update current speeds for display
                self.robot.current_left_speed = left_pps
                self.robot.current_right_speed = right_pps
                
            else:
                # Move to next step
                self.current_step += 1
                self.step_time = 0
                if self.current_step >= len(self.steps):
                    self.robot.current_left_speed = 0
                    self.robot.current_right_speed = 0
                    print("✅ Simulation Complete!")
        
        # Update visualization
        self.update_plot()
    
    def update_plot(self):
        """Update the plot with current robot state"""
        # Update path
        self.path_line.set_data(self.robot.path_x, self.robot.path_y)
        
        # Update robot body
        corners = self.robot.get_robot_corners()
        self.robot_body.set_data(corners[:, 0], corners[:, 1])
        
        # Update robot center
        self.robot_center.center = (self.robot.x, self.robot.y)
        
        # Update heading arrow
        arrow_length = 0.05
        end_x = self.robot.x + arrow_length * np.cos(self.robot.theta)
        end_y = self.robot.y + arrow_length * np.sin(self.robot.theta)
        self.heading_arrow.set_positions((self.robot.x, self.robot.y), (end_x, end_y))
        
        # Update axis limits to follow robot
        if len(self.robot.path_x) > 1:
            margin = 0.2
            min_x = min(self.robot.path_x) - margin
            max_x = max(self.robot.path_x) + margin
            min_y = min(self.robot.path_y) - margin  
            max_y = max(self.robot.path_y) + margin
            self.ax_main.set_xlim(min_x, max_x)
            self.ax_main.set_ylim(min_y, max_y)
        
        # Update info display
        info_str = self.get_info_string()
        self.info_text.set_text(info_str)
    
    def get_info_string(self):
        """Generate information display string"""
        v_left = self.robot.pps_to_linear_speed(self.robot.current_left_speed)
        v_right = self.robot.pps_to_linear_speed(self.robot.current_right_speed)
        v_robot = (v_left + v_right) / 2.0
        omega_robot = (v_right - v_left) / self.robot.wheel_base
        
        info = f"""🤖 ROBOT STATUS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📍 Position:
   X: {self.robot.x:+7.3f} m
   Y: {self.robot.y:+7.3f} m
   
🧭 Heading: {np.degrees(self.robot.theta):+6.1f}°

⚙️  Motor Speeds:
   Left:  {self.robot.current_left_speed:+5.0f} PPS ({v_left:+5.3f} m/s)
   Right: {self.robot.current_right_speed:+5.0f} PPS ({v_right:+5.3f} m/s)

🚀 Robot Motion:
   Linear:  {v_robot:+6.3f} m/s
   Angular: {np.degrees(omega_robot):+6.1f} °/s

⏱️  Step: {self.current_step + 1}/{len(self.steps)}
   Time: {self.simulation_time:6.2f} s
   Step Time: {self.step_time:5.2f} s
   
📊 Distance Traveled: {self.calculate_distance():.3f} m"""
        
        return info
    
    def calculate_distance(self):
        """Calculate total distance traveled"""
        if len(self.robot.path_x) < 2:
            return 0.0
            
        distances = []
        for i in range(1, len(self.robot.path_x)):
            dx = self.robot.path_x[i] - self.robot.path_x[i-1]
            dy = self.robot.path_y[i] - self.robot.path_y[i-1]
            distances.append(np.sqrt(dx*dx + dy*dy))
        
        return sum(distances)
    
    def run(self):
        """Start the simulation"""
        # Create animation
        anim = animation.FuncAnimation(self.fig, self.update_simulation, 
                                     interval=int(self.dt*1000), blit=False)
        plt.tight_layout()
        plt.show()
        return anim

# Example usage and test program
if __name__ == "__main__":
    # Robot parameters
    wheel_radius = 0.03  # 3cm
    pulses_per_rotation = 600
    wheel_base = 0.15  # 15cm between wheels
    
    # Create robot
    robot = DifferentialDriveRobot(wheel_radius, pulses_per_rotation, wheel_base)
    
    # Define movement steps: [left_pps, right_pps, time_seconds]
    movement_steps = [
        [150, 120, 5.0],    # Forward 2 seconds
        [120, 150, 5.0],    # Forward 2 seconds
        [-120, -150, 5.0],    # Forward 2 seconds
        [150, -150, 15.0],    # Forward 2 seconds
        
        [0, 0, 1.0],
        [300, -300, 1.2],   # Turn right 1 second
        [0, 0, 1.0]  ,
        [300, 300, 0.1],    # Forward 2 seconds
        [0, 0, 1.0],
        [300, -300, 1.2],   # Turn right 1 second  
        [300, 300, 0.1],    # Forward 2 seconds
      
        [300, -300, 1.2],   # Turn right 1 second  
        [300, -300, 1.2],   # Turn right 1 second  
        
        [300, 300, 1.5],    # Forward 1.5 seconds
        [-300, 300, 1.0],   # Turn left 1 second
        [300, 300, 1.0],    # Forward 1 second
        [0, 0, 0.5],        # Stop 0.5 second
        [400, 200, 2.0],    # Curve right 2 seconds
        [-300, -300, 1.0],  # Backward 1 second
        [0, 0, 1.0]         # Final stop
    ]
    print("333")
    # Create and run simulation
    simulator = RobotSimulator(robot, movement_steps)
    animation_obj = simulator.run()
