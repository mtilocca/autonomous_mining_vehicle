"""
Vehicle Path Following Simulation
Simulates a 3D vehicle traversing the planned path using Stanley controller
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import math

# Import existing implementations
from slopedTerrainModel import TerrainMap
from A_star import a_star_search
from dijkstra import Dijkstra
from vehicle_dynamics import BicycleModel3D
from low_level_control import StanleyLateralController


class VehiclePathFollowingSimulator:
    """Simulates vehicle following a planned path"""
    
    def __init__(self, terrain_map, path, vehicle_params=None):
        """
        Initialize the simulator
        
        :param terrain_map: TerrainMap object
        :param path: List of (x, y) waypoints
        :param vehicle_params: Dict with vehicle parameters
        """
        self.terrain_map = terrain_map
        self.path = path
        
        # Default vehicle parameters
        if vehicle_params is None:
            vehicle_params = {
                'length': 2.5,
                'velocity': 3.0,
                'controller_gain': 0.8
            }
        
        # Initialize vehicle at start of path
        start = path[0]
        start_z = terrain_map.terrain[int(start[0]), int(start[1])]
        
        self.vehicle = BicycleModel3D(
            length=vehicle_params['length'],
            velocity=vehicle_params['velocity'],
            x=float(start[0]),
            y=float(start[1]),
            z=start_z
        )
        
        # Initialize Stanley controller
        self.controller = StanleyLateralController(
            k_p=vehicle_params['controller_gain'],
            wheelbase_length=vehicle_params['length']
        )
        
        # Simulation parameters
        self.dt = 0.1  # Time step (seconds)
        self.target_velocity = vehicle_params['velocity']
        self.waypoint_threshold = 2.0  # Distance to consider waypoint reached
        
        # History tracking
        self.position_history = []
        self.heading_history = []
        self.velocity_history = []
        self.steering_history = []
        self.cross_track_errors = []
        
        # Simulation state
        self.current_waypoint_index = 0
        self.simulation_time = 0.0
        self.completed = False
        
    def calculate_cross_track_error(self):
        """Calculate perpendicular distance to path"""
        if self.current_waypoint_index >= len(self.path) - 1:
            return 0.0
        
        state = self.vehicle.get_state()
        pos = (state['x'], state['y'])
        
        # Get current path segment
        p1 = self.path[self.current_waypoint_index]
        p2 = self.path[self.current_waypoint_index + 1]
        
        # Vector from p1 to p2
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        if dx == 0 and dy == 0:
            return math.sqrt((pos[0] - p1[0])**2 + (pos[1] - p1[1])**2)
        
        # Parameter for closest point on line segment
        t = max(0, min(1, ((pos[0] - p1[0]) * dx + (pos[1] - p1[1]) * dy) / (dx*dx + dy*dy)))
        
        # Closest point on segment
        closest_x = p1[0] + t * dx
        closest_y = p1[1] + t * dy
        
        # Distance to closest point
        return math.sqrt((pos[0] - closest_x)**2 + (pos[1] - closest_y)**2)
    
    def update_waypoint(self):
        """Check if we should advance to next waypoint"""
        state = self.vehicle.get_state()
        target = self.path[self.current_waypoint_index]
        
        distance = math.sqrt((state['x'] - target[0])**2 + (state['y'] - target[1])**2)
        
        if distance < self.waypoint_threshold:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.path):
                self.completed = True
                self.current_waypoint_index = len(self.path) - 1
                return True
        
        return False
    
    def get_terrain_pitch(self):
        """Calculate terrain pitch at current position"""
        state = self.vehicle.get_state()
        x, y = int(state['x']), int(state['y'])
        
        # Clamp to terrain bounds
        x = max(0, min(self.terrain_map.width - 2, x))
        y = max(0, min(self.terrain_map.length - 2, y))
        
        # Calculate gradient
        dz_dx = self.terrain_map.terrain[x+1, y] - self.terrain_map.terrain[x, y]
        dz_dy = self.terrain_map.terrain[x, y+1] - self.terrain_map.terrain[x, y]
        
        # Calculate pitch based on heading direction
        heading = state['heading']
        pitch = math.atan2(dz_dx * math.cos(heading) + dz_dy * math.sin(heading), 1.0)
        
        return pitch
    
    def step(self):
        """Perform one simulation step"""
        if self.completed:
            return False
        
        # Get current state
        state = self.vehicle.get_state()
        
        # Get upcoming waypoints for controller
        lookahead = min(10, len(self.path) - self.current_waypoint_index)
        waypoints = self.path[self.current_waypoint_index:self.current_waypoint_index + lookahead]
        
        # Compute steering using Stanley controller
        steering_angle = self.controller.compute_steering(
            state['x'], state['y'], state['heading'], waypoints
        )
        
        # Limit steering angle
        max_steering = math.radians(30)  # 30 degrees max
        steering_angle = max(-max_steering, min(max_steering, steering_angle))
        
        # Calculate terrain pitch
        terrain_pitch = self.get_terrain_pitch()
        pitch_rate = (terrain_pitch - state['pitch']) / self.dt
        
        # Update vehicle state
        self.vehicle.update(
            delta=steering_angle,
            velocity=self.target_velocity,
            pitch_rate=pitch_rate,
            roll_rate=0.0,
            dt=self.dt
        )
        
        # Update terrain elevation
        x_idx = max(0, min(self.terrain_map.width-1, int(state['x'])))
        y_idx = max(0, min(self.terrain_map.length-1, int(state['y'])))
        self.vehicle.z = self.terrain_map.terrain[x_idx, y_idx]
        
        # Track history
        new_state = self.vehicle.get_state()
        self.position_history.append((new_state['x'], new_state['y'], new_state['z']))
        self.heading_history.append(new_state['heading'])
        self.velocity_history.append(new_state['velocity'])
        self.steering_history.append(steering_angle)
        self.cross_track_errors.append(self.calculate_cross_track_error())
        
        # Update waypoint
        self.update_waypoint()
        
        # Update simulation time
        self.simulation_time += self.dt
        
        return True
    
    def run_simulation(self, max_steps=5000):
        """Run the complete simulation"""
        print("="*70)
        print("VEHICLE PATH FOLLOWING SIMULATION")
        print("="*70)
        print(f"\nVehicle Parameters:")
        print(f"  Wheelbase: {self.vehicle.length:.2f} m")
        print(f"  Target velocity: {self.target_velocity:.2f} m/s")
        print(f"  Controller gain: {self.controller.k_p:.2f}")
        print(f"\nPath Information:")
        print(f"  Total waypoints: {len(self.path)}")
        print(f"  Start: {self.path[0]}")
        print(f"  Goal: {self.path[-1]}")
        
        print(f"\n🚗 Starting simulation...")
        
        step_count = 0
        while step_count < max_steps and not self.completed:
            self.step()
            step_count += 1
            
            # Progress update every 100 steps
            if step_count % 100 == 0:
                state = self.vehicle.get_state()
                print(f"  Step {step_count}: Position ({state['x']:.1f}, {state['y']:.1f}), "
                      f"Waypoint {self.current_waypoint_index}/{len(self.path)}, "
                      f"Cross-track error: {self.cross_track_errors[-1]:.2f}m")
        
        print(f"\n✅ Simulation completed!")
        print(f"  Total time: {self.simulation_time:.2f} seconds")
        print(f"  Total steps: {step_count}")
        print(f"  Final waypoint: {self.current_waypoint_index}/{len(self.path)}")
        
        return step_count
    
    def visualize_results(self, save_path='vehicle_simulation.png'):
        """Create comprehensive visualization of simulation results"""
        fig = plt.figure(figsize=(20, 12))
        
        # Extract position history
        if self.position_history:
            x_hist, y_hist, z_hist = zip(*self.position_history)
        else:
            return
        
        # 1. 2D Overhead view
        ax1 = fig.add_subplot(2, 3, 1)
        ax1.imshow(self.terrain_map.terrain, cmap='terrain', origin='lower', alpha=0.7)
        
        # Plot planned path
        if self.path:
            path_x, path_y = zip(*self.path)
            ax1.plot(path_y, path_x, 'b--', linewidth=2, label='Planned Path', alpha=0.6)
        
        # Plot vehicle trajectory
        ax1.plot(y_hist, x_hist, 'r-', linewidth=2.5, label='Vehicle Trajectory')
        ax1.plot(y_hist[0], x_hist[0], 'go', markersize=12, label='Start', 
                markeredgecolor='black', markeredgewidth=2)
        ax1.plot(y_hist[-1], x_hist[-1], 'r*', markersize=18, label='End',
                markeredgecolor='black', markeredgewidth=2)
        
        ax1.set_title('Vehicle Trajectory - 2D Overhead', fontsize=14, fontweight='bold')
        ax1.set_xlabel('Y Coordinate')
        ax1.set_ylabel('X Coordinate')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. 3D Trajectory
        ax2 = fig.add_subplot(2, 3, 2, projection='3d')
        X, Y = np.meshgrid(range(self.terrain_map.width), range(self.terrain_map.length))
        ax2.plot_surface(X, Y, self.terrain_map.terrain.T, cmap='terrain', alpha=0.3)
        
        ax2.plot(y_hist, x_hist, z_hist, 'r-', linewidth=3, label='Vehicle Path')
        ax2.scatter(y_hist[0], x_hist[0], z_hist[0], c='green', s=100, label='Start')
        ax2.scatter(y_hist[-1], x_hist[-1], z_hist[-1], c='red', s=100, marker='*', label='End')
        
        ax2.set_title('3D Vehicle Trajectory', fontsize=14, fontweight='bold')
        ax2.set_xlabel('Y')
        ax2.set_ylabel('X')
        ax2.set_zlabel('Elevation')
        ax2.legend()
        
        # 3. Cross-track error over time
        ax3 = fig.add_subplot(2, 3, 3)
        time_steps = np.arange(len(self.cross_track_errors)) * self.dt
        ax3.plot(time_steps, self.cross_track_errors, 'b-', linewidth=2)
        ax3.axhline(y=self.waypoint_threshold, color='r', linestyle='--', 
                   label=f'Waypoint threshold ({self.waypoint_threshold}m)')
        ax3.set_title('Cross-Track Error', fontsize=14, fontweight='bold')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Error (m)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. Steering angle over time
        ax4 = fig.add_subplot(2, 3, 4)
        steering_degrees = [math.degrees(s) for s in self.steering_history]
        ax4.plot(time_steps, steering_degrees, 'g-', linewidth=2)
        ax4.set_title('Steering Angle', fontsize=14, fontweight='bold')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Angle (degrees)')
        ax4.grid(True, alpha=0.3)
        
        # 5. Velocity over time
        ax5 = fig.add_subplot(2, 3, 5)
        ax5.plot(time_steps, self.velocity_history, 'purple', linewidth=2)
        ax5.axhline(y=self.target_velocity, color='r', linestyle='--', 
                   label=f'Target velocity ({self.target_velocity} m/s)')
        ax5.set_title('Vehicle Velocity', fontsize=14, fontweight='bold')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Velocity (m/s)')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 6. Performance metrics
        ax6 = fig.add_subplot(2, 3, 6)
        metrics = {
            'Avg Cross-track\nError (m)': np.mean(self.cross_track_errors),
            'Max Cross-track\nError (m)': np.max(self.cross_track_errors),
            'Avg Steering\n(degrees)': np.mean(np.abs(steering_degrees)),
            'Max Steering\n(degrees)': np.max(np.abs(steering_degrees)),
            'Total Time\n(s)': self.simulation_time,
            'Total Distance\n(m)': sum(math.sqrt((x_hist[i+1]-x_hist[i])**2 + (y_hist[i+1]-y_hist[i])**2)
                                      for i in range(len(x_hist)-1))
        }
        
        metric_names = list(metrics.keys())
        metric_values = list(metrics.values())
        
        bars = ax6.bar(range(len(metrics)), metric_values, 
                      color=['skyblue', 'lightcoral', 'lightgreen', 'orange', 'purple', 'pink'],
                      edgecolor='black', linewidth=1.5)
        ax6.set_xticks(range(len(metrics)))
        ax6.set_xticklabels(metric_names, rotation=45, ha='right', fontsize=9)
        ax6.set_title('Performance Metrics', fontsize=14, fontweight='bold')
        ax6.set_ylabel('Value')
        ax6.grid(True, alpha=0.3, axis='y')
        
        # Add value labels
        for bar, value in zip(bars, metric_values):
            height = bar.get_height()
            ax6.text(bar.get_x() + bar.get_width()/2., height,
                    f'{value:.2f}', ha='center', va='bottom', fontsize=9)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"\n📊 Saved visualization to {save_path}")
        plt.close()
        
        # Print summary statistics
        self.print_summary(metrics)
    
    def print_summary(self, metrics):
        """Print simulation summary statistics"""
        print("\n" + "="*70)
        print("SIMULATION PERFORMANCE SUMMARY")
        print("="*70)
        
        print(f"\nPath Following Performance:")
        print(f"  Average cross-track error: {metrics['Avg Cross-track\nError (m)']:.3f} m")
        print(f"  Maximum cross-track error: {metrics['Max Cross-track\nError (m)']:.3f} m")
        print(f"  RMS cross-track error: {np.sqrt(np.mean(np.array(self.cross_track_errors)**2)):.3f} m")
        
        print(f"\nControl Performance:")
        print(f"  Average steering angle: {metrics['Avg Steering\n(degrees)']:.2f}°")
        print(f"  Maximum steering angle: {metrics['Max Steering\n(degrees)']:.2f}°")
        
        print(f"\nTrajectory Metrics:")
        print(f"  Total simulation time: {metrics['Total Time\n(s)']:.2f} s")
        print(f"  Total distance traveled: {metrics['Total Distance\n(m)']:.2f} m")
        print(f"  Average speed: {metrics['Total Distance\n(m)']/metrics['Total Time\n(s)']:.2f} m/s")
        
        print(f"\nWaypoint Progress:")
        print(f"  Waypoints reached: {self.current_waypoint_index}/{len(self.path)}")
        print(f"  Completion: {(self.current_waypoint_index/len(self.path)*100):.1f}%")
        
        print("="*70)


def run_vehicle_simulation(algorithm='astar'):
    """Run complete vehicle simulation with path planning"""
    print("\n🚀 STARTING COMPLETE AUTONOMOUS NAVIGATION DEMONSTRATION")
    print("="*70)
    
    # Step 1: Create terrain
    print("\n📍 Step 1: Creating terrain...")
    np.random.seed(42)
    terrain_map = TerrainMap(width=100, length=100, slope_percentage=0.5)
    terrain_map.add_random_obstacles(120)
    start_point = (10, 10)
    end_point = (85, 85)
    terrain_map.set_points(start_point, end_point)
    print(f"   ✅ Terrain created: {terrain_map.width}x{terrain_map.length} with {terrain_map.slope_percentage}% slope")
    
    # Step 2: Plan path
    print(f"\n📍 Step 2: Planning path using {algorithm.upper()}...")
    
    if algorithm.lower() == 'astar':
        obstacles_set = set()
        for x in range(terrain_map.width):
            for y in range(terrain_map.length):
                if terrain_map.terrain[x, y] == -1:
                    obstacles_set.add((x, y))
        
        path = a_star_search(start_point, end_point, obstacles_set, 
                           (terrain_map.width, terrain_map.length), movement='8way')
    else:  # dijkstra
        dijkstra = Dijkstra(terrain_map.terrain, start_point, end_point, max_elevation_diff=5.0)
        path = dijkstra.plan()
    
    if not path:
        print("   ❌ No path found!")
        return None
    
    path_length = sum(np.hypot(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]) 
                     for i in range(len(path) - 1))
    print(f"   ✅ Path found: {len(path)} waypoints, {path_length:.2f} units")
    
    # Step 3: Simulate vehicle
    print(f"\n📍 Step 3: Simulating vehicle traversing path...")
    vehicle_params = {
        'length': 2.5,
        'velocity': 1.5,  # Reduced speed for better control
        'controller_gain': 0.5  # Reduced gain to minimize oscillations
    }
    
    simulator = VehiclePathFollowingSimulator(terrain_map, path, vehicle_params)
    steps = simulator.run_simulation(max_steps=5000)
    
    # Step 4: Visualize
    print(f"\n📍 Step 4: Generating visualizations...")
    simulator.visualize_results(f'vehicle_simulation_{algorithm}.png')
    
    print("\n✅ COMPLETE AUTONOMOUS NAVIGATION DEMONSTRATION FINISHED!")
    print("="*70)
    
    return simulator


if __name__ == "__main__":
    # Run simulation with A*
    print("\n" + "🔷"*35)
    print("RUNNING SIMULATION WITH A* ALGORITHM")
    print("🔷"*35)
    sim_astar = run_vehicle_simulation(algorithm='astar')
    
    print("\n\nSimulation complete! Check vehicle_simulation_astar.png for results.")