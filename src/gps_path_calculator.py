import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

# Import your existing path planning algorithms
from A_star import a_star_search
from rrt_star import RRTStar

# Import GPS simulation module
from gps_simulation import GPSRTKSimulator, GPSReading, GPSQuality

# Navigation States
@dataclass
class NavigationState:
    current_position: Tuple[float, float, float]
    gps_position: Tuple[float, float, float]
    target_waypoint: Tuple[float, float]
    planned_path: List[Tuple[int, int]]
    path_index: int
    cross_track_error: float
    gps_quality: GPSQuality

class NavigationMode(Enum):
    PLANNING = "planning"
    FOLLOWING = "following"
    REPLANNING = "replanning"
    ARRIVED = "arrived"

class GPSNavigationSystem:
    """
    GPS-integrated navigation system with selectable path planning algorithms
    """
    
    def __init__(self, terrain_map, algorithm='astar'):
        self.terrain_map = terrain_map
        self.algorithm = algorithm.lower()
        self.gps_sim = GPSRTKSimulator(terrain_map)
        
        # Navigation parameters
        self.waypoint_tolerance = 2.0
        self.replan_threshold = 5.0
        self.max_gps_error_threshold = 10.0
        
        # Current state
        self.current_path = []
        self.current_waypoint_index = 0
        self.navigation_mode = NavigationMode.PLANNING
        self.start_point = None
        self.end_point = None
        
        # History tracking
        self.position_history = []
        self.gps_history = []
        self.navigation_states = []
        
    def set_algorithm(self, algorithm: str):
        """Set the path planning algorithm: 'astar' or 'rrt*'"""
        self.algorithm = algorithm.lower()
        print(f"Algorithm set to: {self.algorithm}")
    
    def plan_path(self, start_point: Tuple[int, int], end_point: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """Plan path using selected algorithm"""
        print(f"Planning path using {self.algorithm.upper()} from {start_point} to {end_point}...")
        
        self.start_point = start_point
        self.end_point = end_point
        
        if self.algorithm == 'astar':
            # Convert terrain to obstacles for A*
            obstacles = set()
            for x in range(self.terrain_map.terrain.shape[0]):
                for y in range(self.terrain_map.terrain.shape[1]):
                    if self.terrain_map.terrain[x, y] == -1:
                        obstacles.add((x, y))
            
            # Use your existing A* implementation
            path = a_star_search(
                start=start_point,
                goal=end_point,
                obstacles=obstacles,
                grid_size=self.terrain_map.terrain.shape,
                movement='8way'
            )
            
        elif self.algorithm == 'rrt*':
            # Use your existing RRT* implementation
            rrt_star = RRTStar(
                terrain=self.terrain_map.terrain,
                start=start_point,
                goal=end_point,
                max_elevation_diff=5.0,
                expand_dis=3.0,
                max_iter=500
            )
            path = rrt_star.plan()
            
            # Convert to integer coordinates for consistency
            if path:
                path = [(int(x), int(y)) for x, y in path]
        else:
            print(f"❌ Unknown algorithm: {self.algorithm}")
            return None
        
        if path:
            self.current_path = path
            self.current_waypoint_index = 0
            self.navigation_mode = NavigationMode.FOLLOWING
            print(f"✅ Path found with {len(path)} waypoints")
            return path
        else:
            print("❌ No path found!")
            return None
    
    def calculate_cross_track_error(self, current_pos: Tuple[float, float]) -> float:
        """Calculate perpendicular distance to current path segment"""
        if (self.current_waypoint_index >= len(self.current_path) - 1 or 
            len(self.current_path) < 2):
            return 0.0
        
        p1 = self.current_path[self.current_waypoint_index]
        p2 = self.current_path[self.current_waypoint_index + 1]
        x, y = current_pos
        
        # Vector from p1 to p2
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        if dx == 0 and dy == 0:
            return math.sqrt((x - p1[0])**2 + (y - p1[1])**2)
        
        # Parameter for closest point on line segment
        t = max(0, min(1, ((x - p1[0]) * dx + (y - p1[1]) * dy) / (dx*dx + dy*dy)))
        
        # Closest point on segment
        closest_x = p1[0] + t * dx
        closest_y = p1[1] + t * dy
        
        # Distance to closest point
        return math.sqrt((x - closest_x)**2 + (y - closest_y)**2)
    
    def advance_waypoint(self, current_pos: Tuple[float, float]) -> bool:
        """Check if we should advance to the next waypoint"""
        if self.current_waypoint_index >= len(self.current_path):
            return False
            
        target_waypoint = self.current_path[self.current_waypoint_index]
        distance_to_waypoint = math.sqrt(
            (current_pos[0] - target_waypoint[0])**2 +
            (current_pos[1] - target_waypoint[1])**2
        )
        
        if distance_to_waypoint < self.waypoint_tolerance:
            self.current_waypoint_index += 1
            print(f"✅ Reached waypoint {self.current_waypoint_index-1}")
            
            if self.current_waypoint_index >= len(self.current_path):
                self.navigation_mode = NavigationMode.ARRIVED
                print("🎯 Destination reached!")
                return False
                
            return True
        
        return False
    
    def should_replan(self, nav_state: NavigationState) -> bool:
        """Determine if replanning is needed"""
        # Replan if cross-track error is too large
        if nav_state.cross_track_error > self.replan_threshold:
            print(f"🔄 Replanning due to cross-track error: {nav_state.cross_track_error:.2f}m")
            return True
            
        # Replan if GPS quality is poor and error is high
        if nav_state.gps_quality in [GPSQuality.NO_FIX, GPSQuality.STANDARD_GPS]:
            gps_error = math.sqrt(
                (nav_state.current_position[0] - nav_state.gps_position[0])**2 +
                (nav_state.current_position[1] - nav_state.gps_position[1])**2
            )
            if gps_error > self.max_gps_error_threshold:
                print(f"🔄 Replanning due to GPS error: {gps_error:.2f}m")
                return True
        
        return False
    
    def simulate_navigation(self, vehicle_speed: float = 1.5, time_step: float = 0.5) -> List[NavigationState]:
        """
        Simulate complete navigation with GPS integration
        
        Args:
            vehicle_speed: Speed in terrain units per second
            time_step: Time between navigation updates (seconds)
        """
        if not self.current_path:
            print("❌ No path planned yet!")
            return []
        
        print(f"🚀 Starting GPS-integrated navigation...")
        print(f"Algorithm: {self.algorithm.upper()}")
        print(f"Vehicle speed: {vehicle_speed} units/sec")
        print(f"GPS update rate: {1/time_step:.1f} Hz")
        
        navigation_states = []
        current_true_pos = [float(self.start_point[0]), float(self.start_point[1]), 0.0]
        
        simulation_step = 0
        max_steps = 1000
        
        while (self.navigation_mode != NavigationMode.ARRIVED and 
               simulation_step < max_steps):
            
            # Get terrain elevation
            x_idx = max(0, min(self.terrain_map.width-1, int(current_true_pos[0])))
            y_idx = max(0, min(self.terrain_map.length-1, int(current_true_pos[1])))
            current_true_pos[2] = self.terrain_map.terrain[x_idx, y_idx]
            
            # Get GPS reading
            gps_reading = self.gps_sim.get_gps_reading(tuple(current_true_pos))
            gps_position = (gps_reading.latitude, gps_reading.longitude, gps_reading.elevation)
            
            # Calculate navigation metrics
            target_waypoint = self.current_path[self.current_waypoint_index] if self.current_waypoint_index < len(self.current_path) else self.end_point
            cross_track_error = self.calculate_cross_track_error((gps_position[0], gps_position[1]))
            
            # Create navigation state
            nav_state = NavigationState(
                current_position=tuple(current_true_pos),
                gps_position=gps_position,
                target_waypoint=target_waypoint,
                planned_path=self.current_path.copy(),
                path_index=self.current_waypoint_index,
                cross_track_error=cross_track_error,
                gps_quality=gps_reading.quality
            )
            
            navigation_states.append(nav_state)
            self.position_history.append(tuple(current_true_pos))
            self.gps_history.append(gps_reading)
            
            # Check for replanning
            if self.should_replan(nav_state):
                self.navigation_mode = NavigationMode.REPLANNING
                
                # Replan from current GPS position
                current_grid_pos = (int(gps_position[0]), int(gps_position[1]))
                if self.plan_path(current_grid_pos, self.end_point):
                    self.navigation_mode = NavigationMode.FOLLOWING
                    print("✅ Replanning successful")
                else:
                    print("❌ Replanning failed, continuing with original path")
                    self.navigation_mode = NavigationMode.FOLLOWING
            
            # Advance waypoint if close enough
            self.advance_waypoint((gps_position[0], gps_position[1]))
            
            # Move vehicle towards target waypoint
            if self.current_waypoint_index < len(self.current_path):
                target = self.current_path[self.current_waypoint_index]
                
                # Calculate direction to target
                dx = target[0] - current_true_pos[0]
                dy = target[1] - current_true_pos[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance > 0:
                    # Move towards target
                    move_distance = min(vehicle_speed * time_step, distance)
                    current_true_pos[0] += (dx / distance) * move_distance
                    current_true_pos[1] += (dy / distance) * move_distance
            
            simulation_step += 1
            
            # Print progress
            if simulation_step % 20 == 0:
                print(f"Step {simulation_step}: GPS Quality={nav_state.gps_quality.name}, "
                      f"Cross-track={cross_track_error:.2f}m, "
                      f"Waypoint={self.current_waypoint_index}/{len(self.current_path)}")
        
        if simulation_step >= max_steps:
            print("⚠️ Simulation stopped due to step limit")
        
        print(f"✅ Navigation completed in {simulation_step} steps")
        return navigation_states
    
    def visualize_results(self, navigation_states: List[NavigationState]):
        """Visualize complete navigation results with multiple views"""
        if not navigation_states:
            print("No navigation data to visualize")
            return
            
        fig = plt.figure(figsize=(20, 12))
        
        # 2D overhead view
        ax1 = fig.add_subplot(231)
        ax1.imshow(self.terrain_map.terrain, cmap='terrain', origin='lower', alpha=0.7)
        
        # Plot planned path
        if self.current_path:
            path_x, path_y = zip(*self.current_path)
            ax1.plot(path_y, path_x, 'b--', linewidth=2, label='Planned Path', alpha=0.8)
        
        # Plot true vs GPS trajectories
        true_positions = [state.current_position for state in navigation_states]
        gps_positions = [state.gps_position for state in navigation_states]
        
        if true_positions:
            true_x, true_y, _ = zip(*true_positions)
            ax1.plot(true_y, true_x, 'g-', linewidth=3, label='True Path')
        
        if gps_positions:
            gps_x, gps_y, _ = zip(*gps_positions)
            ax1.plot(gps_y, gps_x, 'r:', linewidth=2, label='GPS Path')
        
        # Mark start and end
        if self.start_point and self.end_point:
            ax1.plot(self.start_point[1], self.start_point[0], 'go', markersize=10, label='Start')
            ax1.plot(self.end_point[1], self.end_point[0], 'ro', markersize=10, label='End')
        
        ax1.legend()
        ax1.set_title(f'Navigation Overview - {self.algorithm.upper()}')
        ax1.set_xlabel('Y Coordinate')
        ax1.set_ylabel('X Coordinate')
        
        # Cross-track error over time
        ax2 = fig.add_subplot(232)
        cross_track_errors = [state.cross_track_error for state in navigation_states]
        time_steps = range(len(cross_track_errors))
        ax2.plot(time_steps, cross_track_errors, 'b-', linewidth=2)
        ax2.axhline(y=self.replan_threshold, color='r', linestyle='--', 
                   label=f'Replan Threshold ({self.replan_threshold}m)')
        ax2.set_ylabel('Cross-track Error (m)')
        ax2.set_xlabel('Time Step')
        ax2.set_title('Cross-track Error Over Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # GPS quality timeline
        ax3 = fig.add_subplot(233)
        gps_qualities = [state.gps_quality.value for state in navigation_states]
        ax3.plot(time_steps, gps_qualities, 'g-', marker='o', markersize=3)
        ax3.set_ylabel('GPS Quality Level')
        ax3.set_xlabel('Time Step')
        ax3.set_title('GPS Quality Over Time')
        ax3.grid(True, alpha=0.3)
        
        # Position error distribution
        ax4 = fig.add_subplot(234)
        position_errors = []
        for state in navigation_states:
            error = math.sqrt(
                (state.current_position[0] - state.gps_position[0])**2 +
                (state.current_position[1] - state.gps_position[1])**2
            )
            position_errors.append(error)
        
        ax4.hist(position_errors, bins=20, alpha=0.7, edgecolor='black', color='skyblue')
        ax4.axvline(np.mean(position_errors), color='red', linestyle='--', 
                   label=f'Mean: {np.mean(position_errors):.2f}m')
        ax4.set_xlabel('Position Error (m)')
        ax4.set_ylabel('Count')
        ax4.set_title('GPS Position Error Distribution')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 3D trajectory view
        ax5 = fig.add_subplot(235, projection='3d')
        X, Y = np.meshgrid(range(self.terrain_map.width), range(self.terrain_map.length))
        ax5.plot_surface(X, Y, self.terrain_map.terrain, cmap='terrain', alpha=0.3)
        
        if true_positions:
            true_x, true_y, true_z = zip(*true_positions)
            ax5.plot(true_x, true_y, true_z, 'g-', linewidth=3, label='True Path')
        
        if gps_positions:
            gps_x, gps_y, gps_z = zip(*gps_positions)
            ax5.plot(gps_x, gps_y, gps_z, 'r:', linewidth=2, label='GPS Path')
        
        ax5.set_title('3D Navigation Trajectory')
        ax5.legend()
        
        # Algorithm comparison metrics
        ax6 = fig.add_subplot(236)
        
        # Calculate key metrics
        total_path_length = 0
        if self.current_path and len(self.current_path) > 1:
            for i in range(len(self.current_path) - 1):
                dx = self.current_path[i+1][0] - self.current_path[i][0]
                dy = self.current_path[i+1][1] - self.current_path[i][1]
                total_path_length += math.sqrt(dx*dx + dy*dy)
        
        metrics = {
            'Path Length': total_path_length,
            'Mean GPS Error': np.mean(position_errors),
            'Mean Cross-track': np.mean(cross_track_errors),
            'Max Cross-track': np.max(cross_track_errors),
            'Navigation Steps': len(navigation_states)
        }
        
        metric_names = list(metrics.keys())
        metric_values = list(metrics.values())
        
        bars = ax6.bar(metric_names, metric_values, color=['skyblue', 'lightgreen', 'orange', 'red', 'purple'])
        ax6.set_title(f'Navigation Metrics - {self.algorithm.upper()}')
        ax6.set_ylabel('Value')
        
        # Add value labels on bars
        for bar, value in zip(bars, metric_values):
            height = bar.get_height()
            ax6.text(bar.get_x() + bar.get_width()/2., height + height*0.01,
                    f'{value:.2f}', ha='center', va='bottom')
        
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()
        
        # Print detailed statistics
        self.print_navigation_summary(navigation_states, position_errors, cross_track_errors)
    
    def print_navigation_summary(self, navigation_states, position_errors, cross_track_errors):
        """Print comprehensive navigation performance summary"""
        print("\n" + "="*70)
        print(f"NAVIGATION PERFORMANCE SUMMARY - {self.algorithm.upper()}")
        print("="*70)
        
        print(f"Algorithm Used: {self.algorithm.upper()}")
        print(f"Total Navigation Steps: {len(navigation_states)}")
        print(f"Final Status: {self.navigation_mode.value}")
        
        print(f"\nPath Planning Metrics:")
        if self.current_path:
            total_length = sum(
                math.sqrt((self.current_path[i+1][0] - self.current_path[i][0])**2 + 
                         (self.current_path[i+1][1] - self.current_path[i][1])**2)
                for i in range(len(self.current_path) - 1)
            )
            print(f"  Planned path length: {total_length:.2f} units")
            print(f"  Total waypoints: {len(self.current_path)}")
        
        print(f"\nGPS Performance:")
        print(f"  Mean position error: {np.mean(position_errors):.3f} m")
        print(f"  RMS position error: {np.sqrt(np.mean(np.array(position_errors)**2)):.3f} m")
        print(f"  Max position error: {np.max(position_errors):.3f} m")
        print(f"  95th percentile error: {np.percentile(position_errors, 95):.3f} m")
        
        print(f"\nNavigation Performance:")
        print(f"  Mean cross-track error: {np.mean(cross_track_errors):.3f} m")
        print(f"  Max cross-track error: {np.max(cross_track_errors):.3f} m")
        print(f"  Replanning events: {sum(1 for e in cross_track_errors if e > self.replan_threshold)}")
        
        # GPS quality distribution
        quality_counts = {}
        for state in navigation_states:
            quality_name = state.gps_quality.name
            quality_counts[quality_name] = quality_counts.get(quality_name, 0) + 1
        
        print(f"\nGPS Quality Distribution:")
        for quality, count in quality_counts.items():
            percentage = count / len(navigation_states) * 100
            print(f"  {quality}: {count} steps ({percentage:.1f}%)")

# Example usage and demonstration
def demo_gps_navigation():
    """Demonstrate the GPS navigation system with both algorithms"""
    
    # Create terrain (you'll import this from your terrain model)
    from slopedTerrainModel import TerrainMap
    
    # Create test terrain
    terrain_map = TerrainMap(width=100, length=100, slope_percentage=1.0)
    terrain_map.add_random_obstacles(25)
    
    # Test points
    start_point = (10, 10)
    end_point = (85, 85)
    
    print("🧪 Testing GPS Navigation System with Different Algorithms")
    print("="*60)
    
    # Test A* algorithm
    print("\n🔹 Testing A* Algorithm")
    nav_system_astar = GPSNavigationSystem(terrain_map, algorithm='astar')
    
    if nav_system_astar.plan_path(start_point, end_point):
        astar_states = nav_system_astar.simulate_navigation(vehicle_speed=2.0, time_step=0.5)
        nav_system_astar.visualize_results(astar_states)
    
    print("\n" + "-"*60)
    
    # Test RRT* algorithm
    print("\n🔹 Testing RRT* Algorithm")
    nav_system_rrt = GPSNavigationSystem(terrain_map, algorithm='rrt*')
    
    if nav_system_rrt.plan_path(start_point, end_point):
        rrt_states = nav_system_rrt.simulate_navigation(vehicle_speed=2.0, time_step=0.5)
        nav_system_rrt.visualize_results(rrt_states)

if __name__ == "__main__":
    demo_gps_navigation()