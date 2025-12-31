#!/usr/bin/env python3
"""
Simple runner script for autonomous mining vehicle demonstrations
Run this from the project root directory
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Suppress stdout during imports to avoid example code output
import io
from contextlib import redirect_stdout

# Add paths to import modules
sys.path.append(os.path.join(os.path.dirname(__file__), 'terrain'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'navigation_algorithms'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'vehicle_dynamics'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'low_level_control'))

from slopedTerrainModel import TerrainMap

# Import A* with suppressed output
print("Loading modules...")
f = io.StringIO()
with redirect_stdout(f):
    from A_star import a_star_search

# Import Dijkstra - need to check if it has the __lt__ method
try:
    f = io.StringIO()
    with redirect_stdout(f):
        from dijkstra import Dijkstra
    print("✅ Modules loaded successfully")
except Exception as e:
    print(f"❌ Error loading dijkstra module: {e}")
    print("\nPlease run fix_dijkstra.py first:")
    print("  python3 fix_dijkstra.py")
    print("\nThen run this script again.")
    sys.exit(1)

from vehicle_dynamics.vehicle_model import BicycleModel3D
from low_level_control.StanleyLateralController import StanleyLateralController
import math


def create_navigation_comparison():
    """Generate navigation algorithm comparison"""
    print("\n" + "="*70)
    print("GENERATING NAVIGATION COMPARISON")
    print("="*70)
    
    # Create terrain
    print("\n📍 Creating terrain...")
    np.random.seed(42)
    terrain_map = TerrainMap(width=100, length=100, slope_percentage=0)
    terrain_map.add_random_obstacles(150)
    start_point = (10, 10)
    end_point = (85, 85)
    
    print(f"   Terrain: {terrain_map.width}x{terrain_map.length}")
    print(f"   Start: {start_point}, Goal: {end_point}")
    
    # A* path planning
    print("\n🔷 Running A* algorithm...")
    obstacles_set = set()
    for x in range(terrain_map.width):
        for y in range(terrain_map.length):
            if terrain_map.terrain[x, y] == -1:
                obstacles_set.add((x, y))
    
    astar_path = a_star_search(start_point, end_point, obstacles_set, 
                               (terrain_map.width, terrain_map.length), movement='8way')
    
    if not astar_path:
        print("   ❌ A* failed to find path")
        return None, None
    
    astar_length = sum(np.hypot(astar_path[i+1][0] - astar_path[i][0], 
                                astar_path[i+1][1] - astar_path[i][1]) 
                      for i in range(len(astar_path) - 1))
    print(f"   ✅ A* path: {len(astar_path)} waypoints, {astar_length:.2f} units")
    
    # Dijkstra path planning
    print("\n🔷 Running Dijkstra algorithm...")
    dijkstra = Dijkstra(terrain_map.terrain, start_point, end_point, max_elevation_diff=5.0)
    dijkstra_path = dijkstra.plan()
    
    if not dijkstra_path:
        print("   ❌ Dijkstra failed to find path")
        return None, None
    
    dijkstra_length = sum(np.hypot(dijkstra_path[i+1][0] - dijkstra_path[i][0], 
                                   dijkstra_path[i+1][1] - dijkstra_path[i][1]) 
                         for i in range(len(dijkstra_path) - 1))
    print(f"   ✅ Dijkstra path: {len(dijkstra_path)} waypoints, {dijkstra_length:.2f} units")
    
    # Create visualization
    print("\n📊 Creating visualization...")
    fig = plt.figure(figsize=(20, 12))
    
    # 2D comparison
    ax1 = fig.add_subplot(2, 3, 1)
    ax1.imshow(terrain_map.terrain, cmap='terrain', origin='lower', alpha=0.8)
    
    if astar_path:
        x_coords, y_coords = zip(*astar_path)
        ax1.plot(y_coords, x_coords, 'r-', linewidth=2.5, label='A* (8-way)', alpha=0.8)
    if dijkstra_path:
        x_coords, y_coords = zip(*dijkstra_path)
        ax1.plot(y_coords, x_coords, 'b-', linewidth=2.5, label='Dijkstra (4-way)', alpha=0.8)
    
    ax1.plot(start_point[1], start_point[0], 'go', markersize=15, label='Start')
    ax1.plot(end_point[1], end_point[0], 'r*', markersize=20, label='Goal')
    ax1.legend()
    ax1.set_title('Path Comparison - 2D View', fontsize=14, fontweight='bold')
    ax1.set_xlabel('Y Coordinate')
    ax1.set_ylabel('X Coordinate')
    ax1.grid(True, alpha=0.3)
    
    # 3D view
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    X, Y = np.meshgrid(range(terrain_map.width), range(terrain_map.length))
    ax2.plot_surface(X, Y, terrain_map.terrain.T, cmap='terrain', alpha=0.4)
    
    if astar_path:
        x_coords, y_coords = zip(*astar_path)
        z_coords = [terrain_map.terrain[int(x), int(y)] for x, y in astar_path]
        ax2.plot(y_coords, x_coords, z_coords, 'r-', linewidth=3, label='A*')
    if dijkstra_path:
        x_coords, y_coords = zip(*dijkstra_path)
        z_coords = [terrain_map.terrain[int(x), int(y)] for x, y in dijkstra_path]
        ax2.plot(y_coords, x_coords, z_coords, 'b-', linewidth=3, label='Dijkstra')
    
    ax2.set_title('3D Terrain with Paths', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Y')
    ax2.set_ylabel('X')
    ax2.set_zlabel('Elevation')
    ax2.legend()
    
    # Path length comparison
    ax3 = fig.add_subplot(2, 3, 3)
    algorithms = ['A*', 'Dijkstra']
    lengths = [astar_length, dijkstra_length]
    bars = ax3.bar(algorithms, lengths, color=['#FF4444', '#4444FF'], alpha=0.7, edgecolor='black', linewidth=2)
    ax3.set_title('Path Length Comparison', fontsize=14, fontweight='bold')
    ax3.set_ylabel('Path Length (units)')
    ax3.grid(True, alpha=0.3, axis='y')
    
    for bar, length in zip(bars, lengths):
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height,
                f'{length:.1f}', ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # Individual paths
    for idx, (path, algo, color) in enumerate([(astar_path, 'A*', '#FF4444'), 
                                                 (dijkstra_path, 'Dijkstra', '#4444FF')]):
        ax = fig.add_subplot(2, 3, 4 + idx)
        ax.imshow(terrain_map.terrain, cmap='terrain', origin='lower', alpha=0.8)
        ax.plot(start_point[1], start_point[0], 'go', markersize=12, markeredgecolor='black', markeredgewidth=2)
        ax.plot(end_point[1], end_point[0], 'r*', markersize=18, markeredgecolor='black', markeredgewidth=2)
        
        if path:
            x_coords, y_coords = zip(*path)
            ax.plot(y_coords, x_coords, '-', color=color, linewidth=2.5, label=f'{algo} Path')
            ax.set_title(f'{algo}\nLength: {lengths[idx]:.1f} units, Waypoints: {len(path)}', 
                        fontsize=12, fontweight='bold')
        
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('Y Coordinate')
        ax.set_ylabel('X Coordinate')
    
    plt.tight_layout()
    plt.savefig('navigation_comparison.png', dpi=300, bbox_inches='tight')
    print(f"   ✅ Saved: navigation_comparison.png")
    plt.close()
    
    print("\n" + "="*70)
    print("RESULTS")
    print("="*70)
    print(f"A*: {astar_length:.2f} units ({len(astar_path)} waypoints)")
    print(f"Dijkstra: {dijkstra_length:.2f} units ({len(dijkstra_path)} waypoints)")
    diff_percent = ((dijkstra_length-astar_length)/dijkstra_length*100)
    if diff_percent > 0:
        print(f"A* is {diff_percent:.1f}% shorter")
    else:
        print(f"Dijkstra is {-diff_percent:.1f}% shorter")
    print("="*70)
    
    return terrain_map, astar_path


def create_vehicle_simulation(terrain_map, path):
    """Generate vehicle simulation following path"""
    print("\n" + "="*70)
    print("GENERATING VEHICLE SIMULATION")
    print("="*70)
    
    # Initialize vehicle at start
    start = path[0]
    vehicle = BicycleModel3D(length=2.5, velocity=1.5, x=float(start[0]), y=float(start[1]), z=0.0)
    controller = StanleyLateralController(k_p=0.5, wheelbase_length=2.5)
    
    print(f"\n🚗 Vehicle parameters:")
    print(f"   Wheelbase: 2.5 m")
    print(f"   Velocity: 1.5 m/s")
    print(f"   Controller gain: 0.5")
    print(f"   Path waypoints: {len(path)}")
    
    # Simulation
    print(f"\n🚗 Running simulation...")
    dt = 0.1
    current_waypoint_idx = 0
    position_history = []
    cross_track_errors = []
    steering_history = []
    
    max_steps = 1000
    for step in range(max_steps):
        state = vehicle.get_state()
        
        # Get waypoints for controller
        lookahead = min(10, len(path) - current_waypoint_idx)
        waypoints = path[current_waypoint_idx:current_waypoint_idx + lookahead]
        
        if not waypoints:
            break
        
        # Compute steering
        steering = controller.compute_steering(state['x'], state['y'], state['heading'], waypoints)
        steering = max(-math.radians(30), min(math.radians(30), steering))
        
        # Update vehicle
        vehicle.update(steering, 1.5, 0.0, 0.0, dt)
        
        # Track history
        new_state = vehicle.get_state()
        position_history.append((new_state['x'], new_state['y'], new_state['z']))
        steering_history.append(steering)
        
        # Calculate cross-track error
        target = path[current_waypoint_idx]
        cte = math.sqrt((new_state['x'] - target[0])**2 + (new_state['y'] - target[1])**2)
        cross_track_errors.append(cte)
        
        # Advance waypoint
        if cte < 2.0:
            current_waypoint_idx += 1
            if current_waypoint_idx >= len(path):
                print(f"   ✅ Reached goal at step {step}!")
                break
        
        if step % 100 == 0:
            print(f"   Step {step}: Position ({new_state['x']:.1f}, {new_state['y']:.1f}), "
                  f"Waypoint {current_waypoint_idx}/{len(path)}")
    
    print(f"   ✅ Simulation complete: {len(position_history)} steps")
    
    if not position_history:
        print("   ❌ No position history recorded")
        return
    
    # Visualization
    print(f"\n📊 Creating visualization...")
    fig = plt.figure(figsize=(20, 12))
    
    x_hist, y_hist, z_hist = zip(*position_history)
    
    # 2D trajectory
    ax1 = fig.add_subplot(2, 3, 1)
    ax1.imshow(terrain_map.terrain, cmap='terrain', origin='lower', alpha=0.7)
    
    path_x, path_y = zip(*path)
    ax1.plot(path_y, path_x, 'b--', linewidth=2, label='Planned Path', alpha=0.6)
    ax1.plot(y_hist, x_hist, 'r-', linewidth=2.5, label='Vehicle Trajectory')
    ax1.plot(y_hist[0], x_hist[0], 'go', markersize=12, label='Start', markeredgecolor='black', markeredgewidth=2)
    ax1.plot(y_hist[-1], x_hist[-1], 'r*', markersize=18, label='End', markeredgecolor='black', markeredgewidth=2)
    
    ax1.set_title('Vehicle Trajectory - 2D', fontsize=14, fontweight='bold')
    ax1.set_xlabel('Y Coordinate')
    ax1.set_ylabel('X Coordinate')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 3D trajectory
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    X, Y = np.meshgrid(range(terrain_map.width), range(terrain_map.length))
    ax2.plot_surface(X, Y, terrain_map.terrain.T, cmap='terrain', alpha=0.3)
    ax2.plot(y_hist, x_hist, z_hist, 'r-', linewidth=3, label='Vehicle Path')
    ax2.scatter(y_hist[0], x_hist[0], z_hist[0], c='green', s=100, label='Start')
    ax2.scatter(y_hist[-1], x_hist[-1], z_hist[-1], c='red', s=100, marker='*', label='End')
    ax2.set_title('3D Vehicle Trajectory', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Y')
    ax2.set_ylabel('X')
    ax2.set_zlabel('Elevation')
    ax2.legend()
    
    # Cross-track error
    ax3 = fig.add_subplot(2, 3, 3)
    time_steps = np.arange(len(cross_track_errors)) * dt
    ax3.plot(time_steps, cross_track_errors, 'b-', linewidth=2)
    ax3.axhline(y=2.0, color='r', linestyle='--', label='Waypoint threshold')
    ax3.set_title('Cross-Track Error', fontsize=14, fontweight='bold')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Error (m)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Steering angle
    ax4 = fig.add_subplot(2, 3, 4)
    steering_deg = [math.degrees(s) for s in steering_history]
    ax4.plot(time_steps, steering_deg, 'g-', linewidth=2)
    ax4.set_title('Steering Angle', fontsize=14, fontweight='bold')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Angle (degrees)')
    ax4.grid(True, alpha=0.3)
    
    # Performance metrics
    ax5 = fig.add_subplot(2, 3, 5)
    metrics = {
        'Avg CTE\n(m)': np.mean(cross_track_errors),
        'Max CTE\n(m)': np.max(cross_track_errors),
        'Avg Steering\n(°)': np.mean(np.abs(steering_deg)),
        'Distance\n(m)': sum(math.sqrt((x_hist[i+1]-x_hist[i])**2 + (y_hist[i+1]-y_hist[i])**2)
                           for i in range(len(x_hist)-1))
    }
    
    bars = ax5.bar(range(len(metrics)), list(metrics.values()), 
                   color=['skyblue', 'lightcoral', 'lightgreen', 'purple'],
                   edgecolor='black', linewidth=1.5)
    ax5.set_xticks(range(len(metrics)))
    ax5.set_xticklabels(list(metrics.keys()), fontsize=10)
    ax5.set_title('Performance Metrics', fontsize=14, fontweight='bold')
    ax5.grid(True, alpha=0.3, axis='y')
    
    for bar, value in zip(bars, metrics.values()):
        height = bar.get_height()
        ax5.text(bar.get_x() + bar.get_width()/2., height,
                f'{value:.2f}', ha='center', va='bottom', fontsize=9, fontweight='bold')
    
    # Path following quality over distance
    ax6 = fig.add_subplot(2, 3, 6)
    distances = [0]
    for i in range(len(x_hist)-1):
        distances.append(distances[-1] + math.sqrt((x_hist[i+1]-x_hist[i])**2 + (y_hist[i+1]-y_hist[i])**2))
    ax6.plot(distances, cross_track_errors, 'purple', linewidth=2)
    ax6.set_title('Cross-Track Error vs Distance', fontsize=14, fontweight='bold')
    ax6.set_xlabel('Distance Traveled (m)')
    ax6.set_ylabel('Cross-Track Error (m)')
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('vehicle_simulation.png', dpi=300, bbox_inches='tight')
    print(f"   ✅ Saved: vehicle_simulation.png")
    plt.close()
    
    print("\n" + "="*70)
    print("VEHICLE PERFORMANCE")
    print("="*70)
    for key, value in metrics.items():
        print(f"{key.replace(chr(10), ' ')}: {value:.2f}")
    print(f"Waypoints reached: {current_waypoint_idx}/{len(path)} ({current_waypoint_idx/len(path)*100:.1f}%)")
    print(f"Total time: {len(position_history) * dt:.2f} seconds")
    print("="*70)


if __name__ == "__main__":
    print("\n🚀 AUTONOMOUS MINING VEHICLE DEMONSTRATION\n")
    
    # Generate navigation comparison
    terrain_map, astar_path = create_navigation_comparison()
    
    if terrain_map is None or astar_path is None:
        print("\n❌ Failed to generate navigation paths")
        sys.exit(1)
    
    # Generate vehicle simulation
    create_vehicle_simulation(terrain_map, astar_path)
    
    print("\n✅ All visualizations generated successfully!")
    print("   📄 navigation_comparison.png")
    print("   📄 vehicle_simulation.png")
    print("\n")