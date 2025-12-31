"""
Enhanced Navigation Demo - Using Existing Implementations
Tests A* and Dijkstra algorithms with the actual project code
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

# Import existing implementations
from slopedTerrainModel import TerrainMap
from A_star import a_star_search
from dijkstra import Dijkstra


def visualize_comparison(terrain, paths, algorithms, start_point, end_point, filename):
    """Create a comprehensive comparison visualization"""
    fig = plt.figure(figsize=(20, 12))
    
    # Color scheme for different algorithms
    colors = ['#FF4444', '#4444FF', '#44FF44', '#FF44FF']
    
    # 2D Overhead view with all paths
    ax1 = fig.add_subplot(2, 3, 1)
    ax1.imshow(terrain, cmap='terrain', origin='lower', interpolation='nearest', alpha=0.8)
    ax1.set_title('Path Comparison - 2D Overhead View', fontsize=14, fontweight='bold')
    ax1.plot(start_point[1], start_point[0], 'go', markersize=15, label='Start', 
             markeredgecolor='black', markeredgewidth=2)
    ax1.plot(end_point[1], end_point[0], 'r*', markersize=20, label='Goal', 
             markeredgecolor='black', markeredgewidth=2)
    
    for i, (path, algo) in enumerate(zip(paths, algorithms)):
        if path:
            x_coords, y_coords = zip(*path)
            ax1.plot(y_coords, x_coords, '-', color=colors[i], linewidth=2.5, 
                    label=algo, alpha=0.8)
    
    ax1.legend(loc='best', fontsize=10)
    ax1.set_xlabel('Y Coordinate', fontsize=11)
    ax1.set_ylabel('X Coordinate', fontsize=11)
    ax1.grid(True, alpha=0.3)
    
    # 3D View
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    X, Y = np.meshgrid(range(terrain.shape[1]), range(terrain.shape[0]))
    ax2.plot_surface(X, Y, terrain.T, cmap='terrain', alpha=0.4, antialiased=True)
    
    for i, (path, algo) in enumerate(zip(paths, algorithms)):
        if path:
            x_coords, y_coords = zip(*path)
            z_coords = [terrain[int(x), int(y)] if terrain[int(x), int(y)] != -1 else 0 
                       for x, y in path]
            ax2.plot(y_coords, x_coords, z_coords, '-', color=colors[i], 
                    linewidth=3, label=algo)
    
    ax2.set_title('3D Terrain with Planned Paths', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Y', fontsize=10)
    ax2.set_ylabel('X', fontsize=10)
    ax2.set_zlabel('Elevation', fontsize=10)
    ax2.legend(fontsize=9)
    
    # Path metrics comparison
    ax3 = fig.add_subplot(2, 3, 3)
    path_lengths = []
    for path in paths:
        if path:
            length = sum(np.hypot(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]) 
                        for i in range(len(path) - 1))
            path_lengths.append(length)
        else:
            path_lengths.append(0)
    
    bars = ax3.bar(algorithms, path_lengths, color=colors[:len(algorithms)], 
                   alpha=0.7, edgecolor='black', linewidth=2)
    ax3.set_title('Path Length Comparison', fontsize=14, fontweight='bold')
    ax3.set_ylabel('Path Length (units)', fontsize=11)
    ax3.grid(True, alpha=0.3, axis='y')
    
    # Add value labels on bars
    for bar, length in zip(bars, path_lengths):
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height,
                f'{length:.1f}', ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # Individual path visualizations
    for idx, (path, algo) in enumerate(zip(paths, algorithms)):
        ax = fig.add_subplot(2, 3, 4 + idx)
        ax.imshow(terrain, cmap='terrain', origin='lower', interpolation='nearest', alpha=0.8)
        ax.plot(start_point[1], start_point[0], 'go', markersize=12, 
               markeredgecolor='black', markeredgewidth=2)
        ax.plot(end_point[1], end_point[0], 'r*', markersize=18, 
               markeredgecolor='black', markeredgewidth=2)
        
        if path:
            x_coords, y_coords = zip(*path)
            ax.plot(y_coords, x_coords, '-', color=colors[idx], linewidth=2.5, 
                   label=f'{algo} Path')
            ax.set_title(f'{algo}\nLength: {path_lengths[idx]:.1f} units, Waypoints: {len(path)}', 
                        fontsize=12, fontweight='bold')
        else:
            ax.set_title(f'{algo}\nNo path found', fontsize=12, fontweight='bold', color='red')
        
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('Y Coordinate', fontsize=10)
        ax.set_ylabel('X Coordinate', fontsize=10)
    
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✅ Saved visualization to {filename}")
    plt.close()


def run_navigation_demo():
    """Run comprehensive navigation algorithm demo using existing implementations"""
    print("="*70)
    print("AUTONOMOUS MINE TRUCK - NAVIGATION ALGORITHM DEMONSTRATION")
    print("Using Existing Project Implementations")
    print("="*70)
    
    # Create flat terrain with obstacles
    print("\n📍 Creating test terrain...")
    np.random.seed(42)  # For reproducibility
    terrain_map = TerrainMap(width=100, length=100, slope_percentage=0)
    terrain_map.add_random_obstacles(150)
    start_point = (10, 10)
    end_point = (85, 85)
    terrain_map.set_points(start_point, end_point)
    
    # Save terrain
    terrain_map.save_map('demo_terrain.npy')
    print(f"   Terrain size: {terrain_map.width}x{terrain_map.length}")
    print(f"   Start: {start_point}, Goal: {end_point}")
    print(f"   Obstacles: 150 cells")
    print(f"   Terrain saved to: demo_terrain.npy")
    
    # Prepare obstacle set for A*
    obstacles_set = set()
    for x in range(terrain_map.width):
        for y in range(terrain_map.length):
            if terrain_map.terrain[x, y] == -1:
                obstacles_set.add((x, y))
    
    print(f"   Total obstacle cells: {len(obstacles_set)}")
    
    # Test A* algorithm
    print("\n🔷 Testing A* Algorithm...")
    print("   Algorithm: A* with 8-way movement")
    print("   Heuristic: Euclidean distance")
    
    astar_path = a_star_search(start_point, end_point, obstacles_set, 
                               (terrain_map.width, terrain_map.length), movement='8way')
    
    if astar_path:
        astar_length = sum(np.hypot(astar_path[i+1][0] - astar_path[i][0], 
                                    astar_path[i+1][1] - astar_path[i][1]) 
                          for i in range(len(astar_path) - 1))
        straight_line = np.hypot(end_point[0] - start_point[0], end_point[1] - start_point[1])
        efficiency = (straight_line / astar_length) * 100
        
        print(f"   ✅ Path found!")
        print(f"   Path length: {astar_length:.2f} units")
        print(f"   Waypoints: {len(astar_path)}")
        print(f"   Efficiency: {efficiency:.1f}%")
    else:
        print(f"   ❌ No path found")
    
    # Test Dijkstra algorithm
    print("\n🔷 Testing Dijkstra Algorithm...")
    print("   Algorithm: Dijkstra with 4-way movement")
    print("   Max elevation difference: 5.0 units")
    
    dijkstra = Dijkstra(terrain_map.terrain, start_point, end_point, max_elevation_diff=5.0)
    dijkstra_path = dijkstra.plan()
    
    if dijkstra_path:
        dijkstra_length = sum(np.hypot(dijkstra_path[i+1][0] - dijkstra_path[i][0], 
                                       dijkstra_path[i+1][1] - dijkstra_path[i][1]) 
                             for i in range(len(dijkstra_path) - 1))
        straight_line = np.hypot(end_point[0] - start_point[0], end_point[1] - start_point[1])
        efficiency = (straight_line / dijkstra_length) * 100
        
        print(f"   ✅ Path found!")
        print(f"   Path length: {dijkstra_length:.2f} units")
        print(f"   Waypoints: {len(dijkstra_path)}")
        print(f"   Efficiency: {efficiency:.1f}%")
    else:
        print(f"   ❌ No path found")
    
    # Create visualizations
    print("\n📊 Generating comprehensive visualizations...")
    paths = [astar_path, dijkstra_path]
    algorithms = ['A* (8-way)', 'Dijkstra (4-way)']
    
    visualize_comparison(terrain_map.terrain, paths, algorithms, 
                        start_point, end_point, 
                        'navigation_comparison.png')
    
    # Print summary
    print("\n" + "="*70)
    print("RESULTS SUMMARY")
    print("="*70)
    
    if astar_path and dijkstra_path:
        print(f"\n🏆 Both algorithms successfully found paths!")
        
        print(f"\nA* Algorithm (8-way movement):")
        print(f"  - Path length: {astar_length:.2f} units")
        print(f"  - Waypoints: {len(astar_path)}")
        print(f"  - Efficiency: {(straight_line/astar_length)*100:.1f}%")
        print(f"  - First 5 waypoints: {astar_path[:5]}")
        
        print(f"\nDijkstra Algorithm (4-way movement):")
        print(f"  - Path length: {dijkstra_length:.2f} units")
        print(f"  - Waypoints: {len(dijkstra_path)}")
        print(f"  - Efficiency: {(straight_line/dijkstra_length)*100:.1f}%")
        print(f"  - First 5 waypoints: {dijkstra_path[:5]}")
        
        diff = abs(astar_length - dijkstra_length)
        print(f"\nComparison:")
        print(f"  - Path length difference: {diff:.2f} units ({(diff/min(astar_length, dijkstra_length)*100):.1f}%)")
        print(f"  - Straight-line distance: {straight_line:.2f} units")
        
        if astar_length < dijkstra_length:
            print(f"  - A* found a {((dijkstra_length-astar_length)/dijkstra_length*100):.1f}% shorter path")
        elif dijkstra_length < astar_length:
            print(f"  - Dijkstra found a {((astar_length-dijkstra_length)/astar_length*100):.1f}% shorter path")
        else:
            print(f"  - Both algorithms found paths of equal length")
    
    print("\n📁 Generated files:")
    print("   - navigation_comparison.png (visualization)")
    print("   - demo_terrain.npy (terrain data)")
    
    print("\n✅ Demo complete!")
    print("="*70)
    
    return terrain_map, astar_path, dijkstra_path


if __name__ == "__main__":
    terrain_map, astar_path, dijkstra_path = run_navigation_demo()