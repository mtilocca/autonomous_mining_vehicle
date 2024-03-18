import numpy as np
from A_star import AStar
from rrt_star import RRTStar
from terrain_map import TerrainMap
import matplotlib.pyplot as plt
import numpy as np


def visualize_path(terrain, path, start_point, end_point):
    """
    Visualizes the terrain and the path from start to end point.

    :param terrain: 2D numpy array representing the terrain elevation.
    :param path: List of tuples representing the path coordinates.
    :param start_point: Tuple representing the start coordinates.
    :param end_point: Tuple representing the end coordinates.
    """
    plt.figure(figsize=(10, 10))
    
    # Plot the terrain as a heatmap
    plt.imshow(terrain, cmap='terrain', origin='lower', interpolation='nearest')
    plt.colorbar(label='Elevation')
    
    # Plot start and end points
    plt.plot(start_point[1], start_point[0], 'go', markersize=10, label='Start')  # Green for start
    plt.plot(end_point[1], end_point[0], 'ro', markersize=10, label='End')  # Red for end
    
    # Plot the path
    if path is not None:
        # Unzip the path coordinates for plotting
        x_coords, y_coords = zip(*path)
        plt.plot(y_coords, x_coords, 'b-', label='Path')  # Blue for path
    
    plt.legend()
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Path Over Terrain')
    plt.show()

def load_terrain(filename):
    return np.load(filename)

def main():
    # Load terrains
    flat_terrain = load_terrain('flat_terrain_with_obstacles.npy')
    sloped_terrain = load_terrain('sloped_terrain_with_obstacles.npy')

    # Specify start and end points
    start_point = (20, 30)  # Example coordinates
    end_point = (280, 270)  # Example coordinates

    # Choose the terrain and planning algorithm
    terrain = flat_terrain  # or sloped_terrain, depending on the scenario
    algorithm = AStar(start_point, end_point, terrain)  # or RRTStar, as needed

    # Calculate the path
    path = algorithm.find_path()
    
    visualize_path(terrain, path, start_point, end_point)

if __name__ == "__main__":
    main()
