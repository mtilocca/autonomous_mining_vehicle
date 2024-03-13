import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class TerrainMap:
    def __init__(self, width=300, length=300, slope_percentage=0):
        self.width = width
        self.length = length
        self.slope_percentage = slope_percentage
        self.terrain = self.create_map()
        self.start_point = None
        self.end_point = None

    def create_map(self):
        terrain = np.zeros((self.width, self.length))
        if self.slope_percentage != 0:
            for x in range(self.width):
                elevation_increase = (self.slope_percentage / 100) * x
                terrain[x, :] = elevation_increase
        return terrain

    def add_random_obstacles(self, num_obstacles):
        for _ in range(num_obstacles):
            x, y = np.random.randint(0, self.width), np.random.randint(0, self.length)
            self.terrain[x, y] = -1  # Marking the cell as an obstacle

    def set_points(self, start, end):
        """
        Sets the start and end points for the terrain map.

        :param start: A tuple (x, y) representing the start point.
        :param end: A tuple (x, y) representing the end point.
        """
        self.start_point = start
        self.end_point = end

    def visualize(self, title='Terrain Visualization'):
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')
        X, Y = np.meshgrid(range(self.width), range(self.length))
        ax.plot_surface(X, Y, self.terrain, cmap='terrain', alpha=0.7)

        if self.start_point and self.end_point:
            ax.scatter(*self.start_point, self.terrain[self.start_point], color='green', s=100, label='Start')
            ax.scatter(*self.end_point, self.terrain[self.end_point], color='red', s=100, label='End')

        ax.set_title(title)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Elevation')
        ax.legend()
        plt.tight_layout()
        plt.show()

    def save_map(self, filename):
        np.save(filename, self.terrain)

# Example Usage 

# Example usage
terrain_map = TerrainMap()
terrain_map.add_random_obstacles(90)
terrain_map.set_points(start=(20, 30), end=(180, 270))
terrain_map.visualize('Terrain with Start and End Points')
terrain_map.save_map('terrain_with_obstacles_and_points.npy')


# # Create a sloped terrain with a 2.5% incline
# sloped_terrain = TerrainMap(slope_percentage=2.5)
# sloped_terrain.add_random_obstacles(50)
# sloped_terrain.visualize('Sloped Terrain (2.5% Incline) with Obstacles')
# sloped_terrain.save_map('sloped_terrain_with_obstacles.npy')
