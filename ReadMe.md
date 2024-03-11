# Autonomous Mine Truck Simulation

This project aims to simulate an autonomous truck navigating through a mining environment. The simulation includes vehicle dynamics, sensing capabilities, and navigation algorithms to enable the truck to autonomously traverse the terrain while avoiding obstacles. The project is structured to modularize different aspects of the simulation, including low-level control, navigation, and vehicle dynamics.

## Project Structure

The project is organized into several directories, each containing scripts related to different aspects of the simulation:

- `low_level_control/`: Contains controllers that manage the vehicle's motion.
  - `lateral_controller.py`: Manages the vehicle's steering to follow a planned path.
  - `longitudinal_controller.py`: Controls the vehicle's speed to maintain a constant velocity.
- `navigation_algorithms/`: Includes algorithms for path planning and navigation.
  - `A_star.py`: Implements the A* algorithm for grid-based path planning.
  - `rrt_star.py`: Contains the RRT* algorithm for path planning in continuous spaces.
- `sensing/`: (Intended for future development) Will include scripts related to the vehicle's sensing capabilities, such as LiDAR and GPS data processing.
- `src/`: (Intended for additional source files) Can be used for supplementary scripts and utilities.
- `terrain/`: (Intended for terrain data and simulation) Will hold data and scripts related to the simulated mining environment.
- `vehicle_dynamics/`: Contains the simulation of the vehicle's physical behavior.
  - `vehicle_model.py`: Simulates the vehicle dynamics using a bicycle model.

## Getting Started

### Prerequisites

- Python 3.x
- NumPy
- Matplotlib (for visualization)
- Any additional libraries required by specific scripts (e.g., `paho-mqtt` for sensor simulations).

### Installation

1. Clone the repository to your local machine.
2. Ensure you have Python installed, and install any dependencies as needed.

## Future Work

- **Sensing Module Development**: Integration of LiDAR and GPS data processing to enhance the vehicle's environmental awareness.
- **Terrain Simulation**: Incorporation of more complex terrain data to simulate a variety of mining environments.
- **Improved Navigation Algorithms**: Exploration of additional path planning and navigation algorithms for optimized performance.

