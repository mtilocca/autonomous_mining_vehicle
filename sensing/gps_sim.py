import numpy as np
import random
import time
import math
from dataclasses import dataclass
from typing import Tuple, Optional, List
from enum import Enum

class GPSQuality(Enum):
    NO_FIX = 0
    STANDARD_GPS = 1
    DGPS = 2
    RTK_FLOAT = 5
    RTK_FIXED = 4

@dataclass
class GPSReading:
    """Represents a GPS reading with accuracy and quality information"""
    latitude: float
    longitude: float
    elevation: float
    accuracy_horizontal: float  # meters
    accuracy_vertical: float    # meters
    quality: GPSQuality
    num_satellites: int
    timestamp: float
    true_position: Optional[Tuple[float, float, float]] = None

class GPSRTKSimulator:
    """
    Simulates GPS RTK corrected sensing with realistic error models
    """
    
    def __init__(self, terrain_map, base_station_pos=None):
        self.terrain_map = terrain_map
        self.base_station_pos = base_station_pos or (terrain_map.width//2, terrain_map.length//2)
        
        # GPS error characteristics (in meters)
        self.error_params = {
            GPSQuality.STANDARD_GPS: {
                'horizontal_std': 3.0,
                'vertical_std': 5.0,
                'accuracy_95': 5.0
            },
            GPSQuality.DGPS: {
                'horizontal_std': 1.0,
                'vertical_std': 2.0,
                'accuracy_95': 2.0
            },
            GPSQuality.RTK_FLOAT: {
                'horizontal_std': 0.3,
                'vertical_std': 0.5,
                'accuracy_95': 0.6
            },
            GPSQuality.RTK_FIXED: {
                'horizontal_std': 0.02,
                'vertical_std': 0.05,
                'accuracy_95': 0.05
            }
        }
        
        # Simulate atmospheric and multipath effects
        self.atmospheric_error_scale = 1.0
        self.multipath_probability = 0.1
        
    def _calculate_distance_to_base(self, position: Tuple[float, float]) -> float:
        """Calculate distance to RTK base station"""
        dx = position[0] - self.base_station_pos[0]
        dy = position[1] - self.base_station_pos[1]
        return np.sqrt(dx*dx + dy*dy)
    
    def _determine_gps_quality(self, position: Tuple[float, float], 
                              num_satellites: int) -> GPSQuality:
        """Determine GPS quality based on conditions"""
        distance_to_base = self._calculate_distance_to_base(position)
        
        # RTK corrections degrade with distance
        if num_satellites < 4:
            return GPSQuality.NO_FIX
        elif distance_to_base < 10000 and num_satellites >= 8:  # 10km RTK range
            if random.random() > 0.1:  # 90% chance of RTK fixed
                return GPSQuality.RTK_FIXED
            else:
                return GPSQuality.RTK_FLOAT
        elif distance_to_base < 20000 and num_satellites >= 6:  # Extended range
            return GPSQuality.RTK_FLOAT
        elif num_satellites >= 6:
            return GPSQuality.DGPS
        else:
            return GPSQuality.STANDARD_GPS
    
    def _simulate_satellite_count(self, position: Tuple[float, float]) -> int:
        """Simulate number of visible satellites based on terrain"""
        base_satellites = 12
        
        # Terrain blocking (simplified model)
        elevation = self.terrain_map.terrain[int(position[0]), int(position[1])]
        if elevation < -0.5:  # In obstacle
            base_satellites -= 4
        
        # Add some randomness for atmospheric conditions
        satellite_variation = random.randint(-3, 2)
        return max(4, min(16, base_satellites + satellite_variation))
    
    def _add_gps_errors(self, true_pos: Tuple[float, float, float], 
                       quality: GPSQuality) -> Tuple[float, float, float]:
        """Add realistic GPS errors based on quality"""
        if quality == GPSQuality.NO_FIX:
            return true_pos  # Return true position if no fix (shouldn't happen)
        
        params = self.error_params[quality]
        
        # Base error
        h_error = np.random.normal(0, params['horizontal_std'])
        v_error = np.random.normal(0, params['vertical_std'])
        
        # Apply atmospheric scaling
        h_error *= self.atmospheric_error_scale
        v_error *= self.atmospheric_error_scale
        
        # Add multipath effects (sudden jumps)
        if random.random() < self.multipath_probability:
            h_error += np.random.normal(0, params['horizontal_std'] * 2)
        
        # Convert to x,y components
        error_angle = random.uniform(0, 2*np.pi)
        x_error = h_error * np.cos(error_angle)
        y_error = h_error * np.sin(error_angle)
        
        return (
            true_pos[0] + x_error,
            true_pos[1] + y_error,
            true_pos[2] + v_error
        )
    
    def get_gps_reading(self, true_position: Tuple[float, float, float]) -> GPSReading:
        """
        Get a simulated GPS reading for a given true position
        
        Args:
            true_position: (x, y, elevation) in terrain coordinates
            
        Returns:
            GPSReading object with simulated GPS data
        """
        # Clamp position to terrain bounds
        x = max(0, min(self.terrain_map.width-1, true_position[0]))
        y = max(0, min(self.terrain_map.length-1, true_position[1]))
        
        # Get terrain elevation
        terrain_elevation = self.terrain_map.terrain[int(x), int(y)]
        true_pos_clamped = (x, y, terrain_elevation)
        
        # Simulate satellite conditions
        num_satellites = self._simulate_satellite_count(true_pos_clamped)
        quality = self._determine_gps_quality(true_pos_clamped, num_satellites)
        
        # Add GPS errors
        measured_pos = self._add_gps_errors(true_pos_clamped, quality)
        
        # Calculate accuracy estimates
        params = self.error_params[quality]
        
        return GPSReading(
            latitude=measured_pos[0],  # Using x as latitude for simplicity
            longitude=measured_pos[1], # Using y as longitude for simplicity
            elevation=measured_pos[2],
            accuracy_horizontal=params['accuracy_95'],
            accuracy_vertical=params['accuracy_95'] * 1.5,
            quality=quality,
            num_satellites=num_satellites,
            timestamp=time.time(),
            true_position=true_pos_clamped
        )
    
    def simulate_trajectory(self, waypoints: List[Tuple[float, float]], 
                          samples_per_segment: int = 10) -> List[GPSReading]:
        """
        Simulate GPS readings along a trajectory
        
        Args:
            waypoints: List of (x, y) waypoints
            samples_per_segment: Number of GPS samples between waypoints
            
        Returns:
            List of GPSReading objects
        """
        readings = []
        
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            
            for j in range(samples_per_segment):
                t = j / samples_per_segment
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                
                # Get terrain elevation
                terrain_z = self.terrain_map.terrain[int(x), int(y)]
                
                reading = self.get_gps_reading((x, y, terrain_z))
                readings.append(reading)
                
                # Simulate time delay
                time.sleep(0.01)  # 100Hz update rate
        
        return readings
    
    def calculate_position_error(self, gps_reading: GPSReading) -> float:
        """Calculate horizontal position error from GPS reading"""
        if not gps_reading.true_position:
            return 0.0
        
        return math.sqrt(
            (gps_reading.latitude - gps_reading.true_position[0])**2 +
            (gps_reading.longitude - gps_reading.true_position[1])**2
        )
    
    def get_performance_stats(self, readings: List[GPSReading]) -> dict:
        """Get performance statistics for a series of GPS readings"""
        if not readings:
            return {}
        
        errors = [self.calculate_position_error(r) for r in readings]
        quality_counts = {}
        
        for reading in readings:
            quality_name = reading.quality.name
            quality_counts[quality_name] = quality_counts.get(quality_name, 0) + 1
        
        return {
            'mean_error': np.mean(errors),
            'rms_error': np.sqrt(np.mean(np.array(errors)**2)),
            'max_error': np.max(errors),
            'percentile_95': np.percentile(errors, 95),
            'quality_distribution': quality_counts,
            'total_readings': len(readings)
        }