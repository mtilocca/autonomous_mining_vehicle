class AdaptiveCruiseControl3D:
    def __init__(self, target_speed, time_gap, max_acceleration, max_deceleration):
        """
        Initializes the 3D Adaptive Cruise Control system.

        :param target_speed: The desired speed of the vehicle (m/s).
        :param time_gap: The desired time gap to the leading vehicle (seconds).
        :param max_acceleration: Maximum acceleration capacity of the vehicle (m/s^2).
        :param max_deceleration: Maximum deceleration capacity of the vehicle (m/s^2).
        """
        self.target_speed = target_speed
        self.time_gap = time_gap
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration

    def compute_control(self, current_speed, distance_to_leading_vehicle, leading_vehicle_speed, current_z=0.0, leading_vehicle_z=0.0):
        """
        Computes the acceleration or deceleration needed to maintain a safe following distance, target speed, and adjust for elevation changes if necessary.

        :param current_speed: The current speed of the vehicle (m/s).
        :param distance_to_leading_vehicle: The current distance to the leading vehicle (meters), primarily in the xy-plane.
        :param leading_vehicle_speed: The speed of the leading vehicle (m/s).
        :param current_z: The current elevation (z) of the vehicle (optional).
        :param leading_vehicle_z: The elevation (z) of the leading vehicle (optional).
        :return: The acceleration (or deceleration if negative) command (m/s^2).
        """
        # Desired following distance based on current speed and time gap
        desired_distance = current_speed * self.time_gap
        
        # Distance error, considering elevation difference if significant
        elevation_difference = abs(current_z - leading_vehicle_z)
        distance_error = distance_to_leading_vehicle + elevation_difference - desired_distance
        
        # Speed error
        speed_error = self.target_speed - current_speed
        
        # Simple proportional controller for demonstration (P-controller)
        acceleration_command = speed_error + distance_error * 0.1
        
        # Adjust acceleration based on the relative speed of the leading vehicle
        relative_speed = current_speed - leading_vehicle_speed
        if distance_to_leading_vehicle < desired_distance and relative_speed > 0:
            # Decelerate more aggressively if too close and approaching the leading vehicle
            acceleration_command -= relative_speed * 0.5
        
        # Ensure acceleration command is within vehicle capabilities
        acceleration_command = max(min(acceleration_command, self.max_acceleration), -self.max_deceleration)
        
        return acceleration_command
