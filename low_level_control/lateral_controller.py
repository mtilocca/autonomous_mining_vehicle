import math
import numpy as np

class LateralController:
    def __init__(self, k_p:floatt=1.0, wheelbase_length:float=2.0)-> None:
        """
        Initializes the Lateral Controller.

        :param k_p: Proportional gain for the lateral controller.
        :param wheelbase_length: The length between the front and rear wheels of the vehicle.
        """
        self.k_p = k_p
        self.wheelbase_length = wheelbase_length

    def compute_steering(self, vehicle_x : float, vehicle_y : float , vehicle_heading : float, waypoints : float) -> float:
        """
        Computes the steering angle needed to follow the waypoints.

        :param vehicle_x: The current x position of the vehicle.
        :param vehicle_y: The current y position of the vehicle.
        :param vehicle_heading: The current heading of the vehicle in radians.
        :param waypoints: A list of waypoints [(x1, y1), (x2, y2), ...] the vehicle needs to follow.
        :return: The computed steering angle in radians.
        """
        # Find the closest waypoint
        closest_dist = float('inf')
        for waypoint in waypoints:
            dist = np.hypot(waypoint[0] - vehicle_x, waypoint[1] - vehicle_y)
            if dist < closest_dist:
                closest_dist = dist
                closest_waypoint = waypoint

        # Compute the cross track error (assuming closest waypoint as the reference point)
        cross_track_error = np.hypot(vehicle_x - closest_waypoint[0], vehicle_y - closest_waypoint[1])

        # Compute the heading error
        path_angle = math.atan2(closest_waypoint[1] - vehicle_y, closest_waypoint[0] - vehicle_x)
        heading_error = path_angle - vehicle_heading

        # Adjust for the fact that atan2 returns values between -pi and pi
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Proportional control for steering angle
        steering_angle = heading_error + math.atan(self.k_p * cross_track_error / self.wheelbase_length)

        return steering_angle
